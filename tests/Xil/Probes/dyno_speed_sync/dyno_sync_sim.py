"""Dyno / CARLA forced-speed-match study (#323).

THE QUESTION
    The bench holds a real vehicle against a road load. CARLA meanwhile runs its
    own vehicle with its own mass and its own resistance. We tie them together by
    overwriting CARLA's speed with the bench's speed every sync interval -- what
    ``set_target_velocity`` does. How much does that paper over, and what does it
    cost?

WHAT IS REAL HERE AND WHAT IS NOT
    The bench is ``CommonLib.xil.DynoSimulator`` in chassis mode -- the shipped
    simulator, not a copy. The CARLA side is a surrogate: a longitudinal ODE
    written here, parameterised from constants measured off a live 0.9.16 server
    (``CdA = 0.377 m^2``, ``rho = 1.25``, ``mass = 1845 kg``). No CARLA server is
    contacted. This studies the coupling LAW, not CARLA's behaviour.

    Both plants are driven by pedals, from the same driver, because that is what
    the FIXS ego receives -- a CARLA agent writes throttle and brake through
    ``ego.set`` (FIXS_Applications#59). Feeding the two plants anything else
    would compare two different inputs.

    They share a powertrain envelope on purpose. The bench then differs from the
    surrogate in exactly three places -- resistance curve, mass, and the bench's
    torque delivery lag -- and each is separable. Handing the surrogate CARLA's
    real torque map is the next step and belongs with a server-connected run.

WHY THE COUPLING IS NOT FREE
    Between two syncs the surrogate integrates its own equation and drifts; at
    the next sync we snap it back. That snap is a momentum injection nobody
    accounted for:

        F_sync[k] = m_carla * (v_bench[k] - v_carla_before_snap[k]) / T_sync

    It is recorded, never applied -- the mechanism is the assignment. Its
    steady-state part is simply the disagreement between the two resistance
    models, a floor that no sync rate removes.

BASELINES
    free           CARLA runs open loop on its own driver, never synced
    track          CARLA's own driver closes the loop on the bench speed
    forced_coast   CARLA coasts and is snapped: the sync ends up supplying the
                   whole tractive effort. Pessimistic bound, not the architecture.
    forced_driven  CARLA drives itself AND is snapped.   <- the scheme under test

A NOTE ON SYNC RATE
    ``F_sync = m*dv/T`` and ``dv`` accumulates roughly linearly in ``T``, so the
    injected FORCE is very nearly independent of how fast you sync. A faster sync
    buys a smaller speed error, not a smaller disturbance.

Run ``python dyno_sync_sim.py --help`` for the knobs.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import sys
from dataclasses import dataclass

_REPO = os.path.abspath(os.path.join(os.path.dirname(__file__),
                                     '..', '..', '..', '..'))
if _REPO not in sys.path:
    sys.path.insert(0, _REPO)

from CommonLib.xil import (Bench, Dyno, DynoSim, LocalLink,  # noqa: E402
                           RobotDriver, Vehicle)

G = 9.80665


# ---------------------------------------------------------------- parameters

@dataclass
class CarlaParams:
    """CARLA's own vehicle. Values measured on the stock Tesla Model 3.

    ``aero_CdA_m2`` and ``air_density`` come from CARLA's source and were
    confirmed in motion: ``drag/v^2`` constant to 3.2 % over 941 samples.
    ``roll_coeff`` is NOT measured -- CARLA's rolling resistance lives inside the
    tire model and only appears lumped into ``sum(long_force)``. It is a
    parameter here precisely because it is the least-known term, and it decides
    where the resistance gap changes sign.
    """

    mass_kg: float = 1845.0
    aero_CdA_m2: float = 0.377
    air_density: float = 1.25          # CARLA hardcodes this, not 1.225
    roll_coeff: float = 0.012          # UNMEASURED -- see docstring


@dataclass
class RunParams:
    dt_s: float = 0.001                # integration step, both plants
    sync_dt_s: float = 0.010           # how often the speed is overwritten
    driver_kp: float = 0.45            # pedal per (m/s)
    driver_ki: float = 0.25            # pedal per (m/s * s)
    #: Route the reference through the link even in-process, so swapping to UDP
    #: is a transport change and not a code change.
    use_link: bool = True


# ---------------------------------------------------------------- resistance

def carla_resistance_N(p: CarlaParams, v: float) -> float:
    """Signed so it always opposes motion."""
    s = 1.0 if v >= 0.0 else -1.0
    aero = 0.5 * p.air_density * p.aero_CdA_m2 * v * v
    roll = p.roll_coeff * p.mass_kg * G
    return s * (aero + roll)


def steady_state_gap_N(sim, carla, speeds):
    """The part of F_sync that is pure model disagreement, not dynamics.

    At constant speed the bench holds against its road load while CARLA holds
    against its own aero plus rolling. Whatever the two differ by has to come out
    of the sync, for as long as the vehicle sits at that speed.
    """
    return [(v, sim.dyno.resistance(v) - carla_resistance_N(carla, v))
            for v in speeds]


# ---------------------------------------------------------------- components

# ---------------------------------------------------------------- drive cycle

def synthetic_cycle(duration_s=120.0, dt=0.001):
    """Ramp-and-hold segments, steep enough to excite the bench's torque lag."""
    segs = [(13.9, 1.8, 12.0), (0.0, -2.2, 4.0), (27.8, 2.2, 18.0),
            (16.7, -1.5, 10.0), (30.6, 1.2, 14.0), (0.0, -2.5, 6.0)]
    n = int(round(duration_s / dt))
    v = [0.0] * n
    cur, i = 0.0, 0
    while i < n:
        for target, a, cruise in segs:
            steps = int(abs(target - cur) / abs(a) / dt) if a else 0
            for _ in range(steps):
                if i >= n:
                    break
                cur += a * dt
                cur = min(cur, target) if a > 0 else max(cur, target)
                v[i] = cur
                i += 1
            for _ in range(int(cruise / dt)):
                if i >= n:
                    break
                v[i] = cur
                i += 1
            if i >= n:
                break
    return v


def load_cycle_csv(path, dt):
    """Two columns: time [s], speed [m/s]. Linearly resampled onto dt."""
    ts, vs = [], []
    with open(path, newline='') as fh:
        for row in csv.reader(fh):
            if not row or row[0].lstrip().startswith('#'):
                continue
            try:
                ts.append(float(row[0]))
                vs.append(float(row[1]))
            except (ValueError, IndexError):
                continue                      # header row
    if len(ts) < 2:
        raise ValueError('cycle CSV needs at least two numeric rows')
    out, j = [], 0
    for k in range(int((ts[-1] - ts[0]) / dt)):
        t = ts[0] + k * dt
        while j + 1 < len(ts) - 1 and ts[j + 1] < t:
            j += 1
        span = ts[j + 1] - ts[j]
        w = 0.0 if span <= 0 else (t - ts[j]) / span
        out.append(vs[j] + w * (vs[j + 1] - vs[j]))
    return out


# ---------------------------------------------------------------- simulation

TRACE_COLS = ('t', 'v_ref', 'v_dyno', 'v_carla', 'v_carla_presync',
              'throttle', 'brake', 'F_trac', 'F_road', 'F_carla_res',
              'F_carla_trac', 'F_sync', 'x_dyno', 'x_carla')

COUPLINGS = ('free', 'track', 'forced_coast', 'forced_driven')


def build_dyno(**kw):
    """The bench under study: chassis mode, the shipped simulator.

    Keywords are split between the two config objects by name, so a caller says
    ``build_dyno(mass_kg=1800, road_A_N=90)`` without knowing which is which.
    """
    vehicle_keys = set(Vehicle().__dict__)
    v = dict((k, kw[k]) for k in kw if k in vehicle_keys)
    d = dict((k, kw[k]) for k in kw if k not in vehicle_keys)
    return DynoSim(Vehicle(**v), Dyno(mode='chassis', **d))


def simulate(cycle, dyno: DynoSimulator, carla: CarlaParams, run: RunParams,
             coupling: str):
    """Integrate both plants. See COUPLINGS for the four modes."""
    if coupling not in COUPLINGS:
        raise ValueError('coupling must be one of %s' % (COUPLINGS,))

    dt = run.dt_s
    sync_every = max(1, int(round(run.sync_dt_s / dt)))
    dyno.reset()

    bench = Bench(sim=dyno, driver=RobotDriver(run.driver_kp, run.driver_ki))
    drv_c = RobotDriver(run.driver_kp, run.driver_ki)
    link = LocalLink() if run.use_link else None

    # The surrogate shares the bench's powertrain envelope so that the two plants
    # differ only in resistance, mass and the bench's delivery lag.
    carla_powertrain = dyno.vehicle.axle_torque
    r = dyno.vehicle.wheel_radius_m
    max_brake = dyno.vehicle.max_brake_torque_Nm

    v_c = x_d = x_c = 0.0
    rec = {k: [] for k in TRACE_COLS}
    sync_impulse = sync_work = 0.0

    for i, v_ref in enumerate(cycle):
        t = i * dt

        # ---- the bench, across the wire --------------------------------
        if link is not None:
            link.send_reference(v_ref)
            ref = link.recv_reference()
            st = bench.step(ref[0] if ref else 0.0, dt)
            link.send_measurement(st.speed)
            meas = link.recv_measurement()
            v_d = meas[0] if meas else 0.0
        else:
            st = bench.step(v_ref, dt)
            v_d = st.speed
        thr, brk = bench.throttle, bench.brake

        # ---- the CARLA surrogate ---------------------------------------
        if coupling in ('free', 'forced_driven'):
            ct, cb = drv_c.step(v_ref, v_c, dt)
        elif coupling == 'track':
            ct, cb = drv_c.step(v_d, v_c, dt)      # closes the loop on bench speed
        else:                                      # forced_coast
            ct, cb = 0.0, 0.0                      # speed imposed, nothing driving
        w = v_c / r
        Tf, Tr = carla_powertrain(ct, w, w)        # no delivery lag: apply_control
        F_c = (Tf + Tr) / r - cb * 4.0 * max_brake / r
        F_res_c = carla_resistance_N(carla, v_c)
        v_c = max(0.0, v_c + (F_c - F_res_c) / carla.mass_kg * dt)

        x_d += v_d * dt
        x_c += v_c * dt

        v_pre = v_c
        F_sync = 0.0
        if coupling.startswith('forced') and (i + 1) % sync_every == 0:
            dv = v_d - v_c
            F_sync = carla.mass_kg * dv / run.sync_dt_s
            sync_impulse += abs(carla.mass_kg * dv)
            sync_work += 0.5 * carla.mass_kg * (v_d * v_d - v_c * v_c)
            v_c = v_d                              # the overwrite

        for key, val in zip(TRACE_COLS,
                            (t, v_ref, v_d, v_c, v_pre, thr, brk,
                             st.force, st.resistance, F_res_c,
                             F_c, F_sync, x_d, x_c)):
            rec[key].append(val)

    rec['_summary'] = summarise(rec, run, coupling, sync_impulse, sync_work)
    return rec


def summarise(rec, run, coupling, sync_impulse, sync_work):
    dt = run.dt_s
    n = len(rec['t'])
    err = [rec['v_carla_presync'][i] - rec['v_dyno'][i] for i in range(n)]
    abserr = sorted(abs(e) for e in err)
    trac_work = sum(abs(rec['F_trac'][i]) * rec['v_dyno'][i] for i in range(n)) * dt
    syncs = [f for f in rec['F_sync'] if f != 0.0]
    dist_d, dist_c = rec['x_dyno'][-1], rec['x_carla'][-1]
    return {
        'coupling': coupling,
        'duration_s': n * dt,
        'sync_dt_s': run.sync_dt_s,
        'speed_err_rms_mps': math.sqrt(sum(e * e for e in err) / n),
        'speed_err_max_mps': abserr[-1],
        'speed_err_p95_mps': abserr[int(0.95 * (n - 1))],
        'F_sync_mean_N': (sum(syncs) / len(syncs)) if syncs else 0.0,
        'F_sync_rms_N': (math.sqrt(sum(f * f for f in syncs) / len(syncs))
                         if syncs else 0.0),
        'F_sync_max_N': max((abs(f) for f in syncs), default=0.0),
        'F_trac_rms_N': math.sqrt(sum(f * f for f in rec['F_trac']) / n),
        'sync_impulse_Ns': sync_impulse,
        'sync_work_J': sync_work,
        'tractive_work_J': trac_work,
        'sync_work_frac_of_tractive': (sync_work / trac_work) if trac_work else 0.0,
        'distance_dyno_m': dist_d,
        'distance_carla_m': dist_c,
        'distance_err_m': dist_c - dist_d,
        'distance_err_pct': (100.0 * (dist_c - dist_d) / dist_d) if dist_d else 0.0,
    }


# ---------------------------------------------------------------- reporting

def write_trace(rec, outdir, tag, decim):
    os.makedirs(outdir, exist_ok=True)
    path = os.path.join(outdir, 'trace_%s.csv' % tag)
    with open(path, 'w', newline='') as fh:
        w = csv.writer(fh)
        w.writerow(TRACE_COLS)
        for i in range(0, len(rec['t']), decim):
            w.writerow(['%.6g' % rec[c][i] for c in TRACE_COLS])
    return path


def try_plot(runs, gap, outdir):
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        return None

    fig, ax = plt.subplots(4, 1, figsize=(13, 12), constrained_layout=True)
    forced = runs['forced_driven']
    d = max(1, len(forced['t']) // 4000)
    t = forced['t'][::d]

    ax[0].plot(t, forced['v_ref'][::d], 'k--', lw=1.0, label='cycle target')
    ax[0].plot(t, forced['v_dyno'][::d], lw=1.3, label='dyno bench (truth)')
    for name in ('free', 'track'):
        ax[0].plot(t, runs[name]['v_carla'][::d], lw=0.9, label='CARLA, %s' % name)
    ax[0].set_ylabel('speed [m/s]')
    ax[0].legend(ncol=4, fontsize=8)
    ax[0].set_title('speed: the bench is truth, and how CARLA follows it')
    ax[0].grid(alpha=0.3)

    for name in COUPLINGS:
        e = [runs[name]['v_carla_presync'][i] - runs[name]['v_dyno'][i]
             for i in range(0, len(runs[name]['t']), d)]
        ax[1].plot(t[:len(e)], e, lw=0.9, label=name)
    ax[1].set_ylabel('v_carla - v_dyno [m/s]')
    ax[1].legend(fontsize=8)
    ax[1].set_title('speed error: continuous for free/track, pre-snap for forced')
    ax[1].grid(alpha=0.3)

    idx = [i for i, f in enumerate(forced['F_sync']) if f != 0.0]
    ds = max(1, len(idx) // 4000)
    ts = [forced['t'][i] for i in idx[::ds]]
    fs = [forced['F_sync'][i] for i in idx[::ds]]
    ax[2].plot(ts, fs, lw=0.8, label='F_sync (forced_driven)')
    ax[2].axhline(sum(fs) / len(fs) if fs else 0.0, color='C0', ls=':', lw=1.2,
                  label='F_sync mean')
    ax2b = ax[2].twinx()
    ax2b.plot(t, forced['F_trac'][::d], lw=0.8, alpha=0.35, color='C1',
              label='bench tractive force (right axis)')
    ax[2].set_ylabel('F_sync [N]')
    ax2b.set_ylabel('tractive force [N]')
    h1, l1 = ax[2].get_legend_handles_labels()
    h2, l2 = ax2b.get_legend_handles_labels()
    ax[2].legend(h1 + h2, l1 + l2, fontsize=8, loc='upper right')
    ax[2].set_title('what the speed overwrite silently injects, against real effort')
    ax[2].grid(alpha=0.3)

    ax[3].plot([g[0] for g in gap], [g[1] for g in gap], lw=1.6)
    ax[3].axhline(0, color='k', lw=0.8)
    ax[3].set_xlabel('speed [m/s]')
    ax[3].set_ylabel('road load - CARLA resistance [N]')
    ax[3].set_title('steady-state model disagreement: the floor under F_sync')
    ax[3].grid(alpha=0.3)

    path = os.path.join(outdir, 'dyno_sync_overview.png')
    fig.savefig(path, dpi=110)
    plt.close(fig)
    return path


def print_report(summaries, gap, outdir, png):
    print('\n' + '=' * 78)
    print('DYNO / CARLA SPEED-SYNC STUDY')
    print('=' * 78)
    print('\n  steady-state model disagreement (bench road load minus CARLA resistance).')
    print('  This is the floor under the sync force; a faster sync does not remove it.\n')
    print('    %8s %14s' % ('v [m/s]', 'gap [N]'))
    for v, g in gap:
        if int(round(v)) % 5 == 0:
            print('    %8.0f %14.1f' % (v, g))

    print('\n  per-coupling results\n')
    print('    %-14s %8s %9s %9s %10s %10s %10s %10s'
          % ('coupling', 'sync[s]', 'err_rms', 'err_max', 'Fsync_mean',
             'Fsync_rms', 'Fsync_max', 'dist_err_%'))
    for s in summaries:
        print('    %-14s %8.3f %9.4f %9.4f %10.1f %10.1f %10.1f %10.4f'
              % (s['coupling'], s['sync_dt_s'], s['speed_err_rms_mps'],
                 s['speed_err_max_mps'], s['F_sync_mean_N'], s['F_sync_rms_N'],
                 s['F_sync_max_N'], s['distance_err_pct']))

    forced = [s for s in summaries if s['coupling'] == 'forced_driven']
    if forced:
        s = forced[0]
        print('\n  energy the overwrite injects: %.1f kJ against %.1f kJ of tractive'
              ' work  (%.2f %%)'
              % (s['sync_work_J'] / 1e3, s['tractive_work_J'] / 1e3,
                 100 * s['sync_work_frac_of_tractive']))
    print('\n  wrote %s' % outdir)
    if png:
        print('  plot  %s' % png)
    print()


# ---------------------------------------------------------------- entry point

def build_argparser():
    c, r = CarlaParams(), RunParams()
    veh, dyn = Vehicle(), Dyno()
    p = argparse.ArgumentParser(
        description='Dyno / CARLA forced-speed-match study',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    p.add_argument('--out', default=None)
    p.add_argument('--cycle-csv', default=None,
                   help='two-column time[s],speed[m/s]; default is synthetic')
    p.add_argument('--duration', type=float, default=120.0)
    p.add_argument('--dt', type=float, default=r.dt_s)
    p.add_argument('--sync-dt', type=float, default=r.sync_dt_s)
    p.add_argument('--sync-sweep', default='',
                   help='comma-separated sync intervals [s], e.g. 0.005,0.01,0.05')
    p.add_argument('--dyno-mass', type=float, default=veh.mass_kg)
    p.add_argument('--roller-inertia', type=float, default=dyn.roller_inertia_kgm2)
    p.add_argument('--grade-deg', type=float, default=0.0)
    p.add_argument('--road-a', type=float, default=dyn.road_A_N)
    p.add_argument('--road-b', type=float, default=dyn.road_B_Npms)
    p.add_argument('--road-c', type=float, default=dyn.road_C_Npms2)
    p.add_argument('--torque-bw', type=float, default=veh.torque_bandwidth_Hz,
                   help='bench torque delivery bandwidth [Hz]')
    p.add_argument('--carla-mass', type=float, default=c.mass_kg)
    p.add_argument('--carla-cda', type=float, default=c.aero_CdA_m2)
    p.add_argument('--carla-roll', type=float, default=c.roll_coeff)
    p.add_argument('--decim', type=int, default=10)
    p.add_argument('--no-plot', action='store_true')
    return p


def main(argv=None):
    args = build_argparser().parse_args(argv)

    dyno = build_dyno(mass_kg=args.dyno_mass,
                      torque_bandwidth_Hz=args.torque_bw,
                      roller_inertia_kgm2=args.roller_inertia,
                      grade_rad=math.radians(args.grade_deg),
                      road_A_N=args.road_a, road_B_Npms=args.road_b,
                      road_C_Npms2=args.road_c)
    carla = CarlaParams(mass_kg=args.carla_mass, aero_CdA_m2=args.carla_cda,
                        roll_coeff=args.carla_roll)
    run = RunParams(dt_s=args.dt, sync_dt_s=args.sync_dt)

    cycle = (load_cycle_csv(args.cycle_csv, args.dt) if args.cycle_csv
             else synthetic_cycle(args.duration, args.dt))

    outdir = args.out or os.path.join(os.path.dirname(os.path.abspath(__file__)), 'out')
    os.makedirs(outdir, exist_ok=True)

    runs, summaries = {}, []
    for coupling in COUPLINGS:
        rec = simulate(cycle, dyno, carla, run, coupling)
        runs[coupling] = rec
        summaries.append(rec['_summary'])
        write_trace(rec, outdir, coupling, args.decim)

    if args.sync_sweep:
        for s in [float(x) for x in args.sync_sweep.split(',')]:
            rp = RunParams(dt_s=run.dt_s, sync_dt_s=s,
                           driver_kp=run.driver_kp, driver_ki=run.driver_ki)
            summaries.append(
                simulate(cycle, dyno, carla, rp, 'forced_driven')['_summary'])

    gap = steady_state_gap_N(dyno, carla, [float(i) for i in range(0, 41)])
    png = None if args.no_plot else try_plot(runs, gap, outdir)

    with open(os.path.join(outdir, 'summary.json'), 'w') as fh:
        json.dump({'vehicle': dict(dyno.vehicle.__dict__),
                   'dyno': dict(dyno.dyno.__dict__),
                   'carla': dict(carla.__dict__), 'run': dict(run.__dict__),
                   'results': summaries}, fh, indent=2)

    print_report(summaries, gap, outdir, png)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
