"""Does the CARLA side keep up with the bench? (#323)

Two scenarios, because they separate two things:

  free      the reference is a drive cycle and nothing is in front. This isolates
            the tracking law: whatever CARLA and the bench disagree by is the law
            and the plant, with no planner reacting to it.
  leader    an IDM sizes the reference from the gap to a braking leader. Now the
            disagreement has a consequence, because the IDM planned on the ego
            achieving the reference and the ego does not.

                       ┌── drive cycle ──┐
                       │                 ▼
    leader ──▶ [IDM] ──┴──▶ v_ref ──▶ [link] ──▶ [RobotDriver + DynoSimulator]
                  ▲                                        │  v_dyno
                  │                                        ▼
                  └──────── gap ◀── [CARLA surrogate tracking v_dyno]

Two CARLA-side tracking laws, because the choice is live:

  ornl    a = 0.55*(v_dyno - v_ego), mapped to a pedal by a/3.2, applied.
          Lifted from ORNL's carla_standalone_drive.py. Proportional only, so it
          carries a standing offset of (3.2/0.55) x pedal by construction.
  ideal   the ego is handed the speed directly, which is what
          ego.set(speedDesired=...) with stiff Ackermann gains approximates.

Writes one plotly html: both scenarios side by side, both laws in each.

    python tracking_scenarios.py [--out DIR]
"""

from __future__ import annotations

import argparse
import math
import os
import sys
from dataclasses import dataclass

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import dyno_sync_sim as study  # noqa: E402

from CommonLib.xil import DynoVehicle, InProcessPair  # noqa: E402


# ------------------------------------------------------------------- leader

@dataclass
class LeaderParams:
    """A leader that cruises, brakes hard, then resumes."""

    start_gap_m: float = 45.0
    cruise_mps: float = 22.0
    brake_at_s: float = 40.0
    brake_decel: float = 3.5
    hold_mps: float = 6.0
    resume_at_s: float = 55.0
    resume_accel: float = 1.2


def leader_speed(t: float, p: LeaderParams) -> float:
    if t < p.brake_at_s:
        return p.cruise_mps
    if t < p.resume_at_s:
        v = p.cruise_mps - p.brake_decel * (t - p.brake_at_s)
        return max(p.hold_mps, v)
    v = p.hold_mps + p.resume_accel * (t - p.resume_at_s)
    return min(p.cruise_mps, v)


# ---------------------------------------------------------------------- IDM

@dataclass
class IdmParams:
    v0_mps: float = 25.0        # desired free speed
    headway_s: float = 1.5
    accel_max: float = 1.5
    decel_comfort: float = 2.0
    min_gap_m: float = 2.0
    delta: float = 4.0


def idm_accel(v: float, gap: float, closing: float, p: IdmParams) -> float:
    """Standard IDM. ``closing`` is v_ego - v_lead, positive when approaching."""
    gap = max(gap, 1e-3)
    s_star = (p.min_gap_m + max(0.0, v * p.headway_s
                                + v * closing
                                / (2.0 * math.sqrt(p.accel_max * p.decel_comfort))))
    return p.accel_max * (1.0 - (v / p.v0_mps) ** p.delta - (s_star / gap) ** 2)


# ---------------------------------------------------------------- the run

ORNL_KP = 0.55              # DYNO_SPEED_KP
ORNL_MAX_ACCEL = 3.2        # MAX_ACCEL_CMD


def accel_to_pedal(a: float):
    a = max(-ORNL_MAX_ACCEL, min(ORNL_MAX_ACCEL, a))
    return (a / ORNL_MAX_ACCEL, 0.0) if a >= 0 else (0.0, -a / ORNL_MAX_ACCEL)


def run(law: str, scenario: str, duration_s=90.0, dt=0.005,
        leader=LeaderParams(), idm=IdmParams(), carla=study.CarlaParams()):
    """One run. ``law`` is 'ornl' or 'ideal'; ``scenario`` is 'free' or 'leader'."""
    if law not in ('ornl', 'ideal'):
        raise ValueError("law must be 'ornl' or 'ideal'")
    if scenario not in ('free', 'leader'):
        raise ValueError("scenario must be 'free' or 'leader'")

    dyno = study.build_dyno()
    car = DynoVehicle(dyno=dyno)
    link = InProcessPair()
    powertrain = study.envelope_powertrain(dyno.cfg.powertrain,
                                           dyno.cfg.driveline.wheel_radius_m)
    r = dyno.cfg.driveline.wheel_radius_m
    max_brake = dyno.cfg.driveline.max_brake_torque_Nm

    cycle = study.synthetic_cycle(duration_s, dt) if scenario == 'free' else None

    x_l = leader.start_gap_m
    x_d = x_c = v_c = v_ref = 0.0
    rec = {k: [] for k in ('t', 'v_lead', 'v_ref', 'v_dyno', 'v_carla',
                           'gap_dyno', 'gap_carla', 'x_divergence')}

    for i in range(int(duration_s / dt)):
        t = i * dt

        if scenario == 'free':
            v_l = float('nan')
            v_ref = cycle[i] if i < len(cycle) else cycle[-1]
        else:
            v_l = leader_speed(t, leader)
            x_l += v_l * dt
            # The IDM plans against the BENCH's state, because the bench is the
            # vehicle: it is what the reference is for.
            v_ref = max(0.0, v_ref + idm_accel(car.speed_mps, x_l - x_d,
                                               car.speed_mps - v_l, idm) * dt)

        link.simulator.send_reference(v_ref)
        ref = link.dyno.latest_reference()
        st = car.step(ref[0] if ref else 0.0, dt)
        link.dyno.send_measurement(st.speed_mps)
        meas = link.simulator.latest_measurement()
        v_d = meas[0] if meas else 0.0
        x_d += v_d * dt

        # ---- the CARLA side follows the bench --------------------------
        if law == 'ideal':
            v_c = v_d
        else:
            thr, brk = accel_to_pedal(ORNL_KP * (v_d - v_c))
            w = v_c / r
            Tf, Tr = powertrain(thr, brk, w, w)
            F = (Tf + Tr) / r - brk * 4.0 * max_brake / r
            v_c = max(0.0, v_c + (F - study.carla_resistance_N(carla, v_c))
                      / carla.mass_kg * dt)
        x_c += v_c * dt

        for k, val in zip(rec, (t, v_l, v_ref, v_d, v_c,
                                x_l - x_d, x_l - x_c, x_c - x_d)):
            rec[k].append(val)
    return rec


# ------------------------------------------------------------------- report

def summarise(rec, law, scenario):
    err = [rec['v_carla'][i] - rec['v_dyno'][i] for i in range(len(rec['t']))]
    a = sorted(abs(e) for e in err)
    out = {
        'scenario': scenario, 'law': law,
        'speed_err_rms': math.sqrt(sum(e * e for e in err) / len(err)),
        'speed_err_max': a[-1],
        'speed_err_mean': sum(err) / len(err),
        'x_divergence_m': rec['x_divergence'][-1],
    }
    out['min_gap_dyno'] = (min(rec['gap_dyno']) if scenario == 'leader'
                           else float('nan'))
    out['min_gap_carla'] = (min(rec['gap_carla']) if scenario == 'leader'
                            else float('nan'))
    return out


COLOUR = {'ideal': '#2ca02c', 'ornl': '#d62728'}
SCENARIOS = ('free', 'leader')


def write_html(runs, path):
    """One page, scenarios side by side, both laws in each."""
    import plotly.graph_objects as go
    from plotly.subplots import make_subplots

    titles = []
    for row in ('speed', 'CARLA speed minus bench speed',
                'position divergence, x_carla - x_bench', 'gap to leader'):
        for sc in SCENARIOS:
            titles.append('%s  --  %s' % (row, 'free driving' if sc == 'free'
                                          else 'following a leader'))

    fig = make_subplots(rows=4, cols=2, shared_xaxes=True,
                        vertical_spacing=0.055, horizontal_spacing=0.07,
                        subplot_titles=titles)

    for col, sc in enumerate(SCENARIOS, start=1):
        base = runs[(sc, 'ideal')]
        first = col == 1
        if sc == 'leader':
            fig.add_trace(go.Scatter(x=base['t'], y=base['v_lead'], name='leader',
                                     legendgroup='lead', showlegend=first,
                                     line=dict(color='#888', dash='dot')),
                          row=1, col=col)
        fig.add_trace(go.Scatter(x=base['t'], y=base['v_ref'], name='reference',
                                 legendgroup='ref', showlegend=first,
                                 line=dict(color='black', dash='dash', width=1)),
                      row=1, col=col)
        fig.add_trace(go.Scatter(x=base['t'], y=base['v_dyno'], name='bench (dyno)',
                                 legendgroup='bench', showlegend=first,
                                 line=dict(color='#1f77b4', width=2)),
                      row=1, col=col)

        for law in ('ideal', 'ornl'):
            rec = runs[(sc, law)]
            n = len(rec['t'])
            line = dict(color=COLOUR[law], width=1.4)
            fig.add_trace(go.Scatter(x=rec['t'], y=rec['v_carla'],
                                     name='CARLA, %s' % law, legendgroup=law,
                                     showlegend=first, line=line), row=1, col=col)
            fig.add_trace(go.Scatter(
                x=rec['t'], y=[rec['v_carla'][i] - rec['v_dyno'][i]
                               for i in range(n)],
                legendgroup=law, showlegend=False, line=line), row=2, col=col)
            fig.add_trace(go.Scatter(x=rec['t'], y=rec['x_divergence'],
                                     legendgroup=law, showlegend=False,
                                     line=line), row=3, col=col)
            if sc == 'leader':
                fig.add_trace(go.Scatter(x=rec['t'], y=rec['gap_carla'],
                                         legendgroup=law, showlegend=False,
                                         line=line), row=4, col=col)

        if sc == 'leader':
            fig.add_trace(go.Scatter(x=base['t'], y=base['gap_dyno'],
                                     legendgroup='bench', showlegend=False,
                                     line=dict(color='#1f77b4', width=2)),
                          row=4, col=col)
        for row in (2, 3):
            fig.add_hline(y=0.0, line=dict(color='black', width=1),
                          row=row, col=col)

    for row, label in ((1, 'speed [m/s]'), (2, 'v_carla - v_dyno [m/s]'),
                       (3, 'x_carla - x_bench [m]'), (4, 'gap [m]')):
        fig.update_yaxes(title_text=label, row=row, col=1)
    fig.update_xaxes(title_text='time [s]', row=4, col=1)
    fig.update_xaxes(title_text='time [s]', row=4, col=2)
    fig.update_layout(
        height=1150, hovermode='x unified',
        title='Does the CARLA side keep up with the bench? '
              'Left: free driving. Right: following a braking leader.',
        legend=dict(orientation='h', y=1.05))
    fig.write_html(path, include_plotlyjs='cdn')
    return path


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split(chr(10))[0])
    ap.add_argument('--out', default=None)
    ap.add_argument('--duration', type=float, default=90.0)
    ap.add_argument('--dt', type=float, default=0.005)
    args = ap.parse_args(argv)

    outdir = args.out or os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                      'out')
    os.makedirs(outdir, exist_ok=True)

    runs = {(sc, law): run(law, sc, args.duration, args.dt)
            for sc in SCENARIOS for law in ('ideal', 'ornl')}

    print('\nCARLA following the bench. Leader brakes at %.0f s.\n'
          % LeaderParams().brake_at_s)
    print('  %-8s %-7s %10s %10s %10s %13s %12s %13s'
          % ('scenario', 'law', 'err_rms', 'err_max', 'err_mean', 'x_diverge[m]',
             'min_gap_bench', 'min_gap_carla'))
    for sc in SCENARIOS:
        for law in ('ideal', 'ornl'):
            r = summarise(runs[(sc, law)], law, sc)
            print('  %-8s %-7s %10.4f %10.4f %10.4f %13.2f %12.2f %13.2f'
                  % (r['scenario'], r['law'], r['speed_err_rms'],
                     r['speed_err_max'], r['speed_err_mean'],
                     r['x_divergence_m'], r['min_gap_dyno'], r['min_gap_carla']))

    path = write_html(runs, os.path.join(outdir, 'tracking_scenarios.html'))
    print('\n  wrote %s\n' % path)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
