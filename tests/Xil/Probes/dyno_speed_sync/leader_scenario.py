"""Car-following against a braking leader: does the CARLA side keep up? (#323)

The question this answers is the one that matters for safety. An IDM sizes the
ego's speed from the gap to a leader, assuming the ego will achieve it. It will
not: the reference goes to the bench, the bench's vehicle takes time to follow,
and CARLA then takes more time to follow the bench. The gap closes further than
the IDM planned for.

    leader ──▶ [IDM] ──▶ v_ref ──▶ [link] ──▶ [RobotDriver + DynoSimulator]
                  ▲                                        │  v_dyno
                  │                                        ▼
                  └──────── gap ◀── [CARLA surrogate tracking v_dyno]

Two CARLA-side tracking laws, because the choice is live:

  ornl    a = 0.55*(v_dyno - v_ego), mapped to a pedal by a/3.2, applied.
          Lifted from ORNL's carla_standalone_drive.py. Proportional only, so it
          carries a standing offset of (3.2/0.55) x pedal by construction.
  ideal   the ego is handed the speed directly, which is what
          ego.set(speedDesired=...) with stiff Ackermann gains approximates.

Writes a plotly html: speed traces and gap to leader, for both laws.

    python leader_scenario.py [--out DIR] [--no-open]
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


def run(law: str, duration_s=90.0, dt=0.005,
        leader=LeaderParams(), idm=IdmParams(), carla=study.CarlaParams()):
    """One scenario. ``law`` is 'ornl' or 'ideal'."""
    dyno = study.build_dyno()
    car = DynoVehicle(dyno=dyno)
    link = InProcessPair()
    powertrain = study.envelope_powertrain(dyno.cfg.powertrain,
                                           dyno.cfg.driveline.wheel_radius_m)
    r = dyno.cfg.driveline.wheel_radius_m
    max_brake = dyno.cfg.driveline.max_brake_torque_Nm

    x_l = leader.start_gap_m
    x_d = x_c = 0.0
    v_c = 0.0
    v_ref = 0.0
    rec = {k: [] for k in ('t', 'v_lead', 'v_ref', 'v_dyno', 'v_carla',
                           'gap_dyno', 'gap_carla')}

    for i in range(int(duration_s / dt)):
        t = i * dt
        v_l = leader_speed(t, leader)
        x_l += v_l * dt

        # The IDM plans against the BENCH's state, because the bench is the
        # vehicle: it is what the reference is for.
        gap_d = x_l - x_d
        v_ref = max(0.0, v_ref + idm_accel(car.speed_mps, gap_d,
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
                                x_l - x_d, x_l - x_c)):
            rec[k].append(val)
    return rec


# ------------------------------------------------------------------- report

def summarise(rec, law):
    err = [rec['v_carla'][i] - rec['v_dyno'][i] for i in range(len(rec['t']))]
    a = sorted(abs(e) for e in err)
    return {
        'law': law,
        'speed_err_rms': math.sqrt(sum(e * e for e in err) / len(err)),
        'speed_err_max': a[-1],
        'speed_err_mean': sum(err) / len(err),
        'min_gap_dyno': min(rec['gap_dyno']),
        'min_gap_carla': min(rec['gap_carla']),
        'gap_err_max': max(abs(rec['gap_carla'][i] - rec['gap_dyno'][i])
                           for i in range(len(rec['t']))),
    }


def write_html(runs, path):
    import plotly.graph_objects as go
    from plotly.subplots import make_subplots

    fig = make_subplots(
        rows=3, cols=1, shared_xaxes=True, vertical_spacing=0.06,
        subplot_titles=('speed: leader, IDM reference, bench, CARLA',
                        'gap to leader',
                        'CARLA speed minus bench speed'))

    base = runs['ideal']
    fig.add_trace(go.Scatter(x=base['t'], y=base['v_lead'], name='leader',
                             line=dict(color='#888', dash='dot')), row=1, col=1)
    fig.add_trace(go.Scatter(x=base['t'], y=base['v_ref'], name='IDM reference',
                             line=dict(color='black', dash='dash', width=1)),
                  row=1, col=1)
    fig.add_trace(go.Scatter(x=base['t'], y=base['v_dyno'], name='bench (dyno)',
                             line=dict(color='#1f77b4', width=2)), row=1, col=1)

    colour = {'ideal': '#2ca02c', 'ornl': '#d62728'}
    for law, rec in runs.items():
        fig.add_trace(go.Scatter(x=rec['t'], y=rec['v_carla'],
                                 name='CARLA, %s' % law,
                                 line=dict(color=colour[law], width=1.4)),
                      row=1, col=1)
        fig.add_trace(go.Scatter(x=rec['t'], y=rec['gap_carla'],
                                 name='gap, CARLA %s' % law,
                                 line=dict(color=colour[law], width=1.4),
                                 showlegend=False), row=2, col=1)
        fig.add_trace(go.Scatter(
            x=rec['t'], y=[rec['v_carla'][i] - rec['v_dyno'][i]
                           for i in range(len(rec['t']))],
            name='err, %s' % law, line=dict(color=colour[law], width=1.4),
            showlegend=False), row=3, col=1)

    fig.add_trace(go.Scatter(x=base['t'], y=base['gap_dyno'], name='gap, bench',
                             line=dict(color='#1f77b4', width=2),
                             showlegend=False), row=2, col=1)
    fig.add_hline(y=0.0, line=dict(color='black', width=1), row=2, col=1)

    fig.update_yaxes(title_text='speed [m/s]', row=1, col=1)
    fig.update_yaxes(title_text='gap [m]', row=2, col=1)
    fig.update_yaxes(title_text='v_carla - v_dyno [m/s]', row=3, col=1)
    fig.update_xaxes(title_text='time [s]', row=3, col=1)
    fig.update_layout(
        height=900, hovermode='x unified',
        title='Car following a braking leader: bench is the vehicle, '
              'CARLA follows it',
        legend=dict(orientation='h', y=1.06))
    fig.write_html(path, include_plotlyjs='cdn')
    return path


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--out', default=None)
    ap.add_argument('--duration', type=float, default=90.0)
    ap.add_argument('--dt', type=float, default=0.005)
    args = ap.parse_args(argv)

    outdir = args.out or os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                      'out')
    os.makedirs(outdir, exist_ok=True)

    runs = {law: run(law, args.duration, args.dt) for law in ('ideal', 'ornl')}

    print('\nCARLA following the bench, leader brakes at %.0f s\n'
          % LeaderParams().brake_at_s)
    print('  %-7s %11s %11s %11s %12s %12s %11s'
          % ('law', 'err_rms', 'err_max', 'err_mean', 'min_gap_dyno',
             'min_gap_carla', 'gap_err_max'))
    for law in ('ideal', 'ornl'):
        s = summarise(runs[law], law)
        print('  %-7s %11.4f %11.4f %11.4f %12.2f %12.2f %11.3f'
              % (s['law'], s['speed_err_rms'], s['speed_err_max'],
                 s['speed_err_mean'], s['min_gap_dyno'], s['min_gap_carla'],
                 s['gap_err_max']))

    path = write_html(runs, os.path.join(outdir, 'leader_scenario.html'))
    print('\n  wrote %s\n' % path)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
