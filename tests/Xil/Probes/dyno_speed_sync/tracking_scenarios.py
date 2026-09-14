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

WHAT WE COMMAND CARLA WITH
These are the two things a controller can write through ``ego.set``, which is
the same choice ``COMMAND_SHAPE`` makes in ego_agent_controller.py:

  'speed'    we hand CARLA a speed and its own Ackermann controller closes the
             loop. Modelled at kp 50, ki 5 -- the gains the MLK app ships -- with
             the demanded acceleration clipped by what the vehicle can actually
             deliver. That clip is what makes it a controller rather than an
             assignment, and it tracks closely but NOT exactly.
  'pedals'   we close the loop ourselves and write throttle and brake:
             a = kp*(v_dyno - v_ego) + ki*integral, mapped to a pedal by a/3.2.
             kp and the 3.2 are lifted from ORNL's carla_standalone_drive.py;
             the integral term is NOT theirs. Their law is proportional only,
             which leaves a standing offset of (3.2/kp) x pedal -- measured at
             -0.44 m/s mean and 40 m of position divergence over 90 s. Set
             PEDAL_KI to 0 to get their law back.

AND WATCH THE TORQUE, NOT ONLY THE SPEED
Tracking well is not the same as tracking honestly. The two plants differ by 33%
in effective inertia and their resistance curves cross, so for CARLA to hold the
bench's speed it must produce a DIFFERENT force than the bench did. A stiff loop
will deliver that difference without complaint, and the speed trace will look
perfect while CARLA is being driven to a torque no real vehicle would produce.
Row 2 of the figure is there to catch that: wheel torque, both sides, same axis.

A third option exists and is not here: overwriting CARLA's speed outright, which
is what set_target_velocity does. It belongs in dyno_sync_sim.py's forced
couplings, where its cost is measured. An earlier revision of this file had it
as an 'ideal' law, which made every error metric read 0.0000 -- true, and a
tautology, because an assignment is not a controller.

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

from CommonLib.xil import Bench, LocalLink  # noqa: E402


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

#: The pedal law. kp and the acceleration cap are from ORNL's
#: carla_standalone_drive.py; the integral term is ours, and 0 restores theirs.
PEDAL_KP = 0.55             # DYNO_SPEED_KP
PEDAL_KI = 0.5
PEDAL_MAX_ACCEL = 3.2       # MAX_ACCEL_CMD

#: CARLA's own Ackermann speed loop, as the MLK app configures it.
SPEED_KP, SPEED_KI = 50.0, 5.0

#: What we write through ego.set. Same choice as COMMAND_SHAPE in the app.
COMMANDS = ('speed', 'pedals')
LABEL = {'speed': 'CARLA, speed command',
         'pedals': 'CARLA, pedal command'}
COLOUR = {'speed': '#2ca02c', 'pedals': '#d62728'}


def accel_to_pedal(a: float):
    a = max(-PEDAL_MAX_ACCEL, min(PEDAL_MAX_ACCEL, a))
    return (a / PEDAL_MAX_ACCEL, 0.0) if a >= 0 else (0.0, -a / PEDAL_MAX_ACCEL)


def run(command: str, scenario: str, duration_s=90.0, dt=0.005,
        leader=LeaderParams(), idm=IdmParams(), carla=study.CarlaParams()):
    """One run.

    ``command`` is what we write to CARLA: 'speed' or 'pedals'.
    ``scenario`` is 'free' or 'leader'.
    """
    if command not in COMMANDS:
        raise ValueError('command must be one of %s' % (COMMANDS,))
    if scenario not in ('free', 'leader'):
        raise ValueError("scenario must be 'free' or 'leader'")

    sim = study.build_dyno()
    bench = Bench(sim=sim)
    link = LocalLink()
    powertrain = sim.vehicle.axle_torque
    r = sim.vehicle.wheel_radius_m
    max_brake = sim.vehicle.max_brake_torque_Nm

    cycle = study.synthetic_cycle(duration_s, dt) if scenario == 'free' else None

    x_l = leader.start_gap_m
    x_d = x_c = v_c = v_ref = 0.0
    integ = 0.0          # the tracking law's integral, whichever law is running
    rec = {k: [] for k in ('t', 'v_lead', 'v_ref', 'v_dyno', 'v_carla',
                           'gap_dyno', 'gap_carla', 'x_divergence',
                           'T_bench', 'T_carla', 'T_carla_capacity')}

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
            v_ref = max(0.0, v_ref + idm_accel(bench.speed, x_l - x_d,
                                               bench.speed - v_l, idm) * dt)

        link.send_reference(v_ref)
        ref = link.recv_reference()
        st = bench.step(ref[0] if ref else 0.0, dt)
        link.send_measurement(st.speed)
        meas = link.recv_measurement()
        v_d = meas[0] if meas else 0.0
        x_d += v_d * dt

        # ---- the CARLA side follows the bench --------------------------
        e = v_d - v_c
        w = v_c / r
        cap_f, cap_r = powertrain(1.0, w, w)
        a_max = (cap_f + cap_r) / r / carla.mass_kg
        a_min = -4.0 * max_brake / r / carla.mass_kg
        if command == 'speed':
            # The controller asks for an acceleration; the vehicle delivers what
            # it can. That clip is what stops this being an assignment.
            if a_min < SPEED_KP * e + SPEED_KI * integ < a_max:
                integ += e * dt
            a_cmd = max(a_min, min(a_max, SPEED_KP * e + SPEED_KI * integ))
            F = carla.mass_kg * a_cmd
        else:
            a_raw = PEDAL_KP * e + PEDAL_KI * integ
            if -PEDAL_MAX_ACCEL < a_raw < PEDAL_MAX_ACCEL:
                integ += e * dt
            thr, brk = accel_to_pedal(PEDAL_KP * e + PEDAL_KI * integ)
            Tf, Tr = powertrain(thr, w, w)
            F = (Tf + Tr) / r - brk * 4.0 * max_brake / r
        v_c = max(0.0, v_c + (F - study.carla_resistance_N(carla, v_c))
                  / carla.mass_kg * dt)

        # Torque on the WHEEL AXIS -- what the bench actually measures, which
        # is the powertrain's net torque less what went into spinning the wheel
        # up rather than reaching the road:
        #
        #     J_wheel * alpha = T_drive - T_brake - T_axle
        #
        # An earlier revision plotted sum(drive) - sum(brake) instead, which is
        # one term upstream of the transducer. Only 1.7 % different here, but it
        # is not the measured quantity and on an axle dyno the wheel inertia is
        # precisely what the hub unit senses.
        #
        # The CARLA surrogate is a point mass with no wheel, so its propulsive
        # force at the road times the radius is the same quantity: contact
        # torque, with no inertia term to subtract because there is no wheel.
        T_bench = sum(st.axle_torque)
        T_carla = F * r
        x_c += v_c * dt

        for k, val in zip(rec, (t, v_l, v_ref, v_d, v_c,
                                x_l - x_d, x_l - x_c, x_c - x_d,
                                T_bench, T_carla, (cap_f + cap_r))):
            rec[k].append(val)
    return rec


# ------------------------------------------------------------------- report

def summarise(rec, command, scenario):
    err = [rec['v_carla'][i] - rec['v_dyno'][i] for i in range(len(rec['t']))]
    a = sorted(abs(e) for e in err)
    out = {
        'scenario': scenario, 'command': command,
        'speed_err_rms': math.sqrt(sum(e * e for e in err) / len(err)),
        'speed_err_max': a[-1],
        'speed_err_mean': sum(err) / len(err),
        'x_divergence_m': rec['x_divergence'][-1],
        'T_carla_peak': max(abs(v) for v in rec['T_carla']),
        'T_bench_peak': max(abs(v) for v in rec['T_bench']),
        'T_ratio_peak': max(abs(rec['T_carla'][i]) for i in range(len(rec['t'])))
        / max(1e-9, max(abs(v) for v in rec['T_bench'])),
        'T_frac_of_capacity': max(
            abs(rec['T_carla'][i]) / max(1e-9, rec['T_carla_capacity'][i])
            for i in range(len(rec['t']))),
    }
    out['min_gap_dyno'] = (min(rec['gap_dyno']) if scenario == 'leader'
                           else float('nan'))
    out['min_gap_carla'] = (min(rec['gap_carla']) if scenario == 'leader'
                            else float('nan'))
    return out


SCENARIOS = ('free', 'leader')


def write_html(runs, path):
    """One page, scenarios side by side, both laws in each."""
    import plotly.graph_objects as go
    from plotly.subplots import make_subplots

    titles = []
    for row in ('speed',
                'torque on the wheel axis (what the bench measures)',
                'CARLA speed minus bench speed',
                'position divergence, x_carla - x_bench',
                'gap to leader'):
        for sc in SCENARIOS:
            titles.append('%s  --  %s' % (row, 'free driving' if sc == 'free'
                                          else 'following a leader'))

    fig = make_subplots(rows=5, cols=2, shared_xaxes='all',
                        vertical_spacing=0.045, horizontal_spacing=0.07,
                        subplot_titles=titles)

    for col, sc in enumerate(SCENARIOS, start=1):
        base = runs[(sc, COMMANDS[0])]
        first = col == 1
        # Names belong to column 1 only. Column 2 repeats every trace, so
        # naming both put each entry in the legend twice under one label.
        def named(label, _first=first):
            return dict(name=label, legendgroup=label, showlegend=_first)

        if sc == 'leader':
            fig.add_trace(go.Scatter(x=base['t'], y=base['v_lead'],
                                     line=dict(color='#888', dash='dot'),
                                     **named('leader')), row=1, col=col)
        fig.add_trace(go.Scatter(x=base['t'], y=base['v_ref'],
                                 line=dict(color='black', dash='dash', width=1),
                                 **named('reference')), row=1, col=col)
        fig.add_trace(go.Scatter(x=base['t'], y=base['v_dyno'],
                                 line=dict(color='#1f77b4', width=2),
                                 **named('bench (dyno)')), row=1, col=col)
        fig.add_trace(go.Scatter(x=base['t'], y=base['T_bench'],
                                 legendgroup='bench (dyno)', showlegend=False,
                                 line=dict(color='#1f77b4', width=2)),
                      row=2, col=col)

        for cmd in COMMANDS:
            rec = runs[(sc, cmd)]
            n = len(rec['t'])
            line = dict(color=COLOUR[cmd], width=1.4)
            fig.add_trace(go.Scatter(x=rec['t'], y=rec['v_carla'], line=line,
                                     **named(LABEL[cmd])), row=1, col=col)
            fig.add_trace(go.Scatter(x=rec['t'], y=rec['T_carla'],
                                     legendgroup=LABEL[cmd], showlegend=False,
                                     line=line), row=2, col=col)
            fig.add_trace(go.Scatter(
                x=rec['t'], y=[rec['v_carla'][i] - rec['v_dyno'][i]
                               for i in range(n)],
                legendgroup=LABEL[cmd], showlegend=False,
                line=line), row=3, col=col)
            fig.add_trace(go.Scatter(x=rec['t'], y=rec['x_divergence'],
                                     legendgroup=LABEL[cmd],
                                     showlegend=False, line=line), row=4, col=col)
            if sc == 'leader':
                fig.add_trace(go.Scatter(x=rec['t'], y=rec['gap_carla'],
                                         legendgroup=LABEL[cmd],
                                         showlegend=False, line=line),
                              row=5, col=col)

        if sc == 'leader':
            fig.add_trace(go.Scatter(x=base['t'], y=base['gap_dyno'],
                                     legendgroup='bench (dyno)',
                                     showlegend=False,
                                     line=dict(color='#1f77b4', width=2)),
                          row=5, col=col)
        for row in (2, 3, 4):
            fig.add_hline(y=0.0, line=dict(color='black', width=1),
                          row=row, col=col)

    for row, label in ((1, 'speed [m/s]'), (2, 'wheel torque [Nm]'),
                       (3, 'v_carla - v_dyno [m/s]'),
                       (4, 'x_carla - x_bench [m]'), (5, 'gap [m]')):
        fig.update_yaxes(title_text=label, row=row, col=1)

    # Ticks and a crosshair on every panel, not just the bottom one. Reading a
    # divergence off row 3 against the brake event in row 1 is the whole job,
    # and that needs the time axis legible where you are looking.
    t_end = max(r['t'][-1] for r in runs.values())
    step = 10.0 if t_end <= 120.0 else 20.0
    fig.update_xaxes(
        showticklabels=True, dtick=step, tick0=0.0, range=[0.0, t_end],
        showgrid=True, gridcolor='rgba(0,0,0,0.12)',
        showspikes=True, spikemode='across', spikesnap='cursor',
        spikethickness=1, spikecolor='rgba(0,0,0,0.45)', spikedash='dot',
        ticks='outside', ticklen=4)
    fig.update_yaxes(showgrid=True, gridcolor='rgba(0,0,0,0.12)',
                     zeroline=True, zerolinecolor='rgba(0,0,0,0.35)')
    for col in (1, 2):
        fig.update_xaxes(title_text='time [s]', row=5, col=col)

    fig.update_layout(
        height=1500, hovermode='x unified', spikedistance=-1,
        plot_bgcolor='white',
        title='Does the CARLA side keep up with the bench? '
              'Left: free driving. Right: following a braking leader.',
        legend=dict(orientation='h', y=1.045))
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

    runs = {(sc, cmd): run(cmd, sc, args.duration, args.dt)
            for sc in SCENARIOS for cmd in COMMANDS}

    print('\nCARLA following the bench. Leader brakes at %.0f s.\n'
          % LeaderParams().brake_at_s)
    print('  %-8s %-8s %9s %9s %12s %10s %10s %8s %8s'
          % ('scenario', 'command', 'err_rms', 'err_mean', 'x_diverge[m]',
             'T_bench_pk', 'T_carla_pk', 'ratio', '%capac'))
    for sc in SCENARIOS:
        for cmd in COMMANDS:
            r = summarise(runs[(sc, cmd)], cmd, sc)
            print('  %-8s %-8s %9.4f %9.4f %12.2f %10.0f %10.0f %8.2f %7.0f%%'
                  % (r['scenario'], r['command'], r['speed_err_rms'],
                     r['speed_err_mean'], r['x_divergence_m'],
                     r['T_bench_peak'], r['T_carla_peak'], r['T_ratio_peak'],
                     100 * r['T_frac_of_capacity']))

    path = write_html(runs, os.path.join(outdir, 'tracking_scenarios.html'))
    print('\n  wrote %s\n' % path)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
