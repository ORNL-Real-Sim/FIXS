"""Invariants for the dyno / CARLA speed-sync study (#323).

These pin the properties the study's conclusions rest on, so that changing the
model cannot silently stop the conclusions following from it. They are not
accuracy tests against measured data -- there is none yet.

The bench itself is tested in ``tests/Python/unit/test_xil_dyno.py``; this file
only covers the coupling. Anything here that would also hold for the bench alone
belongs there instead.

Run:  pytest tests/Xil/Probes/dyno_speed_sync/ -q
"""

import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import dyno_sync_sim as sim  # noqa: E402

from CommonLib.xil import DynoSimulator  # noqa: E402


def _cycle(secs=60.0):
    return sim.synthetic_cycle(secs, 0.001)


# ------------------------------------------------------ the bench is shipped

def test_the_study_drives_the_shipped_bench_not_a_copy():
    """If this ever stops being the CommonLib simulator, the study is measuring
    something the rest of FIXS does not use."""
    assert isinstance(sim.build_dyno(), DynoSimulator)
    assert sim.build_dyno().cfg.mode == 'chassis'


def test_both_plants_are_driven_by_pedals():
    """A CARLA agent writes throttle and brake, so the comparison has to feed
    both plants that and not a force."""
    d = sim.PedalDriver(0.45, 0.25)
    thr, brk = d.step(10.0, 0.0, 0.001)
    assert thr > 0.0 and brk == 0.0
    thr, brk = d.step(0.0, 10.0, 0.001)
    assert brk > 0.0 and thr == 0.0
    assert 0.0 <= thr <= 1.0 and 0.0 <= brk <= 1.0


# ---------------------------------------------------------- the disagreement

def test_the_resistance_gap_changes_sign():
    """The central physical claim: the mismatch is not a constant offset.

    CARLA's rolling term dominates low down and the bench's C dominates high up,
    so the disagreement crosses zero. If it stopped crossing, one bias correction
    would fix the coupling and the study's conclusion would be wrong.
    """
    gap = sim.steady_state_gap_N(sim.build_dyno(), sim.CarlaParams(),
                                 [float(v) for v in range(0, 41)])
    assert gap[0][1] < 0.0 < gap[-1][1]
    crossings = [v for (v, g), (_, gn) in zip(gap, gap[1:]) if g * gn < 0]
    assert len(crossings) == 1, 'expected one sign change, got %s' % crossings


def _matched():
    """A bench and a surrogate whose resistance curves are the same function.

    CARLA's aero is 0.5*rho*CdA*v^2 with rho hardcoded to 1.25, so setting the
    bench's C to that and zeroing A, B and CARLA's rolling makes them identical.
    """
    c = sim.CarlaParams(roll_coeff=0.0)

    def over(cfg):
        cfg.road_load.A_N = 0.0
        cfg.road_load.B_Npms = 0.0
        cfg.road_load.C_Npms2 = 0.5 * c.air_density * c.aero_CdA_m2
        cfg.chassis.vehicle_mass_kg = c.mass_kg
        cfg.chassis.roller_inertia_kgm2 = 0.0
    return over, c


def test_matching_the_resistance_models_cuts_the_sync_force():
    over, c = _matched()
    run = sim.RunParams()
    matched = sim.simulate(_cycle(40), sim.build_dyno(over), c, run,
                           'forced_driven')['_summary']['F_sync_rms_N']
    stock = sim.simulate(_cycle(40), sim.build_dyno(), sim.CarlaParams(), run,
                         'forced_driven')['_summary']['F_sync_rms_N']
    assert matched < stock / 3.0


def test_torque_delivery_lag_is_a_second_independent_source():
    """Matching the resistance does not drive F_sync to zero, and that matters.

    The bench delivers torque through a second-order lag; the surrogate applies
    it immediately, the way CARLA's apply_control does. The plants differ
    dynamically even with identical resistance, and the sync pays for it.
    Widening the bench's torque bandwidth shrinks it, which is what identifies
    the lag as the cause rather than the resistance.
    """
    over, c = _matched()
    run = sim.RunParams()

    def slow_cfg(cfg):
        over(cfg)
        cfg.driveline.torque_bandwidth_Hz = 2.0

    def fast_cfg(cfg):
        over(cfg)
        cfg.driveline.torque_bandwidth_Hz = 50.0

    slow = sim.simulate(_cycle(40), sim.build_dyno(slow_cfg), c, run,
                        'forced_driven')['_summary']['F_sync_rms_N']
    fast = sim.simulate(_cycle(40), sim.build_dyno(fast_cfg), c, run,
                        'forced_driven')['_summary']['F_sync_rms_N']
    assert slow > 1.0, 'matched resistance should still leave a lag residual'
    assert fast < slow / 2.0, 'a faster torque loop should shrink it'


# ---------------------------------------------------------------- the point

def test_sync_force_barely_moves_with_sync_rate_but_the_error_scales():
    """The result the study exists to establish.

    F_sync = m*dv/T and dv grows with T, so the injected FORCE is roughly
    invariant while the speed ERROR scales with T. Syncing faster buys a tighter
    speed match and does nothing about the disturbance. The force is not exactly
    invariant -- the plants also differ dynamically, which puts content in dv
    that does not accumulate linearly -- so the bound here is loose on purpose.
    """
    cycle = _cycle(60)
    dyno, carla = sim.build_dyno(), sim.CarlaParams()
    out = {T: sim.simulate(cycle, dyno, carla, sim.RunParams(sync_dt_s=T),
                           'forced_driven')['_summary']
           for T in (0.005, 0.05)}

    force = out[0.05]['F_sync_rms_N'] / out[0.005]['F_sync_rms_N']
    error = out[0.05]['speed_err_rms_mps'] / out[0.005]['speed_err_rms_mps']
    assert 0.75 < force < 1.3, 'force should be roughly invariant, got %.3f' % force
    assert error > 5.0, 'error should scale with T, got %.3f' % error


def test_forced_driven_beats_forced_coast_by_a_wide_margin():
    """Coasting CARLA makes the sync supply the whole tractive effort. That is
    the pessimistic bound, not the architecture."""
    cycle = _cycle(60)
    args = (cycle, sim.build_dyno(), sim.CarlaParams(), sim.RunParams())
    coast = sim.simulate(*args, 'forced_coast')['_summary']
    driven = sim.simulate(*args, 'forced_driven')['_summary']
    assert driven['F_sync_rms_N'] < coast['F_sync_rms_N'] / 2.0


def test_forced_tracks_speed_far_better_than_a_closed_loop_driver():
    cycle = _cycle(60)
    args = (cycle, sim.build_dyno(), sim.CarlaParams(), sim.RunParams())
    track = sim.simulate(*args, 'track')['_summary']
    forced = sim.simulate(*args, 'forced_driven')['_summary']
    assert forced['speed_err_rms_mps'] < track['speed_err_rms_mps'] / 20.0


def test_distance_error_stays_small_under_forced_sync():
    s = sim.simulate(_cycle(60), sim.build_dyno(), sim.CarlaParams(),
                     sim.RunParams(), 'forced_driven')['_summary']
    assert abs(s['distance_err_pct']) < 0.05


# ------------------------------------------------------------------ plumbing

def test_unknown_coupling_is_rejected():
    with pytest.raises(ValueError):
        sim.simulate([0.0], sim.build_dyno(), sim.CarlaParams(),
                     sim.RunParams(), 'nonsense')


def test_cycle_csv_roundtrip(tmp_path):
    p = tmp_path / 'cycle.csv'
    p.write_text('time,speed\n0,0\n1,10\n2,10\n')
    out = sim.load_cycle_csv(str(p), 0.01)
    assert len(out) == 200
    assert out[0] == pytest.approx(0.0, abs=1e-9)
    assert out[100] == pytest.approx(10.0, abs=0.2)


def test_main_writes_its_outputs(tmp_path):
    rc = sim.main(['--duration', '5', '--no-plot', '--out', str(tmp_path)])
    assert rc == 0
    assert (tmp_path / 'summary.json').is_file()
    for coupling in sim.COUPLINGS:
        assert (tmp_path / ('trace_%s.csv' % coupling)).is_file()
