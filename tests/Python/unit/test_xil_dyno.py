"""Invariants for the simulated dyno bench (CommonLib/xil, #323).

Physics checks, not regression snapshots. Each one pins a property something
downstream relies on, so that changing the model cannot quietly change what a
coupling built on it is entitled to assume.

Run:  pytest tests/Python/unit/test_xil_dyno.py -v
"""

import math
import os
import sys

import pytest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__),
                                                '..', '..', '..')))

from CommonLib.xil import (  # noqa: E402
    NWHEEL, RL, AxleDynoParams, ChassisDynoParams, DrivelineParams, DynoConfig,
    DynoSimulator, PowertrainParams, RoadLoadParams,
)

DT = 0.001


def _chassis(**kw):
    return DynoSimulator(DynoConfig(mode='chassis', **kw))


def _axle(**kw):
    return DynoSimulator(DynoConfig(mode='axle', **kw))


def _run(dyno, thr, brk, secs, omega_cmd=None):
    s = None
    for _ in range(int(secs / DT)):
        s = dyno.step(thr, brk, DT, omega_cmd=omega_cmd)
    return s


# ------------------------------------------------------------------- config

def test_mode_must_be_one_of_two():
    with pytest.raises(ValueError):
        DynoSimulator(DynoConfig(mode='rollers'))


def test_chassis_refuses_a_speed_command():
    """Passing omega_cmd to chassis mode means the caller has the wrong picture
    of who owns the body, and silently ignoring it would hide that."""
    with pytest.raises(ValueError):
        _chassis().step(0.5, 0.0, DT, omega_cmd=[10.0] * NWHEEL)


def test_axle_requires_a_speed_command():
    with pytest.raises(ValueError):
        _axle().step(0.5, 0.0, DT)


def test_an_unstable_servo_is_rejected_at_construction():
    """A speed loop crossing over near its own actuator oscillates, and that
    oscillation looks like a physics result. Refuse to build it."""
    bad = AxleDynoParams(speed_loop_Hz=60.0, current_loop_Hz=25.0)
    with pytest.raises(ValueError, match='phase margin'):
        DynoSimulator(DynoConfig(mode='axle', axle=bad))


def test_servo_gains_scale_with_inertia():
    """Gains are derived so the requested bandwidth is what you actually get,
    whatever the inertia."""
    a = AxleDynoParams(speed_loop_Hz=20.0)
    kp1, _ = a.gains(1.0)
    kp2, _ = a.gains(4.0)
    assert kp2 == pytest.approx(4.0 * kp1)
    assert kp1 / 1.0 == pytest.approx(2 * math.pi * 20.0)


# ------------------------------------------------------------------ chassis

def test_a_parked_vehicle_stays_parked():
    """Road load must not accelerate a stopped car. It is a reaction force, and
    summing it as a signed term is the easy way to make one roll backwards."""
    s = _run(_chassis(), 0.0, 0.0, 5.0)
    assert s.speed_mps == pytest.approx(0.0, abs=1e-9)
    assert s.at_standstill
    assert s.tractive_force_N == pytest.approx(0.0, abs=1e-9)


def test_brake_brings_it_to_rest_and_holds():
    d = _chassis()
    _run(d, 1.0, 0.0, 6.0)
    assert d.v > 10.0
    s = _run(d, 0.0, 1.0, 8.0)
    assert s.speed_mps == pytest.approx(0.0, abs=1e-9)
    assert s.at_standstill


def test_terminal_speed_is_where_tractive_effort_meets_road_load():
    """The one closed-form check available: hold full throttle long enough and
    the vehicle settles where drive force equals road load."""
    d = _chassis()
    s = _run(d, 1.0, 0.0, 400.0)
    v = s.speed_mps
    assert 30.0 < v < 90.0, 'terminal speed %.1f m/s is not plausible' % v
    assert abs(s.tractive_force_N) < 25.0, 'not settled: net %.1f N' % s.tractive_force_N
    assert s.road_load_N == pytest.approx(sum(s.drive_torque_Nm)
                                          / d.cfg.driveline.wheel_radius_m, rel=0.02)


def test_road_load_matches_its_own_coefficients():
    d = _chassis(road_load=RoadLoadParams(A_N=100.0, B_Npms=2.0, C_Npms2=0.5))
    assert d.road_load_N(10.0) == pytest.approx(100.0 + 20.0 + 50.0)
    assert d.road_load_N(-10.0) == pytest.approx(-(100.0 + 20.0 + 50.0))


def test_roller_inertia_slows_the_acceleration():
    """It is referred to the road through r^2 and adds to the mass, so a heavier
    roller must accelerate more slowly on identical torque."""
    light = _chassis(chassis=ChassisDynoParams(roller_inertia_kgm2=0.0))
    heavy = _chassis(chassis=ChassisDynoParams(roller_inertia_kgm2=200.0))
    assert _run(heavy, 1.0, 0.0, 3.0).speed_mps < _run(light, 1.0, 0.0, 3.0).speed_mps


def test_wheels_are_rigidly_coupled_to_the_roller():
    d = _chassis()
    s = _run(d, 0.6, 0.0, 4.0)
    for w in s.wheel_omega_radps:
        assert w == pytest.approx(s.speed_mps / d.cfg.driveline.wheel_radius_m)


def test_grade_is_an_applied_force_and_can_roll_the_vehicle_back():
    d = _chassis(chassis=ChassisDynoParams(grade_rad=math.radians(15.0)))
    s = _run(d, 0.0, 0.0, 4.0)
    assert s.speed_mps < -0.5, 'a steep hill with no brake should roll it back'


# --------------------------------------------------------------------- axle

def test_the_servo_holds_the_commanded_speed():
    s = _run(_axle(), 0.3, 0.0, 3.0, omega_cmd=[20.0] * NWHEEL)
    for w in s.wheel_omega_radps:
        assert w == pytest.approx(20.0, abs=1e-3)


def test_at_steady_state_the_absorber_takes_exactly_what_the_vehicle_gives():
    """Zero acceleration means the load cell reads the absorber torque, negated.
    If these two ever disagree at steady state, an inertia is unaccounted for."""
    s = _run(_axle(), 0.3, 0.0, 3.0, omega_cmd=[20.0] * NWHEEL)
    for j in range(NWHEEL):
        assert s.axle_torque_Nm[j] == pytest.approx(-s.dyno_torque_Nm[j], rel=1e-6)


def test_measured_torque_tracks_the_pedal():
    hold = [20.0] * NWHEEL
    low = _run(_axle(), 0.2, 0.0, 3.0, omega_cmd=hold).axle_torque_Nm[RL]
    high = _run(_axle(), 0.6, 0.0, 3.0, omega_cmd=hold).axle_torque_Nm[RL]
    assert high > low > 0.0
    assert high == pytest.approx(3.0 * low, rel=0.05)


def test_the_bench_owns_no_body_in_axle_mode():
    s = _run(_axle(), 0.3, 0.0, 1.0, omega_cmd=[20.0] * NWHEEL)
    assert s.speed_mps is None


def test_the_servo_follows_a_changing_command():
    d = _axle()
    cmd = 5.0
    for k in range(6000):
        cmd = 5.0 + 15.0 * (k * DT) / 6.0
        s = d.step(0.3, 0.0, DT, omega_cmd=[cmd] * NWHEEL)
    assert s.wheel_omega_radps[RL] == pytest.approx(cmd, abs=0.05)


def test_brake_never_spins_the_wheel_backwards():
    d = _axle()
    _run(d, 0.4, 0.0, 2.0, omega_cmd=[20.0] * NWHEEL)
    s = _run(d, 0.0, 1.0, 4.0, omega_cmd=[0.0] * NWHEEL)
    for w in s.wheel_omega_radps:
        assert w > -1e-3, 'brake drove the wheel backwards to %.4f rad/s' % w


# --------------------------------------------------------------- powertrain

def test_the_envelope_is_constant_torque_then_constant_power():
    p = PowertrainParams(rear_peak_torque_Nm=1000.0, rear_peak_power_W=10.0e3,
                         front_share=0.0)
    d = DynoSimulator(DynoConfig(mode='axle', powertrain=p))
    # base speed is where peak power meets peak torque: 10 kW / 1000 Nm = 10 rad/s
    below = _run(d, 1.0, 0.0, 2.0, omega_cmd=[5.0] * NWHEEL).drive_torque_Nm[RL]
    d.reset()
    above = _run(d, 1.0, 0.0, 2.0, omega_cmd=[40.0] * NWHEEL).drive_torque_Nm[RL]
    assert below == pytest.approx(500.0, rel=0.02)     # 1000 Nm axle, split 50/50
    assert above == pytest.approx(125.0, rel=0.02)     # 10 kW / 40 rad/s / 2


def test_a_caller_can_replace_the_powertrain_entirely():
    """The hook for making the bench agree with another simulator's torque map
    instead of guessing at it."""
    calls = []

    def flat(throttle, brake, w_f, w_r):
        calls.append((throttle, w_r))
        return 0.0, 800.0

    d = DynoSimulator(DynoConfig(mode='axle'), powertrain=flat)
    s = _run(d, 0.5, 0.0, 2.0, omega_cmd=[20.0] * NWHEEL)
    assert calls, 'the override was never called'
    assert s.drive_torque_Nm[RL] == pytest.approx(400.0, rel=1e-3)
    assert s.drive_torque_Nm[0] == pytest.approx(0.0, abs=1e-6)


def test_torque_delivery_lags_the_command():
    """A step in pedal must not appear instantly at the wheel; the lag is a
    first-class error source for a speed-matched coupling."""
    d = _axle(driveline=DrivelineParams(torque_bandwidth_Hz=2.0))
    first = d.step(1.0, 0.0, DT, omega_cmd=[20.0] * NWHEEL).drive_torque_Nm[RL]
    settled = _run(d, 1.0, 0.0, 4.0, omega_cmd=[20.0] * NWHEEL).drive_torque_Nm[RL]
    assert abs(first) < 0.05 * abs(settled)


# ------------------------------------------------------------------- resets

def test_reset_restores_a_known_state():
    d = _chassis()
    _run(d, 1.0, 0.0, 5.0)
    d.reset(speed_mps=12.0)
    assert d.v == pytest.approx(12.0)
    assert d.t == 0.0
    for w in d.omega:
        assert w == pytest.approx(12.0 / d.cfg.driveline.wheel_radius_m)


def test_dt_must_be_positive():
    with pytest.raises(ValueError):
        _chassis().step(0.0, 0.0, 0.0)
