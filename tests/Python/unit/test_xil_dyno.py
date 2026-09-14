"""Invariants for the simulated dyno bench (CommonLib/xil, #323).

Physics checks, not regression snapshots. Each pins a property something
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
    DynoSimulator, DynoVehicle, DynoVehicleConfig, PowertrainParams,
    envelope_powertrain,
    RoadResistanceParams, RobotDriver, RobotDriverParams,
)

DT = 0.001


def _chassis(**kw):
    return DynoSimulator(DynoConfig(mode='chassis', **kw))


def _axle(**kw):
    return DynoSimulator(DynoConfig(mode='axle', **kw))


def _run(dyno, thr, brk, secs):
    s = None
    for _ in range(int(secs / DT)):
        s = dyno.step(thr, brk, DT)
    return s


# ------------------------------------------------------------------- config

def test_mode_must_be_one_of_two():
    with pytest.raises(ValueError):
        DynoSimulator(DynoConfig(mode='rollers'))


def test_dt_must_be_positive():
    with pytest.raises(ValueError):
        _chassis().step(0.0, 0.0, 0.0)


def test_a_bad_wheel_radius_is_rejected():
    with pytest.raises(ValueError):
        DynoSimulator(DynoConfig(driveline=DrivelineParams(wheel_radius_m=0.0)))


# --------------------------------------------------------------- sub-stepping

def test_the_bench_chops_a_large_dt_into_steps_it_can_integrate():
    """A caller sets the outer rate and should not need to know the bench's time
    constants. Integrated whole, a 0.1 s step diverges -- the speed reached 1e81
    before the clamps snapped it to zero, which reads as a stopped vehicle."""
    d = _chassis()
    assert d.step(0.5, 0.0, 0.001).substeps == 1
    assert d.step(0.5, 0.0, 0.100).substeps > 1


def test_the_answer_does_not_depend_on_the_callers_rate():
    ref = None
    for dt in (0.001, 0.02, 0.05, 0.1, 0.2):
        car = DynoVehicle()
        for _ in range(int(40.0 / dt)):
            s = car.step(15.0, dt)
        if ref is None:
            ref = s.speed_mps
        assert s.speed_mps == pytest.approx(ref, abs=1e-3), \
            'dt=%g gave %.6f against %.6f at 1 ms' % (dt, s.speed_mps, ref)


def test_the_inner_step_follows_the_fastest_dynamic():
    slow = DynoSimulator(DynoConfig(
        driveline=DrivelineParams(torque_bandwidth_Hz=1.0)))
    fast = DynoSimulator(DynoConfig(
        driveline=DrivelineParams(torque_bandwidth_Hz=20.0)))
    assert fast.inner_dt < slow.inner_dt
    assert slow.inner_dt == pytest.approx(0.1)


# ------------------------------------------------------------------ chassis

def test_a_parked_vehicle_stays_parked():
    """Road resistance must not accelerate a stopped car. It is a reaction
    force, and summing it as a signed term makes one roll backwards."""
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


def test_terminal_speed_is_where_tractive_effort_meets_road_resistance():
    """The one closed-form check available: hold full throttle long enough and
    the vehicle settles where drive force equals road resistance.

    Stepped at 10 ms rather than 1 ms -- the bench sub-steps internally, so the
    answer is the same and the test does not need 400 000 iterations to say so.
    """
    d = _chassis()
    for _ in range(40000):                      # 400 s
        s = d.step(1.0, 0.0, 0.01)
    assert abs(s.tractive_force_N) < 25.0, 'not settled: %.1f N' % s.tractive_force_N
    assert s.road_resistance_N == pytest.approx(
        sum(s.drive_torque_Nm) / d.cfg.driveline.wheel_radius_m, rel=0.02)


def test_the_speed_limiter_holds_the_top_speed():
    """Real EVs are limited well below what their power would reach. Without
    one this bench settled at 229 km/h against an EV6's actual 185."""
    d = _chassis()
    for _ in range(40000):
        s = d.step(1.0, 0.0, 0.01)
    limit = d.cfg.powertrain.max_speed_mps
    assert s.speed_mps <= limit
    assert s.speed_mps > limit - d.cfg.powertrain.limiter_taper_mps - 0.5


def test_full_throttle_delivers_the_whole_vehicle():
    """front_share splits the demand; it must not scale each axle's capability.
    Doing that gave 51 % of the car at full throttle -- 3280 of 6400 Nm."""
    p = PowertrainParams()
    Tf, Tr = envelope_powertrain(p, 0.36)(1.0, 0.0, 0.0, 0.0)
    capability = p.front_peak_torque_Nm + p.rear_peak_torque_Nm
    assert (Tf + Tr) > 0.99 * capability


def test_the_share_is_honoured_below_the_caps():
    p = PowertrainParams()
    Tf, Tr = envelope_powertrain(p, 0.36)(0.5, 0.0, 0.0, 0.0)
    assert Tf / (Tf + Tr) == pytest.approx(p.front_share, abs=1e-6)


def test_a_zero_share_means_that_axle_is_not_driven():
    """Rear-drive-only is a real vehicle, and an earlier spill-over rule sent it
    torque anyway."""
    for share, driven, idle in ((0.0, 1, 0), (1.0, 0, 1)):
        T = envelope_powertrain(PowertrainParams(front_share=share),
                                0.36)(1.0, 0.0, 0.0, 0.0)
        assert T[idle] == pytest.approx(0.0)
        assert T[driven] > 0.0


def test_road_resistance_matches_its_own_coefficients():
    d = _chassis(road_resistance=RoadResistanceParams(A_N=100.0, B_Npms=2.0,
                                                      C_Npms2=0.5))
    assert d.road_resistance_N(10.0) == pytest.approx(170.0)
    assert d.road_resistance_N(-10.0) == pytest.approx(-170.0)


def test_roller_inertia_slows_the_acceleration():
    """Referred to the road through r^2, so a heavier roller must accelerate
    more slowly on identical torque."""
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
    assert _run(d, 0.0, 0.0, 4.0).speed_mps < -0.5


# --------------------------------------------------------------------- axle

def test_an_axle_bench_spins_up_on_its_own():
    """The hubs absorb the road-resistance share, so the wheels accelerate and
    the bench is a whole vehicle again."""
    s = _run(_axle(), 0.5, 0.0, 6.0)
    assert s.wheel_omega_radps[RL] > 10.0
    assert s.speed_mps > 3.0
    assert s.road_resistance_N > 0.0


def test_the_simulated_mass_is_what_the_hubs_have_to_accelerate():
    """There is no body on an axle dyno, so the translational inertia is added
    electrically. A heavier simulated vehicle must spin up more slowly."""
    light = _axle(axle=AxleDynoParams(simulated_mass_kg=500.0))
    heavy = _axle(axle=AxleDynoParams(simulated_mass_kg=4000.0))
    assert heavy.axle_inertia_kgm2 > light.axle_inertia_kgm2
    assert (_run(heavy, 0.5, 0.0, 3.0).wheel_omega_radps[RL]
            < _run(light, 0.5, 0.0, 3.0).wheel_omega_radps[RL])


def test_an_axle_bench_holds_still_with_no_pedal():
    s = _run(_axle(), 0.0, 0.0, 5.0)
    assert s.at_standstill
    for w in s.wheel_omega_radps:
        assert w == pytest.approx(0.0, abs=1e-9)


def test_brake_never_spins_the_wheel_backwards():
    d = _axle()
    _run(d, 0.4, 0.0, 3.0)
    s = _run(d, 0.0, 1.0, 6.0)
    for w in s.wheel_omega_radps:
        assert w > -1e-3, 'brake drove the wheel backwards to %.4f rad/s' % w


# --------------------------------------------------------------- powertrain

def test_the_envelope_is_constant_torque_then_constant_power():
    """Base speed is where peak power meets peak torque: 10 kW / 1000 Nm."""
    p = PowertrainParams(rear_peak_torque_Nm=1000.0, rear_peak_power_W=10.0e3,
                         front_share=0.0)
    d = DynoSimulator(DynoConfig(mode='axle', powertrain=p))
    d.omega = [5.0] * NWHEEL                    # below base speed
    below = d.step(1.0, 0.0, 4.0).drive_torque_Nm[RL]
    d = DynoSimulator(DynoConfig(mode='axle', powertrain=p))
    d.omega = [40.0] * NWHEEL                   # above it
    above = d.step(1.0, 0.0, 0.001).drive_torque_Nm[RL]
    assert below > above
    assert above <= 125.0 * 1.05                # 10 kW / 40 rad/s, halved by the diff


def test_a_caller_can_replace_the_powertrain_entirely():
    """The hook for making the bench agree with another simulator's torque map
    instead of guessing at it."""
    calls = []

    def flat(throttle, brake, w_f, w_r):
        calls.append((throttle, w_r))
        return 0.0, 800.0

    d = DynoSimulator(DynoConfig(mode='chassis'), powertrain=flat)
    s = _run(d, 0.5, 0.0, 3.0)
    assert calls, 'the override was never called'
    assert s.drive_torque_Nm[RL] == pytest.approx(400.0, rel=1e-3)
    assert s.drive_torque_Nm[0] == pytest.approx(0.0, abs=1e-6)


def test_torque_delivery_lags_the_command():
    d = _chassis(driveline=DrivelineParams(torque_bandwidth_Hz=2.0))
    first = d.step(1.0, 0.0, DT).drive_torque_Nm[RL]
    settled = _run(d, 1.0, 0.0, 4.0).drive_torque_Nm[RL]
    assert abs(first) < 0.05 * abs(settled)


# ------------------------------------------------------------- robot driver

def test_the_driver_drives_the_speed_error_to_zero():
    """The whole specification. A proportional-only law would sit permanently
    below the reference by however much error makes the pedal it needs."""
    car = DynoVehicle()
    for _ in range(int(60.0 / DT)):
        s = car.step(15.0, DT)
    assert s.speed_mps == pytest.approx(15.0, abs=1e-3)


def test_the_pedal_is_pinned_by_physics_not_by_history():
    """Under road resistance a steady speed needs exactly the pedal whose
    tractive force balances it, so the same setpoint reached three different
    ways must settle on the same pedal."""
    parked = []
    for ramp in (None, 5.0, 20.0):
        car = DynoVehicle()
        for k in range(int(60.0 / DT)):
            t = k * DT
            ref = 15.0 if ramp is None else min(15.0, 15.0 * t / ramp)
            car.step(ref, DT)
        parked.append(car.last_throttle)
    assert max(parked) - min(parked) < 1e-4, 'history-dependent pedal: %s' % parked


def test_throttle_and_brake_are_never_both_applied():
    drv = RobotDriver()
    for ref, v in ((20.0, 0.0), (0.0, 20.0), (10.0, 10.0), (5.0, 5.2)):
        thr, brk = drv.step(ref, v, DT)
        assert thr == 0.0 or brk == 0.0
        assert 0.0 <= thr <= 1.0 and 0.0 <= brk <= 1.0


def test_the_integral_does_not_wind_up_against_a_rail():
    drv = RobotDriver()
    for _ in range(20000):
        drv.step(100.0, 0.0, DT)                # unreachable: pedal saturated
    assert abs(drv.integral) < 10.0, \
        'integral wound to %.1f while saturated' % drv.integral


def test_a_standstill_reference_is_held_on_the_brake():
    """A PI chasing zero from zero dithers; a driver stops."""
    car = DynoVehicle()
    for _ in range(int(5.0 / DT)):
        s = car.step(0.0, DT)
    assert s.speed_mps == pytest.approx(0.0, abs=1e-9)
    assert car.last_brake > 0.0 and car.last_throttle == 0.0


def test_separate_throttle_and_brake_ceilings():
    drv = RobotDriver(RobotDriverParams(max_throttle=0.25, max_brake=1.0))
    for _ in range(2000):
        thr, _ = drv.step(50.0, 0.0, DT)
    assert thr == pytest.approx(0.25)


def test_the_vehicle_composes_driver_and_bench():
    car = DynoVehicle(DynoVehicleConfig())
    car.step(10.0, DT)
    assert car.last_throttle > 0.0
    car.reset(speed_mps=7.0)
    assert car.speed_mps == pytest.approx(7.0)
    assert car.driver.integral == 0.0


def test_the_vehicle_can_fail_to_reach_the_reference():
    """The gap between asked and achieved is the point of having a plant."""
    car = DynoVehicle()
    for _ in range(int(3.0 / DT)):
        s = car.step(60.0, DT)                  # far beyond reach in 3 s
    assert s.speed_mps < 40.0
    assert car.last_throttle == pytest.approx(1.0)
