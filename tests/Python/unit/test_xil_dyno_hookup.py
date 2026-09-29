"""XilSetup: a dynamometer in the ego controller's loop (#24).

``fixsxil.dynosim()`` builds the simulated bench from the parameters it is given;
it reads no scenario. What the scenario decides -- whether a bench is in the loop,
and where a udp application's rig is -- comes from ``fixs.xil.enabled()`` and
``fixs.config.get('xil')``.

And one refusal: a dynamometer in the controller's loop is not a plant that
owns the ego, so EnableXil with Dynamics anything but virenv is two different
answers to who computes the ego's motion, and is refused rather than resolved.

    python -m pytest tests/Python/unit/test_xil_dyno_hookup.py
"""
from __future__ import annotations

import os
import sys

import pytest
import yaml

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.abspath(os.path.join(_HERE, '..', '..', '..'))
for _p in (_ROOT, os.path.join(_ROOT, 'CommonLib')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import fixs                                                    # noqa: E402
from fixs import xil as fixsxil                                 # noqa: E402
from CommonLib.ConfigHelper import ConfigHelper                 # noqa: E402

BASE = {
    "SimulationSetup": {"EnableRealSim": True, "SimulationEndTime": 100,
                        "SelectedTrafficSimulator": "SUMO"},
    "ApplicationSetup": {
        "EnableApplicationLayer": True,
        "VehicleSubscription": [{"type": "ego", "attribute": {"all": ["true"]},
                                 "ip": ["127.0.0.1"], "port": [430]}]},
    "EgoSetup": {"Id": "ego", "Dynamics": "virenv",
                 "ActuationSource": "user", "Controller": "ctl.py"},
    "CarlaSetup": {"EnableCosimulation": True, "CarlaTimeStep": 0.05},
}


def write(tmp_path, xil=None, dynamics="virenv", name="c.yaml"):
    doc = yaml.safe_load(yaml.safe_dump(BASE))
    doc["EgoSetup"]["Dynamics"] = dynamics
    if xil is not None:
        doc["XilSetup"] = xil
    p = tmp_path / name
    p.write_text(yaml.safe_dump(doc), encoding="utf-8")
    return str(p)


def xilOn(transport="inprocess", port=420, ip="127.0.0.1"):
    return {"EnableXil": True, "Transport": transport,
            "VehicleSubscription": [{"type": "ego",
                                     "attribute": {"id": ["ego"]},
                                     "ip": [ip], "port": [port]}]}


# ------------------------------------------------------------ what it builds

def test_dynosim_needs_no_scenario(monkeypatch):
    """A udp app's stand-in rig builds it with no yaml at all."""
    monkeypatch.delenv('FIXS_CONFIG_YAML', raising=False)
    d = fixsxil.dynosim()
    try:
        assert d.sim is not None
    finally:
        d.close()


def test_the_rig_address_comes_from_the_subscription(tmp_path):
    xil = fixs.config.get('xil', write(tmp_path, xilOn('udp', ip='192.168.140.24', port=4420)))
    assert xil['transport'] == 'udp'
    assert (xil['ip'], xil['port']) == ('192.168.140.24', 4420)


def test_an_unknown_transport_is_refused(tmp_path):
    with pytest.raises(SystemExit) as e:
        fixs.config.get('xil', write(tmp_path, xilOn('carrier-pigeon')))
    assert 'Transport' in str(e.value)


# ------------------------------------------------------------ what it answers

def test_the_bench_holds_the_speed_it_is_given():
    """The whole point, end to end: ask for 15 m/s and the bench gets there.

    And the torque it takes is the road load, which is checkable: the dyno
    absorbs A + B*v + C*v^2, and F*r must equal what the axles produced.
    """
    d = fixsxil.dynosim()
    try:
        for _ in range(400):                    # 20 s at the CARLA step
            got = d.exchange(15.0, 0.0, 0.05)
        assert got == pytest.approx(15.0, abs=0.05)
        assert d.misses == 0

        sim = d.sim
        state = sim.step_pedals(sim.throttle, sim.brake, 0.05)
        road = sim.dyno.resistance(15.0)
        assert sum(state.axle_torque) == pytest.approx(
            road * sim.vehicle.wheel_radius_m, rel=0.02)
    finally:
        d.close()


# ------------------------------------------ the bench, on either integrator

def test_dynamics_xil_is_refused(tmp_path):
    """'xil' as a Dynamics VALUE is unimplemented, bench or no bench. Unrelated
    to who integrates the ego -- see the two tests below, which both pass."""
    with pytest.raises(SystemExit):
        ConfigHelper().getConfig(
            write(tmp_path, xilOn(), dynamics="xil", name="xil.yaml"))


def test_traffic_with_a_bench_is_accepted(tmp_path):
    """A bench is not a plant that owns the ego, so it does not need the
    VIRTUAL ENVIRONMENT to own one either (#24).

    This was refused until the passive driver existed, on the reading that a
    cell only makes sense while CARLA's physics move the ego. The sentence
    holds for the traffic simulator word for word: it integrates position,
    heading and everything lateral, the driver runs passive -- no agent, no
    steering, no obstacle sweep -- and the cell answers the one question left,
    the longitudinal one. Both rungs then run the same controller file and the
    same cell, which is the whole point of having the pair."""
    cfg = ConfigHelper()
    cfg.getConfig(write(tmp_path, xilOn(), dynamics="traffic",
                        name="traffic.yaml"))
    assert cfg.Xil_setup['EnableXil'] is True
    assert cfg.Ego_setup['Dynamics'] == 'traffic'


def test_virenv_with_a_bench_is_accepted(tmp_path):
    cfg = ConfigHelper()
    cfg.getConfig(write(tmp_path, xilOn()))
    assert cfg.Xil_setup['EnableXil'] is True
    assert cfg.Xil_setup['Transport'] == 'inprocess'
    assert cfg.Ego_setup['Dynamics'] == 'virenv'


def test_transport_defaults_to_inprocess(tmp_path):
    """A yaml that says EnableXil and nothing else runs the bench here, which
    is the case that needs no hardware."""
    cfg = ConfigHelper()
    cfg.getConfig(write(tmp_path, {"EnableXil": True}))
    assert cfg.Xil_setup['Transport'] == 'inprocess'


def test_the_bridge_says_which_yaml_it_is_running(tmp_path):
    """mainVirCarla exports its -f so a controller loaded in-process cannot read
    a DIFFERENT scenario than the bridge hosting it. Without this the fallback
    above fires mid-run: measured, the bridge died on its first controlled tick
    with FileNotFoundError: 'config.yaml'."""
    src = os.path.join(_ROOT, 'Carla', 'VirEnv', 'mainVirCarla.py')
    with open(src, encoding='utf-8') as fh:
        text = fh.read()
    assert "os.environ['FIXS_CONFIG_YAML'] = os.path.abspath(args.configPath)" in text


def test_the_caller_says_what_vehicle_is_on_the_bench():
    """Otherwise every run gets the default car, and an ablation -- put a light
    vehicle on it and see whether the drive comes back -- cannot be run at all.
    """
    d = fixsxil.dynosim(vehicle={'mass_kg': 900.0, 'torque_bandwidth_Hz': 40.0},
                        dyno={'road_A_N': 0.0, 'roller_inertia_kgm2': 0.0})
    try:
        assert d.sim.vehicle.mass_kg == 900.0
        assert d.sim.vehicle.driveline.torque_bandwidth_Hz == 40.0
        assert d.sim.dyno.road_A_N == 0.0
        assert d.sim.dyno.roller_inertia_kgm2 == 0.0
    finally:
        d.close()


def test_a_misspelled_bench_parameter_fails_the_run():
    """Ignoring it would leave the bench quietly on its defaults, and the run
    would look like it tested the vehicle the caller described."""
    d = None
    try:
        with pytest.raises(TypeError):
            d = fixsxil.dynosim(vehicle={'mass': 900.0})
    finally:
        if d is not None:
            d.close()


def test_a_light_bench_reaches_its_reference_far_sooner():
    """The ablation itself, in miniature: strip the mass and the torque delay
    and the bench stops being a plant. That is what makes it a control -- if a
    light bench still changes a run, the bench is not what changed it."""
    heavy = fixsxil.dynosim()
    light = fixsxil.dynosim(vehicle={'mass_kg': 400.0, 'torque_bandwidth_Hz': 40.0},
                            dyno={'roller_inertia_kgm2': 0.0})
    try:
        def stepsTo(d, target):
            for n in range(1, 2001):
                if d.exchange(target, 0.0, 0.05) >= 0.95 * target:
                    return n
            return None
        nHeavy, nLight = stepsTo(heavy, 10.0), stepsTo(light, 10.0)
        assert nLight is not None and nHeavy is not None
        assert nLight < nHeavy
    finally:
        heavy.close()
        light.close()


def test_enabled_answers_the_scenario_not_the_client(tmp_path):
    """The flag is about the SCENARIO. A rig that brings its own cell client
    still gets a true answer, so control flow that depends on a bench being in
    the loop does not have to test whether OUR client was built."""
    assert fixsxil.enabled(write(tmp_path, xilOn())) is True
    assert fixsxil.enabled(write(tmp_path, dict(xilOn(), EnableXil=False),
                                 name='off.yaml')) is False
    assert fixsxil.enabled(write(tmp_path, name='none.yaml')) is False


def test_enabled_refuses_to_guess(tmp_path, monkeypatch):
    """Whether a bench is declared is a fact about the scenario; a scenario that
    cannot be read does not answer it."""
    monkeypatch.delenv('FIXS_CONFIG_YAML', raising=False)
    with pytest.raises(fixs.FixsError):
        fixsxil.enabled()


# -- the robot driver, capped to an envelope (#24) ---------------------------

def test_the_robot_cap_reaches_the_cell():
    """On a real cell the robot is the one of the three that is yours to set."""
    d = fixsxil.dynosim(driver={"max_throttle": 0.27})
    try:
        assert d.sim.driver.max_throttle == 0.27
    finally:
        d.close()


def test_the_envelope_holds_at_any_speed():
    """Quoted in m/s^2, and it has to mean the same thing at 20 m/s as off the
    line -- which is why it is not a pedal cap. A pedal bounds TORQUE, and the
    acceleration that buys falls away as road load and the power limit take
    their share."""
    from CommonLib.xil.dynosim import Dyno, DynoSim
    from CommonLib.xil.vehicle import Vehicle
    from CommonLib.xil.driver import RobotDriver

    def peaks(**kw):
        sim = DynoSim(Vehicle(mass_kg=2100.0),
                      Dyno(road_A_N=111.0, roller_inertia_kgm2=40.0),
                      RobotDriver(**kw))
        slow, fast, p = [], [], 0.0
        for _ in range(400):
            v = sim.step(25.0, 0.1).speed
            (slow if v < 5.0 else fast).append((v - p) / 0.1)
            p = v
        return max(slow), max(fast)

    assert min(peaks()) > 5.0, 'uncapped, this cell is quick'
    slow, fast = peaks(max_accel_mps2=1.8)
    assert slow < 2.0 and fast < 2.0, (slow, fast)
    assert abs(slow - fast) < 0.3, 'the envelope must not sag with speed'


def test_the_envelope_ramps_rather_than_clamping():
    """Clamping the reference to the measured speed looks equivalent and is
    not: it parks the setpoint permanently just ahead of actual, the integrator
    winds on that standing error, and the pedal grows until the vehicle exceeds
    the very rate the clamp was meant to impose. Measured that way, a 2.0 cap
    delivered 2.92. The ramp is what keeps the error small enough for the rate
    to be the ramp's."""
    from CommonLib.xil.dynosim import Dyno, DynoSim
    from CommonLib.xil.vehicle import Vehicle
    from CommonLib.xil.driver import RobotDriver

    sim = DynoSim(Vehicle(mass_kg=2100.0),
                  Dyno(road_A_N=111.0, roller_inertia_kgm2=40.0),
                  RobotDriver(max_accel_mps2=1.8, max_decel_mps2=1.8))
    p, worst = 0.0, 0.0
    for _ in range(400):
        v = sim.step(25.0, 0.1).speed
        worst = max(worst, (v - p) / 0.1)
        p = v
    assert worst < 2.0, worst


# -- openpilot's hold: slow and not asked to accelerate means brake -----------

HOLD = {'stop_speed_mps': 0.3, 'stop_accel_mps2': 0.1}


def test_the_hold_is_off_by_default():
    a, b = fixsxil.dynosim(), fixsxil.dynosim()
    for _ in range(40):
        assert a.exchange(0.15, 0.0, 0.05) == b.exchange(0.15, 1.0, 0.05)


def test_the_hold_keeps_a_slow_small_command_at_rest():
    d = fixsxil.dynosim(driver=HOLD)
    assert max(d.exchange(0.15, 0.0, 0.05) for _ in range(60)) == 0.0


def test_the_hold_lets_go_when_asked_to_accelerate():
    d = fixsxil.dynosim(driver=HOLD)
    assert [d.exchange(0.15, 1.5, 0.05) for _ in range(60)][-1] > 0.1


def test_the_hold_only_applies_while_slow():
    d = fixsxil.dynosim(driver=HOLD)
    for _ in range(200):
        d.exchange(1.0, 1.5, 0.05)
    assert d.exchange(1.0, 0.0, 0.05) > 0.9
