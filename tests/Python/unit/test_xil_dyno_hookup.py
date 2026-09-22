"""XilSetup: a dynamometer in the ego controller's loop (#24).

``fixsxil.dyno()`` turns a scenario's XilSetup into the bench the controller talks
to. What is worth testing is not the bench -- test_xil_dyno.py does that -- but
the three things the hookup can get wrong without failing:

  - a scenario that declares no bench must hand back None, not a default one,
  - a scenario that declares one must reach the endpoint IT names,
  - a bench that never answers must not invent motion.

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

def test_no_xilsetup_at_all_means_no_bench(tmp_path):
    assert fixsxil.dyno(write(tmp_path)) is None


def test_enablexil_false_means_no_bench(tmp_path):
    """Off is off. A default bench would silently change every run's plant."""
    assert fixsxil.dyno(write(tmp_path, dict(xilOn(), EnableXil=False))) is None


@pytest.mark.parametrize("transport", ["inprocess", "udp", "tcp"])
def test_each_transport_builds(tmp_path, transport):
    d = fixsxil.dyno(write(tmp_path, xilOn(transport), name=transport + ".yaml"))
    try:
        assert d.transport == transport
        assert (d.sim is not None) == (transport == 'inprocess')
    finally:
        d.close()


def test_the_endpoint_comes_from_the_subscription(tmp_path):
    """Not a constant here: the yaml says where the cell is."""
    port = 5399
    d = fixsxil.dyno(write(tmp_path, xilOn('udp', port=port)))
    try:
        assert d.link._peer[1] == port or d.link._peer[0] == '127.0.0.1'
    finally:
        d.close()


def test_an_unknown_transport_is_refused(tmp_path):
    with pytest.raises(SystemExit) as e:
        fixsxil.dyno(write(tmp_path, xilOn('carrier-pigeon')))
    assert 'Transport' in str(e.value)


# ------------------------------------------------------------ what it answers

def test_inprocess_bench_holds_the_speed_it_is_given(tmp_path):
    """The whole point, end to end: ask for 15 m/s and the bench gets there.

    And the torque it takes is the road load, which is checkable: the dyno
    absorbs A + B*v + C*v^2, and F*r must equal what the axles produced.
    """
    d = fixsxil.dyno(write(tmp_path, xilOn()))
    try:
        for _ in range(400):                    # 20 s at the CARLA step
            got = d.exchange(15.0, 0.05)
        assert got == pytest.approx(15.0, abs=0.05)
        assert d.misses == 0

        sim = d.sim
        state = sim.step_pedals(sim.throttle, sim.brake, 0.05)
        road = sim.dyno.resistance(15.0)
        assert sum(state.axle_torque) == pytest.approx(
            road * sim.vehicle.wheel_radius_m, rel=0.02)
    finally:
        d.close()


def test_a_silent_bench_gives_the_reference_straight_back(tmp_path):
    """No answer must not become no motion, and must not become invented
    motion either. The reference returns unchanged -- the run behaves as though
    no bench were attached -- and the miss is counted, because a run that ends
    with a large count did not test what it claims to have tested."""
    d = fixsxil.dyno(write(tmp_path, xilOn('tcp', port=5398)))
    try:
        assert d.exchange(12.0, 0.05) == pytest.approx(12.0)
        assert d.misses == 1
        assert d.age() is None
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


def test_an_unreadable_scenario_is_not_read_as_no_bench(tmp_path, monkeypatch):
    """The dangerous direction. Whether a bench is declared is a fact about the
    scenario; a scenario that cannot be read does not answer it, and guessing
    'no' would quietly run the plant the yaml did not ask for."""
    monkeypatch.delenv('FIXS_CONFIG_YAML', raising=False)
    with pytest.raises(fixs.FixsError) as e:
        fixsxil.dyno()
    assert 'FIXS_CONFIG_YAML' in str(e.value)

    monkeypatch.setenv('FIXS_CONFIG_YAML', str(tmp_path / 'nope.yaml'))
    with pytest.raises(fixs.FixsError):
        fixsxil.dyno()


def test_the_bridge_says_which_yaml_it_is_running(tmp_path):
    """mainVirCarla exports its -f so a controller loaded in-process cannot read
    a DIFFERENT scenario than the bridge hosting it. Without this the fallback
    above fires mid-run: measured, the bridge died on its first controlled tick
    with FileNotFoundError: 'config.yaml'."""
    src = os.path.join(_ROOT, 'Carla', 'VirEnv', 'mainVirCarla.py')
    with open(src, encoding='utf-8') as fh:
        text = fh.read()
    assert "os.environ['FIXS_CONFIG_YAML'] = os.path.abspath(args.configPath)" in text


def test_the_yaml_says_what_vehicle_is_on_the_bench(tmp_path):
    """Otherwise every run gets the default car, and an ablation -- put a light
    vehicle on it and see whether the drive comes back -- cannot be run at all.
    """
    cfg = dict(xilOn(), Vehicle={'mass_kg': 900.0, 'torque_bandwidth_Hz': 40.0},
               Dyno={'road_A_N': 0.0, 'roller_inertia_kgm2': 0.0})
    d = fixsxil.dyno(write(tmp_path, cfg))
    try:
        assert d.sim.vehicle.mass_kg == 900.0
        assert d.sim.vehicle.driveline.torque_bandwidth_Hz == 40.0
        assert d.sim.dyno.road_A_N == 0.0
        assert d.sim.dyno.roller_inertia_kgm2 == 0.0
    finally:
        d.close()


def test_a_misspelled_bench_parameter_fails_the_run(tmp_path):
    """Ignoring it would leave the bench quietly on its defaults, and the run
    would look like it tested the vehicle the yaml described."""
    d = None
    try:
        with pytest.raises(TypeError):
            d = fixsxil.dyno(write(tmp_path, dict(xilOn(),
                                                  Vehicle={'mass': 900.0})))
    finally:
        if d is not None:
            d.close()


def test_a_light_bench_reaches_its_reference_far_sooner(tmp_path):
    """The ablation itself, in miniature: strip the mass and the torque delay
    and the bench stops being a plant. That is what makes it a control -- if a
    light bench still changes a run, the bench is not what changed it."""
    heavy = fixsxil.dyno(write(tmp_path, xilOn(), name='heavy.yaml'))
    light = fixsxil.dyno(write(tmp_path, dict(
        xilOn(), Vehicle={'mass_kg': 400.0, 'torque_bandwidth_Hz': 40.0},
        Dyno={'roller_inertia_kgm2': 0.0}), name='light.yaml'))
    try:
        def stepsTo(d, target):
            for n in range(1, 2001):
                if d.exchange(target, 0.05) >= 0.95 * target:
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


def test_enabled_refuses_to_guess_like_dyno_does(tmp_path, monkeypatch):
    monkeypatch.delenv('FIXS_CONFIG_YAML', raising=False)
    with pytest.raises(fixs.FixsError):
        fixsxil.enabled()


# -- the robot driver, capped to an envelope (#24) ---------------------------

def test_the_robot_driver_comes_from_the_yaml(tmp_path):
    """Vehicle and Dyno were settable and the ROBOT was not, so a bench could
    not be held inside an acceleration envelope without pretending the car had
    less torque than it has. On a real cell the robot is the one of the three
    that is yours to set."""
    y = dict(xilOn(), Driver={"max_throttle": 0.27, "max_brake": 0.27})
    cfg = ConfigHelper()
    cfg.getConfig(write(tmp_path, y, name="driver.yaml"))
    assert cfg.Xil_setup["Driver"] == {"max_throttle": 0.27, "max_brake": 0.27}


def test_the_yaml_cap_reaches_the_cell(tmp_path):
    d = fixsxil.dyno(write(tmp_path,
                           dict(xilOn(), Driver={"max_throttle": 0.27}),
                           name="capped.yaml"))
    try:
        assert d.sim.driver.max_throttle == 0.27
    finally:
        d.close()


def test_a_capped_robot_holds_the_envelope():
    """The cap is on the PEDAL, so what it buys is an acceleration envelope.
    Uncapped this cell reaches 7.4 m/s^2 -- more than a traffic simulator
    holding the ego to 2.0 will deliver, and the gap does not close because the
    cell integrates on from a speed the ego never had."""
    from CommonLib.xil.dynosim import Dyno, DynoSim
    from CommonLib.xil.vehicle import Vehicle
    from CommonLib.xil.driver import RobotDriver

    def peak(**kw):
        sim = DynoSim(Vehicle(mass_kg=2100.0),
                      Dyno(road_A_N=111.0, roller_inertia_kgm2=40.0),
                      RobotDriver(**kw))
        a, p = 0.0, 0.0
        for _ in range(400):
            v = sim.step(20.0, 0.1).speed
            a = max(a, (v - p) / 0.1)
            p = v
        return a

    assert peak() > 5.0, "uncapped, this cell is quick"
    assert peak(max_throttle=0.27) < 2.0, "capped, it stays inside the envelope"
