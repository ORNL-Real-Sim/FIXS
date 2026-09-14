"""XilSetup: a dynamometer in the ego controller's loop (#24).

``fixs.dyno()`` turns a scenario's XilSetup into the bench the controller talks
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
    assert fixs.dyno(write(tmp_path)) is None


def test_enablexil_false_means_no_bench(tmp_path):
    """Off is off. A default bench would silently change every run's plant."""
    assert fixs.dyno(write(tmp_path, dict(xilOn(), EnableXil=False))) is None


@pytest.mark.parametrize("transport", ["inprocess", "udp", "tcp"])
def test_each_transport_builds(tmp_path, transport):
    d = fixs.dyno(write(tmp_path, xilOn(transport), name=transport + ".yaml"))
    try:
        assert d.transport == transport
        assert (d.sim is not None) == (transport == 'inprocess')
    finally:
        d.close()


def test_the_endpoint_comes_from_the_subscription(tmp_path):
    """Not a constant here: the yaml says where the cell is."""
    port = 5399
    d = fixs.dyno(write(tmp_path, xilOn('udp', port=port)))
    try:
        assert d.link._peer[1] == port or d.link._peer[0] == '127.0.0.1'
    finally:
        d.close()


def test_an_unknown_transport_is_refused(tmp_path):
    with pytest.raises(SystemExit) as e:
        fixs.dyno(write(tmp_path, xilOn('carrier-pigeon')))
    assert 'Transport' in str(e.value)


# ------------------------------------------------------------ what it answers

def test_inprocess_bench_holds_the_speed_it_is_given(tmp_path):
    """The whole point, end to end: ask for 15 m/s and the bench gets there.

    And the torque it takes is the road load, which is checkable: the dyno
    absorbs A + B*v + C*v^2, and F*r must equal what the axles produced.
    """
    d = fixs.dyno(write(tmp_path, xilOn()))
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
    d = fixs.dyno(write(tmp_path, xilOn('tcp', port=5398)))
    try:
        assert d.exchange(12.0, 0.05) == pytest.approx(12.0)
        assert d.misses == 1
        assert d.age() is None
    finally:
        d.close()


# ----------------------------------------------------------- the one refusal

@pytest.mark.parametrize("dynamics", ["traffic", "xil"])
def test_a_bench_needs_the_virenv_to_own_the_ego(tmp_path, dynamics):
    """EnableXil puts a bench in the CONTROLLER's loop. The virtual environment
    still integrates position, heading and everything lateral. Saying otherwise
    is a run with two answers to who computes the ego's motion."""
    with pytest.raises(SystemExit):
        ConfigHelper().getConfig(
            write(tmp_path, xilOn(), dynamics=dynamics, name=dynamics + ".yaml"))


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
        fixs.dyno()
    assert 'FIXS_CONFIG_YAML' in str(e.value)

    monkeypatch.setenv('FIXS_CONFIG_YAML', str(tmp_path / 'nope.yaml'))
    with pytest.raises(fixs.FixsError):
        fixs.dyno()


def test_the_bridge_says_which_yaml_it_is_running(tmp_path):
    """mainVirCarla exports its -f so a controller loaded in-process cannot read
    a DIFFERENT scenario than the bridge hosting it. Without this the fallback
    above fires mid-run: measured, the bridge died on its first controlled tick
    with FileNotFoundError: 'config.yaml'."""
    src = os.path.join(_ROOT, 'Carla', 'VirEnv', 'mainVirCarla.py')
    with open(src, encoding='utf-8') as fh:
        text = fh.read()
    assert "os.environ['FIXS_CONFIG_YAML'] = os.path.abspath(args.configPath)" in text
