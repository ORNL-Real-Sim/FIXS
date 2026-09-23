"""A scenario yaml that declares no CARLA half, and a run that believes it.

run_cosim reads every CarlaSetup key through a default, so until now an absent
CarlaSetup block was indistinguishable from one spelling the defaults out -- and
the defaults are a complete, plausible CARLA. A traffic-only scenario therefore
launched CARLA and started a bridge on DEFAULT_BRIDGE_PORT, which TrafficLayer
(same file, no defaults) then refused: "declares no subscription on port 4440. It
declares: [4430]."

This is not --sumo-only by another name. --sumo-only drops the bridge from a
scenario that HAS one; this is a scenario that never had one, which is a choice it
makes rather than a lesser run.

Nothing here launches anything: the question is answered from a text file.
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "..", "scripts", "cosim"))
import run_cosim  # noqa: E402


def _cosim_yaml(tmp_path, name="cosim.yaml", client_port=4440):
    """A yaml with a CARLA half: CarlaSetup, and a subscription on its bridge port."""
    path = tmp_path / name
    path.write_text(
        "SimulationSetup:\n"
        "  TrafficSimulatorPort: 1337\n"
        "ApplicationSetup:\n"
        "  VehicleSubscription:\n"
        "  - type: \"ego\"\n"
        "    ip: [\"127.0.0.1\"]\n"
        f"    port: [{client_port}]\n"
        "CarlaSetup:\n"
        "  CarlaServerIP: 127.0.0.1\n"
        "  CarlaServerPort: 2000\n"
        f"  CarlaClientPort: {client_port}\n",
        encoding="utf-8")
    return str(path)


def _traffic_only_yaml(tmp_path, name="traffic_only.yaml"):
    """The shape of the eco-driving ladder's rung 1: SUMO, and no CarlaSetup at all."""
    path = tmp_path / name
    path.write_text(
        "SimulationSetup:\n"
        "  TrafficSimulatorPort: 1337\n"
        "ApplicationSetup:\n"
        "  VehicleSubscription:\n"
        "  - type: \"ego\"\n"
        "    ip: [\"127.0.0.1\"]\n"
        "    port: [4430]\n",
        encoding="utf-8")
    return str(path)


# --------------------------------------------------------------------------- #
# the predicate
# --------------------------------------------------------------------------- #
def test_a_carla_yaml_declares_a_carla_half(tmp_path):
    assert run_cosim.scenario_has_carla(_cosim_yaml(tmp_path)) is True


def test_a_yaml_without_carlasetup_declares_none(tmp_path):
    assert run_cosim.scenario_has_carla(_traffic_only_yaml(tmp_path)) is False


def test_an_empty_carlasetup_still_declares_one(tmp_path):
    """Writing the block is the affirmative act; every key in it has a default."""
    path = tmp_path / "empty_block.yaml"
    path.write_text("SimulationSetup:\n  TrafficSimulatorPort: 1337\n"
                    "CarlaSetup:\n", encoding="utf-8")
    assert run_cosim.scenario_has_carla(str(path)) is True


def test_a_yaml_not_written_yet_is_not_a_traffic_only_one(tmp_path):
    """generate_config_yaml always emits a CarlaSetup, so 'no file' must not read
    as 'no CARLA' - that would suppress it for the run that creates the file."""
    assert run_cosim.scenario_has_carla(str(tmp_path / "not_generated.yaml")) is True


# --------------------------------------------------------------------------- #
# the bridge port: absent, not defaulted
# --------------------------------------------------------------------------- #
def test_the_bridge_port_comes_from_a_carla_yaml(tmp_path):
    traci, bridge = run_cosim.read_stack_ports(_cosim_yaml(tmp_path, client_port=4441))
    assert (traci, bridge) == (1337, 4441)


def test_a_traffic_only_yaml_has_no_bridge_port(tmp_path):
    """None, not DEFAULT_BRIDGE_PORT: defaulting it is what started a bridge on
    4440 against a file that subscribes nothing to it."""
    traci, bridge = run_cosim.read_stack_ports(_traffic_only_yaml(tmp_path))
    assert traci == 1337
    assert bridge is None


# --------------------------------------------------------------------------- #
# the setup summary
# --------------------------------------------------------------------------- #
def _args(**kw):
    ns = argparse.Namespace(sumo_only=False, engine=None,
                            carla_host=None, carla_port=None)
    for k, v in kw.items():
        setattr(ns, k, v)
    return ns


def test_the_summary_keeps_the_carla_rows_for_a_carla_yaml(tmp_path):
    derived = run_cosim.derived_from_yaml(_cosim_yaml(tmp_path), {}, _args())
    assert derived["sumo_only"] is False


def test_the_summary_drops_the_carla_rows_for_a_traffic_only_yaml(tmp_path):
    """run_profile._carla_row_hidden already keys off this, so the CARLA and engine
    rows stop describing work that will not happen."""
    derived = run_cosim.derived_from_yaml(_traffic_only_yaml(tmp_path), {}, _args())
    assert derived["sumo_only"] is True


def test_sumo_only_still_drops_them_for_a_carla_yaml(tmp_path):
    derived = run_cosim.derived_from_yaml(_cosim_yaml(tmp_path), {},
                                          _args(sumo_only=True))
    assert derived["sumo_only"] is True
