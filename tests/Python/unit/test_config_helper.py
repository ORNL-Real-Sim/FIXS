import pytest
import os
from CommonLib.ConfigHelper import ConfigHelper

SIMPLE_ECHO_CONFIG = os.path.join(
    os.path.dirname(__file__), '..', 'SimpleEchoClient', 'config.yaml'
)


def test_load_simple_echo_config():
    """ConfigHelper can load the SimpleEchoClient config without errors."""
    if not os.path.exists(SIMPLE_ECHO_CONFIG):
        pytest.skip("config file not found")
    ch = ConfigHelper()
    ch.getConfig(SIMPLE_ECHO_CONFIG)
    assert ch.simulation_setup is not None


def test_config_simulation_setup_keys():
    """Required SimulationSetup fields are populated after loading config."""
    if not os.path.exists(SIMPLE_ECHO_CONFIG):
        pytest.skip("config file not found")
    ch = ConfigHelper()
    ch.getConfig(SIMPLE_ECHO_CONFIG)

    assert ch.simulation_setup['SelectedTrafficSimulator'] == 'SUMO'
    assert ch.simulation_setup['TrafficSimulatorIP'] == '127.0.0.1'
    assert isinstance(ch.simulation_setup['TrafficSimulatorPort'], int)
    assert ch.simulation_setup['EnableRealSim'] is True
    assert ch.simulation_setup['EnableExternalDynamics'] is True


def test_config_vehicle_message_field():
    """VehicleMessageField is a non-empty list of strings."""
    if not os.path.exists(SIMPLE_ECHO_CONFIG):
        pytest.skip("config file not found")
    ch = ConfigHelper()
    ch.getConfig(SIMPLE_ECHO_CONFIG)

    fields = ch.simulation_setup['VehicleMessageField']
    assert isinstance(fields, list)
    assert len(fields) > 0
    for f in fields:
        assert isinstance(f, str)


def test_config_application_setup():
    """ApplicationSetup is populated with VehicleSubscription list."""
    if not os.path.exists(SIMPLE_ECHO_CONFIG):
        pytest.skip("config file not found")
    ch = ConfigHelper()
    ch.getConfig(SIMPLE_ECHO_CONFIG)

    assert ch.application_setup['EnableApplicationLayer'] is True
    subs = ch.application_setup['VehicleSubscription']
    assert isinstance(subs, list)
    assert len(subs) > 0
    first = subs[0]
    assert 'type' in first
    assert 'ip' in first
    assert 'port' in first


def test_config_defaults_for_missing_fields():
    """ConfigHelper returns sensible defaults for fields not present in yaml."""
    if not os.path.exists(SIMPLE_ECHO_CONFIG):
        pytest.skip("config file not found")
    ch = ConfigHelper()
    ch.getConfig(SIMPLE_ECHO_CONFIG)

    # Warm-up keys not in config — no warm-up (#86)
    assert ch.simulation_setup['WarmUpUntilEgoEntry'] is False
    assert ch.simulation_setup['WarmUpTime'] == 0
    # SimulationEndTime not in config — should default to 90000
    assert ch.simulation_setup['SimulationEndTime'] == 90000.0


def test_removed_simulation_mode_is_refused(tmp_path):
    """A config still setting SimulationMode is refused, not silently ignored (#86).

    Silently dropping it would leave a run that still works and is merely, and
    mysteriously, much slower — the whole warm-up would run in full sync.
    """
    cfg = tmp_path / "legacy_mode.yaml"
    cfg.write_text(
        "SimulationSetup:\n"
        "  SelectedTrafficSimulator: \"SUMO\"\n"
        "  SimulationMode: 4\n"
        "  SimulationModeParameter: 450\n"
    )
    ch = ConfigHelper()
    with pytest.raises(ValueError, match="WarmUpTime"):
        ch.getConfig(str(cfg))


def test_warmup_keys_are_parsed(tmp_path):
    """Both warm-up triggers round-trip through the parser (#86)."""
    cfg = tmp_path / "warmup.yaml"
    cfg.write_text(
        "SimulationSetup:\n"
        "  SelectedTrafficSimulator: \"SUMO\"\n"
        "  WarmUpUntilEgoEntry: true\n"
        "  WarmUpTime: 28985\n"
    )
    ch = ConfigHelper()
    ch.getConfig(str(cfg))
    assert ch.simulation_setup['WarmUpUntilEgoEntry'] is True
    assert ch.simulation_setup['WarmUpTime'] == 28985


def test_reset_config():
    """resetConfig clears all setup dicts."""
    if not os.path.exists(SIMPLE_ECHO_CONFIG):
        pytest.skip("config file not found")
    ch = ConfigHelper()
    ch.getConfig(SIMPLE_ECHO_CONFIG)
    ch.resetConfig()
    assert len(ch.simulation_setup) == 0
    assert len(ch.application_setup) == 0
