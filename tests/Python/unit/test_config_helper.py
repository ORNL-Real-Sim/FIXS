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

    # SimulationMode not in config — should default to 0
    assert ch.simulation_setup['SimulationMode'] == 0
    # SimulationEndTime not in config — should default to 90000
    assert ch.simulation_setup['SimulationEndTime'] == 90000.0


def test_reset_config():
    """resetConfig clears all setup dicts."""
    if not os.path.exists(SIMPLE_ECHO_CONFIG):
        pytest.skip("config file not found")
    ch = ConfigHelper()
    ch.getConfig(SIMPLE_ECHO_CONFIG)
    ch.resetConfig()
    assert len(ch.simulation_setup) == 0
    assert len(ch.application_setup) == 0
