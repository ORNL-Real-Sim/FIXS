"""fixs.config.get: the scenario yaml, read-only, by lowercase names.

    python -m pytest tests/Python/unit/test_fixs_config.py
"""
import os
import sys

import pytest
import yaml

_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
for _p in (_ROOT, os.path.join(_ROOT, 'CommonLib')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import fixs                                                    # noqa: E402

DOC = {
    "SimulationSetup": {"EnableRealSim": True, "SimulationEndTime": 100,
                        "SelectedTrafficSimulator": "SUMO",
                        "TrafficSimulatorIP": "127.0.0.1"},
    "ApplicationSetup": {
        "EnableApplicationLayer": True,
        "VehicleSubscription": [{"type": "ego", "attribute": {"all": ["true"]},
                                 "ip": ["127.0.0.1"], "port": [4430]}]},
    "XilSetup": {"EnableXil": True, "Transport": "udp",
                 "VehicleSubscription": [{"type": "ego", "attribute": {"id": ["ego"]},
                                          "ip": ["192.168.140.24", "10.0.0.1"],
                                          "port": [4420]}]},
    "EgoSetup": {"Id": "ego", "Dynamics": "virenv"},
    "CarlaSetup": {"EnableCosimulation": True, "CarlaTimeStep": 0.05},
}


@pytest.fixture
def path(tmp_path):
    p = tmp_path / "scenario.yaml"
    p.write_text(yaml.safe_dump(DOC), encoding="utf-8")
    return str(p)


def test_sections_and_keys_are_lowercase(path):
    assert fixs.config.get('xil', path)['transport'] == 'udp'
    assert fixs.config.get('xil', path)['enable_xil'] is True
    assert fixs.config.get('carla', path)['carla_time_step'] == 0.05
    assert fixs.config.get('simulation', path)['traffic_simulator_ip'] == '127.0.0.1'


def test_ip_and_port_are_the_first_subscriptions(path):
    xil = fixs.config.get('xil', path)
    assert (xil['ip'], xil['port']) == ('192.168.140.24', 4420)
    assert xil['vehicle_subscription'][0]['ip'] == ['192.168.140.24', '10.0.0.1']


def test_defaults_are_applied(tmp_path):
    p = tmp_path / "bare.yaml"
    p.write_text(yaml.safe_dump(dict(DOC, XilSetup={"EnableXil": True})), encoding="utf-8")
    assert fixs.config.get('xil', str(p))['transport'] == 'inprocess'


def test_an_unwritten_parsed_section_still_answers(tmp_path):
    p = tmp_path / "noxil.yaml"
    p.write_text(yaml.safe_dump({k: v for k, v in DOC.items() if k != 'XilSetup'}),
                 encoding="utf-8")
    assert fixs.config.get('xil', str(p))['enable_xil'] is False


def test_the_result_is_read_only_and_a_copy(path):
    xil = fixs.config.get('xil', path)
    with pytest.raises(TypeError):
        xil['transport'] = 'tcp'
    xil['vehicle_subscription'][0]['ip'][0] = 'changed'
    assert fixs.config.get('xil', path)['ip'] == '192.168.140.24'


def test_an_unknown_section_names_the_ones_there_are(path):
    with pytest.raises(fixs.FixsError) as e:
        fixs.config.get('nope', path)
    assert 'xil' in str(e.value)


def test_it_reads_the_yaml_the_run_is_using(path, monkeypatch):
    monkeypatch.setenv('FIXS_CONFIG_YAML', path)
    assert fixs.config.get('xil')['transport'] == 'udp'


def test_no_scenario_is_an_error_not_a_guess(monkeypatch):
    monkeypatch.delenv('FIXS_CONFIG_YAML', raising=False)
    with pytest.raises(fixs.FixsError):
        fixs.config.get('xil')
