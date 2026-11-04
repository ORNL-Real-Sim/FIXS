"""ConfigHelper.cpp resolves EgoSetup from the legacy per-backend keys as ConfigHelper.py does.

The C++ parse reads CarMakerSetup and CarlaSetup a second time, after every
section has been parsed, so a section clobbered on the way shows up here.

    python -m pytest tests/VirEnv/test_ego_setup_parity.py
"""

import glob
import os
import subprocess
import sys

import pytest
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.abspath(os.path.join(HERE, '..', '..'))
sys.path.insert(0, REPO_ROOT)
sys.path.insert(0, HERE)
import cpp_build                                                    # noqa: E402
from CommonLib.ConfigHelper import ConfigHelper                     # noqa: E402

PROBE_CONFIGS = sorted(glob.glob(os.path.join(
    REPO_ROOT, 'tests', 'Sumo', 'Probes', 'TrafficLayer_SUMO_Carla', 'config_*.yaml')))


def _runCpp(path):
    return subprocess.run([cpp_build.ensureBuilt()['ego_dump'], str(path)],
                          capture_output=True, text=True, timeout=30)


def _cppEgo(path):
    proc = _runCpp(path)
    assert proc.returncode == 0, proc.stdout + proc.stderr
    line = [ln for ln in proc.stdout.splitlines() if ln.startswith('EGO|')]
    assert len(line) == 1, proc.stdout
    keys = ('Id', 'Type', 'Dynamics', 'ActuationSource', 'Controller')
    return dict(zip(keys, line[0].split('|')[1:]))


@pytest.mark.parametrize("path", PROBE_CONFIGS, ids=os.path.basename)
def test_legacy_carla_ego_keys_resolve_the_same(path):
    py = ConfigHelper()
    try:
        py.getConfig(path)
    except SystemExit as refused:
        proc = _runCpp(path)
        assert proc.returncode != 0 and 'ERROR: EgoSetup' in proc.stdout, (str(refused), proc.stdout)
        return
    cpp = _cppEgo(path)
    for k in ('Dynamics', 'ActuationSource', 'Controller'):
        assert cpp[k] == (py.Ego_setup[k] or ''), k


def test_legacy_carmaker_ego_id_survives_later_sections(tmp_path):
    doc = {
        "SimulationSetup": {"SelectedTrafficSimulator": "SUMO",
                            "VehicleMessageField": ["id", "speed", "speedDesired"]},
        "CarMakerSetup": {"EgoId": "cm_ego", "EgoType": "cm_type"},
        "SumoSetup": {"SpeedMode": 0},
        "CarlaSetup": {"EnableCosimulation": False},
    }
    p = tmp_path / 'cm_legacy.yaml'
    p.write_text(yaml.safe_dump(doc, sort_keys=False), encoding='utf-8')
    cpp = _cppEgo(p)
    assert (cpp['Id'], cpp['Type']) == ('cm_ego', 'cm_type')
