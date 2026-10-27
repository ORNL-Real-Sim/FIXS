"""An app can report a config overlay with its scenario; run_cosim runs the stack on a merged copy."""
import json
import os
import sys

import yaml

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "..", "Carla"))
import app_catalog  # noqa: E402
import run_cosim  # noqa: E402
import CommonLib.fixs as fixs  # noqa: E402
from CommonLib.ConfigHelper import ConfigHelper  # noqa: E402
from CommonLib.fixs.sumo import build  # noqa: E402

BASE = """# a comment
SimulationSetup:
  SimulationEndTime: 29900
CarlaSetup:
  RealtimePacing: false
  EnableSpectatorFollow: true
  InterestedIds: ["ego"]
"""
OVERLAY = {"CarlaSetup": {"RealtimePacing": True, "SpectatorHeight": 75},
           "ViewSetup": {"Views": [{"Name": "chase", "Type": "chase", "Target": "ego"}]}}


def test_merge_keeps_unnamed_keys_and_replaces_lists():
    out = run_cosim._merged({"A": {"x": 1, "y": [1, 2]}, "B": 2}, {"A": {"y": [3]}, "C": 4})
    assert out == {"A": {"x": 1, "y": [3]}, "B": 2, "C": 4}


def test_run_config_is_a_merged_copy_the_config_parser_reads(tmp_path, monkeypatch):
    monkeypatch.setattr(app_catalog, "app_root", lambda: str(tmp_path))
    src = tmp_path / "config_demo.yaml"
    src.write_text(BASE, encoding="utf-8")
    out = run_cosim.write_run_config(str(src), OVERLAY, "demo")
    assert os.path.basename(out) == "config_demo.yaml" and out != str(src)
    assert src.read_text(encoding="utf-8") == BASE
    doc = yaml.safe_load(open(out, encoding="utf-8"))
    assert doc["CarlaSetup"] == {"RealtimePacing": True, "EnableSpectatorFollow": True,
                                 "InterestedIds": ["ego"], "SpectatorHeight": 75}
    assert doc["SimulationSetup"]["SimulationEndTime"] == 29900
    h = ConfigHelper()
    h.getConfig(out)
    assert h.Carla_setup["RealtimePacing"] is True
    assert h.Carla_setup["SpectatorHeight"] == 75.0


def test_lists_of_scalars_stay_inline_for_the_line_based_checks(tmp_path, monkeypatch):
    monkeypatch.setattr(app_catalog, "app_root", lambda: str(tmp_path))
    src = tmp_path / "config_demo.yaml"
    src.write_text(BASE + "  CarlaClientPort: 4440\nVehicleSubscription:\n  - type: ego\n    port: [4440]\n",
                   encoding="utf-8")
    text = open(run_cosim.write_run_config(str(src), OVERLAY, "demo"), encoding="utf-8").read()
    assert "port: [4440]" in text and "InterestedIds: [ego]" in text


def test_in_a_run_the_copy_is_in_the_run_dir_and_the_run_record_names_it(tmp_path, monkeypatch):
    import hashlib
    from types import SimpleNamespace
    run_dir = tmp_path / "run"
    run_dir.mkdir()
    monkeypatch.setattr(run_cosim, "_RUN", SimpleNamespace(dir=str(run_dir), id="demo-1", lease={}))
    src = tmp_path / "config_demo.yaml"
    src.write_text(BASE, encoding="utf-8")
    out = run_cosim.write_run_config(str(src), OVERLAY, "demo")
    assert os.path.dirname(out) == str(run_dir / "overlay")
    run_cosim._record_run(str(src), out)
    rec = json.loads((run_dir / run_cosim.RUN_RECORD).read_text(encoding="utf-8"))
    assert rec["user_yaml"] == str(src) and rec["stack_yaml"] == out
    assert rec["stack_yaml_sha256"] == hashlib.sha256(open(out, "rb").read()).hexdigest()
    assert (run_dir / run_cosim.STACK_YAML_COPY).read_text(encoding="utf-8") == open(out, encoding="utf-8").read()


def test_overlay_rides_on_the_scenario_report(tmp_path, monkeypatch):
    handoff = tmp_path / "handoff.json"
    monkeypatch.setenv("FIXS_HANDOFF", str(handoff))
    monkeypatch.setattr(fixs, "_handoffExtra", {})
    fixs.launch.overlayConfig(OVERLAY)
    cfg = tmp_path / "s.sumocfg"
    cfg.write_text("<configuration/>", encoding="utf-8")
    build._report(cfg)
    answer = json.loads(handoff.read_text(encoding="utf-8"))
    assert answer == {"sumocfg": str(cfg.resolve()), "configOverlay": OVERLAY}


def test_overlay_calls_merge(tmp_path, monkeypatch):
    handoff = tmp_path / "handoff.json"
    monkeypatch.setenv("FIXS_HANDOFF", str(handoff))
    monkeypatch.setattr(fixs, "_handoffExtra", {})
    fixs.launch.overlayConfig({"SimulationSetup": {"SimulationEndTime": 29500}})
    fixs.launch.overlayConfig(OVERLAY)
    fixs.launch.overlayConfig({"CarlaSetup": {"SpectatorHeight": 60}})
    cfg = tmp_path / "s.sumocfg"
    cfg.write_text("<configuration/>", encoding="utf-8")
    build._report(cfg)
    got = json.loads(handoff.read_text(encoding="utf-8"))["configOverlay"]
    assert got["SimulationSetup"] == {"SimulationEndTime": 29500}
    assert got["CarlaSetup"] == {"RealtimePacing": True, "SpectatorHeight": 60}
    assert got["ViewSetup"] == OVERLAY["ViewSetup"]


def test_no_overlay_reports_the_scenario_only(tmp_path, monkeypatch):
    handoff = tmp_path / "handoff.json"
    monkeypatch.setenv("FIXS_HANDOFF", str(handoff))
    monkeypatch.setattr(fixs, "_handoffExtra", {})
    cfg = tmp_path / "s.sumocfg"
    cfg.write_text("<configuration/>", encoding="utf-8")
    build._report(cfg)
    assert json.loads(handoff.read_text(encoding="utf-8")) == {"sumocfg": str(cfg.resolve())}


CONTROLLER = """import json, os
tmp = os.environ["FIXS_HANDOFF"] + ".tmp"
open(tmp, "w").write(json.dumps({"configOverlay": json.loads(%r)}))
os.replace(tmp, os.environ["FIXS_HANDOFF"])
"""


def test_start_app_returns_the_reported_overlay(tmp_path, monkeypatch):
    app_dir = tmp_path / "apps" / "demo"
    app_dir.mkdir(parents=True)
    (app_dir / "ctl.py").write_text(CONTROLLER % json.dumps(OVERLAY), encoding="utf-8")
    monkeypatch.setattr(app_catalog, "app_root", lambda: str(tmp_path))
    proc, sumocfg, overlay = run_cosim.start_app({"id": "demo", "dir": "demo", "launch": "ctl.py"},
                                                 timeout=30)
    proc.wait(timeout=30)
    assert sumocfg is None
    assert overlay == OVERLAY
