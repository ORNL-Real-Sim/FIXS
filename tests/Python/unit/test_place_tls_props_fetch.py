"""place_tls.py run on its own fetches the shared placement.yaml, and refuses the
pre-manifest placement unless --legacy-placement is given (FIXS#384).

    python -m pytest tests/Python/unit/test_place_tls_props_fetch.py
"""
from __future__ import annotations

import hashlib
import json
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "..", "Carla"))
import import_map  # noqa: E402
import place_tls  # noqa: E402
import props  # noqa: E402


MAP = "mlk_notexture_uturn"
HEAD_BP = "/Game/FIXS/Props/FIXS_TrafficLight_Head"
UASSET = b"fake-uasset-bytes"

SHARED_MANIFEST = f"""
engine: {{unreal: "4.26.2", carla: "0.9.15"}}
install_root: /Game/FIXS/Props
traffic_lights:
  blueprint: {HEAD_BP}
  group_blueprint: /Game/Carla/Static/TrafficLight/Streetlights_01/BP_TrafficLightGroup
  z_offset_cm: 500
  flip_yaw_180: false
""".encode()


def _library(reachable):
    """A stand-in for import_map.repo_file serving the map library's props/."""
    files = {
        "placement.yaml": SHARED_MANIFEST,
        "provenance.json": json.dumps(
            {"exported": {"md5": hashlib.md5(UASSET).hexdigest()}}).encode(),
        "FIXS_TrafficLight_Head.uasset": UASSET,
    }
    calls = []

    def repo_file(repo, path, ref="main"):
        calls.append(path)
        return files.get(path.rsplit("/", 1)[-1]) if reachable else None
    return repo_file, calls


@pytest.fixture
def machine(tmp_path, monkeypatch):
    """A source-build machine with the map cooked, an empty props cache, and a map
    bundle that ships no placement.yaml (as mlk does)."""
    root = tmp_path / "carla"
    props_cache = tmp_path / "props"
    map_cache = tmp_path / "maps" / MAP
    map_cache.mkdir(parents=True)
    table = tmp_path / "traffic_light_table.csv"
    table.write_text("junction_id,link_id,x,y,z,heading\n", encoding="utf-8")

    monkeypatch.setenv("FIXS_PROPS_CACHE", str(props_cache))
    monkeypatch.setattr(import_map, "map_cache_dir", lambda name: str(map_cache))
    monkeypatch.setattr(place_tls.env, "reexec_under_configured", lambda *a, **k: None)
    monkeypatch.setattr(place_tls.env, "load_config",
                        lambda: {"mode": "source", "carla_root": str(root),
                                 "ue4_root": str(tmp_path / "ue4")})
    monkeypatch.setattr(import_map, "map_is_imported", lambda carla_root, name: True)
    monkeypatch.setattr(place_tls.subprocess, "call",
                        lambda *a, **k: pytest.fail("editor launched"))
    return {"root": root, "table": table, "props_cache": props_cache,
            "map_cache": map_cache}


def _run_main(monkeypatch, table, *extra):
    monkeypatch.setattr(sys, "argv", ["place_tls.py", "--map", MAP,
                                      "--tl-table", str(table), *extra])
    return place_tls.main()


def test_standalone_fetches_the_shared_manifest(machine, monkeypatch):
    """After a clean props cache, a direct run places from the library's manifest
    (FIXS head at +500), not the stock pole at +300."""
    repo_file, calls = _library(reachable=True)
    monkeypatch.setattr(import_map, "repo_file", repo_file)
    seen = {}

    def fake_place(name, tl_table, *a, bundle_dirs=(), **k):
        seen["manifest"] = props.find_manifest(*bundle_dirs)
        return 0
    monkeypatch.setattr(place_tls, "place_tls", fake_place)

    assert _run_main(monkeypatch, machine["table"]) == 0
    assert any(p.endswith("placement.yaml") for p in calls), "props were never fetched"
    assert seen["manifest"], "no placement.yaml reachable from the bundle dirs"
    settings = props.tl_settings(props.load(seen["manifest"]), seen["manifest"])
    assert settings["blueprint"] == HEAD_BP
    assert settings["z_offset_cm"] == 500


def test_no_manifest_anywhere_refuses(machine, monkeypatch, capsys):
    """Library unreachable and nothing cached: refuse, naming the flag."""
    repo_file, _ = _library(reachable=False)
    monkeypatch.setattr(import_map, "repo_file", repo_file)
    with pytest.raises(SystemExit) as exc:
        _run_main(monkeypatch, machine["table"])
    assert "--legacy-placement" in str(exc.value)
    assert props.LEGACY_TL_BLUEPRINT not in capsys.readouterr().out


def test_legacy_placement_flag_falls_back(machine, monkeypatch, capsys):
    """With --legacy-placement the pre-manifest numbers are used, as before."""
    repo_file, _ = _library(reachable=False)
    monkeypatch.setattr(import_map, "repo_file", repo_file)
    marker = place_tls.tls_marker(str(machine["root"]), MAP)
    os.makedirs(os.path.dirname(marker), exist_ok=True)
    open(marker, "w").close()

    assert _run_main(monkeypatch, machine["table"], "--legacy-placement") == 0
    out = capsys.readouterr().out
    assert props.LEGACY_TL_BLUEPRINT in out
    assert "already placed" in out


def test_resolve_props_refuses_by_default(tmp_path):
    with pytest.raises(props.ManifestError, match="--legacy-placement"):
        place_tls.resolve_props(str(tmp_path), None, bundle_dirs=[str(tmp_path)])
    settings, manifest_path, fingerprint = place_tls.resolve_props(
        str(tmp_path), None, bundle_dirs=[str(tmp_path)], allow_legacy=True)
    assert settings["blueprint"] == props.LEGACY_TL_BLUEPRINT
    assert settings["z_offset_cm"] == props.LEGACY_TL_Z_OFFSET_CM
    assert manifest_path is None and fingerprint is None
