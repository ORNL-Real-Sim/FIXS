"""A bundle stages its CARLA half only, and a restage clears what it replaces.

A Digital-Twin-Library map ships as `carla/` + `sumo/`. Extracted whole into
CARLA's Import/, it leaves a second descriptor at Import/carla/<name>.json
beside the one already there, CARLA's Import.py cooks every descriptor it
finds, and the repeat pass crashes Unreal. Observed on mlk_notexture_uturn;
mechanism in FIXS#358.

    python -m pytest tests/Python/unit/test_import_map_bundle_staging.py
"""
from __future__ import annotations

import json
import os
import sys
import zipfile

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "..", "Carla"))
import import_map  # noqa: E402


MAP = "mlk_notexture_uturn"


def _descriptorText():
    return json.dumps({"maps": [{"name": MAP,
                                 "xodr": f"{MAP}/{MAP}.xodr",
                                 "source": f"{MAP}/{MAP}.fbx"}],
                       "props": []})


def _bundleZip(path):
    """A release zip in the library's layout: a carla/ half and a sumo/ half."""
    with zipfile.ZipFile(path, "w") as z:
        z.writestr(f"carla/{MAP}.json", _descriptorText())
        z.writestr(f"carla/{MAP}/{MAP}.fbx", "new-fbx")
        z.writestr(f"carla/{MAP}/{MAP}.xodr", "new-xodr")
        z.writestr(f"sumo/{MAP}.net.xml", "<net/>")
        z.writestr(f"sumo/{MAP}.sumocfg", "<configuration/>")
    return path


def _stagedAlready(importDir):
    """What a previous import left behind, as the real one had it."""
    os.makedirs(os.path.join(importDir, MAP), exist_ok=True)
    with open(os.path.join(importDir, MAP + ".json"), "w") as fh:
        fh.write(_descriptorText())
    for leaf in (f"{MAP}.fbx", f"{MAP}.xodr"):
        with open(os.path.join(importDir, MAP, leaf), "w") as fh:
            fh.write("old")


def _carlaRoot(tmp_path):
    root = tmp_path / "Carla"
    (root / "Import").mkdir(parents=True)
    return str(root)


def _descriptorsUnder(importDir):
    """Every CARLA map descriptor Import.py would find, at any depth."""
    found = []
    for base, _dirs, files in os.walk(importDir):
        for f in files:
            if f.lower().endswith(".json") and f.lower() != "roadpainter_decals.json":
                found.append(os.path.relpath(os.path.join(base, f), importDir))
    return sorted(found)


def test_a_bundle_does_not_leave_two_descriptors(tmp_path, monkeypatch):
    carlaRoot = _carlaRoot(tmp_path)
    importDir = os.path.join(carlaRoot, "Import")
    _stagedAlready(importDir)
    zipPath = _bundleZip(str(tmp_path / "bundle.zip"))

    monkeypatch.setenv("FIXS_MAP_CACHE", str(tmp_path / "cache"))
    import_map.stage_package(carlaRoot, MAP, package_dir=zipPath)

    found = _descriptorsUnder(importDir)
    assert found == [MAP + ".json"], (
        "Import.py cooks every descriptor it finds; a bundle must not add a "
        "second one under carla/. Found: %s" % found)


def test_the_staged_package_is_the_new_one(tmp_path, monkeypatch):
    carlaRoot = _carlaRoot(tmp_path)
    importDir = os.path.join(carlaRoot, "Import")
    _stagedAlready(importDir)
    zipPath = _bundleZip(str(tmp_path / "bundle.zip"))

    monkeypatch.setenv("FIXS_MAP_CACHE", str(tmp_path / "cache"))
    import_map.stage_package(carlaRoot, MAP, package_dir=zipPath)

    with open(os.path.join(importDir, MAP, f"{MAP}.fbx")) as fh:
        assert fh.read() == "new-fbx", "the old staging was cooked instead"


def test_clear_staging_removes_descriptor_and_assets(tmp_path):
    carlaRoot = _carlaRoot(tmp_path)
    importDir = os.path.join(carlaRoot, "Import")
    _stagedAlready(importDir)
    assert import_map.staged_import_paths(carlaRoot, MAP)

    import_map.clear_staging(carlaRoot, MAP)

    assert import_map.staged_import_paths(carlaRoot, MAP) == []
    assert not os.path.exists(os.path.join(importDir, MAP))
    assert not os.path.exists(os.path.join(importDir, MAP + ".json"))


def test_clear_staging_on_nothing_is_not_an_error(tmp_path):
    carlaRoot = _carlaRoot(tmp_path)
    import_map.clear_staging(carlaRoot, MAP)          # must not raise


def test_a_flat_package_is_unchanged(tmp_path):
    """The legacy layout has no carla/ half and must stage exactly as before."""
    flat = tmp_path / "flat.zip"
    with zipfile.ZipFile(flat, "w") as z:
        z.writestr(f"{MAP}.json", _descriptorText())
        z.writestr(f"{MAP}/{MAP}.fbx", "flat-fbx")
        z.writestr(f"{MAP}/{MAP}.xodr", "flat-xodr")
    assert import_map._carla_half(str(flat), MAP) == str(flat)
