"""Manifest compatibility: schema 1 must not move, schema 2 must generalize.

The point of these tests is one promise made in #313 - a repo with a committed
apps/apps.json (FIXS_Applications has one) upgrades to a FIXS that understands
fixs.json and notices nothing at all. Not "mostly works": identical normalized
output, and zero warnings. So the fixture below is the real FIXS_Applications
manifest, not a reduced one, and the assertions cover the resolved folder as well
as the parsed fields.
"""
import io
import json
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__),
                                "..", "..", "..", "scripts", "cosim"))
import app_catalog  # noqa: E402


# The FIXS_Applications manifest as committed today, trimmed only of its long
# _comment. Every structural feature it uses is here: an app that relies on the
# id-is-the-map-name default, one with an explicit maps/configs/launch/
# requirements, per-app sumo_args including a null that drops a convention flag,
# and defaults.
LEGACY_MANIFEST = {
    "schema": 1,
    "apps": [
        {"id": "roosevelt", "title": "Roosevelt Rd (Chicago, IL) co-sim",
         "sumo_args": {"--lateral-resolution": "0.25",
                       "--collision.check-junctions": "true"},
         "defaults": {"engine": "cpp"}},
        {"id": "atlanta", "title": "Atlanta co-sim",
         "sumo_args": {"--lateral-resolution": "0.25",
                       "--collision.check-junctions": "true"},
         "defaults": {"engine": "cpp"}},
        {"id": "mlk_eco_driving", "title": "MLK arterial eco-driving",
         "requirements": "requirements.txt",
         "maps": ["mlk"],
         "launch": "run_mlk_eco_driving",
         "configs": [
             {"path": "MLK_Sumo_Scenario/config_Sumo_Carla_ecoDriving.yaml",
              "title": "SUMO + CARLA, eco-driving controller", "engine": "cpp"},
             {"path": "MLK_Sumo_Scenario/config_Sumo_Carla_dSPACE.yaml",
              "title": "SUMO + CARLA + dSPACE XIL", "engine": "cpp"},
         ],
         "sumo_args": {"--lateral-resolution": "0",
                       "--collision.check-junctions": None,
                       "--time-to-teleport": "30"},
         "defaults": {"engine": "cpp"}},
    ],
}


def _write(path, doc):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with io.open(path, "w", encoding="utf-8") as f:
        json.dump(doc, f)


def _legacy_repo(tmp_path, doc=None):
    """A repo laid out the FIXS_Applications way: apps/apps.json + apps/<id>/."""
    root = str(tmp_path / "legacy_repo")
    _write(os.path.join(root, "apps", "apps.json"), doc or LEGACY_MANIFEST)
    for app in (doc or LEGACY_MANIFEST)["apps"]:
        os.makedirs(os.path.join(root, "apps", app.get("dir") or app["id"]),
                    exist_ok=True)
    return root


# --------------------------------------------------------------------------- #
# schema 1 must not move
# --------------------------------------------------------------------------- #
def test_legacy_manifest_loads_with_no_warnings(tmp_path, capsys):
    """The promise: an untouched apps/apps.json is silent on load.

    Guards the specific regression that `declared != SCHEMA` would have caused -
    every FIXS_Applications launch printing 'schema 1, this FIXS understands 2'."""
    root = _legacy_repo(tmp_path)
    apps = app_catalog.load_catalog(root)
    assert [a["id"] for a in apps] == ["roosevelt", "atlanta", "mlk_eco_driving"]
    assert capsys.readouterr().out == ""


def test_legacy_manifest_fields_are_unchanged(tmp_path):
    """Every normalized field, spelled out - so a future edit to _normalize_app
    that changes schema-1 output fails here rather than in someone's co-sim."""
    apps = {a["id"]: a for a in app_catalog.load_catalog(_legacy_repo(tmp_path))}

    roosevelt = apps["roosevelt"]
    assert roosevelt["title"] == "Roosevelt Rd (Chicago, IL) co-sim"
    assert roosevelt["dir"] == "roosevelt"          # defaults to the id
    assert roosevelt["maps"] == ["roosevelt"]       # id doubles as the map name
    assert roosevelt["configs"] == []
    assert roosevelt["requirements"] is None
    assert roosevelt["launch"] is None
    assert roosevelt["defaults"] == {"engine": "cpp"}
    assert roosevelt["sumo_args"] == {"--lateral-resolution": "0.25",
                                      "--collision.check-junctions": "true"}

    mlk = apps["mlk_eco_driving"]
    assert mlk["maps"] == ["mlk"]                   # explicit, not the id
    assert mlk["launch"] == "run_mlk_eco_driving"
    assert mlk["requirements"] == "requirements.txt"
    assert [c["path"] for c in mlk["configs"]] == [
        "MLK_Sumo_Scenario/config_Sumo_Carla_ecoDriving.yaml",
        "MLK_Sumo_Scenario/config_Sumo_Carla_dSPACE.yaml"]
    # a null value drops a convention flag and must survive as None, not "" or absent
    assert mlk["sumo_args"]["--collision.check-junctions"] is None


def test_legacy_app_dir_is_repo_apps_id(tmp_path):
    """schema 1 keeps resolving <repo>/apps/<dir> - the rule that predates #313."""
    root = _legacy_repo(tmp_path)
    apps = {a["id"]: a for a in app_catalog.load_catalog(root)}
    assert app_catalog.app_dir(apps["mlk_eco_driving"], root) == \
        os.path.join(root, "apps", "mlk_eco_driving")


def test_legacy_manifest_without_declared_schema(tmp_path, capsys):
    """No 'schema' key in apps.json -> schema 1 by filename, still silent."""
    doc = {k: v for k, v in LEGACY_MANIFEST.items() if k != "schema"}
    root = _legacy_repo(tmp_path, doc)
    apps = {a["id"]: a for a in app_catalog.load_catalog(root)}
    assert app_catalog.app_dir(apps["roosevelt"], root) == \
        os.path.join(root, "apps", "roosevelt")
    assert capsys.readouterr().out == ""


def test_apps_json_still_found_when_no_fixs_json(tmp_path):
    root = _legacy_repo(tmp_path)
    assert app_catalog.catalog_path(root) == \
        os.path.join(root, "apps", "apps.json")


# --------------------------------------------------------------------------- #
# schema 2 generalizes
# --------------------------------------------------------------------------- #
def test_fixs_json_wins_over_apps_json(tmp_path):
    """Both present (the state a repo is in mid-migration): fixs.json is read."""
    root = _legacy_repo(tmp_path)
    _write(os.path.join(root, "fixs.json"), {"schema": 2, "apps": []})
    assert app_catalog.catalog_path(root) == os.path.join(root, "fixs.json")


def test_env_override_wins_over_both(tmp_path, monkeypatch):
    root = _legacy_repo(tmp_path)
    _write(os.path.join(root, "fixs.json"), {"schema": 2, "apps": []})
    monkeypatch.setenv("FIXS_APPS_JSON", r"D:\elsewhere\manifest.json")
    assert app_catalog.catalog_path(root) == r"D:\elsewhere\manifest.json"




def test_declared_schema_wins_over_the_filename(tmp_path):
    """schema 1 written into fixs.json still resolves the schema-1 way -
    including keeping `launch` optional - so the two forms stay independent of
    which file they are written in."""
    root = str(tmp_path / "mixed_repo")
    _write(os.path.join(root, "fixs.json"),
           {"schema": 1, "apps": [{"id": "eco", "dir": "eco"}]})
    app, = app_catalog.load_catalog(root)
    assert app_catalog.app_dir(app, root) == os.path.join(root, "apps", "eco")


def test_missing_manifest_is_not_an_error(tmp_path):
    """App awareness is an enhancement, not a precondition - a repo with no
    manifest at all runs generic, which is what a fresh integration does."""
    assert app_catalog.load_catalog(str(tmp_path / "empty_repo")) == []


def test_unknown_schema_warns_once_and_still_loads(tmp_path, capsys):
    root = str(tmp_path / "future_repo")
    _write(os.path.join(root, "fixs.json"),
           {"schema": 99, "apps": [{"launch": "a/run_eco"}]})
    apps = app_catalog.load_catalog(root)
    assert [a["id"] for a in apps] == ["eco"]        # advisory, never fatal
    out = capsys.readouterr().out
    assert "schema 99" in out and "not one this FIXS knows" in out
