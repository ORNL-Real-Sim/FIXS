"""Declaring an application: discovery, naming, scaffolding, and writing.

The wizard's job is not to save typing - four JSON fields are not a burden. It is
to hand over the FIXS_PYTHON / FIXS_CONFIG_YAML / FIXS_HANDOFF contract in the
file where it is needed, because nothing about an empty "apps": [] reveals that
contract exists. So the scaffold is tested for CONTENT, not just for existence.
"""
import io
import json
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__),
                                "..", "..", "..", "scripts", "cosim"))
import app_catalog  # noqa: E402
import app_setup  # noqa: E402


def _repo(tmp_path, launchers=(), name="r"):
    root = str(tmp_path / name)
    os.makedirs(root, exist_ok=True)
    for rel in launchers:
        p = os.path.join(root, *rel.split("/"))
        os.makedirs(os.path.dirname(p), exist_ok=True)
        io.open(p + ".bat", "w").write("")
        io.open(p + ".sh", "w").write("")
    return root


def _answers(monkeypatch, *seq):
    """Feed the wizard a fixed script of answers."""
    it = iter(seq)
    monkeypatch.setattr(app_setup, "_ask", lambda p, d="": (next(it, "") or d))


# --------------------------------------------------------------------------- #
# discovery
# --------------------------------------------------------------------------- #
def test_finds_launcher_pairs_as_one_entry(tmp_path):
    """A .bat and .sh of the same name ARE one launcher - which is why the
    manifest spells it without a suffix."""
    root = _repo(tmp_path, ["projects/autolab/run_ctrl", "tools/build"])
    assert app_setup.find_launchers(root) == ["projects/autolab/run_ctrl",
                                              "tools/build"]


def test_the_engine_bundle_is_not_searched(tmp_path):
    """FIXS/ is full of .bat files and none of them is an application."""
    root = _repo(tmp_path, ["run_mine"])
    os.makedirs(os.path.join(root, "FIXS", "Carla"))
    io.open(os.path.join(root, "FIXS", "Carla", "launch_carla.bat"), "w").write("")
    assert app_setup.find_launchers(root) == ["run_mine"]


def test_already_declared_launchers_are_not_offered(tmp_path):
    root = _repo(tmp_path, ["a/run_one", "a/run_two"])
    assert app_setup.find_launchers(root, declared=["a/run_one"]) == ["a/run_two"]


# --------------------------------------------------------------------------- #
# naming
# --------------------------------------------------------------------------- #
@pytest.mark.parametrize("launch, expected", [
    ("projects/autolab/controlA/run_ctrl", "ctrl"),
    ("a/drive.sh", "drive"),
    ("run_eco_mpc", "eco_mpc"),
])
def test_id_comes_from_the_launcher(launch, expected):
    assert app_setup.derive_id(launch, "Some Title", []) == expected


def test_id_falls_back_to_the_title_when_the_launcher_gives_none():
    assert app_setup.derive_id("a/run_", "Eco Driving MPC", []) == "eco_driving_mpc"


def test_a_colliding_id_is_suffixed_not_shared():
    """The id is a directory under ~/.fixs/apps/ - two apps sharing one would
    share the other's remembered scenario choice."""
    assert app_setup.derive_id("a/run_ctrl", "t", ["ctrl"]) == "ctrl_2"
    assert app_setup.derive_id("a/run_ctrl", "t", ["ctrl", "ctrl_2"]) == "ctrl_3"


@pytest.mark.parametrize("title, expected", [
    ("AutoLab control A", "autolab_control_a"),
    ("  Eco--Driving!! ", "eco_driving"),
    ("", ""),
])
def test_slug_produces_a_path_safe_key(title, expected):
    assert app_setup.slug(title) == expected


# --------------------------------------------------------------------------- #
# scaffolding - the part that actually matters
# --------------------------------------------------------------------------- #
def test_scaffold_writes_both_halves(tmp_path):
    root = _repo(tmp_path)
    written = app_setup.scaffold(root, "projects/autolab/run_ctrl", "AutoLab", "ctrl")
    assert len(written) == 2
    for ext in (".bat", ".sh"):
        assert os.path.isfile(os.path.join(root, "projects", "autolab",
                                           "run_ctrl" + ext))


@pytest.mark.parametrize("ext", [".bat", ".sh"])
def test_the_scaffold_documents_the_contract(tmp_path, ext):
    """Every variable an app is handed must be named in the file it is handed to."""
    root = _repo(tmp_path)
    app_setup.scaffold(root, "run_ctrl", "AutoLab", "ctrl")
    body = io.open(os.path.join(root, "run_ctrl" + ext), encoding="utf-8").read()
    for var in ("FIXS_PYTHON", "FIXS_CONFIG_YAML", "FIXS_HANDOFF"):
        assert var in body, f"{ext} scaffold never mentions {var}"
    assert "sumocfg" in body                      # how to report a scenario
    assert "Do NOT launch SUMO" in body           # the port-collision trap


def test_scaffold_never_overwrites(tmp_path):
    """Also how you add the missing half to a one-platform app."""
    root = _repo(tmp_path)
    target = os.path.join(root, "run_ctrl.sh")
    io.open(target, "w").write("mine")
    written = app_setup.scaffold(root, "run_ctrl", "AutoLab", "ctrl")
    assert io.open(target).read() == "mine"
    assert [os.path.basename(p) for p in written] == ["run_ctrl.bat"]


# --------------------------------------------------------------------------- #
# writing the manifest
# --------------------------------------------------------------------------- #
def test_entry_omits_empty_fields(tmp_path):
    e = app_setup.build_entry("a/run_x", title="", description=None, maps=None)
    assert e == {"launch": "a/run_x"}


def test_add_entry_creates_the_manifest_and_seeds_the_pin(tmp_path):
    root = _repo(tmp_path)
    m = os.path.join(root, "fixs.json")
    app_setup.add_entry(m, {"launch": "a/run_x"},
                        repo="ORNL-Real-Sim/FIXS", version="v0.10.0-alpha")
    doc = json.load(io.open(m, encoding="utf-8"))
    assert doc["schema"] == app_catalog.SCHEMA
    assert doc["fixs"] == {"repo": "ORNL-Real-Sim/FIXS", "version": "v0.10.0-alpha"}
    assert doc["apps"] == [{"launch": "a/run_x"}]


def test_add_entry_never_repins_an_existing_manifest(tmp_path):
    """The repo's declared engine is the repo's; adding an app must not move it."""
    root = _repo(tmp_path)
    m = os.path.join(root, "fixs.json")
    json.dump({"schema": 2, "fixs": {"repo": "me/fork", "version": "v0.9.0-alpha"},
               "apps": []}, io.open(m, "w", encoding="utf-8"))
    app_setup.add_entry(m, {"launch": "a/run_x"},
                        repo="ORNL-Real-Sim/FIXS", version="v0.10.0-alpha")
    doc = json.load(io.open(m, encoding="utf-8"))
    assert doc["fixs"] == {"repo": "me/fork", "version": "v0.9.0-alpha"}


def test_add_entry_refuses_a_schema_1_manifest(tmp_path):
    """schema 1's 'launch' and 'dir' mean something else; a silent mix would
    resolve app folders to places that do not exist."""
    root = _repo(tmp_path)
    m = os.path.join(root, "apps", "apps.json")
    os.makedirs(os.path.dirname(m))
    json.dump({"schema": 1, "apps": []}, io.open(m, "w", encoding="utf-8"))
    with pytest.raises(ValueError, match="schema-1"):
        app_setup.add_entry(m, {"launch": "a/run_x"})


def test_add_entry_appends_rather_than_replaces(tmp_path):
    root = _repo(tmp_path)
    m = os.path.join(root, "fixs.json")
    app_setup.add_entry(m, {"launch": "a/run_one"})
    app_setup.add_entry(m, {"launch": "a/run_two"})
    doc = json.load(io.open(m, encoding="utf-8"))
    assert [a["launch"] for a in doc["apps"]] == ["a/run_one", "a/run_two"]


# --------------------------------------------------------------------------- #
# the walkthrough, end to end
# --------------------------------------------------------------------------- #
def test_wizard_scaffolds_and_declares(tmp_path, monkeypatch, capsys):
    root = _repo(tmp_path)
    monkeypatch.setenv("FIXS_APPS_JSON", os.path.join(root, "fixs.json"))
    _answers(monkeypatch, "projects/autolab/run_ctrl", "AutoLab control A",
             "Eco-driving MPC variant", "y", "y")
    assert app_setup.run_wizard(root, map_name="RP_Ver0529",
                                repo="ORNL-Real-Sim/FIXS",
                                version="v0.10.0-alpha") == "ctrl"
    app, = app_catalog.load_catalog(root)
    assert app["id"] == "ctrl"
    assert app["title"] == "AutoLab control A"
    assert app["note"] == "Eco-driving MPC variant"
    assert app["maps"] == ["RP_Ver0529"]
    assert app_catalog.launch_command(app, root)[0] is not None   # resolvable


def test_wizard_declines_the_map_when_the_app_is_generic(tmp_path, monkeypatch):
    root = _repo(tmp_path)
    monkeypatch.setenv("FIXS_APPS_JSON", os.path.join(root, "fixs.json"))
    _answers(monkeypatch, "run_ctrl", "Ctrl", "", "n", "y")
    app_setup.run_wizard(root, map_name="RP_Ver0529")
    app, = app_catalog.load_catalog(root)
    assert app["maps"] == []          # no hint: runs on anything


def test_wizard_writes_nothing_when_declined(tmp_path, monkeypatch):
    root = _repo(tmp_path)
    monkeypatch.setenv("FIXS_APPS_JSON", os.path.join(root, "fixs.json"))
    _answers(monkeypatch, "run_ctrl", "Ctrl", "", "n", "n")
    assert app_setup.run_wizard(root) is None
    assert not os.path.exists(os.path.join(root, "fixs.json"))
    assert not os.path.exists(os.path.join(root, "run_ctrl.bat"))


def test_wizard_picks_an_existing_launcher(tmp_path, monkeypatch):
    root = _repo(tmp_path, ["projects/autolab/run_ctrl"])
    monkeypatch.setenv("FIXS_APPS_JSON", os.path.join(root, "fixs.json"))
    # no map_name -> the "only on this map?" question is skipped
    _answers(monkeypatch, "1", "AutoLab", "", "y")
    assert app_setup.run_wizard(root) == "ctrl"
    app, = app_catalog.load_catalog(root)
    assert app["launch"] == "projects/autolab/run_ctrl"


def test_wizard_warns_when_only_one_platform_half_exists(tmp_path, monkeypatch, capsys):
    root = _repo(tmp_path)
    io.open(os.path.join(root, "run_ctrl.bat"), "w").write("")
    monkeypatch.setenv("FIXS_APPS_JSON", os.path.join(root, "fixs.json"))
    _answers(monkeypatch, "1", "Ctrl", "", "y")
    app_setup.run_wizard(root)
    out = capsys.readouterr().out
    assert "only the .bat half" in out


# --------------------------------------------------------------------------- #
# things a real repo throws at it
# --------------------------------------------------------------------------- #
def test_the_front_door_is_not_offered_as_an_application(tmp_path):
    """FIXS.bat/FIXS.sh sit at the repo root beside real launchers. Offering
    '1) FIXS' is both wrong and the kind of wrong someone accepts because it is
    the only entry on the list."""
    root = _repo(tmp_path, ["run_mine"])
    io.open(os.path.join(root, "FIXS.bat"), "w").write("")
    io.open(os.path.join(root, "FIXS.sh"), "w").write("")
    assert app_setup.find_launchers(root) == ["run_mine"]


def test_a_launcher_with_a_space_is_not_offered(tmp_path):
    """`launch` is shlex-split so an app can pass itself arguments, so a space in
    the path would silently become an argument boundary."""
    root = _repo(tmp_path, ["run_ok"])
    io.open(os.path.join(root, "Demo controller.sh"), "w").write("")
    assert app_setup.find_launchers(root) == ["run_ok"]


def test_a_typed_path_at_the_pick_prompt_is_rejected_not_swallowed(
        tmp_path, monkeypatch, capsys):
    """Typing a path where a number belongs used to fall through to 'create a new
    launcher', which then asked for that same path again - reading as the wizard
    ignoring the answer."""
    root = _repo(tmp_path, ["a/run_one"])
    monkeypatch.setenv("FIXS_APPS_JSON", os.path.join(root, "fixs.json"))
    _answers(monkeypatch, "projects/demo/run_ctrl", "1", "Demo", "", "y")
    assert app_setup.run_wizard(root) == "one"
    assert "is not one of 1-1" in capsys.readouterr().out


def test_a_typed_extension_is_dropped(tmp_path, monkeypatch, capsys):
    root = _repo(tmp_path)
    monkeypatch.setenv("FIXS_APPS_JSON", os.path.join(root, "fixs.json"))
    _answers(monkeypatch, "projects/demo/run_ctrl.bat", "Demo", "", "y")
    app_setup.run_wizard(root)
    app, = app_catalog.load_catalog(root)
    assert app["launch"] == "projects/demo/run_ctrl"      # both halves, one entry
    assert "dropping the extension" in capsys.readouterr().out


def test_a_spaced_path_is_refused_then_accepted(tmp_path, monkeypatch, capsys):
    root = _repo(tmp_path)
    monkeypatch.setenv("FIXS_APPS_JSON", os.path.join(root, "fixs.json"))
    _answers(monkeypatch, "Demo controller", "projects/demo/run_ctrl", "Demo", "", "y")
    assert app_setup.run_wizard(root) == "ctrl"
    assert "no spaces" in capsys.readouterr().out
