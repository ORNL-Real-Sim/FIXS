"""The engine flags a front end drives run_cosim with (#78).

A GUI must not reimplement the engine, so the engine answers as data instead:
--list apps|maps|setups and --json on --list/--doctor/--version, --setup with every
answer given (--carla-mode ...), --stop-file, and --app-args. What these pin down:

  * --json output is ONE parseable document on stdout, even when the readers print
    notes on the way (those go to stderr) - a front end parses stdout blindly.
  * listings are reads: they create nothing on disk.
  * a non-interactive setup refuses what the wizard refuses, and never prompts.
"""
import io
import json
import os
import sys
from contextlib import redirect_stdout

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__),
                                "..", "..", "..", "scripts", "cosim"))
import doctor  # noqa: E402
import env_setup  # noqa: E402
import import_map  # noqa: E402
import run_cosim  # noqa: E402
import run_profile  # noqa: E402


@pytest.fixture
def home(tmp_path, monkeypatch):
    """A private ~/.fixs: carla.json, run_profiles.json, apps/ and maps/ all live
    under tmp_path, so nothing here reads or writes the real one."""
    fixs = tmp_path / ".fixs"
    fixs.mkdir()
    monkeypatch.setattr(env_setup, "CONFIG_DIR", str(fixs))
    monkeypatch.setattr(env_setup, "CONFIG_PATH", str(fixs / "carla.json"))
    monkeypatch.delenv("FIXS_MAP_CACHE", raising=False)
    monkeypatch.delenv("FIXS_APPS_JSON", raising=False)
    return fixs


def _json_stdout(fn, *a, **kw):
    buf = io.StringIO()
    with redirect_stdout(buf):
        rc = fn(*a, **kw)
    return rc, json.loads(buf.getvalue())


# --------------------------------------------------------------------------- #
# --list setups
# --------------------------------------------------------------------------- #
def test_list_setups_is_one_json_document_even_when_the_store_talks(home, capsys):
    """An old store with step_length makes load_doc print an upgrade note. That
    note must not land in the middle of the JSON a front end is parsing."""
    (home / "run_profiles.json").write_text(json.dumps({
        "schema": 1, "last": "b",
        "setups": {"a": {"app": "x", "map": "m1", "updated": "2026-01-01T00:00:00"},
                   "b": {"app": None, "map": "m2", "sumo_gui": False,
                         "step_length": 0.05, "updated": "2026-01-02T00:00:00"}}}),
        encoding="utf-8")
    rc = run_cosim.print_listing("setups", as_json=True)
    out, err = capsys.readouterr()
    data = json.loads(out)
    assert rc == 0
    assert data["last"] == "b"
    assert [s["name"] for s in data["setups"]] == ["b", "a"]      # last first
    assert data["setups"][0]["sumo_gui"] is False
    assert "headless" in data["setups"][0]["summary"]
    assert "step_length" in err                                    # the note, on stderr


def test_list_setups_with_no_store_is_empty_not_an_error(home):
    rc, data = _json_stdout(run_cosim.print_listing, "setups", as_json=True)
    assert rc == 0 and data["setups"] == [] and data["last"] is None


# --------------------------------------------------------------------------- #
# --list apps
# --------------------------------------------------------------------------- #
def test_list_apps_names_the_staged_path_without_staging(home, tmp_path, monkeypatch):
    """--config must name the machine-local copy run_cosim stages a yaml to, so the
    listing says where that is - but only a run may create it."""
    repo = tmp_path / "repo"
    (repo / "ctrl").mkdir(parents=True)
    (repo / "ctrl" / "run_ctrl.py").write_text("")
    (repo / "ctrl" / "scen.yaml").write_text("SimulationSetup: {}\n")
    (repo / "fixs.json").write_text(json.dumps({
        "schema": 2,
        "apps": [{"id": "ctrl", "title": "Controller", "launch": "ctrl/run_ctrl.py",
                  "maps": ["mlk"],
                  "configs": [{"path": "ctrl/scen.yaml", "title": "the scenario",
                               "engine": "py"}]}]}), encoding="utf-8")
    monkeypatch.setenv("FIXS_APPS_JSON", str(repo / "fixs.json"))
    rc, data = _json_stdout(run_cosim.print_listing, "apps", as_json=True)
    app = data["apps"][0]
    assert rc == 0 and app["id"] == "ctrl" and app["maps"] == ["mlk"]
    cfg = app["configs"][0]
    assert cfg["title"] == "the scenario" and cfg["engine"] == "py"
    assert cfg["staged"] == str(home / "apps" / "ctrl" / "scen.yaml")
    assert not (home / "apps").exists()                            # nothing staged


# --------------------------------------------------------------------------- #
# --list maps
# --------------------------------------------------------------------------- #
def _cook(carla_root, name):
    umap = carla_root / "Unreal" / "CarlaUE4" / "Content" / name / "Maps" / name
    umap.mkdir(parents=True)
    (umap / f"{name}.umap").write_text("")


def test_list_local_maps_reads_cooked_and_cached_and_creates_nothing(tmp_path, home):
    carla = tmp_path / "carla"
    _cook(carla, "town_a")
    _cook(carla, "Carla")                   # CARLA's own content: never a map
    (carla / "Unreal" / "CarlaUE4" / "Content" / "half").mkdir()   # no .umap
    maps = home / "maps"
    (maps / "town_a").mkdir(parents=True)
    (maps / "town_a" / "bundle.zip").write_text("x")
    (maps / "only_cached").mkdir()
    (maps / "only_cached" / "sumo").mkdir()
    (maps / "empty").mkdir()                # nothing to run from
    got = import_map.list_local_maps(str(carla), "source")
    assert got == [{"name": "only_cached", "cooked": False, "cached": True},
                   {"name": "town_a", "cooked": True, "cached": True}]


def test_list_local_maps_does_not_create_the_cache(tmp_path, home):
    assert import_map.list_local_maps(None, None) == []
    assert not (home / "maps").exists()


def test_list_maps_merges_the_library_by_cooked_name(home):
    (home / "maps" / "mlk_no_signal").mkdir(parents=True)
    (home / "maps" / "mlk_no_signal" / "sumo").mkdir()
    (home / "catalog.json").write_text(json.dumps({"maps": [
        {"location": "mlk", "title": "MLK Blvd", "map_name": "mlk_no_signal"},
        {"location": "i24", "title": "I-24"}]}), encoding="utf-8")
    rc, data = _json_stdout(run_cosim.print_listing, "maps", as_json=True)
    by = {m["name"]: m for m in data["maps"]}
    assert by["mlk_no_signal"]["cached"] and by["mlk_no_signal"]["library"]
    assert by["mlk_no_signal"]["location"] == "mlk"
    assert by["i24"] == {"name": "i24", "cooked": False, "cached": False,
                         "library": True, "title": "I-24", "location": "i24"}


# --------------------------------------------------------------------------- #
# --doctor --json, --version --json
# --------------------------------------------------------------------------- #
def test_doctor_report_as_data():
    rep = doctor.Report()
    rep.add("FIXS", "bundle", doctor.OK, "fine")
    rep.add("CARLA", "reachable", doctor.WARN, "not up")
    assert rep.worst == doctor.WARN and not rep.failed
    d = rep.as_dict()
    assert d["worst"] == "WARN"
    assert d["rows"][1] == {"section": "CARLA", "label": "reachable",
                            "status": "WARN", "detail": "not up"}
    rep.add("SUMO", "binary", doctor.FAIL, "missing")
    assert rep.worst == doctor.FAIL


def test_doctor_json_keeps_check_chatter_off_stdout(monkeypatch, capsys):
    def fake_collect(*_a, **_kw):
        print("a check that talks")
        rep = doctor.Report()
        rep.add("FIXS", "bundle", doctor.FAIL, "missing")
        return rep
    monkeypatch.setattr(doctor, "_collect", fake_collect)
    rc = doctor.run({}, env_setup, "root", "maps", "localhost", 2000, "v",
                    role="traffic", as_json=True)
    out, err = capsys.readouterr()
    data = json.loads(out)
    assert rc == 1 and data["worst"] == "FAIL" and data["role"] == "traffic"
    assert "a check that talks" in err


def test_fingerprint_is_data_and_print_renders_it(monkeypatch, capsys):
    monkeypatch.setattr(run_cosim.subprocess, "call", lambda *a, **k: 0)
    fp = run_cosim.fingerprint({"mode": "source", "carla_root": "C:/carla",
                                "ue4_root": "C:/ue4", "python": sys.executable},
                               "localhost", 2000)
    assert fp["carla_mode"] == "source" and fp["ue4_root"] == "C:/ue4"
    assert fp["modules"] == {m: True for m in
                             ("carla", "traci", "yaml", "pandas", "shapely")}
    json.dumps(fp)                                                 # serializable
    run_cosim.print_fingerprint({"mode": "source"}, "localhost", 2000)
    assert "mode source, endpoint localhost:2000" in capsys.readouterr().out


# --------------------------------------------------------------------------- #
# --setup --carla-mode: no prompts
# --------------------------------------------------------------------------- #
def test_setup_refuses_an_unknown_mode(home):
    with pytest.raises(SystemExit):
        env_setup.setup_from_args("remote")


def test_setup_packaged_refuses_a_folder_without_carla(home, tmp_path):
    with pytest.raises(SystemExit, match="no CarlaUE4 launcher"):
        env_setup.setup_from_args("packaged", carla_root=str(tmp_path))
    assert not os.path.exists(env_setup.CONFIG_PATH)


def test_setup_source_needs_both_roots(home, tmp_path, monkeypatch):
    monkeypatch.delenv("UE4_ROOT", raising=False)
    with pytest.raises(SystemExit, match="--ue4-root"):
        env_setup.setup_from_args("source", carla_root=str(tmp_path))


def test_setup_client_writes_carla_json_without_asking(home, monkeypatch):
    monkeypatch.setattr(env_setup, "ensure_carla", lambda *a, **k: None)
    monkeypatch.setattr(env_setup, "_warn_if_incomplete", lambda py: None)
    monkeypatch.setattr("builtins.input", lambda *a: pytest.fail("prompted"))
    assert env_setup.setup_from_args("client", python=sys.executable) == 0
    with open(env_setup.CONFIG_PATH, encoding="utf-8") as f:
        cfg = json.load(f)
    assert cfg == {"mode": "client", "python": sys.executable}


def test_setup_refuses_a_python_that_is_not_there(home, tmp_path):
    with pytest.raises(SystemExit, match="no python at"):
        env_setup.setup_from_args("client", python=str(tmp_path / "python.exe"))


# --------------------------------------------------------------------------- #
# --app-args
# --------------------------------------------------------------------------- #
def test_app_args_reach_the_environment(home, monkeypatch, capsys):
    """The contract of FIXS_Applications' run_cosim.bat/.sh: an app reads its own
    arguments from COSIM_APP_ARGS, which start_app's copy of os.environ carries."""
    monkeypatch.delenv("COSIM_APP_ARGS", raising=False)
    monkeypatch.setattr(sys, "argv", ["run_cosim.py", "--app-args",
                                      "--penetrationRate 0.2", "--list", "setups",
                                      "--json"])
    assert run_cosim.main() == 0
    assert os.environ["COSIM_APP_ARGS"] == "--penetrationRate 0.2"
    json.loads(capsys.readouterr().out)
    monkeypatch.delenv("COSIM_APP_ARGS")


# --------------------------------------------------------------------------- #
# the ego controller preflight
# --------------------------------------------------------------------------- #
class _Parsed:
    def __init__(self, **ego):
        self.Ego_setup = ego


@pytest.mark.parametrize("spec", [
    "apps/app/ctrl.py",
    "apps/app/ctrl.py --command-shape pedals",       # the options tail is not the path
    "apps/app/ctrl.py:MyController",                 # nor is the attribute
])
def test_a_missing_controller_stops_the_run_before_anything_starts(
        tmp_path, monkeypatch, spec):
    """Reported from the window: a yaml staged from another branch of the app repo
    named a controller this checkout does not have. The bridge found out only
    after SUMO, TrafficLayer, the app and CARLA were up, and the stack blamed
    CARLA. The engine now refuses first, resolving exactly as the bridge does."""
    monkeypatch.setattr(run_cosim, "_read_scenario_config",
                        lambda _y: _Parsed(ActuationSource="user", Controller=spec))
    with pytest.raises(SystemExit, match="ego controller is not here"):
        run_cosim.check_ego_controller("scen.yaml", cwd=str(tmp_path))
    (tmp_path / "apps" / "app").mkdir(parents=True)
    (tmp_path / "apps" / "app" / "ctrl.py").write_text("")
    run_cosim.check_ego_controller("scen.yaml", cwd=str(tmp_path))       # present: fine


@pytest.mark.parametrize("ego", [
    {"ActuationSource": "simulator", "Controller": "apps/app/gone.py"},  # not loaded
    {"ActuationSource": "user", "Controller": "mypkg.controllers:Eco"},  # imported, not opened
    {"ActuationSource": "user", "Controller": ""},                       # wire actuation
])
def test_controllers_the_bridge_would_not_open_are_left_alone(tmp_path, monkeypatch, ego):
    monkeypatch.setattr(run_cosim, "_read_scenario_config", lambda _y: _Parsed(**ego))
    run_cosim.check_ego_controller("scen.yaml", cwd=str(tmp_path))


# --------------------------------------------------------------------------- #
# self-update goes through whichever front door the repo has
# --------------------------------------------------------------------------- #
@pytest.mark.parametrize("present,expected", [
    (["FIXS.bat", "FIXS.sh", "run_cosim.bat", "run_cosim.sh"], "FIXS"),   # new wins
    (["run_cosim.bat", "run_cosim.sh"], "run_cosim"),                     # legacy repo
])
def test_self_update_uses_the_front_door_the_repo_has(tmp_path, monkeypatch,
                                                      present, expected):
    for name in present:
        (tmp_path / name).write_text("")
    monkeypatch.setattr(run_cosim, "APP_ROOT", str(tmp_path))
    calls = []
    monkeypatch.setattr(run_cosim.subprocess, "call",
                        lambda cmd, *a, **k: calls.append(cmd) or 0)
    assert run_cosim._run_initialize("v0.10.0") is True
    assert os.path.basename(calls[0][-3]).startswith(expected)
    assert calls[0][-2:] == ["--update-fixs", "v0.10.0"]


def test_self_update_says_which_front_doors_it_looked_for(tmp_path, monkeypatch, capsys):
    monkeypatch.setattr(run_cosim, "APP_ROOT", str(tmp_path))
    assert run_cosim._run_initialize("v0.10.0") is False
    assert "FIXS." in capsys.readouterr().out
