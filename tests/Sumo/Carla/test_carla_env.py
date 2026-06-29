"""
Tier-1 tests for the CARLA env setup/config layer (carla_env_setup.py) and the
launch-command resolution in run_cosim.py.

These are pure-stdlib: NO CARLA server, NO GPU, NO map asset, and NO interactive
prompt or GUI (the folder picker is never invoked here). They use a temp config
path and fake CARLA/UE4 trees, so they run on any computer. Run with:

    pytest test_carla_env.py
"""
import os
import platform
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
# the co-sim runtime lives at the repo root: FIXS_root/Carla
CARLA = os.path.normpath(os.path.join(HERE, "..", "..", "..", "Carla"))
sys.path.insert(0, CARLA)

import carla_env_setup as env  # noqa: E402
import run_cosim  # noqa: E402  (imports carla_env_setup; does NOT import the carla wheel)

WIN = platform.system() == "Windows"


# ---------------------------------------------------------------- config io

def test_config_roundtrip(tmp_path, monkeypatch):
    """save_config then load_config returns the same dict."""
    cfg_path = tmp_path / ".fixs" / "carla.json"
    monkeypatch.setattr(env, "CONFIG_DIR", str(cfg_path.parent))
    monkeypatch.setattr(env, "CONFIG_PATH", str(cfg_path))

    cfg = {"mode": "packaged", "carla_root": str(tmp_path / "carla")}
    env.save_config(cfg)
    assert cfg_path.is_file()
    assert env.load_config() == cfg


def test_load_config_missing(tmp_path, monkeypatch):
    """No file -> None (the first-run signal run_cosim keys off)."""
    monkeypatch.setattr(env, "CONFIG_PATH", str(tmp_path / "nope.json"))
    assert env.load_config() is None


def test_load_config_rejects_garbage(tmp_path, monkeypatch):
    """A malformed / incomplete config is treated as 'not configured'."""
    bad = tmp_path / "carla.json"
    bad.write_text('{"mode": "weird"}', encoding="utf-8")
    monkeypatch.setattr(env, "CONFIG_PATH", str(bad))
    assert env.load_config() is None


# --------------------------------------------------------- path resolving

def _make_packaged(root):
    """Create a fake packaged CARLA tree for this OS; return the launcher path."""
    name = "CarlaUE4.exe" if WIN else "CarlaUE4.sh"
    os.makedirs(root, exist_ok=True)
    exe = os.path.join(root, name)
    open(exe, "w").close()
    return exe


def test_packaged_exe_found(tmp_path):
    root = str(tmp_path / "carla")
    exe = _make_packaged(root)
    assert env.packaged_exe(root) == exe


def test_packaged_exe_missing(tmp_path):
    assert env.packaged_exe(str(tmp_path / "empty")) is None


def test_source_paths_shape(tmp_path):
    uproject, editor = env.source_paths(str(tmp_path / "carla"), str(tmp_path / "ue4"))
    assert uproject.endswith(os.path.join("Unreal", "CarlaUE4", "CarlaUE4.uproject"))
    assert editor.endswith("UE4Editor.exe" if WIN else "UE4Editor")


# --------------------------------------------- run_cosim launch resolution

def test_carla_command_packaged(tmp_path):
    """Packaged mode resolves to the launcher + rpc-port flag."""
    root = str(tmp_path / "carla")
    exe = _make_packaged(root)
    cfg = {"mode": "packaged", "carla_root": root}
    cmd = run_cosim._carla_command(cfg, 2000, render_offscreen=False)
    assert cmd[0] == exe
    assert "-carla-rpc-port=2000" in cmd


def test_carla_command_offscreen_flag(tmp_path):
    root = str(tmp_path / "carla")
    _make_packaged(root)
    cfg = {"mode": "packaged", "carla_root": root}
    cmd = run_cosim._carla_command(cfg, 2000, render_offscreen=True)
    assert "-RenderOffScreen" in cmd


def test_carla_command_source(tmp_path):
    """Source mode resolves to UE4Editor <uproject> -game."""
    carla_root = tmp_path / "carla"
    ue4_root = tmp_path / "ue4"
    uproject, editor = env.source_paths(str(carla_root), str(ue4_root))
    os.makedirs(os.path.dirname(uproject), exist_ok=True)
    os.makedirs(os.path.dirname(editor), exist_ok=True)
    open(uproject, "w").close()
    open(editor, "w").close()

    cfg = {"mode": "source", "carla_root": str(carla_root), "ue4_root": str(ue4_root)}
    cmd = run_cosim._carla_command(cfg, 2000, render_offscreen=False)
    assert cmd[0] == editor
    assert cmd[1] == uproject
    assert "-game" in cmd


def test_carla_command_packaged_missing_raises(tmp_path):
    cfg = {"mode": "packaged", "carla_root": str(tmp_path / "empty")}
    with pytest.raises(FileNotFoundError):
        run_cosim._carla_command(cfg, 2000, render_offscreen=False)


# ------------------------------------- generic python / wheel resolution

def test_python_can_import_self():
    """The running interpreter can import os (sanity of the subprocess probe)."""
    assert env._python_can_import(sys.executable, ("os", "sys"))
    assert not env._python_can_import(sys.executable, ("a_module_that_does_not_exist_xyz",))


def test_conda_candidates_includes_current():
    """Candidate discovery always includes the current interpreter, all real."""
    cands = env._conda_candidates()
    assert os.path.normcase(sys.executable) in {os.path.normcase(c) for c in cands}
    assert all(os.path.isfile(c) for c in cands)


def test_find_source_wheel_prefers_tag(tmp_path):
    """Wheel auto-resolution picks one matching the interpreter's cpXY tag."""
    dist = tmp_path / "PythonAPI" / "carla" / "dist"
    dist.mkdir(parents=True)
    tag = env._python_tag(sys.executable)  # e.g. cp310
    (dist / f"carla-0.9.15-{tag}-{tag}-win_amd64.whl").write_text("", encoding="utf-8")
    (dist / "carla-0.9.15-cp38-cp38-win_amd64.whl").write_text("", encoding="utf-8")
    picked = env.find_source_wheel(str(tmp_path), sys.executable)
    assert picked is not None and tag in os.path.basename(picked)


def test_find_source_wheel_absent(tmp_path):
    assert env.find_source_wheel(str(tmp_path / "nope"), sys.executable) is None


def test_maybe_reexec_noop_same_interpreter():
    """No re-exec when already on the configured python, or when it's missing /
    unset (must simply return, never SystemExit)."""
    run_cosim.maybe_reexec({"python": sys.executable})
    run_cosim.maybe_reexec({"python": os.path.join(os.sep, "no", "such", "python")})
    run_cosim.maybe_reexec({})


def test_ensure_runtime_noop_when_python_valid(monkeypatch):
    """A config whose python can import carla is returned unchanged - no setup."""
    monkeypatch.setattr(env, "_python_can_import", lambda py, mods: True)
    called = {"resolve": False}
    monkeypatch.setattr(env, "resolve_python",
                        lambda: called.__setitem__("resolve", True) or sys.executable)
    cfg = {"mode": "source", "carla_root": "x", "python": sys.executable}
    out = run_cosim.ensure_runtime(dict(cfg))
    assert out == cfg and called["resolve"] is False


def test_frame_from_table_centroid_and_span(tmp_path):
    """TL-table framing: centroid + span, with --no-net-offset mapping y -> -y."""
    csv_path = tmp_path / "tl.csv"
    csv_path.write_text(
        "junction_id,link_id,x,y,z,heading\n"
        "j,0,100,200,10,0\n"
        "j,1,300,400,30,0\n", encoding="utf-8")
    cx, cy, cz, span, anchor = run_cosim._frame_from_table(str(csv_path), no_net_offset=True)
    assert cx == 200.0 and cy == -300.0 and cz == 20.0   # y negated, averaged
    assert span == 200.0                                  # max(200, 200)
    # without no_net_offset, y is kept as-is
    _, cy2, _, _, _ = run_cosim._frame_from_table(str(csv_path), no_net_offset=False)
    assert cy2 == 300.0


def test_frame_from_table_missing_file():
    assert run_cosim._frame_from_table("nope.csv", no_net_offset=True) is None


def test_frame_from_table_picks_busiest_junction(tmp_path):
    """Default framing zooms to the junction with the most signal heads."""
    csv_path = tmp_path / "tl.csv"
    csv_path.write_text(
        "junction_id,link_id,x,y,z,heading\n"
        "A,0,0,0,0,0\n"                       # junction A: 1 head, far away
        "B,0,1000,1000,5,0\n"                 # junction B: 3 heads (busiest)
        "B,1,1010,1000,5,0\n"
        "B,2,1020,1000,5,0\n", encoding="utf-8")
    cx, cy, cz, span, anchor = run_cosim._frame_from_table(str(csv_path), no_net_offset=True)
    assert cx == 1010.0 and cy == -1000.0   # centred on B, not the A/B centroid
    assert span == 20.0 and "B" in anchor    # tight span -> close view

    # whole=True frames everything instead
    _, _, _, span_all, anchor_all = run_cosim._frame_from_table(
        str(csv_path), no_net_offset=True, whole=True)
    assert span_all == 1020.0 and "network" in anchor_all


def test_ensure_runtime_repairs_missing_python(monkeypatch, tmp_path):
    """A stale config without a usable python is repaired via resolve_python +
    ensure_carla, and the result is persisted (CARLA paths preserved)."""
    monkeypatch.setattr(env, "_python_can_import", lambda py, mods: False)
    monkeypatch.setattr(env, "resolve_python", lambda: sys.executable)
    monkeypatch.setattr(env, "ensure_carla", lambda py, mode, root: None)
    saved = {}
    monkeypatch.setattr(env, "save_config", lambda c: saved.update(c))
    cfg = {"mode": "source", "carla_root": "C:/src_ext/Carla", "ue4_root": "C:/ue4"}
    out = run_cosim.ensure_runtime(dict(cfg))
    assert out["python"] == sys.executable
    assert out["carla_root"] == "C:/src_ext/Carla" and out["ue4_root"] == "C:/ue4"
    assert saved.get("python") == sys.executable  # persisted
