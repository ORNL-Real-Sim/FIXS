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
