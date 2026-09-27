"""
Tier-1 tests for the CARLA env setup/config layer (carla_env_setup.py) and the
launch-command resolution in run_cosim.py.

These are pure-stdlib: NO CARLA server, NO GPU, NO map asset, and NO interactive
prompt or GUI (the folder picker is never invoked here). They use a temp config
path and fake CARLA/UE4 trees, so they run on any computer. Run with:

    pytest test_carla_env.py
"""
import json
import os
import platform
import shutil
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
# the co-sim runtime lives at the repo root: FIXS_root/Carla
CARLA = os.path.normpath(os.path.join(HERE, "..", "..", "..", "Carla"))
sys.path.insert(0, CARLA)

import carla_env_setup as env  # noqa: E402
import run_cosim  # noqa: E402  (imports carla_env_setup; does NOT import the carla wheel)
import import_map  # noqa: E402  (stdlib + carla_env_setup; no carla wheel)
import place_tls  # noqa: E402  (stdlib + carla_env_setup; no carla wheel)

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


def _make_source(tmp_path):
    """Create a fake source CARLA + UE4 tree; return (carla_root, ue4_root, uproject, editor)."""
    carla_root = str(tmp_path / "carla")
    ue4_root = str(tmp_path / "ue4")
    uproject, editor = env.source_paths(carla_root, ue4_root)
    for path in (uproject, editor):
        os.makedirs(os.path.dirname(path), exist_ok=True)
        open(path, "w").close()
    return carla_root, ue4_root, uproject, editor


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
    carla_root, ue4_root, uproject, editor = _make_source(tmp_path)

    cfg = {"mode": "source", "carla_root": carla_root, "ue4_root": ue4_root}
    cmd = run_cosim._carla_command(cfg, 2000, render_offscreen=False)
    assert cmd[0] == editor
    assert cmd[1] == uproject
    assert "-game" in cmd


def test_carla_command_source_disables_renderdoc_prompt(tmp_path):
    """Source launches suppress UE4's RenderDoc plugin (#311).

    CARLA's uproject enables RenderDocPlugin, whose loader asks for a renderdoc.dll
    it cannot find by opening a modal file dialog at startup - which blocks the game
    thread until a human cancels it. -DisableFrameTraceCapture returns before the
    search, so the dialog is never reachable.
    """
    carla_root, ue4_root, _, _ = _make_source(tmp_path)

    cfg = {"mode": "source", "carla_root": carla_root, "ue4_root": ue4_root}
    cmd = run_cosim._carla_command(cfg, 2000, render_offscreen=False)
    assert "-DisableFrameTraceCapture" in cmd


def test_carla_command_packaged_carries_no_editor_flags(tmp_path):
    """A packaged build never loads the plugin (UncookedOnly), so it needs no flag."""
    root = str(tmp_path / "carla")
    _make_packaged(root)
    cfg = {"mode": "packaged", "carla_root": root}
    cmd = run_cosim._carla_command(cfg, 2000, render_offscreen=False)
    assert "-DisableFrameTraceCapture" not in cmd


def _res_flags(cmd):
    return [c for c in cmd if c == "-windowed" or c.startswith(("-ResX=", "-ResY="))]


def test_carla_command_default_window_is_1280x720(tmp_path):
    """No carla_res saved -> 1280x720, not UE4's desktop-sized window."""
    root = str(tmp_path / "carla")
    _make_packaged(root)
    cfg = {"mode": "packaged", "carla_root": root}
    cmd = run_cosim._carla_command(cfg, 2000, render_offscreen=False)
    assert _res_flags(cmd) == ["-windowed", "-ResX=1280", "-ResY=720"]


def test_carla_command_uses_saved_res_packaged_and_source(tmp_path):
    root = str(tmp_path / "pkg")
    _make_packaged(root)
    cmd = run_cosim._carla_command({"mode": "packaged", "carla_root": root,
                                    "carla_res": "960x540"}, 2000, render_offscreen=False)
    assert _res_flags(cmd) == ["-windowed", "-ResX=960", "-ResY=540"]

    carla_root, ue4_root, _, _ = _make_source(tmp_path)
    cmd = run_cosim._carla_command({"mode": "source", "carla_root": carla_root,
                                    "ue4_root": ue4_root, "carla_res": "1600x900"},
                                   2000, render_offscreen=False)
    assert _res_flags(cmd) == ["-windowed", "-ResX=1600", "-ResY=900"]


def test_carla_command_bad_saved_res_falls_back(tmp_path, capsys):
    root = str(tmp_path / "carla")
    _make_packaged(root)
    cfg = {"mode": "packaged", "carla_root": root, "carla_res": "big"}
    cmd = run_cosim._carla_command(cfg, 2000, render_offscreen=False)
    assert _res_flags(cmd) == ["-windowed", "-ResX=1280", "-ResY=720"]
    assert "'big' is not WxH" in capsys.readouterr().out


def test_carla_command_offscreen_has_no_window_flags(tmp_path):
    """No window to size: the offscreen command is left as it was."""
    root = str(tmp_path / "carla")
    _make_packaged(root)
    cfg = {"mode": "packaged", "carla_root": root, "carla_res": "960x540"}
    assert _res_flags(run_cosim._carla_command(cfg, 2000, render_offscreen=True)) == []


@pytest.mark.parametrize("text, want", [("1280x720", "1280x720"), (" 960X540 ", "960x540")])
def test_res_arg_normalises(text, want):
    assert run_cosim._res_arg(text) == want


@pytest.mark.parametrize("text", ["1280", "1280x", "x720", "0x720", "1280x-1", "1280*720", ""])
def test_res_arg_rejects(text):
    import argparse
    with pytest.raises(argparse.ArgumentTypeError):
        run_cosim._res_arg(text)


def test_saved_carla_res(tmp_path, monkeypatch):
    """What setup carries over: a valid carla_res from the file on disk, else None."""
    path = tmp_path / "carla.json"
    monkeypatch.setattr(env, "CONFIG_PATH", str(path))
    assert env._saved_carla_res() is None                      # no file
    path.write_text('{"mode": "weird", "carla_res": "1600x900"}', encoding="utf-8")
    assert env._saved_carla_res() == "1600x900"                # kept even if load_config rejects
    path.write_text('{"mode": "packaged", "carla_res": "huge"}', encoding="utf-8")
    assert env._saved_carla_res() is None
    path.write_text('[]', encoding="utf-8")
    assert env._saved_carla_res() is None


def test_carla_command_packaged_missing_raises(tmp_path):
    cfg = {"mode": "packaged", "carla_root": str(tmp_path / "empty")}
    with pytest.raises(FileNotFoundError):
        run_cosim._carla_command(cfg, 2000, render_offscreen=False)


# ------------------------------------- generic python / wheel resolution

def test_python_can_import_self():
    """The running interpreter can import os (sanity of the subprocess probe)."""
    assert env._python_can_import(sys.executable, ("os", "sys"))
    assert not env._python_can_import(sys.executable, ("a_module_that_does_not_exist_xyz",))


def test_python_candidates_includes_current():
    """Candidate discovery always includes the current interpreter, all real."""
    cands = env._python_candidates()
    assert os.path.normcase(sys.executable) in {os.path.normcase(c) for c in cands}
    assert all(os.path.isfile(c) for c in cands)


def test_python_candidates_deduped_by_real_path():
    """One binary reached under several names (conda's bin/python -> bin/python3)
    is offered once, not once per name."""
    reals = [os.path.normcase(os.path.realpath(c)) for c in env._python_candidates()]
    assert len(reals) == len(set(reals))


def test_python_candidates_offer_system_python():
    """A system interpreter on PATH is a candidate: on Linux it is frequently the
    only one that can import traci/sumolib (apt puts them in dist-packages)."""
    sys_py = shutil.which("python3") or shutil.which("python")
    if not sys_py:
        pytest.skip("no python/python3 on PATH")
    reals = {os.path.normcase(os.path.realpath(c)) for c in env._python_candidates()}
    assert os.path.normcase(os.path.realpath(sys_py)) in reals


def test_interpreter_kind_env_private_base_and_system_shared(tmp_path, monkeypatch):
    """A named conda env is FIXS-private; the base env under the same root and
    anything outside conda are shared (so installs into them are gated)."""
    root = tmp_path / "miniconda3"
    named = env._env_python(str(root / "envs" / "realsim"))
    base = env._env_python(str(root))
    for p in (named, base):
        os.makedirs(os.path.dirname(p), exist_ok=True)
        open(p, "w").close()
    monkeypatch.setattr(env, "_conda_roots", lambda: [str(root)])

    label, shared = env._interpreter_kind(named)
    assert "realsim" in label and shared is False
    label, shared = env._interpreter_kind(base)
    assert "BASE" in label and shared is True
    label, shared = env._interpreter_kind(str(tmp_path / "usr" / "bin" / "python3"))
    assert label == "SYSTEM python" and shared is True


def test_confirm_install_gates_shared_interpreter(monkeypatch, capsys):
    """A private env installs unasked. A shared one must be confirmed, warns why,
    and declines when there is no console to answer on."""
    monkeypatch.setattr(env, "_interpreter_kind", lambda py: ("conda env 'realsim'", False))
    assert env._confirm_install("py", "carla==0.9.15") is True

    monkeypatch.setattr(env, "_interpreter_kind", lambda py: ("SYSTEM python", True))
    monkeypatch.setattr("builtins.input", lambda *_: "y")
    assert env._confirm_install("py", "carla==0.9.15") is True
    monkeypatch.setattr("builtins.input", lambda *_: "")
    assert env._confirm_install("py", "carla==0.9.15") is False
    assert "WARNING" in capsys.readouterr().out

    def _no_console(*_):
        raise EOFError
    monkeypatch.setattr("builtins.input", _no_console)
    assert env._confirm_install("py", "carla==0.9.15") is False


def test_confirm_install_question_always_asks(monkeypatch):
    """An explicit question is put even to a private env - that is how the source
    build's client/server-match reinstall stays opt-in."""
    monkeypatch.setattr(env, "_interpreter_kind", lambda py: ("conda env 'realsim'", False))
    monkeypatch.setattr("builtins.input", lambda *_: "n")
    assert env._confirm_install("py", "wheel", "reinstall? [y/N]: ") is False


# ------------------------------------------------------------ uv env (opt-in)

def _make_venv(env_dir, uv=True):
    """A fake venv: its python file and a pyvenv.cfg, uv-made or not."""
    py = env._venv_python(str(env_dir))
    os.makedirs(os.path.dirname(py), exist_ok=True)
    open(py, "w").close()
    cfg = "home = /base\n" + ("uv = 0.8.8\n" if uv else "") + "version_info = 3.10.18\n"
    (env_dir / "pyvenv.cfg").write_text(cfg, encoding="utf-8")
    return py


@pytest.fixture
def fixs_home(tmp_path, monkeypatch):
    """~/.fixs redirected into tmp: the flag file and the uv envs dir."""
    home = tmp_path / ".fixs"
    home.mkdir()
    monkeypatch.setattr(env, "ENV_FLAG_PATH", str(home / "env.json"))
    monkeypatch.setattr(env, "UV_ENVS_DIR", str(home / "envs"))
    return home


@pytest.mark.parametrize("content, expected", [
    (None, False),                       # no file: conda, as before
    ('{"use_uv": true}', True),
    ('{"use_uv": false}', False),
    ('{"use_uv": "yes"}', False),        # only a JSON true opts in
    ('["use_uv"]', False),
    ("not json", False),
])
def test_use_uv_reads_the_flag_file(fixs_home, content, expected):
    if content is not None:
        (fixs_home / "env.json").write_text(content, encoding="utf-8")
    assert env.use_uv() is expected


def test_interpreter_kind_venvs_are_private(fixs_home, tmp_path):
    """A uv env under ~/.fixs/envs, and any other venv, is FIXS-private: installing
    into it reaches nothing else, so it must not get the system-python warning."""
    named = _make_venv(fixs_home / "envs" / "fixs_applications")
    label, shared = env._interpreter_kind(named)
    assert label == "uv env 'fixs_applications'" and shared is False

    other = _make_venv(tmp_path / "proj" / ".venv", uv=False)
    label, shared = env._interpreter_kind(other)
    assert label.startswith("venv") and shared is False


def test_pip_cmd_goes_through_uv_for_a_uv_venv(fixs_home, tmp_path, monkeypatch):
    """A uv venv has no pip, so `python -m pip` fails there; a plain venv keeps it."""
    monkeypatch.setattr(env, "_find_uv", lambda: "uv")
    uv_py = _make_venv(fixs_home / "envs" / "x")
    assert env._pip_cmd(uv_py) == ["uv", "pip", "install", "--python", uv_py]

    plain = _make_venv(tmp_path / "plain", uv=False)
    assert env._pip_cmd(plain) == [plain, "-m", "pip", "install"]

    # uv gone: fall back to pip rather than to nothing.
    monkeypatch.setattr(env, "_find_uv", lambda: None)
    assert env._pip_cmd(uv_py) == [uv_py, "-m", "pip", "install"]


def _no_conda(monkeypatch):
    def _boom(*_):
        raise AssertionError("the conda path was consulted")
    monkeypatch.setattr(env, "_named_env_python", _boom)
    monkeypatch.setattr(env, "_find_conda", _boom)


def test_resolve_python_uv_flag_binds_the_uv_env(fixs_home, monkeypatch):
    (fixs_home / "env.json").write_text('{"use_uv": true}', encoding="utf-8")
    monkeypatch.setenv("FIXS_ENV_NAME", "fixs_applications")
    py = _make_venv(fixs_home / "envs" / "fixs_applications")
    _no_conda(monkeypatch)
    assert env._resolve_python() == py


def test_resolve_python_uv_flag_creates_the_env_from_the_lock(fixs_home, tmp_path,
                                                               monkeypatch):
    (fixs_home / "env.json").write_text('{"use_uv": true}', encoding="utf-8")
    monkeypatch.setenv("FIXS_ENV_NAME", "fixs_applications")
    (tmp_path / "uv.lock").write_text("", encoding="utf-8")
    monkeypatch.setattr(env, "UV_PROJECT", str(tmp_path))
    monkeypatch.setattr(env, "_find_uv", lambda: "uv")
    monkeypatch.setattr("builtins.input", lambda *_: "")
    made = []

    def _sync(uv, name):
        made.append(name)
        _make_venv(fixs_home / "envs" / name)
        return True
    monkeypatch.setattr(env, "_uv_create_env", _sync)
    _no_conda(monkeypatch)

    py = env._resolve_python()
    assert made == ["fixs_applications"]
    assert py == env._venv_python(str(fixs_home / "envs" / "fixs_applications"))


def test_resolve_python_uv_flag_without_uv_falls_back_to_conda(fixs_home, monkeypatch,
                                                               capsys):
    (fixs_home / "env.json").write_text('{"use_uv": true}', encoding="utf-8")
    monkeypatch.setattr(env, "_find_uv", lambda: None)
    monkeypatch.setattr(env, "_named_env_python", lambda name: "conda_py")
    assert env._resolve_python() == "conda_py"
    assert "uv is not installed" in capsys.readouterr().out


def test_resolve_python_without_flag_ignores_uv_envs(fixs_home, monkeypatch):
    """No flag, no change: a uv env on disk does not outrank the conda env."""
    monkeypatch.setenv("FIXS_ENV_NAME", "fixs_applications")
    _make_venv(fixs_home / "envs" / "fixs_applications")
    monkeypatch.setattr(env, "_named_env_python", lambda name: "conda_py")
    assert env._resolve_python() == "conda_py"


# ---------------------------------- setup asks: conda, uv or a system python

@pytest.mark.parametrize("content, expected", [
    (None, None),                        # nothing saved yet
    ('{"env_manager": "conda"}', "conda"),
    ('{"env_manager": "uv"}', "uv"),
    ('{"env_manager": "system"}', "system"),
    ('{"env_manager": "pip"}', None),    # not one of the three
    ('{"use_uv": true}', "uv"),          # dev_v0.10.0 / hand-made form
    ('{"use_uv": false}', "conda"),
    ('{"use_uv": 1}', None),             # only a JSON true/false is an answer there
    ('{"use_uv": "yes"}', None),
    ('{"env_manager": "system", "use_uv": true}', "system"),   # the new key wins
    ('{}', None),
    ('["uv"]', None),
])
def test_env_choice_reads_the_saved_answer(fixs_home, content, expected):
    if content is not None:
        (fixs_home / "env.json").write_text(content, encoding="utf-8")
    assert env.env_choice() == expected


@pytest.fixture
def ask(fixs_home, tmp_path, monkeypatch):
    """_ask_env_manager with conda/uv presence, the typed answers and both
    installers controlled. Returns a function:
        ask(answers, conda=..., uv=..., installs=...) -> (choice, prompts)
    `answers` is one answer or a list, one per prompt; EOFError as an answer raises
    it. `installs` is what the installer returns (None = it failed); every call is
    recorded in ask.installed as (tool, folder)."""
    (tmp_path / "uv.lock").write_text("", encoding="utf-8")
    monkeypatch.setattr(env, "UV_PROJECT", str(tmp_path))
    home = tmp_path / "home"
    home.mkdir()
    monkeypatch.setenv("HOME", str(home))
    monkeypatch.setenv("USERPROFILE", str(home))
    installed = []

    def _run(answers, conda="conda.exe", uv="uv.exe", installs="new_exe"):
        monkeypatch.setattr(env, "_find_conda", lambda: conda)
        monkeypatch.setattr(env, "_find_uv", lambda: uv)
        monkeypatch.setattr(env, "_install_uv",
                            lambda d: installed.append(("uv", d)) or installs)
        monkeypatch.setattr(env, "_install_miniforge",
                            lambda d: installed.append(("conda", d)) or installs)
        queue = list(answers) if isinstance(answers, list) else [answers]
        prompts = []

        def _input(prompt=""):
            prompts.append(prompt)
            answer = queue.pop(0)
            if answer is EOFError:
                raise EOFError
            return answer
        monkeypatch.setattr("builtins.input", _input)
        return env._ask_env_manager(), prompts
    _run.installed = installed
    _run.home = home
    return _run


@pytest.mark.parametrize("conda, uv, default", [
    ("conda.exe", "uv.exe", "conda"),    # both: conda, as setup did before
    ("conda.exe", None, "conda"),
    (None, "uv.exe", "uv"),              # only uv: uv
    (None, None, "uv"),                  # neither: uv, the lighter install
])
def test_ask_enter_takes_the_detected_default(ask, fixs_home, conda, uv, default):
    # Enter; the 'neither' row then reaches the install offer and the folder: Enter.
    choice, prompts = ask(["", "", ""], conda=conda, uv=uv)
    assert choice == default
    assert prompts[0] == "Enter 1, 2 or 3 [%s]: " % ("2" if default == "uv" else "1")
    assert env.env_choice() == default
    assert [t for t, _ in ask.installed] == (["uv"] if not uv and not conda else [])


def test_ask_neither_says_so(ask, fixs_home, capsys):
    ask("3", conda=None, uv=None)
    assert "neither conda nor uv is installed" in capsys.readouterr().out


def test_ask_default_is_the_saved_answer(ask, fixs_home):
    """--update-python re-asks, offering what was chosen last time."""
    (fixs_home / "env.json").write_text('{"env_manager": "system"}', encoding="utf-8")
    choice, prompts = ask("")
    assert choice == "system" and prompts == ["Enter 1, 2 or 3 [3]: "]


def test_ask_system_is_saved_and_installs_nothing(ask, fixs_home):
    choice, _ = ask("3", conda=None, uv=None)
    assert choice == "system" and ask.installed == []
    assert env.env_choice() == "system"


def test_ask_saves_env_manager_drops_use_uv_keeps_the_rest(ask, fixs_home):
    (fixs_home / "env.json").write_text('{"use_uv": true, "other": 7}', encoding="utf-8")
    assert ask("1")[0] == "conda"
    assert json.loads((fixs_home / "env.json").read_text(encoding="utf-8")) == \
        {"env_manager": "conda", "other": 7}


def test_ask_installs_uv_into_the_default_folder(ask, fixs_home):
    choice, prompts = ask(["2", "", ""], uv=None)
    assert choice == "uv"
    assert "Should FIXS install it for you" in prompts[1]
    assert f"[{env._uv_bin_dir()}]" in prompts[2]
    assert ask.installed == [("uv", env._uv_bin_dir())]
    assert json.loads((fixs_home / "env.json").read_text(encoding="utf-8"))["uv_dir"] \
        == env._uv_bin_dir()


def test_ask_installs_uv_into_a_folder_the_user_types(ask, fixs_home, tmp_path):
    tools = str(tmp_path / "my tools" / "uv")
    choice, _ = ask(["2", "y", '"%s"' % tools], uv=None)    # quotes are stripped
    assert choice == "uv" and ask.installed == [("uv", tools)]
    assert json.loads((fixs_home / "env.json").read_text(encoding="utf-8"))["uv_dir"] \
        == tools


def test_ask_installs_miniforge_into_the_default_folder(ask, fixs_home):
    choice, prompts = ask(["1", "", ""], conda=None)
    default = os.path.join(str(ask.home), "miniforge3")
    assert choice == "conda" and ask.installed == [("conda", default)]
    assert f"[{default}]" in prompts[2]
    assert json.loads((fixs_home / "env.json").read_text(encoding="utf-8"))[
        "conda_root"] == default


def test_ask_conda_folder_must_be_empty(ask, fixs_home, tmp_path, capsys):
    """Miniforge refuses a folder with files in it, so that is asked again."""
    full = tmp_path / "full"
    full.mkdir()
    (full / "something").write_text("", encoding="utf-8")
    fresh = str(tmp_path / "fresh")
    choice, prompts = ask(["1", "", str(full), fresh], conda=None)
    assert choice == "conda" and ask.installed == [("conda", fresh)]
    assert len(prompts) == 4
    assert "not empty" in capsys.readouterr().out


def test_ask_conda_folder_without_spaces(ask, fixs_home, tmp_path, capsys):
    """Miniforge's Linux installer stops on a path with spaces (measured on Ubuntu
    20.04), so it is asked again here - before the download, not after it."""
    fresh = str(tmp_path / "forge")
    choice, prompts = ask(["1", "", str(tmp_path / "my forge"), fresh], conda=None)
    assert choice == "conda" and ask.installed == [("conda", fresh)]
    assert len(prompts) == 4
    assert "path with spaces" in capsys.readouterr().out


def test_ask_uv_folder_may_have_spaces(ask, fixs_home, tmp_path):
    """The check is conda's: uv installs anywhere."""
    spaced = str(tmp_path / "my tools")
    ask(["2", "", spaced], uv=None)
    assert ask.installed == [("uv", spaced)]


@pytest.mark.parametrize("tool, answers, installs, steps", [
    ("uv", ["2", "n"], "new_exe", "THE-UV-COMMAND"),              # declined
    ("uv", ["2", EOFError], "new_exe", "THE-UV-COMMAND"),         # no console
    ("uv", ["2", "", EOFError], "new_exe", "THE-UV-COMMAND"),     # no folder given
    ("uv", ["2", "", ""], None, "THE-UV-COMMAND"),                # install failed
    ("conda", ["1", "n"], "new_exe", "THE-CONDA-STEPS"),
    ("conda", ["1", "", ""], None, "THE-CONDA-STEPS"),
])
def test_ask_missing_tool_not_installed_stops_with_the_manual_steps(
        ask, fixs_home, monkeypatch, tool, answers, installs, steps):
    """The user named the tool, so setup stops rather than binding another one -
    and says how to install it by hand, and to run run_cosim again."""
    monkeypatch.setattr(env, "_uv_install_command", lambda: "THE-UV-COMMAND")
    monkeypatch.setattr(env, "_miniforge_install_steps", lambda: "THE-CONDA-STEPS")
    with pytest.raises(SystemExit) as exc:
        ask(answers, conda=None, uv=None, installs=installs)
    assert f"{tool} is not installed" in str(exc.value)
    assert steps in str(exc.value) and "run run_cosim again" in str(exc.value)
    assert not (fixs_home / "env.json").exists()


def test_ask_uv_without_a_lock_stops(ask, fixs_home, tmp_path, monkeypatch):
    monkeypatch.setattr(env, "UV_PROJECT", str(tmp_path / "old_fixs"))
    with pytest.raises(SystemExit) as exc:
        ask("2", uv=None)
    assert "cannot build a uv env" in str(exc.value)
    assert ask.installed == [] and not (fixs_home / "env.json").exists()


def test_ask_invalid_answer_stops(ask, fixs_home):
    with pytest.raises(SystemExit) as exc:
        ask("4")
    assert "expected 1, 2 or 3" in str(exc.value)
    assert not (fixs_home / "env.json").exists()


def test_ask_without_a_console_keeps_the_saved_answer(ask, fixs_home):
    assert ask(EOFError)[0] is None
    assert not (fixs_home / "env.json").exists()
    (fixs_home / "env.json").write_text('{"env_manager": "uv"}', encoding="utf-8")
    assert ask(EOFError)[0] == "uv"


def test_resolve_python_asks_only_when_told(fixs_home, monkeypatch):
    """setup / --update-python ask; everything else reads the saved answer."""
    monkeypatch.setenv("FIXS_ENV_NAME", "fixs_applications")
    uv_py = _make_venv(fixs_home / "envs" / "fixs_applications")
    monkeypatch.setattr(env, "_named_env_python", lambda name: "conda_py")
    asked = []
    monkeypatch.setattr(env, "_ask_env_manager", lambda: asked.append(1) or "uv")

    assert env._resolve_python() == "conda_py" and asked == []
    assert env._resolve_python(ask=True) == uv_py and asked == [1]


def test_resolve_python_system_skips_conda(fixs_home, monkeypatch):
    """'system' goes straight to the pythons already on the machine."""
    (fixs_home / "env.json").write_text('{"env_manager": "system"}', encoding="utf-8")
    _no_conda(monkeypatch)
    monkeypatch.setattr(env, "_python_candidates", lambda: ["sys_py"])
    monkeypatch.setattr(env, "_python_can_import", lambda py, mods: True)
    monkeypatch.setattr(env, "_interpreter_kind", lambda py: ("SYSTEM python", True))
    assert env._resolve_python() == "sys_py"


def test_ensure_runtime_asks_on_update_python_not_on_repair(monkeypatch):
    seen = []
    monkeypatch.setattr(env, "resolve_python", lambda ask=False: seen.append(ask) or "py")
    monkeypatch.setattr(env, "ensure_carla", lambda *a: None)
    monkeypatch.setattr(env, "save_config", lambda cfg: None)
    monkeypatch.setattr(env, "_python_can_import", lambda *a: False)
    env.ensure_runtime({"mode": "client", "python": None})
    env.ensure_runtime({"mode": "client", "python": None}, force=True)
    assert seen == [False, True]


# ------------------------------ where FIXS put conda / uv, found again later

def test_conda_roots_include_the_saved_root(fixs_home, tmp_path):
    root = tmp_path / "my conda"
    root.mkdir()
    (fixs_home / "env.json").write_text(json.dumps({"conda_root": str(root)}),
                                        encoding="utf-8")
    assert os.path.normcase(str(root)) in [os.path.normcase(r) for r in env._conda_roots()]


def test_find_uv_looks_in_the_saved_dir(fixs_home, tmp_path, monkeypatch):
    monkeypatch.setattr(env.shutil, "which", lambda name: None)
    d = tmp_path / "tools"
    d.mkdir()
    exe = d / ("uv.exe" if platform.system() == "Windows" else "uv")
    exe.write_text("", encoding="utf-8")
    (fixs_home / "env.json").write_text(json.dumps({"uv_dir": str(d)}), encoding="utf-8")
    assert env._find_uv() == str(exe)


# ----------------------------------------------- the installers, per OS

@pytest.mark.parametrize("system, expected", [
    ("Windows", 'powershell -ExecutionPolicy ByPass -c '
                '"irm https://astral.sh/uv/install.ps1 | iex"'),
    ("Linux", "curl -LsSf https://astral.sh/uv/install.sh | sh"),
])
def test_uv_install_command_per_os(monkeypatch, system, expected):
    monkeypatch.setattr(env.platform, "system", lambda: system)
    assert env._uv_install_command() == expected


@pytest.mark.parametrize("system, needle", [
    ("Windows", "Miniforge3-Windows-x86_64.exe"),
    ("Linux", "bash Miniforge3-$(uname)-$(uname -m).sh"),
])
def test_miniforge_install_steps_per_os(monkeypatch, system, needle):
    monkeypatch.setattr(env.platform, "system", lambda: system)
    assert needle in env._miniforge_install_steps()


@pytest.mark.parametrize("system, machine, asset", [
    ("Windows", "AMD64", "Miniforge3-Windows-x86_64.exe"),
    ("Linux", "x86_64", "Miniforge3-Linux-x86_64.sh"),
    ("Linux", "aarch64", "Miniforge3-Linux-aarch64.sh"),
    ("Darwin", "arm64", "Miniforge3-MacOSX-arm64.sh"),
])
def test_miniforge_asset_per_os(monkeypatch, system, machine, asset):
    monkeypatch.setattr(env.platform, "system", lambda: system)
    monkeypatch.setattr(env.platform, "machine", lambda: machine)
    assert env._miniforge_asset() == asset


class _Resp:
    """urlopen's response: read(n) in chunks, then b''."""
    def __init__(self, data):
        self.data = data

    def read(self, n=-1):
        if n is None or n < 0:
            n = len(self.data)
        out, self.data = self.data[:n], self.data[n:]
        return out

    def __enter__(self):
        return self

    def __exit__(self, *a):
        return False


def test_downloads_name_a_user_agent(monkeypatch, tmp_path):
    """astral.sh answers Python's default agent with 403; name one."""
    seen = []

    def _urlopen(req, timeout=None):
        seen.append((req.full_url, req.get_header("User-agent")))
        return _Resp(b"x")
    monkeypatch.setattr("urllib.request.urlopen", _urlopen)
    assert env._download("https://astral.sh/uv/install.sh", str(tmp_path / "s"))
    assert seen == [("https://astral.sh/uv/install.sh", "FIXS-setup")]


def _exe_name(system, tool):
    if tool == "uv":
        return "uv.exe" if system == "Windows" else "uv"
    return os.path.join("Scripts", "conda.exe") if system == "Windows" \
        else os.path.join("bin", "conda")


@pytest.mark.parametrize("system", ["Windows", "Linux"])
def test_install_uv_runs_the_official_installer(monkeypatch, tmp_path, system):
    """Downloads the OS's script and runs it with UV_INSTALL_DIR = the folder asked
    for; returns the uv it put there, and removes the script."""
    monkeypatch.setattr(env.platform, "system", lambda: system)
    monkeypatch.setattr(env.shutil, "which", lambda name: "/usr/bin/" + name)
    target = tmp_path / "uv home"
    urls, runs = [], []

    def _urlopen(url, timeout=None):
        url = getattr(url, "full_url", url)     # a urllib Request
        urls.append(url)
        return _Resp(b"echo installer")
    monkeypatch.setattr("urllib.request.urlopen", _urlopen)

    def _call(cmd, env=None):
        with open(cmd[-1], "rb") as f:
            runs.append((cmd, env["UV_INSTALL_DIR"], f.read()))
        target.mkdir()
        (target / _exe_name(system, "uv")).write_text("", encoding="utf-8")
        return 0
    monkeypatch.setattr(env.subprocess, "call", _call)

    assert env._install_uv(str(target)) == str(target / _exe_name(system, "uv"))
    (cmd, install_dir, script), = runs
    assert script == b"echo installer" and install_dir == str(target)
    if system == "Windows":
        assert urls == ["https://astral.sh/uv/install.ps1"]
        assert cmd[1:5] == ["-NoProfile", "-ExecutionPolicy", "Bypass", "-File"]
        assert cmd[-1].endswith(".ps1")
    else:
        assert urls == ["https://astral.sh/uv/install.sh"]
        assert cmd[0] == "sh" and cmd[-1].endswith(".sh")
    assert not os.path.exists(cmd[-1])


def test_install_uv_on_linux_without_curl_or_wget_says_what_to_install(
        monkeypatch, tmp_path, capsys):
    monkeypatch.setattr(env.platform, "system", lambda: "Linux")
    monkeypatch.setattr(env.shutil, "which", lambda name: None)

    def _no_download(*a, **k):
        raise AssertionError("downloaded although the installer cannot run")
    monkeypatch.setattr("urllib.request.urlopen", _no_download)
    assert env._install_uv(str(tmp_path)) is None
    assert "sudo apt install -y curl" in capsys.readouterr().out


@pytest.mark.parametrize("failure", ["download", "exit", "no_uv"])
def test_install_uv_failures_return_none(monkeypatch, tmp_path, failure):
    monkeypatch.setattr(env.platform, "system", lambda: "Linux")
    monkeypatch.setattr(env.shutil, "which", lambda name: "/usr/bin/" + name)

    def _urlopen(url, timeout=None):
        url = getattr(url, "full_url", url)     # a urllib Request
        if failure == "download":
            raise OSError("offline")
        return _Resp(b"")
    monkeypatch.setattr("urllib.request.urlopen", _urlopen)

    def _call(cmd, env=None):
        if failure != "no_uv":
            (tmp_path / "uv").write_text("", encoding="utf-8")
        return 1 if failure == "exit" else 0
    monkeypatch.setattr(env.subprocess, "call", _call)
    assert env._install_uv(str(tmp_path)) is None


def _miniforge_release(name, payload, digest=True):
    import hashlib
    asset = {"name": name, "size": len(payload),
             "browser_download_url": "https://example.invalid/" + name}
    if digest:
        asset["digest"] = "sha256:" + hashlib.sha256(payload).hexdigest()
    return json.dumps({"assets": [{"name": "other.sh"}, asset]}).encode()


@pytest.mark.parametrize("system", ["Windows", "Linux"])
def test_install_miniforge_verifies_and_runs_the_installer(monkeypatch, tmp_path,
                                                            system):
    monkeypatch.setattr(env.platform, "system", lambda: system)
    monkeypatch.setattr(env.platform, "machine", lambda: "x86_64")
    name = env._miniforge_asset()
    payload = b"miniforge installer bytes"
    prefix = tmp_path / "mini forge"

    def _urlopen(url, timeout=None):
        url = getattr(url, "full_url", url)     # a urllib Request
        if url == env.MINIFORGE_RELEASE_API:
            return _Resp(_miniforge_release(name, payload))
        assert url == "https://example.invalid/" + name
        return _Resp(payload)
    monkeypatch.setattr("urllib.request.urlopen", _urlopen)
    runs = []

    def _call(cmd, env=None):
        runs.append(cmd)
        conda = prefix / _exe_name(system, "conda")
        conda.parent.mkdir(parents=True)
        conda.write_text("", encoding="utf-8")
        return 0
    monkeypatch.setattr(env.subprocess, "call", _call)

    assert env._install_miniforge(str(prefix)) == str(prefix / _exe_name(system, "conda"))
    (cmd,) = runs
    if system == "Windows":
        # One string, /D last and unquoted even though the folder has a space.
        assert isinstance(cmd, str)
        assert cmd.endswith(f" /S /D={prefix}")
        assert "/AddToPath=0" in cmd and "/RegisterPython=0" in cmd
        assert "/InstallationType=JustMe" in cmd
    else:
        assert cmd[0] == "bash" and cmd[2:] == ["-b", "-p", str(prefix)]


@pytest.mark.parametrize("problem", ["sha_mismatch", "no_digest", "no_asset",
                                     "api_down"])
def test_install_miniforge_refuses_what_it_cannot_verify(monkeypatch, tmp_path,
                                                         problem):
    monkeypatch.setattr(env.platform, "system", lambda: "Linux")
    monkeypatch.setattr(env.platform, "machine", lambda: "x86_64")
    name = env._miniforge_asset()

    def _urlopen(url, timeout=None):
        url = getattr(url, "full_url", url)     # a urllib Request
        if url == env.MINIFORGE_RELEASE_API:
            if problem == "api_down":
                raise OSError("offline")
            release_name = "not-this.sh" if problem == "no_asset" else name
            return _Resp(_miniforge_release(release_name, b"good",
                                            digest=problem != "no_digest"))
        return _Resp(b"tampered")
    monkeypatch.setattr("urllib.request.urlopen", _urlopen)

    def _never(*a, **k):
        raise AssertionError("ran an installer it could not verify")
    monkeypatch.setattr(env.subprocess, "call", _never)
    assert env._install_miniforge(str(tmp_path / "mf")) is None


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


def test_reexec_noop_same_interpreter():
    """No re-exec when already on the configured python, or when it is missing /
    unset (must simply return, never SystemExit).

    Lives on carla_env_setup now, not run_cosim: every entry point -- import_map,
    place_tls, load_opendrive_world -- needs the same relaunch, so run_cosim keeping
    a private copy meant which script you started decided which interpreter you got
    (see the note at run_cosim.py:1126). This test still named the old private copy.
    """
    env.reexec_under_configured(__file__, {"python": sys.executable})
    env.reexec_under_configured(__file__,
                                {"python": os.path.join(os.sep, "no", "such", "python")})
    env.reexec_under_configured(__file__, {})


def test_ensure_runtime_noop_when_python_valid(monkeypatch):
    """A config whose python can import carla is returned unchanged - no setup."""
    monkeypatch.setattr(env, "_python_can_import", lambda py, mods: True)
    called = {"resolve": False}
    monkeypatch.setattr(env, "resolve_python",
                        lambda ask=False: called.__setitem__("resolve", True) or sys.executable)
    cfg = {"mode": "source", "carla_root": "x", "python": sys.executable}
    out = env.ensure_runtime(dict(cfg))
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


class _FakeWorld:
    """Only what _frame_from_map touches: world.get_map().get_spawn_points(),
    each spawn exposing .location.x/.y/.z. No carla server involved."""

    class _Loc:
        def __init__(self, x, y, z):
            self.x, self.y, self.z = x, y, z

    class _Spawn:
        def __init__(self, loc):
            self.location = loc

    class _Map:
        def __init__(self, spawns):
            self._spawns = spawns

        def get_spawn_points(self):
            return self._spawns

    def __init__(self, points=(), raises=False):
        self._spawns = [self._Spawn(self._Loc(*p)) for p in points]
        self._raises = raises

    def get_map(self):
        if self._raises:
            raise RuntimeError("map not queryable yet")
        return self._Map(self._spawns)


def test_frame_from_map_centroid_and_span():
    """No-signal fallback: centroid + span of the map's spawn points, so a map with
    no traffic lights still gets framed instead of leaving the camera at the origin."""
    cx, cy, cz, span, anchor = run_cosim._frame_from_map(
        _FakeWorld([(0, 0, 0), (100, 40, 10)]))
    assert (cx, cy, cz) == (50.0, 20.0, 5.0)
    assert span == 100.0                       # max(x-range 100, y-range 40)
    assert "map centre" in anchor and "2 spawn points" in anchor


def test_frame_from_map_no_spawn_points():
    assert run_cosim._frame_from_map(_FakeWorld([])) is None


def test_frame_from_map_unqueryable_map():
    """A server that cannot answer get_map() degrades to 'no framing', not a crash."""
    assert run_cosim._frame_from_map(_FakeWorld(raises=True)) is None


# ---------------------------------------------- map import (no real cook)

def test_map_is_imported_detects_umap(tmp_path):
    """map_is_imported keys off the cooked .umap under the source Content tree."""
    root = str(tmp_path / "carla")
    assert not import_map.map_is_imported(root, "RP_Ver0529")
    umap = import_map.cooked_map_path(root, "RP_Ver0529")
    os.makedirs(os.path.dirname(umap), exist_ok=True)
    open(umap, "w").close()
    assert import_map.map_is_imported(root, "RP_Ver0529")


def test_stage_package_from_local_dir(tmp_path):
    """A local package dir is copied into <carla_root>/Import (descriptor + assets)."""
    carla_root = tmp_path / "carla"
    (carla_root / "Import").mkdir(parents=True)
    pkg = tmp_path / "pkg"
    (pkg / "Assets").mkdir(parents=True)
    (pkg / "RP_Ver0529.json").write_text('{"maps":[]}', encoding="utf-8")
    (pkg / "Assets" / "RP_Ver0529.xodr").write_text("<x/>", encoding="utf-8")
    import_map.stage_package(str(carla_root), "RP_Ver0529", package_dir=str(pkg))
    assert (carla_root / "Import" / "RP_Ver0529.json").is_file()
    assert (carla_root / "Import" / "Assets" / "RP_Ver0529.xodr").is_file()


def test_gh_release_ref_parsing():
    """github release-asset URLs parse to (repo, tag, asset); others -> None."""
    ref = import_map._gh_release_ref(
        "https://github.com/ORNL-Real-Sim/FIXS_Applications/releases/download/"
        "map-RP_Ver0529/RP_Ver0529_carla_import.zip")
    assert ref == ("ORNL-Real-Sim/FIXS_Applications", "map-RP_Ver0529",
                   "RP_Ver0529_carla_import.zip")
    assert import_map._gh_release_ref("https://example.com/foo.zip") is None


def test_stage_from_path_accepts_zip(tmp_path):
    """A hand-downloaded .zip is extracted into Import/ (the manual ORNL path)."""
    src = tmp_path / "pkg"
    (src / "Assets").mkdir(parents=True)
    (src / "RP_Ver0529.json").write_text("{}", encoding="utf-8")
    (src / "Assets" / "RP_Ver0529.xodr").write_text("<x/>", encoding="utf-8")
    zpath = tmp_path / "RP_Ver0529_carla_import.zip"
    with __import__("zipfile").ZipFile(zpath, "w") as z:
        z.write(src / "RP_Ver0529.json", "RP_Ver0529.json")
        z.write(src / "Assets" / "RP_Ver0529.xodr", "Assets/RP_Ver0529.xodr")
    import_dir = tmp_path / "carla" / "Import"
    import_dir.mkdir(parents=True)
    import_map._stage_from_path(str(zpath), str(import_dir))
    assert (import_dir / "RP_Ver0529.json").is_file()
    assert (import_dir / "Assets" / "RP_Ver0529.xodr").is_file()


def test_stage_package_pick_uses_selector_not_gh(monkeypatch, tmp_path):
    """--package-pick forces the manual file selector and never calls gh."""
    carla_root = tmp_path / "carla"
    (carla_root / "Import").mkdir(parents=True)
    pkg = tmp_path / "pkg"
    pkg.mkdir()
    (pkg / "RP_Ver0529.json").write_text("{}", encoding="utf-8")
    monkeypatch.setattr(import_map, "_select_package", lambda name, url: str(pkg))
    monkeypatch.setattr(import_map, "_try_gh_download",
                        lambda url: (_ for _ in ()).throw(AssertionError("gh used!")))
    import_map.stage_package(str(carla_root), "RP_Ver0529",
                             package_url="https://x/y.zip", package_pick=True)
    assert (carla_root / "Import" / "RP_Ver0529.json").is_file()


def test_stage_package_noop_when_already_staged(tmp_path):
    """Already staged and no source given: use that copy, write nothing.

    Asserted on the directory, not on the message. The defect this guards is
    FIXS#358's second descriptor -- CARLA cooks every descriptor in Import/, so
    a repeat import cooked the map twice and crashed Unreal. A prose assertion
    tracks the wording instead, and went red when the wording was rewritten
    while the behaviour was correct throughout.
    """
    carla_root = tmp_path / "carla"
    import_dir = carla_root / "Import"
    import_dir.mkdir(parents=True)
    descriptor = import_dir / "RP_Ver0529.json"
    descriptor.write_text("{}", encoding="utf-8")

    assert import_map.stage_package(str(carla_root), "RP_Ver0529") == str(import_dir)
    assert descriptor.read_text(encoding="utf-8") == "{}"
    assert sorted(q.name for q in import_dir.iterdir()) == ["RP_Ver0529.json"]


def test_ensure_map_rejects_packaged(monkeypatch, tmp_path):
    """Importing requires a source build - packaged config is refused."""
    monkeypatch.setattr(import_map.env, "load_config",
                        lambda: {"mode": "packaged", "carla_root": str(tmp_path)})
    with pytest.raises(SystemExit):
        import_map.ensure_map("RP_Ver0529")


def test_ensure_map_noop_when_present(monkeypatch, tmp_path, capsys):
    """Already-imported map short-circuits without cooking."""
    root = tmp_path / "carla"
    umap = import_map.cooked_map_path(str(root), "RP_Ver0529")
    os.makedirs(os.path.dirname(umap), exist_ok=True)
    open(umap, "w").close()
    monkeypatch.setattr(import_map.env, "load_config",
                        lambda: {"mode": "source", "carla_root": str(root),
                                 "ue4_root": str(tmp_path / "ue4")})
    # run_import must NOT be called
    monkeypatch.setattr(import_map, "run_import",
                        lambda *a, **k: (_ for _ in ()).throw(AssertionError("cooked!")))
    assert import_map.ensure_map("RP_Ver0529") == 0
    assert "already imported" in capsys.readouterr().out


def test_ensure_map_force_reimports_when_present(monkeypatch, tmp_path):
    """force=True re-cooks even an already-imported map: it moves the old content
    aside, imports fresh, and reports success when the .umap is produced."""
    root = tmp_path / "carla"
    umap = import_map.cooked_map_path(str(root), "RP_Ver0529")
    os.makedirs(os.path.dirname(umap), exist_ok=True)
    open(umap, "w").close()
    monkeypatch.setattr(import_map.env, "load_config",
                        lambda: {"mode": "source", "carla_root": str(root),
                                 "ue4_root": str(tmp_path / "ue4")})
    monkeypatch.setattr(import_map, "stage_package", lambda *a, **k: None)
    called = {"import": False}

    def fake_import(cr, ue, nm):  # simulate a successful cook re-creating the umap
        called["import"] = True
        os.makedirs(os.path.dirname(umap), exist_ok=True)
        with open(umap, "w") as f:
            f.write("fresh")
        return 0

    monkeypatch.setattr(import_map, "run_import", fake_import)
    assert import_map.ensure_map("RP_Ver0529", force=True) == 0
    assert called["import"] is True
    assert os.path.isfile(umap)
    assert not os.path.isdir(import_map.cooked_content_dir(str(root), "RP_Ver0529") + ".bak_reimport")


def test_ensure_map_restores_backup_on_failed_reimport(monkeypatch, tmp_path):
    """If the re-cook fails to produce the umap, the previous map is restored."""
    root = tmp_path / "carla"
    umap = import_map.cooked_map_path(str(root), "RP_Ver0529")
    os.makedirs(os.path.dirname(umap), exist_ok=True)
    open(umap, "w").close()
    monkeypatch.setattr(import_map.env, "load_config",
                        lambda: {"mode": "source", "carla_root": str(root),
                                 "ue4_root": str(tmp_path / "ue4")})
    monkeypatch.setattr(import_map, "stage_package", lambda *a, **k: None)
    monkeypatch.setattr(import_map, "run_import", lambda *a, **k: 1)  # cook fails, no umap
    with pytest.raises(SystemExit):
        import_map.ensure_map("RP_Ver0529", force=True)
    assert os.path.isfile(umap)  # restored


def test_read_map_config(tmp_path):
    """A map.txt declares the package + url for the wrappers."""
    p = tmp_path / "map.txt"
    p.write_text("# the roosevelt map\npackage=RP_Ver0529\n"
                 "url=https://x/y.zip\n\n", encoding="utf-8")
    mc = import_map.read_map_config(str(p))
    assert mc["package"] == "RP_Ver0529" and mc["url"] == "https://x/y.zip"


# ----------------------------------------------- traffic-light placement

def test_tls_content_path_and_marker(tmp_path):
    """Content path + placement marker resolve under the map's cooked content."""
    assert place_tls.content_map_path("RP_Ver0529") == "/Game/RP_Ver0529/Maps/RP_Ver0529/RP_Ver0529"
    root = str(tmp_path / "carla")
    assert not place_tls.tls_placed(root, "RP_Ver0529")
    marker = place_tls.tls_marker(root, "RP_Ver0529")
    os.makedirs(os.path.dirname(marker), exist_ok=True)
    open(marker, "w").close()
    assert place_tls.tls_placed(root, "RP_Ver0529")


def test_place_tls_noop_when_marker_present(monkeypatch, tmp_path, capsys):
    """Already-placed (marker) short-circuits without launching the editor."""
    root = tmp_path / "carla"
    # map present
    umap = import_map.cooked_map_path(str(root), "RP_Ver0529")
    os.makedirs(os.path.dirname(umap), exist_ok=True)
    open(umap, "w").close()
    # marker present
    open(place_tls.tls_marker(str(root), "RP_Ver0529"), "w").close()
    table = tmp_path / "tl.csv"
    table.write_text("junction_id,link_id,x,y,z,heading\n", encoding="utf-8")
    monkeypatch.setattr(place_tls.env, "load_config",
                        lambda: {"mode": "source", "carla_root": str(root),
                                 "ue4_root": str(tmp_path / "ue4")})
    monkeypatch.setattr(place_tls.subprocess, "call",
                        lambda *a, **k: (_ for _ in ()).throw(AssertionError("editor launched!")))
    assert place_tls.place_tls("RP_Ver0529", str(table)) == 0
    assert "already placed" in capsys.readouterr().out


def test_place_tls_rejects_packaged(monkeypatch, tmp_path):
    """Placement needs a source build (it saves the umap via the editor)."""
    table = tmp_path / "tl.csv"
    table.write_text("x\n", encoding="utf-8")
    monkeypatch.setattr(place_tls.env, "load_config",
                        lambda: {"mode": "packaged", "carla_root": str(tmp_path)})
    with pytest.raises(SystemExit):
        place_tls.place_tls("RP_Ver0529", str(table))


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
    monkeypatch.setattr(env, "resolve_python", lambda ask=False: sys.executable)
    monkeypatch.setattr(env, "ensure_carla", lambda py, mode, root: None)
    saved = {}
    monkeypatch.setattr(env, "save_config", lambda c: saved.update(c))
    cfg = {"mode": "source", "carla_root": "C:/src_ext/Carla", "ue4_root": "C:/ue4"}
    out = env.ensure_runtime(dict(cfg))
    assert out["python"] == sys.executable
    assert out["carla_root"] == "C:/src_ext/Carla" and out["ue4_root"] == "C:/ue4"
    assert saved.get("python") == sys.executable  # persisted


# --------------------------------------------------------------- --purge-map

def _fake_map(carla_root, name, cooked=True, umap=True, staged=True, cache_root=None,
              cached=False):
    """Lay down the on-disk traces one imported map leaves, each half optional, so
    a test can build the half-states a purge exists to clean up."""
    if cooked:
        cfg = import_map.package_descriptor(carla_root, name)
        os.makedirs(os.path.dirname(cfg), exist_ok=True)
        with open(cfg, "w", encoding="utf-8") as f:
            f.write("{}")
        if umap:
            path = import_map.cooked_map_path(carla_root, name)
            os.makedirs(os.path.dirname(path), exist_ok=True)
            with open(path, "w", encoding="utf-8") as f:
                f.write("umap")
    if staged:
        stage = os.path.join(carla_root, "Import", name)
        os.makedirs(stage, exist_ok=True)
        with open(os.path.join(stage, name + ".fbx"), "w", encoding="utf-8") as f:
            f.write("fbx")
        with open(os.path.join(carla_root, "Import", name + ".json"),
                  "w", encoding="utf-8") as f:
            f.write("{}")
    if cached and cache_root:
        sumo = os.path.join(cache_root, name, "sumo")
        os.makedirs(sumo, exist_ok=True)
        with open(os.path.join(sumo, name + ".sumocfg"), "w", encoding="utf-8") as f:
            f.write("<configuration/>")


def _labels(record):
    return sorted(label for label, _p, _b in record["pieces"])


def test_purge_candidates_skips_carlas_own_content(tmp_path, monkeypatch):
    """CARLA's own Content/Carla/ ships a Config/Carla.Package.json exactly like an
    imported package does, so the descriptor alone cannot be the test - the engine's
    own content must never be offered for deletion."""
    monkeypatch.setenv("FIXS_MAP_CACHE", str(tmp_path / "cache"))
    root = str(tmp_path / "carla")
    _fake_map(root, "Carla", umap=False, staged=False)       # the engine's own
    _fake_map(root, "mlk_no_signal", cache_root=str(tmp_path / "cache"))
    names = [r["name"] for r in import_map.purge_candidates(root, mode="source")]
    assert names == ["mlk_no_signal"]


def test_purge_candidates_does_not_create_the_cache_dir(tmp_path, monkeypatch):
    """Taking an inventory must not bring into being the thing it reports on:
    _map_cache_dir(name) CREATES its folder, so using it here would invent an empty
    cache for every map and then offer it up."""
    cache = tmp_path / "cache"
    monkeypatch.setenv("FIXS_MAP_CACHE", str(cache))
    root = str(tmp_path / "carla")
    _fake_map(root, "mlk_no_signal")
    records = import_map.purge_candidates(root, mode="source")
    assert not (cache / "mlk_no_signal").exists()
    assert "cache" not in _labels(records[0])


def test_purge_candidates_offers_a_cook_that_died_partway(tmp_path, monkeypatch):
    """Content/<name>/ with a descriptor but no .umap is a crashed cook - exactly
    the wreckage this command exists to clear - so it is listed, and flagged."""
    monkeypatch.setenv("FIXS_MAP_CACHE", str(tmp_path / "cache"))
    root = str(tmp_path / "carla")
    _fake_map(root, "half_cooked", umap=False)
    records = import_map.purge_candidates(root, mode="source")
    assert [r["name"] for r in records] == ["half_cooked"]
    assert records[0]["cooked_umap"] is False


def test_purge_candidates_ignores_a_lone_descriptor(tmp_path, monkeypatch):
    """CARLA's Import/ holds descriptors that name no map (roadpainter_decals.json).
    A .json with no folder beside it must not conjure a purge candidate."""
    monkeypatch.setenv("FIXS_MAP_CACHE", str(tmp_path / "cache"))
    root = str(tmp_path / "carla")
    os.makedirs(os.path.join(root, "Import"), exist_ok=True)
    with open(os.path.join(root, "Import", "roadpainter_decals.json"),
              "w", encoding="utf-8") as f:
        f.write("{}")
    assert import_map.purge_candidates(root, mode="source") == []


def test_purge_candidates_finds_a_cache_with_no_carla(tmp_path, monkeypatch):
    """carla_root=None is a valid call: a client-mode machine has no local CARLA but
    still has a bundle cache, and reclaiming it is the point of asking."""
    cache = tmp_path / "cache"
    monkeypatch.setenv("FIXS_MAP_CACHE", str(cache))
    _fake_map(None, "x", cooked=False, staged=False)   # no CARLA halves at all
    os.makedirs(str(cache / "roosevelt_full" / "sumo"), exist_ok=True)
    with open(str(cache / "roosevelt_full" / "sumo" / "r.sumocfg"),
              "w", encoding="utf-8") as f:
        f.write("<configuration/>")
    records = import_map.purge_candidates(None, mode="client")
    assert [r["name"] for r in records] == ["roosevelt_full"]
    assert _labels(records[0]) == ["cache"]


def test_purge_sweeps_the_reimport_backup(tmp_path, monkeypatch):
    """A failed re-import leaves Content/<name>.bak_reimport - a full second copy of
    the map. Purging the map without it would strand that copy forever."""
    monkeypatch.setenv("FIXS_MAP_CACHE", str(tmp_path / "cache"))
    root = str(tmp_path / "carla")
    _fake_map(root, "mlk_no_signal")
    backup = import_map.cooked_content_dir(root, "mlk_no_signal") + ".bak_reimport"
    os.makedirs(backup, exist_ok=True)
    with open(os.path.join(backup, "old.uasset"), "w", encoding="utf-8") as f:
        f.write("old")
    records = import_map.purge_candidates(root, mode="source")
    assert backup in [p for _l, p, _b in records[0]["pieces"]]
    import_map.purge_maps(records)
    assert not os.path.exists(backup)


def test_purge_keeps_the_cache_unless_asked(tmp_path, monkeypatch):
    """The default deletes the CARLA halves only. The bundle cache costs a download
    to rebuild AND is read at run time (the .sumocfg lives there), so it takes its
    own yes."""
    cache = tmp_path / "cache"
    monkeypatch.setenv("FIXS_MAP_CACHE", str(cache))
    root = str(tmp_path / "carla")
    _fake_map(root, "mlk_no_signal", cache_root=str(cache), cached=True)
    records = import_map.purge_candidates(root, mode="source")
    assert _labels(records[0]) == ["cache", "cooked", "staged", "staged"]

    removed, failed = import_map.purge_maps(records)
    assert failed == []
    assert not os.path.isdir(import_map.cooked_content_dir(root, "mlk_no_signal"))
    assert not os.path.exists(os.path.join(root, "Import", "mlk_no_signal"))
    assert not os.path.exists(os.path.join(root, "Import", "mlk_no_signal.json"))
    assert (cache / "mlk_no_signal" / "sumo" / "mlk_no_signal.sumocfg").exists()
    assert all(label != "cache" for _n, label, _p, _b in removed)


def test_purge_drops_the_cache_when_asked(tmp_path, monkeypatch):
    cache = tmp_path / "cache"
    monkeypatch.setenv("FIXS_MAP_CACHE", str(cache))
    root = str(tmp_path / "carla")
    _fake_map(root, "mlk_no_signal", cache_root=str(cache), cached=True)
    records = import_map.purge_candidates(root, mode="source")
    _removed, failed = import_map.purge_maps(records, drop_cache=True)
    assert failed == []
    assert not (cache / "mlk_no_signal").exists()


def test_purge_reports_a_failure_instead_of_raising(tmp_path, monkeypatch):
    """The inventory is taken before the deletion, so a path can vanish in between.
    That must be reported per item, not abort the whole purge."""
    monkeypatch.setenv("FIXS_MAP_CACHE", str(tmp_path / "cache"))
    root = str(tmp_path / "carla")
    _fake_map(root, "a")
    _fake_map(root, "b")
    records = import_map.purge_candidates(root, mode="source")
    shutil.rmtree(import_map.cooked_content_dir(root, "a"))   # vanishes underneath
    removed, failed = import_map.purge_maps(records)
    assert [name for name, _l, _p, _e in failed] == ["a"]
    assert not os.path.isdir(import_map.cooked_content_dir(root, "b"))   # b still done


def test_record_bytes_excludes_the_cache_by_default(tmp_path, monkeypatch):
    cache = tmp_path / "cache"
    monkeypatch.setenv("FIXS_MAP_CACHE", str(cache))
    root = str(tmp_path / "carla")
    _fake_map(root, "m", cache_root=str(cache), cached=True)
    record = import_map.purge_candidates(root, mode="source")[0]
    assert import_map.record_bytes(record) < import_map.record_bytes(record, True)


@pytest.mark.parametrize("answer,expected", [
    ("2", [1]),
    ("1,3,4", [0, 2, 3]),
    (" 1 , 3 ", [0, 2]),
    ("2-4", [1, 2, 3]),
    ("1,3-5", [0, 2, 3, 4]),
    ("all", [0, 1, 2, 3, 4]),
    ("a", [0, 1, 2, 3, 4]),
    ("3,3", [2]),
    ("1,", [0]),        # a trailing comma has one reading; do not re-ask over it
])
def test_parse_selection_accepts(answer, expected):
    assert import_map.parse_selection(answer, 5) == expected


@pytest.mark.parametrize("answer", ["", "0", "6", "4-2", "1-9", "x", "1,x", "-"])
def test_parse_selection_rejects(answer):
    """None means re-ask. Nothing here may be guessed at: the next step deletes."""
    assert import_map.parse_selection(answer, 5) is None
