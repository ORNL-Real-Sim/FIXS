"""`launch` can name a controller's .py, and the argv it produces actually spawns.

An application's controller used to need a wrapper script whose whole content was
"call python on the .py beside me". That was not a choice an app made; it was
forced, and the forcing is platform-level: CreateProcess does not consult PATHEXT,
so a bare .py as argv[0] fails on Windows with WinError 193, and writing
`launch: "python my_controller.py"` resolves the FIRST token and looks for a
`python.bat` in the app folder.

So the interesting assertion is not the shape of the list - it is that the list
RUNS. test_py_launch_actually_spawns does that, and test_bare_py_is_what_fails is
its negative control: the same path without the interpreter is exactly the failure
this change removes, and on a platform where that form works anyway the control
skips rather than pretending it proved something.
"""
import os
import subprocess
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "..", "Carla"))
import app_catalog  # noqa: E402


CONTROLLER = "print('controller ran')\n"


def _app(tmp_path, launch, name="my_controller.py", body=CONTROLLER):
    """A manifest entry plus the file it names, under a throwaway repo root."""
    app_dir = tmp_path / "apps" / "demo"
    app_dir.mkdir(parents=True, exist_ok=True)
    if name:
        (app_dir / name).write_text(body, encoding="utf-8")
    return {"id": "demo", "dir": "demo", "launch": launch}


def test_py_is_run_under_this_interpreter(tmp_path):
    app = _app(tmp_path, "my_controller.py --penetrationRate 0.0")
    argv, cwd = app_catalog.launch_command(app, root=str(tmp_path))
    assert argv[0] == sys.executable
    assert argv[1].endswith("my_controller.py")
    assert os.path.isfile(argv[1])
    # The app's own arguments still pass through untouched.
    assert argv[2:] == ["--penetrationRate", "0.0"]
    assert cwd == str(tmp_path / "apps" / "demo")


def test_py_launch_actually_spawns(tmp_path):
    """The point of the change: Popen the argv exactly as start_app does."""
    app = _app(tmp_path, "my_controller.py")
    argv, cwd = app_catalog.launch_command(app, root=str(tmp_path))
    out = subprocess.run(argv, cwd=cwd, capture_output=True, text=True)
    assert out.returncode == 0, out.stderr
    assert "controller ran" in out.stdout


def test_bare_py_is_what_fails(tmp_path):
    """Negative control: without the interpreter, this is the original failure."""
    app = _app(tmp_path, "my_controller.py")
    argv, cwd = app_catalog.launch_command(app, root=str(tmp_path))
    bare = argv[1:]                      # drop sys.executable
    if os.name != "nt":
        pytest.skip("a +x .py with a shebang is spawnable here; the bug is Windows")
    with pytest.raises(OSError):
        subprocess.Popen(bare, cwd=cwd)


def test_exit_code_reaches_the_caller(tmp_path):
    """start_app polls proc.poll(); a controller that dies must look dead."""
    app = _app(tmp_path, "my_controller.py", body="import sys\nsys.exit(3)\n")
    argv, cwd = app_catalog.launch_command(app, root=str(tmp_path))
    assert subprocess.run(argv, cwd=cwd, capture_output=True).returncode == 3


def test_extensionless_still_gets_the_platform_script(tmp_path):
    """The existing convention is untouched: no extension -> .bat / .sh."""
    script = "run_demo.bat" if os.name == "nt" else "run_demo.sh"
    app = _app(tmp_path, "run_demo", name=script, body="")
    argv, _cwd = app_catalog.launch_command(app, root=str(tmp_path))
    assert argv[0].endswith(script)
    assert argv[0] != sys.executable


def test_named_script_is_not_prefixed(tmp_path):
    """A wrapper named outright is still run as itself, not handed to python."""
    script = "run_demo.bat" if os.name == "nt" else "run_demo.sh"
    app = _app(tmp_path, script, name=script, body="")
    argv, _cwd = app_catalog.launch_command(app, root=str(tmp_path))
    assert argv[0].endswith(script)
    assert len(argv) == 1


def test_missing_py_starts_nothing(tmp_path):
    """A launch that names a file that is not there must not become a spawn."""
    app = _app(tmp_path, "absent.py", name=None)
    assert app_catalog.launch_command(app, root=str(tmp_path)) == (None, None)
