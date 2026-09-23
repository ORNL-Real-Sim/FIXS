"""gui/core.py - the toolkit-free half of the FIXS window (#78).

The window is only as trustworthy as the flags it passes and the way it stops a
run, so those are what is pinned here, without Qt:

  * a saved setup opened and run untouched is exactly `--profile NAME`;
  * Stop goes through --stop-file and unwinds the run's `finally` blocks - the
    teardown of SUMO, TrafficLayer and CARLA - which a signal from a GUI parent
    cannot do on Windows;
  * Force stop takes the whole process tree, not just the python at its root.
"""
import os
import subprocess
import sys
import textwrap
import threading
import time

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__),
                                "..", "..", "..", "scripts", "cosim"))
from gui import core  # noqa: E402

COSIM = os.path.abspath(os.path.join(os.path.dirname(__file__),
                                     "..", "..", "..", "scripts", "cosim"))


# --------------------------------------------------------------------------- #
# RunRequest -> flags
# --------------------------------------------------------------------------- #
def test_untouched_saved_setup_is_just_its_profile():
    assert core.RunRequest(setup="mlk_default").args() == ["--profile", "mlk_default"]


def test_new_run_with_no_app():
    assert core.RunRequest(app="", map="uga").args() == \
        ["--fresh", "--no-app", "--map", "uga"]


def test_every_field_becomes_its_flag():
    req = core.RunRequest(setup="s", app="mlk_eco_driving", map="mlk",
                          config="C:/x/scen.yaml", sumo_gui=False, sumo_only=True,
                          engine="py", fast=True, app_args="--rate 0.2", log=True)
    assert req.args() == ["--profile", "s", "--app", "mlk_eco_driving",
                          "--map", "mlk", "--config", "C:/x/scen.yaml",
                          "--no-sumo-gui", "--sumo-only", "--engine", "py", "--fast",
                          "--app-args", "--rate 0.2", "--log"]


def test_taking_the_app_off_a_saved_setup():
    assert core.RunRequest(setup="s", app="").args() == ["--profile", "s", "--no-app"]


def test_empty_app_args_is_still_passed():
    """An explicit empty string overrides an inherited COSIM_APP_ARGS."""
    assert core.RunRequest(setup="s", app_args="").args()[-2:] == ["--app-args", ""]


# --------------------------------------------------------------------------- #
# reading the log
# --------------------------------------------------------------------------- #
@pytest.mark.parametrize("line,prev,tag,stream", [
    ("[SUMO] sumo -c x.sumocfg", None, "SUMO", "SUMO"),
    ("[cosim]   OK   SUMO running", None, "cosim", "FIXS"),
    ("[TL] warm-up done", None, "TL", "TrafficLayer"),
    ("[APP] controller up", None, "APP", "App"),
    ("Traceback (most recent call last):", "APP", "APP", "App"),   # stays with its speaker
    ("[VCE] tick 12", None, "VCE", "Bridge"),
    ("plain text", None, None, None),
])
def test_tag_and_stream(line, prev, tag, stream):
    assert core.tag_of(line, prev) == tag
    assert core.stream_of(tag) == stream


@pytest.mark.parametrize("line,level", [
    ("[cosim]   DEAD SUMO exited immediately", "problem"),
    ("Traceback (most recent call last):", "problem"),
    ("[doctor] FAIL libsumo", "problem"),
    ("[cosim] WARN mlk.yaml lists the bridge port", "warning"),
    ("[cosim]   OK   TrafficLayer running", None),
])
def test_level(line, level):
    assert core.level_of(line) == level


# --------------------------------------------------------------------------- #
# running and stopping
# --------------------------------------------------------------------------- #
# A stand-in for run_cosim's main: arms the real watch_stop_file, holds a
# "stack" open in a polling loop, and tears it down in `finally` - the shape every
# run_cosim run has. The watcher is armed INSIDE the outer try, as run_cosim arms it
# inside main(), which its __main__ wraps: a stop that was already requested fires
# the moment the watcher starts, and that must still land in the handler below.
_CHILD = textwrap.dedent("""
    import os, sys, time
    sys.path.insert(0, {cosim!r})
    import run_cosim
    stop = sys.argv[sys.argv.index("--stop-file") + 1]
    try:
        run_cosim.watch_stop_file(stop, poll_s=0.1)
        print("[cosim] stack up", flush=True)
        try:
            while True:
                time.sleep(1.0)
        finally:
            print("[cosim] stopping SUMO ...", flush=True)
    except KeyboardInterrupt:
        if run_cosim._STOP["requested"]:
            print("[cosim] stopped.", flush=True)
            sys.exit(130)
        raise
""")


def _collect(runner, args):
    lines, done, rc = [], threading.Event(), {}

    def on_exit(code):
        rc["rc"] = code
        done.set()
    runner.start(args, lines.append, on_exit)
    return lines, done, rc


def _wait_for(pred, timeout=30):
    end = time.time() + timeout
    while time.time() < end:
        if pred():
            return True
        time.sleep(0.05)
    return False


def test_stop_file_unwinds_the_finally_blocks(tmp_path):
    child = tmp_path / "fake_run_cosim.py"
    child.write_text(_CHILD.format(cosim=COSIM))
    r = core.Runner(engine=str(child))
    lines, done, rc = _collect(r, [])
    assert _wait_for(lambda: "[cosim] stack up" in lines)
    assert r.running and "--stop-file" in r.argv
    assert r.request_stop()
    assert done.wait(20), "the run did not stop"
    assert rc["rc"] == 130
    assert "[cosim] stopping SUMO ..." in lines           # teardown ran
    assert lines[-1] == "[cosim] stopped."
    assert not os.path.exists(r.stop_file)                # cleaned up


def test_stop_requested_before_the_watcher_is_armed_still_stops(tmp_path):
    """A request that arrives while the engine is still starting (before its
    re-exec, say) is honoured the moment the watcher is armed, not lost."""
    child = tmp_path / "slow_start.py"
    child.write_text("import time; time.sleep(1.5)\n" + _CHILD.format(cosim=COSIM))
    r = core.Runner(engine=str(child))
    lines, done, rc = _collect(r, [])
    assert r.request_stop()
    assert done.wait(20)
    assert rc["rc"] == 130


def test_each_run_gets_a_fresh_stop_file(tmp_path):
    r = core.Runner(engine=str(tmp_path / "x.py"))
    a = r.command([])
    b = r.command([])
    assert a[-1] != b[-1] and not os.path.exists(a[-1])


def _alive(pid):
    if os.name == "nt":
        out = subprocess.run(["tasklist", "/FI", f"PID eq {pid}", "/NH"],
                             capture_output=True, text=True).stdout
        return str(pid) in out
    try:
        os.kill(pid, 0)
        return True
    except OSError:
        return False


def test_force_stop_takes_the_whole_tree(tmp_path):
    """SUMO and TrafficLayer are grandchildren of the window; killing only the
    python at the root would leave them holding their ports."""
    child = tmp_path / "tree.py"
    child.write_text(textwrap.dedent("""
        import subprocess, sys, time
        g = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(60)"])
        print(f"grandchild {g.pid}", flush=True)
        time.sleep(60)
    """))
    r = core.Runner(engine=str(child))
    lines, done, rc = _collect(r, [])
    assert _wait_for(lambda: any(l.startswith("grandchild") for l in lines))
    gpid = int(next(l for l in lines if l.startswith("grandchild")).split()[1])
    assert _alive(gpid)
    r.kill()
    assert done.wait(20)
    assert _wait_for(lambda: not _alive(gpid), timeout=10), "grandchild survived"


def test_a_run_reads_no_stdin(tmp_path):
    """The engine decides whether it may prompt from stdin.isatty(); a closed stdin
    must make an unguarded input() fail fast, never wait."""
    child = tmp_path / "ask.py"
    child.write_text("try:\n    input('? ')\nexcept EOFError:\n    print('EOF')\n")
    r = core.Runner(engine=str(child))
    lines, done, rc = _collect(r, [])
    assert done.wait(20)
    assert lines == ["? EOF"] or lines[-1].endswith("EOF")


def test_child_env_drops_the_reexec_guard(monkeypatch):
    monkeypatch.setenv("FIXS_REEXEC", "1")
    env = core.child_env()
    assert "FIXS_REEXEC" not in env and env["PYTHONUNBUFFERED"] == "1"
