"""
gui/core.py - everything the FIXS window does, with no toolkit in it.

The window is a front end over run_cosim, not a second implementation of it: every
button is one run of the engine with flags, the same command a terminal user would
type. This module owns that contract - which flags a form turns into, how the child
is started and stopped, how its output is read - so it can be tested without Qt and
kept if the toolkit ever changes. app.py is only widgets on top of it.

Three rules the child processes follow, each because the alternative hangs or leaks:

  * stdin is closed. The engine decides "may I ask a human?" from stdin.isatty(),
    and every prompt it has answers EOF with a default or a clean exit. An open
    pipe nobody writes to would instead park an unguarded input() forever.
  * a run is stopped through --stop-file, never a signal. Windows cannot deliver
    Ctrl+C to a child from a GUI parent, and CTRL_BREAK ends python without running
    the `finally` blocks that shut SUMO, TrafficLayer and CARLA down.
  * a run gets its own process group, so the last-resort kill takes the whole tree
    (SUMO, TrafficLayer, the app) rather than orphaning it.
"""
import json
import os
import re
import shlex
import signal
import subprocess
import sys
import tempfile
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ENGINE = os.path.join(os.path.dirname(HERE), "run_cosim.py")
IS_WINDOWS = os.name == "nt"

# The engine's own re-exec guard. A window opened under the configured python by a
# re-exec inherits it, and passing it on would stop every run from switching to the
# configured env - which is the engine's decision to make, per run, not ours.
_STRIP_ENV = ("FIXS_REEXEC",)


class EngineError(RuntimeError):
    """A query the engine could not answer; str() is what it said on stderr."""


def child_env(extra=None):
    env = {k: v for k, v in os.environ.items() if k not in _STRIP_ENV}
    # Line-buffered and UTF-8, so the log pane gets each line as it is printed and
    # a path with a non-ASCII character does not end a run with UnicodeEncodeError.
    env["PYTHONUNBUFFERED"] = "1"
    env["PYTHONIOENCODING"] = "utf-8"
    if extra:
        env.update(extra)
    return env


def engine_cwd(engine=ENGINE):
    """Where a run starts: the application repo that holds FIXS/, as it is for the
    front door. The engine resolves its own paths either way; this is where its
    RealSim_tmp/ (logs, handoff files) lands."""
    sys.path.insert(0, os.path.dirname(engine))
    try:
        import fixs_paths
        root = fixs_paths.app_root(os.path.dirname(engine))
    finally:
        sys.path.pop(0)
    return root if os.path.isdir(root) else os.path.dirname(engine)


def format_command(argv):
    """argv as the user would type it, for display and copy-paste."""
    return subprocess.list2cmdline(argv) if IS_WINDOWS else shlex.join(argv)


# --------------------------------------------------------------------------- #
# Queries: short runs that print one JSON document.
# --------------------------------------------------------------------------- #
def query(args, engine=ENGINE, python=None, timeout=120):
    """Run the engine with `args` + --json and return the parsed document.

    Raises EngineError with the tail of stderr when it exits non-zero without a
    document - except that --doctor exits 1 on a FAIL row and still prints one,
    and that document is the answer, so a parseable stdout always wins."""
    cmd = [python or sys.executable, engine, *args, "--json"]
    kwargs = {}
    if IS_WINDOWS:
        kwargs["creationflags"] = subprocess.CREATE_NO_WINDOW
    try:
        out = subprocess.run(cmd, stdin=subprocess.DEVNULL, capture_output=True,
                             timeout=timeout, cwd=engine_cwd(engine), env=child_env(),
                             **kwargs)
    except subprocess.TimeoutExpired:
        raise EngineError(f"{format_command(cmd)} did not answer in {timeout} s")
    text = out.stdout.decode("utf-8", errors="replace").strip()
    try:
        return json.loads(text)
    except ValueError:
        err = out.stderr.decode("utf-8", errors="replace").strip().splitlines()
        raise EngineError("\n".join(err[-12:]) or
                          f"exit {out.returncode} and no JSON on stdout")


def list_setups(**kw):
    return query(["--list", "setups"], **kw)


def list_apps(**kw):
    return query(["--list", "apps"], **kw)


def list_maps(**kw):
    return query(["--list", "maps"], **kw)


def version(**kw):
    return query(["--version"], **kw)


def doctor(**kw):
    return query(["--doctor"], timeout=180, **kw)


# --------------------------------------------------------------------------- #
# What to run
# --------------------------------------------------------------------------- #
class RunRequest:
    """One Run press, as engine flags.

    `setup` names a saved setup (--profile); None starts a new one (--fresh), in
    which case `app` "" means no application (--no-app), as it does to take the
    application off a saved one. Every other field is None
    for "leave it to the setup / the scenario yaml", which is how a saved setup
    runs exactly as saved when nothing on the form was changed.

    A value passed with a saved setup is saved INTO it by the engine (its
    _apply_cli), except sumo_gui, which the engine treats as this run only."""

    def __init__(self, setup=None, app=None, map=None, config=None, sumo_gui=None,
                 sumo_only=False, engine=None, fast=False, app_args=None, log=False,
                 extra=()):
        self.setup = setup
        self.app = app
        self.map = map
        self.config = config
        self.sumo_gui = sumo_gui
        self.sumo_only = sumo_only
        self.engine = engine
        self.fast = fast
        self.app_args = app_args
        self.log = log
        self.extra = list(extra)

    def args(self):
        a = ["--profile", self.setup] if self.setup else ["--fresh"]
        if self.app == "":
            a.append("--no-app")
        elif self.app:
            a += ["--app", self.app]
        if self.map:
            a += ["--map", self.map]
        if self.config:
            a += ["--config", self.config]
        if self.sumo_gui is not None:
            a.append("--sumo-gui" if self.sumo_gui else "--no-sumo-gui")
        if self.sumo_only:
            a.append("--sumo-only")
        if self.engine:
            a += ["--engine", self.engine]
        if self.fast:
            a.append("--fast")
        if self.app_args is not None:
            a += ["--app-args", self.app_args]
        if self.log:
            a.append("--log")
        return a + self.extra


# --------------------------------------------------------------------------- #
# Reading the log
# --------------------------------------------------------------------------- #
# Every engine line starts with its speaker in brackets: [cosim], [SUMO], [TL],
# [VCE], [APP], [SYNC], [setup], [import] ... and the stack's own children print
# theirs. What is not tagged (a traceback, a child's raw output) belongs to
# whichever speaker came before it, which is how a traceback stays with its APP.
_TAG = re.compile(r"^\s*\[([A-Za-z][\w .-]{0,15})\]")
_PROBLEM = re.compile(r"\b(FAIL(ED)?|DEAD|ERROR|Error|Traceback|Exception|FATAL)\b")
_WARNING = re.compile(r"\b(WARN(ING)?|Warning)\b")

# Log filters: label -> the tags it shows. None = everything.
STREAMS = {
    "FIXS": ("cosim", "setup", "import", "apps", "serve", "doctor", "purge", "gui",
             "peer", "fixs"),
    "SUMO": ("SUMO",),
    "TrafficLayer": ("TL",),
    "Bridge": ("VCE", "SYNC", "bridge"),
    "App": ("APP",),
}


def tag_of(line, previous=None):
    """The speaker of `line`: its [tag], else the previous line's speaker."""
    m = _TAG.match(line)
    return m.group(1) if m else previous


def level_of(line):
    """'problem', 'warning' or None."""
    if _PROBLEM.search(line):
        return "problem"
    if _WARNING.search(line):
        return "warning"
    return None


def stream_of(tag):
    for label, tags in STREAMS.items():
        if tag in tags:
            return label
    return None


# --------------------------------------------------------------------------- #
# Running
# --------------------------------------------------------------------------- #
class _Job:
    """A Windows Job Object holding one run's whole process tree.

    Needed because a tree kill by parent pid (taskkill /T) cannot reach processes
    whose parent has already exited - and that is exactly the leftover worth
    killing: an engine that died after starting the app leaves the app running,
    parentless, holding the output pipe open. Every process the engine starts
    joins its job automatically (nothing in FIXS asks to break away), so ending the
    job ends all of them, whoever their parent was."""

    def __init__(self, proc):
        import ctypes
        from ctypes import wintypes
        k32 = ctypes.WinDLL("kernel32", use_last_error=True)
        k32.CreateJobObjectW.restype = wintypes.HANDLE
        k32.CreateJobObjectW.argtypes = (wintypes.LPVOID, wintypes.LPCWSTR)
        k32.AssignProcessToJobObject.argtypes = (wintypes.HANDLE, wintypes.HANDLE)
        k32.TerminateJobObject.argtypes = (wintypes.HANDLE, wintypes.UINT)
        k32.CloseHandle.argtypes = (wintypes.HANDLE,)
        self._k32 = k32
        self.handle = k32.CreateJobObjectW(None, None)
        if not self.handle or not k32.AssignProcessToJobObject(
                self.handle, wintypes.HANDLE(int(proc._handle))):
            err = ctypes.get_last_error()
            self.close()
            raise OSError(err, "could not put the run in a job")

    def terminate(self):
        if self.handle:
            self._k32.TerminateJobObject(self.handle, 1)

    def close(self):
        if self.handle:
            self._k32.CloseHandle(self.handle)
            self.handle = None


class Runner:
    """One engine process and everything it starts, its output delivered a line at
    a time.

    The run is over when the ENGINE exits, not when its output pipe closes: the
    pipe stays open as long as any descendant holds it, and a descendant the engine
    failed to stop would otherwise leave the window waiting forever with nothing
    left to stop. Anything still running a moment after the engine has exited is
    such a leftover - the engine stops everything it starts, the CARLA it launched
    included - and is ended with the rest of the tree.

    Callbacks run on worker threads; a UI must hand them to its own thread
    (app.py does it with a queued Qt signal)."""

    LEFTOVER_GRACE_S = 3.0

    def __init__(self, engine=ENGINE, python=None):
        self.engine = engine
        self.python = python or sys.executable
        self.proc = None
        self.argv = None
        self.stop_file = None
        self.stop_requested_at = None
        self.started_at = None
        self._job = None

    @property
    def running(self):
        return self.proc is not None and self.proc.poll() is None

    def command(self, args, stoppable=True):
        """The full argv a run of `args` uses. `stoppable` adds --stop-file, which
        is fresh per run: a stale file would stop the next run before it began."""
        argv = [self.python, self.engine, *args]
        if stoppable:
            fd, path = tempfile.mkstemp(prefix="fixs_stop_", suffix=".flag")
            os.close(fd)
            os.remove(path)          # its APPEARANCE is the request
            self.stop_file = path
            argv += ["--stop-file", path]
        else:
            self.stop_file = None
        return argv

    def start(self, args, on_line, on_exit, stoppable=True):
        if self.running:
            raise RuntimeError("a run is already in progress")
        self.argv = self.command(args, stoppable)
        self.stop_requested_at = None
        kwargs = {}
        if IS_WINDOWS:
            # Own process group; no console window of its own - its output comes
            # here, and the children that have windows (sumo-gui, CARLA) still
            # open them.
            kwargs["creationflags"] = (subprocess.CREATE_NEW_PROCESS_GROUP
                                       | subprocess.CREATE_NO_WINDOW)
        else:
            # Own session: the group outlives its leader, so killpg still reaches
            # what the engine left behind after the engine itself has gone.
            kwargs["start_new_session"] = True
        self.proc = subprocess.Popen(self.argv, stdin=subprocess.DEVNULL,
                                     stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                     cwd=engine_cwd(self.engine), env=child_env(),
                                     **kwargs)
        self._job = None
        if IS_WINDOWS:
            try:
                self._job = _Job(self.proc)
            except OSError as exc:
                on_line(f"[gui] note: could not group this run's processes ({exc}); "
                        f"leftovers from a crash may need Machine > Clean up.")
        self.started_at = time.monotonic()
        proc, read_done = self.proc, threading.Event()
        threading.Thread(target=self._read, args=(proc, on_line, read_done),
                         name="fixs-run-reader", daemon=True).start()
        threading.Thread(target=self._wait, args=(proc, on_line, on_exit, read_done),
                         name="fixs-run-waiter", daemon=True).start()
        return self.argv

    @staticmethod
    def _read(proc, on_line, done):
        try:
            for raw in iter(proc.stdout.readline, b""):
                text = raw.decode("utf-8", errors="replace").rstrip("\r\n")
                # A progress bar redraws with \r; show where it ended up.
                on_line(text.rsplit("\r", 1)[-1])
        except (OSError, ValueError):
            pass
        done.set()

    def _wait(self, proc, on_line, on_exit, read_done):
        rc = proc.wait()
        if not read_done.wait(self.LEFTOVER_GRACE_S):
            on_line("[gui] the run has ended, but processes it started are still "
                    "running; stopping them.")
            self._end_tree(proc)
            if not read_done.wait(10):
                on_line("[gui] some of them did not stop; use Machine > Clean up.")
        if self._job is not None:
            self._job.close()
            self._job = None
        if self.stop_file and os.path.exists(self.stop_file):
            try:
                os.remove(self.stop_file)
            except OSError:
                pass
        on_exit(rc)

    def request_stop(self):
        """Ask the run to shut its stack down, exactly as Ctrl+C would. Returns
        False when there is nothing to ask (not running, or not stoppable)."""
        if not self.running or not self.stop_file:
            return False
        if self.stop_requested_at is None:
            with open(self.stop_file, "w") as f:
                f.write("stop\n")
            self.stop_requested_at = time.monotonic()
        return True

    def kill(self):
        """Last resort: end the whole process tree now. Nothing gets to clean up,
        so anything outside the tree may be left - run_cosim --cleanup sweeps it."""
        if self.proc is not None:
            self._end_tree(self.proc)

    def _end_tree(self, proc):
        if self._job is not None:
            self._job.terminate()
        elif IS_WINDOWS:
            subprocess.call(["taskkill", "/T", "/F", "/PID", str(proc.pid)],
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                            creationflags=subprocess.CREATE_NO_WINDOW)
        else:
            try:
                os.killpg(proc.pid, signal.SIGKILL)
            except (ProcessLookupError, PermissionError):
                pass
        try:
            proc.kill()
        except OSError:
            pass
