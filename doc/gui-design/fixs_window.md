# The FIXS window (`FIXS.bat --gui`)

One window for running a co-sim and, next, for setting its scenario up (#78).
Two tabs are working today (**Run** and **Machine**). The third, **Scenario**, is
where the yaml editor designed in [wireframes.md](wireframes.md) (#121, #142) will
go.

```
FIXS.bat --gui          (Windows)      ./FIXS.sh --gui          (Linux)
run_cosim.py --gui      (from a source checkout: scripts/cosim/run_cosim.py --gui)
```

Double-clicking `FIXS.bat` (or running `./FIXS.sh` with no arguments from a
terminal) prints the common options and asks for them once. Type `gui` there to
open the window.

## How it is built: the engine is the backend

The window does not implement co-simulation. Each button starts `run_cosim` with
flags, the same command a terminal user would type, and the Run tab shows that
command before it runs. The engine therefore behaves identically from a terminal
and from the window, and nothing has to be kept in sync between the two.

| Layer | Where | What it owns |
|---|---|---|
| Qt widgets | `scripts/cosim/gui/app.py` | layout, and turning the form into a request |
| toolkit-free core | `scripts/cosim/gui/core.py` | form -> flags, starting / reading / stopping the child, log tagging |
| engine | `scripts/cosim/run_cosim.py` | everything a run does |

The Scenario editor will use the same split. It will load and save yaml through
`CommonLib/ConfigHelper.py` and write back with comments preserved, so it never
duplicates the engine's defaults.

## Engine hooks the window relies on

These are ordinary `run_cosim` flags. Scripts can use them too.

| Flag | What it does |
|---|---|
| `--list apps\|maps\|setups` | the declared apps (with the staged path of each scenario yaml), the maps on this machine merged with the map library, the saved run setups |
| `--json` | with `--list`, `--doctor`, `--version`: one JSON document on stdout; anything else printed on the way goes to stderr |
| `--stop-file PATH` | stops the run as Ctrl+C would, as soon as `PATH` exists (env `FIXS_STOP_FILE`) |
| `--setup --carla-mode packaged\|source\|client` | CARLA setup with no prompts: `--carla-root`, `--ue4-root`, `--env-python` |
| `--app-args "..."` | arguments for the application, passed to it as `COSIM_APP_ARGS` (the contract FIXS_Applications' `run_cosim.bat` already uses) |

**Why a stop file and not a signal.** Every teardown in a run (SUMO,
TrafficLayer, the bridge, the app, CARLA) sits in a `finally` block, and only
Ctrl+C unwinds those. On Windows a GUI parent can't deliver Ctrl+C to a child,
and `CTRL_BREAK` kills Python without running any `finally`. The stop file
becomes a KeyboardInterrupt in the run's main thread instead, so it goes through
exactly the Ctrl+C path. Measured on a SUMO + TrafficLayer stack: stopped in
1.5 s, exit 130, no process left behind.

If a run doesn't stop within 20 s (it is blocked in one long subprocess call, such
as a cook or the standalone bridge), **Stop** turns into **Force stop**, which ends
the whole process tree. **Machine > Clean up** (`--cleanup`) sweeps up anything
left over.

## Rules for the child process

- **stdin is closed.** The engine uses `stdin.isatty()` to decide whether it may
  ask a human, and every prompt answers EOF with a default or a clean exit. An
  open pipe that nobody writes to would park an unguarded `input()` forever.
- **Own process group.** Force stop uses it to take the whole tree.
- **`FIXS_REEXEC` is not passed on.** Each run decides for itself whether to
  switch to the configured python, as it does from a terminal.

## Installing the toolkit

The window needs PySide6 (Qt for Python). `environment.yml` includes
`PySide6-Essentials`. For an env built before that line, the first `--gui`
installs it:

1. The python that ran `--gui` already has PySide6: the window opens there.
2. Else, the env in `~/.fixs/carla.json` has it: the window opens there.
3. Else, it installs `PySide6-Essentials` into that env and opens there. This
   goes through `env_setup`'s usual install rule: a FIXS conda env is written to
   without asking, a shared interpreter only after a yes.

**Windows + conda: ICU.** Qt6Core imports ICU by its unversioned names from
Windows' own `icuuc.dll`. A conda env that has the `icu` package (libxml2 pulls it
in) puts ICU 75 on the DLL path, and that copy exports only versioned names.
Importing Qt then fails with *"DLL load failed ... The specified procedure could
not be found"*. `gui.prefer_system_icu()` loads the System32 copy first. It
affects only the window's own process.

**Linux.** Qt 6.5+ needs `libxcb-cursor0` from the system
(`sudo apt install libxcb-cursor0`). Without it Qt reports that it *could not
load the Qt platform plugin "xcb"*. A headless host (a CARLA render box over SSH)
doesn't need the window; it keeps using `--serve` / `--peer`.

## Tests

- `tests/Python/unit/test_frontend_hooks.py`: the engine flags above.
- `tests/Python/unit/test_gui_core.py`: flags, log tagging, and stop / force-stop
  against real child processes.
- `tests/Python/unit/test_gui_window.py`: the window against a canned backend,
  offscreen. Skipped when PySide6 isn't installed.
