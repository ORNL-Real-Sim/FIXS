"""
carla_env_setup.py - one-time (or reconfigure) CARLA environment setup.

Prompts for the CARLA flavour and folder(s), validates them, and saves the choice
to ~/.fixs/carla.json. run_cosim.py reads that config and launches seamlessly; if
no config exists, run_cosim.py invokes this on the first run.

Three flavours:
  packaged  a released build (CarlaUE4.exe / .sh) - stock maps
  source    an Unreal source build - the only one that can cook a custom map
  client    no CARLA on this machine at all; it runs on another host and is
            reached over the network. The traffic stack (SUMO, TrafficLayer,
            VirCarlaEnv) still runs here, so this machine needs the carla PYTHON
            client but no install, no Unreal, and no GPU.

Run this any time to switch CARLA (packaged <-> source build, or a different
install/version):

    python carla_env_setup.py                  # interactive
    python carla_env_setup.py --show           # print the current config
    python carla_env_setup.py --update-python  # rebind the env, keep the CARLA paths

Everything that runs a co-sim runs under the interpreter recorded here - see
reexec_under_configured, which every entry point calls first, so which script you
start with cannot change the env you end up in.

The config is stored per-machine outside any repo, so every FIXS app on this
computer reuses it and it is never git-tracked.
"""
import argparse
import hashlib
import json
import os
import platform
import shutil
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
# the canonical conda spec ships at the FIXS root (one level up from Carla/).
ENV_YML = os.path.normpath(os.path.join(HERE, "..", "environment.yml"))

CONFIG_DIR = os.path.join(os.path.expanduser("~"), ".fixs")
CONFIG_PATH = os.path.join(CONFIG_DIR, "carla.json")

# SUMO-side deps that come from the realsim env regardless of CARLA flavour.
SUMO_MODULES = ("traci", "sumolib")


# ----------------------------------------------------------------- config io

def load_config():
    """Return the saved CARLA env dict, or None if not configured / invalid."""
    try:
        with open(CONFIG_PATH, encoding="utf-8") as f:
            cfg = json.load(f)
    except (OSError, ValueError):
        return None
    mode = cfg.get("mode")
    # 'client' has no local CARLA to point at - that is the whole point of it - so
    # it is the one mode that is complete without a carla_root.
    if mode == "client":
        return cfg
    return cfg if mode in ("packaged", "source") and cfg.get("carla_root") else None


def save_config(cfg):
    os.makedirs(CONFIG_DIR, exist_ok=True)
    with open(CONFIG_PATH, "w", encoding="utf-8") as f:
        json.dump(cfg, f, indent=2)
    print(f"[setup] saved CARLA env -> {CONFIG_PATH}")


# ------------------------------------------------------- the configured python
# carla.json names ONE interpreter, and every FIXS entry point must run under it -
# it is the only env that has the carla client, the SUMO clients and whatever an
# application's requirements.txt added. Which script you happen to start with
# (run_cosim, import_map, place_tls, ...) must not change the answer.
#
# The per-OS wrappers cannot enforce that: they are `exec python <script>`, so they
# run under whatever python is on PATH. So the rule lives here, in the module that
# owns the config, and each entry point calls it as its first act.

REEXEC_GUARD = "FIXS_REEXEC"

# A conda env has no pyvenv.cfg, so CPython leaves ENABLE_USER_SITE on and puts the
# PER-USER site directory (%APPDATA%\Python\PythonXY\site-packages on Windows,
# ~/.local/lib/pythonX.Y/site-packages elsewhere) AHEAD of the env's own
# site-packages. One `pip install --user` therefore shadows an env-installed package
# in every env on the machine at once, and naming the right interpreter here is not
# enough to stop it: a carla built from a source tree landed in the user directory
# and won over the wheel this config installed, so the client spoke a different
# protocol version than the server and died inside libcarla on ImageTmpl.h's
# `GetWidth() * GetHeight() == size()` assertion - a C++ assert, so it took the
# process down instead of raising something python could report. The version banner
# said so ("Client API version = <hash>" vs "Simulator API version = 0.9.15.2") but
# CARLA only warns there and connects anyway.
#
# carla.json names ONE interpreter; that interpreter has to mean one set of packages.
# Two moves, because an env var alone cannot repair a process that has already booted:
#   - export PYTHONNOUSERSITE, so every child - the re-exec below, TrafficLayer, the
#     app's own launch command, the placers - starts without the directory at all;
#   - drop it from THIS process's sys.path, for the case where we are already on the
#     configured interpreter and so never re-exec.

USER_SITE_OPT_OUT = "PYTHONNOUSERSITE"


def quarantine_user_site():
    """Keep per-user site-packages out of this run. Returns the paths dropped.

    Empty on a healthy machine, and empty in a child we re-exec'd, which never
    added the directory - so the caller's notice prints once, where it is news."""
    os.environ[USER_SITE_OPT_OUT] = "1"
    try:
        import site
        user_site = getattr(site, "USER_SITE", None) or site.getusersitepackages()
    except Exception:
        return []          # no usable site module: nothing to quarantine
    if not user_site:
        return []
    target = os.path.normcase(os.path.normpath(user_site))
    dropped = [p for p in sys.path
               if p and os.path.normcase(os.path.normpath(p)) == target]
    for path in dropped:
        sys.path.remove(path)
    return dropped


# Run at import, not from a call each entry point has to remember: this module is the
# first FIXS import in every one of them, and the quarantine has to beat `import carla`
# on ALL paths - including the ones that answer and exit before
# reexec_under_configured. run_cosim --version is the sharp case: its whole job is to
# report which packages a run will use ("what to paste into a bug report"), and it
# returns at the --doctor/--version branch, well above the re-exec - so it was
# fingerprinting the shadowed copy and calling it present.
USER_SITE_DROPPED = quarantine_user_site()
_python_reported = False


def report_python(tag="fixs"):
    """Name the interpreter this run uses - once, and only when something had been
    shadowing it.

    Which directory got dropped is our problem, not the reader's. The question a
    shadowed import makes unanswerable is "which python am I actually getting",
    so answer that and say nothing else. Silent on a machine with no user-site
    install, which is most of them."""
    global _python_reported
    if not USER_SITE_DROPPED or _python_reported:
        return
    _python_reported = True
    print(f"[{tag}] python: {sys.executable}")


def configured_python():
    """The interpreter carla.json names, if it is on disk. Else None."""
    py = (load_config() or {}).get("python")
    return py if py and os.path.isfile(py) else None


def _same_python(a, b):
    return os.path.normcase(os.path.normpath(a)) == os.path.normcase(os.path.normpath(b))


def reexec_under_configured(script, cfg=None, drop=(), tag="fixs"):
    """Re-run `script` under the configured interpreter, if we are not on it already.

    Does NOT return when it switches: the child's exit code becomes ours. Call it
    before importing carla or anything else from the env, and before prompting -
    re-execing after a prompt would ask the same question twice.

    `cfg` lets a caller that has already loaded the config pass it in; `drop` names
    arguments the child must not see again (run_cosim's --reconfigure has already
    been honoured by the time we switch). REEXEC_GUARD stops a config that points
    at a shim or a symlink - where the path comparison cannot tell parent from
    child - from re-execing forever.

    The user-site quarantine already happened at import. Naming the interpreter is
    done here, and only on the paths that RETURN - if we re-exec, sys.executable is
    not the python that ends up running, and the switch line below names the one
    that does."""
    target = (cfg if cfg is not None else load_config() or {}).get("python")
    if not target or not os.path.isfile(target):
        report_python(tag)
        return                       # nothing configured yet, or it has been removed
    if _same_python(target, sys.executable) or os.environ.get(REEXEC_GUARD) == "1":
        report_python(tag)
        return
    print(f"[{tag}] switching to the configured python env:\n        {target}")
    cmd = [target, os.path.abspath(script), *[a for a in sys.argv[1:] if a not in drop]]
    sys.exit(subprocess.call(cmd, env=dict(os.environ, **{REEXEC_GUARD: "1"})))


# ------------------------------------------------------------- path resolving

def packaged_exe(carla_root):
    """The packaged CARLA server executable under carla_root, or None."""
    if platform.system() == "Windows":
        cands = [os.path.join(carla_root, "CarlaUE4.exe"),
                 os.path.join(carla_root, "WindowsNoEditor", "CarlaUE4.exe")]
    else:
        cands = [os.path.join(carla_root, "CarlaUE4.sh"),
                 os.path.join(carla_root, "LinuxNoEditor", "CarlaUE4.sh")]
    return next((c for c in cands if os.path.isfile(c)), None)


def source_paths(carla_root, ue4_root):
    """(uproject, ue4editor) paths for a source build."""
    uproject = os.path.join(carla_root, "Unreal", "CarlaUE4", "CarlaUE4.uproject")
    if platform.system() == "Windows":
        editor = os.path.join(ue4_root, "Engine", "Binaries", "Win64", "UE4Editor.exe")
    else:
        editor = os.path.join(ue4_root, "Engine", "Binaries", "Linux", "UE4Editor")
    return uproject, editor


# Carried by every UE4Editor launch FIXS makes. CARLA's CarlaUE4.uproject enables
# UE4's RenderDocPlugin, and its loader looks for renderdoc.dll in the Engine.ini
# cvar, then in the registry, then - having found neither, which is the case on
# every machine without RenderDoc installed - by ASKING: a modal "Locate main
# RenderDoc executable..." file dialog. It opens at PostConfigInit, long before the
# engine has a window of its own, and is built through COM - which the game thread
# has not initialized that early, so the dialog usually fails to be created and the
# launch is none the wiser. When some module loaded ahead of it did initialize COM,
# the dialog appears and blocks the game thread until a human cancels it: measured
# at 138 s on a run nobody was watching, against the 180 s wait_for_port budget,
# and unbounded on the placers, which give the editor no timeout at all (#311).
# -DisableFrameTraceCapture makes the loader return before it searches anything.
#
# What that gives up is RenderDoc frame capture from a FIXS-launched editor, which
# no co-sim run uses - capturing a frame means driving the editor by hand anyway.
# Source builds only: a packaged CarlaUE4.exe never loads the plugin (its module is
# UncookedOnly), so that path needs nothing.
EDITOR_LAUNCH_FLAGS = ["-DisableFrameTraceCapture"]


# ------------------------------------------------ python interpreter / carla
# The CARLA + SUMO clients live in a conda env (built from environment.yml). The
# env name is NOT fixed (it may be `realsim`, `realsim_dev`, ...), so we resolve
# the interpreter by *capability* - we scan standard conda locations, the system
# interpreters and the current interpreter, then test which one can actually
# import the modules. This is fully generic: it works on any machine / any
# cloner, with a manual picker fallback when auto-detection comes up empty.
#
# System interpreters are offered because on Linux they are frequently the only
# ones that can import traci/sumolib - apt's SUMO packages drop their bindings
# into the system dist-packages - so a box without conda has nothing else to
# bind. They are also exactly the interpreters we must not install into without
# asking: see _interpreter_kind / _confirm_install below.

def _env_python(env_dir):
    if platform.system() == "Windows":
        return os.path.join(env_dir, "python.exe")
    return os.path.join(env_dir, "bin", "python")


def _python_can_import(py_exe, modules):
    """True if py_exe can import every module in `modules`."""
    try:
        r = subprocess.run([py_exe, "-c", "import " + ", ".join(modules)],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=90)
        return r.returncode == 0
    except Exception:
        return False


def _python_tag(py_exe):
    """e.g. 'cp310' for the interpreter's CPython version (best-effort)."""
    try:
        out = subprocess.check_output(
            [py_exe, "-c", "import sys;print('cp%d%d' % sys.version_info[:2])"],
            text=True, stderr=subprocess.DEVNULL, timeout=30).strip()
        return out or None
    except Exception:
        return None


def _conda_roots():
    """Conda/mamba install roots discovered from env vars + the usual locations."""
    roots = []
    for var in ("CONDA_PREFIX", "CONDA_ROOT", "MAMBA_ROOT_PREFIX"):
        if os.environ.get(var):
            # a prefix like .../envs/foo -> also include the install root above it
            roots.append(os.environ[var])
            roots.append(os.path.dirname(os.path.dirname(os.environ[var])))
    home = os.path.expanduser("~")
    roots += [os.path.join(home, n) for n in
              ("miniconda3", "anaconda3", "miniforge3", "mambaforge",
               os.path.join("AppData", "Local", "miniconda3"),
               os.path.join("AppData", "Local", "anaconda3"))]
    seen, out = set(), []
    for r in roots:
        if r and os.path.isdir(r) and os.path.normcase(r) not in seen:
            seen.add(os.path.normcase(r))
            out.append(r)
    return out


def _system_candidates():
    """Non-conda interpreters: whatever `python3` / `python` resolve to on PATH,
    plus the usual absolute locations.

    On Linux these routinely carry traci/sumolib, because that is where apt's
    SUMO packages install them. Scanning conda alone therefore reported 'no env
    with the co-sim deps' on a machine that had a perfectly usable interpreter
    sitting at /usr/bin/python3."""
    cands = [shutil.which(n) for n in ("python3", "python")]
    if platform.system() != "Windows":
        cands += ["/usr/bin/python3", "/usr/local/bin/python3", "/usr/bin/python"]
    # Windows Store alias stubs are 0-byte reparse points that pop open the
    # Store when executed - never hand one to the import probe.
    return [c for c in cands
            if c and "windowsapps" not in c.replace("\\", "/").lower()]


def _python_candidates():
    """Candidate python executables: the current interpreter, every conda env
    under the discovered roots (and each root's base env), then the system
    interpreters.

    Existing files only, deduped by their symlink-resolved target rather than by
    path text: conda ships bin/python -> bin/python3 -> bin/python3.N, so the
    same binary reached under two names used to be offered as two choices."""
    cands = [sys.executable]
    for root in _conda_roots():
        cands.append(_env_python(root))  # base env
        envs = os.path.join(root, "envs")
        if os.path.isdir(envs):
            for name in sorted(os.listdir(envs)):
                cands.append(_env_python(os.path.join(envs, name)))
    cands += _system_candidates()
    seen, out = set(), []
    for c in cands:
        c = os.path.normpath(c)
        if not os.path.isfile(c):
            continue
        key = os.path.normcase(os.path.realpath(c))
        if key not in seen:
            seen.add(key)
            out.append(c)
    return out


def _interpreter_kind(py_exe):
    """(label, shared) for a candidate interpreter: which env it belongs to, and
    whether that env is SHARED - a system python (owned by the OS and its
    package manager) or a conda base env (shared by every other env on the
    machine). Installing into a shared interpreter reaches well beyond FIXS,
    which is why every install path gates on this via _confirm_install."""
    real = os.path.normcase(os.path.realpath(py_exe))
    roots = _conda_roots()
    # Named envs first: <root>/envs/<name> also lives under <root>, so testing
    # the roots first would report every named env as the base env.
    for root in roots:
        envs = os.path.normcase(os.path.realpath(os.path.join(root, "envs"))) + os.sep
        if real.startswith(envs):
            return "conda env '%s'" % real[len(envs):].split(os.sep)[0], False
    for root in roots:
        if real.startswith(os.path.normcase(os.path.realpath(root)) + os.sep):
            return "conda BASE env (%s)" % os.path.basename(os.path.normpath(root)), True
    return "SYSTEM python", True


def _canonical_env_name():
    """The env to create and bind: $FIXS_ENV_NAME, else the name in the shipped
    environment.yml (defaults to 'realsim').

    The override exists so an application repo can put its OWN env in front - one
    built from this same spec, plus whatever its apps need on top - without this
    engine knowing that env exists. FIXS is told the name by its caller or uses its
    own; it never learns an application's. Keeping the app extras out of 'realsim'
    also keeps that env a faithful test of environment.yml, which is what a FIXS
    developer needs it to be."""
    override = (os.environ.get("FIXS_ENV_NAME") or "").strip()
    if override:
        return override
    try:
        with open(ENV_YML, encoding="utf-8") as f:
            for line in f:
                if line.strip().startswith("name:"):
                    return line.split(":", 1)[1].strip()
    except OSError:
        pass
    return "realsim"


def _named_env_python(name):
    """python.exe of a conda env called `name`, searched across roots, or None."""
    for root in _conda_roots():
        py = _env_python(os.path.join(root, "envs", name))
        if os.path.isfile(py):
            return py
    return None


def _find_conda():
    """Locate a conda/mamba executable, or None."""
    for var in ("CONDA_EXE", "MAMBA_EXE"):
        exe = os.environ.get(var)
        if exe and os.path.isfile(exe):
            return exe
    for name in ("conda", "mamba"):
        found = shutil.which(name)
        if found:
            return found
    for root in _conda_roots():
        for sub in (("Scripts", "conda.exe"), ("condabin", "conda.bat"), ("bin", "conda")):
            c = os.path.join(root, *sub)
            if os.path.isfile(c):
                return c
    return None


def _conda_create_env(conda_exe, yml_path, name):
    """Create the env from the spec, named `name`.

    -n is required, not cosmetic: it overrides the spec's own `name:`, which is how
    the same environment.yml can build the engine's env and an application repo's
    without either restating python=, the channels or channel_priority. Without it
    a caller that set FIXS_ENV_NAME got an env called 'realsim' created, then failed
    to find the name it asked for, and fell through to binding something else."""
    cmd = [conda_exe, "env", "create", "-n", name, "-f", yml_path]
    print(f"[setup] {' '.join(cmd)}")
    print("[setup] creating the env can take several minutes ...")
    return subprocess.call(cmd) == 0


def resolve_python():
    """Resolve the interpreter that runs the co-sim, then report the two ways the
    result can differ from what was asked for: a different env than FIXS_ENV_NAME
    named (_warn_if_not_requested), and an env missing co-sim modules
    (_warn_if_incomplete). Both checks live here rather than in the branches below
    so that every path into _resolve_python - canonical env, freshly created env,
    ranked fallback, manual pick - is covered by the same one."""
    py = _resolve_python()
    _warn_if_not_requested(py, _canonical_env_name())
    _warn_if_incomplete(py)
    return py


def ensure_runtime(cfg, force=False):
    """Make sure `cfg` names a usable interpreter, re-resolving it if not.

    Two callers, one job. Automatically: repair a config whose python is gone or
    can no longer import carla - a deleted env, an env rebuilt without the client,
    or a config written by a setup older than env resolution. On demand
    (`force`, i.e. --update-python): rebind the interpreter even when the current
    one works, which is how you move to a newly created 'fixs_applications' or off
    an env you picked by mistake.

    Either way the CARLA and UE4 paths are kept: which CARLA this machine has is a
    separate question from which python drives it, and re-asking it here is how a
    user updating an env ends up re-picking folders they never wanted to change."""
    py = cfg.get("python")
    usable = bool(py) and os.path.isfile(py) and _python_can_import(py, ("carla",))
    if usable and not force:
        return cfg
    if force:
        print(f"[setup] updating the python env (CARLA paths kept).\n"
              f"        current: {py or '(none recorded)'}")
    else:
        print("[setup] saved config has no usable python env (carla not importable); "
              "resolving it now (CARLA paths kept) ...")
    cfg["python"] = resolve_python()
    # .get: 'client' mode has no carla_root by design (CARLA is on another host).
    wheel = ensure_carla(cfg["python"], cfg["mode"], cfg.get("carla_root"))
    if wheel:
        cfg["carla_wheel"] = wheel
    elif force:
        # A rebind that installs nothing must not leave the previous env's wheel
        # behind describing this one.
        cfg.pop("carla_wheel", None)
    save_config(cfg)
    print(f"[setup] python: {cfg['python']}")
    return cfg


def _resolve_python():
    """Resolve the interpreter that runs the co-sim.

    Order:
      1. the canonical env (FIXS_ENV_NAME, else environment.yml's name) if it exists;
      2. else, if conda is available, offer to create it from environment.yml;
      3. else fall back to any conda env that already has the co-sim deps
         (carla + SUMO), then to a manual python picker.
    This keeps the reproducible 'realsim' path primary while staying usable on
    machines that named their env differently."""
    name = _canonical_env_name()

    # 1. canonical env already installed -> good to go.
    py = _named_env_python(name)
    if py:
        print(f"[setup] found the '{name}' env: {py}")
        return py

    # 2. not installed, but conda is here -> offer to create it from the spec.
    conda = _find_conda()
    if conda and os.path.isfile(ENV_YML):
        print(f"[setup] the '{name}' env is not installed (conda found: {conda}).")
        ans = input(f"        create it now from {ENV_YML}? [Y/n]: ").strip().lower()
        if ans in ("", "y", "yes"):
            if _conda_create_env(conda, ENV_YML, name):
                py = _named_env_python(name)
                if py:
                    print(f"[setup] created '{name}': {py}")
                    return py
            else:
                # Say it failed HERE, where the cause is. Left to fall through in
                # silence, the next steps pick some other interpreter and the
                # first evidence of the failure is a co-sim behaving oddly hours
                # later - which is exactly how a box ended up running without
                # pyyaml and blaming a scenario yaml that was correct.
                print(f"[setup] 'conda env create -f {ENV_YML}' FAILED. Its output "
                      f"is above; a solve failure usually names the conflict.")
            print("[setup] env creation did not produce a usable interpreter; "
                  "falling back to detection.")
    elif not conda:
        print("[setup] conda/mamba not found on PATH.")

    # 3. fall back: any interpreter that already imports the co-sim deps - conda
    #    env, conda base or system python - else pick.
    cands = _python_candidates()
    full = [p for p in cands if _python_can_import(p, ("carla",) + SUMO_MODULES)]
    sumo_only = [p for p in cands if p not in full and _python_can_import(p, SUMO_MODULES)]
    ranked = full + sumo_only
    if len(ranked) == 1:
        label, _ = _interpreter_kind(ranked[0])
        print(f"[setup] using python env: {ranked[0]}  [{label}]")
        return ranked[0]
    if len(ranked) > 1:
        print("[setup] found these python envs with the co-sim deps:")
        for i, p in enumerate(ranked):
            deps = "carla+sumo" if p in full else "sumo only"
            label, shared = _interpreter_kind(p)
            note = "  <- shared, not FIXS-private" if shared else ""
            print(f"   [{i}] {p}")
            print(f"       ({deps} | {label}){note}")
        print("       FIXS may pip-install into whichever you pick; it says what, "
              "and asks\n       first before writing into a shared one.")
        sel = input(f"pick 0-{len(ranked) - 1} (default 0): ").strip()
        return ranked[int(sel)] if sel.isdigit() and int(sel) < len(ranked) else ranked[0]

    print("[setup] no python env with the co-sim deps found automatically.")
    return _no_env_fallback(name)


def _warn_if_not_requested(py_exe, name):
    """Say so when the interpreter bound is not the env that was asked for.

    Only reachable when FIXS_ENV_NAME named an env that does not exist yet and the
    fallbacks picked something else - typically the engine's own 'realsim', which
    ranks first because it has carla and the SUMO clients. Left silent, an
    application's extra packages would then be installed into the engine env, which
    is the one thing the override exists to prevent."""
    requested = (os.environ.get("FIXS_ENV_NAME") or "").strip()
    if not requested or not py_exe:
        return
    # The env's NAME is the last component of its root, so compare that, not the
    # tail of the whole path: endswith() also accepted '.../envs/my_fixs_applications'
    # as 'fixs_applications' and stayed silent about a genuinely different env.
    if os.path.normcase(os.path.basename(_env_root(py_exe))) == os.path.normcase(requested):
        return
    print(f"[setup] NOTE: '{requested}' was requested (FIXS_ENV_NAME) but is not what "
          f"got bound:\n        {py_exe}\n"
          f"        Anything an application installs will land there. Create "
          f"'{requested}' with:\n"
          f"            conda env create -n {requested} -f {ENV_YML}")


# Imported by run_cosim/ConfigHelper and the TL-table generator. Missing any of
# them does not stop the run, it degrades it in ways that name something else:
# no yaml parser makes every scenario setting read as its default (including
# CarlaServerIP -> localhost), and no pandas/shapely turns traffic-light sync off.
RUNTIME_MODULES = ("yaml", "pandas", "shapely", "traci", "sumolib")

# The distribution that provides a module, where the two names differ.
PIP_NAME = {"yaml": "pyyaml"}


def missing_runtime(py_exe):
    """Which of RUNTIME_MODULES `py_exe` cannot import."""
    return [m for m in RUNTIME_MODULES if not _python_can_import(py_exe, (m,))]


def _warn_if_incomplete(py_exe):
    """Say which co-sim modules the bound interpreter cannot import.

    The ranked fallback in _resolve_python accepts an env for importing carla and
    the SUMO clients alone, and its 'sumo only' entries do not even have carla - so
    picking [4] from that list can bind an env that is missing yaml, pandas or
    shapely with no comment at all. None of those stop a run; they degrade it in
    ways that name something else (see RUNTIME_MODULES above), and setup is the one
    moment where saying so costs a single line instead of an afternoon.

    carla is deliberately not checked here: ensure_carla installs it right after
    this, so its absence now is expected, not a defect."""
    if not py_exe:
        return
    lacks = missing_runtime(py_exe)
    if not lacks:
        return
    pkgs = " ".join(PIP_NAME.get(m, m) for m in lacks)
    print(f"[setup] NOTE: this interpreter cannot import: {', '.join(lacks)}\n"
          f"        {py_exe}\n"
          f"        The co-sim will still start, and will misbehave in ways that name "
          f"something else: no yaml makes every scenario setting read as its default "
          f"(CarlaServerIP -> localhost), and no pandas/shapely turns traffic-light "
          f"sync off.\n"
          f"        Fix it with:\n"
          f"            \"{py_exe}\" -m pip install {pkgs}")


# Where an application's applied-dependency stamp lives, relative to the env root.
APP_DEPS_STAMP_DIR = ".fixs_app_deps"


def _env_root(py_exe):
    """The env directory holding py_exe (<env>\\python.exe, or <env>/bin/python)."""
    d = os.path.dirname(os.path.abspath(py_exe))
    if os.path.basename(d).lower() in ("bin", "scripts"):
        d = os.path.dirname(d)
    return d


def ensure_app_deps(py_exe, app_id, req_path, refresh=False):
    """Install an application's extra packages into the interpreter that will run it.

    environment.yml deliberately does not carry them - they belong to the
    application, not to FIXS, and pushing them upstream would put every consumer's
    engine env at the mercy of one app's plotting stack. An app declares its own
    with a 'requirements' path in apps.json; an app that declares none costs
    nothing here.

    The stamp is written INSIDE the env, not into ~/.fixs. That is the whole point:
    recreating the env destroys the packages AND the stamp together, so the next run
    reinstalls. A stamp kept outside would still match a hash it no longer describes,
    and the deps would be skipped silently - which is the failure this is meant to
    avoid, not cause.

    Returns True when the interpreter has the app's declared deps."""
    if not (py_exe and app_id and req_path):
        return True
    if not os.path.isfile(req_path):
        print(f"[setup] app '{app_id}' declares requirements '{req_path}', "
              f"which does not exist - skipping.")
        return True
    try:
        with open(req_path, "rb") as f:
            digest = hashlib.sha256(f.read()).hexdigest()
    except OSError as e:
        print(f"[setup] cannot read {req_path}: {e}")
        return False

    stamp = os.path.join(_env_root(py_exe), APP_DEPS_STAMP_DIR, app_id)
    if not refresh:
        try:
            with open(stamp, encoding="utf-8") as f:
                if f.read().strip() == digest:
                    return True          # unchanged since the last apply
        except OSError:
            pass                          # no stamp, or unreadable -> apply

    if not _confirm_install(py_exe, f"'{app_id}' dependencies "
                                    f"(-r {os.path.basename(req_path)})"):
        print(f"[setup] '{app_id}' dependencies not installed; the app will run "
              f"without them.\n"
              f"        Install them yourself with:\n"
              f"            \"{py_exe}\" -m pip install -r \"{req_path}\"")
        return False
    print(f"[setup] applying '{app_id}' dependencies "
          f"({os.path.basename(req_path)}) to {py_exe} ...")
    rc = subprocess.call([py_exe, "-m", "pip", "install", "-r", req_path])
    if rc != 0:
        # Loud and specific: the alternative is an ImportError minutes into a run,
        # naming a module rather than the app whose requirements never applied.
        print(f"[setup] FAILED to install '{app_id}' dependencies (pip exit {rc}).\n"
              f"        Install them by hand, or re-run with --refresh-deps:\n"
              f"            \"{py_exe}\" -m pip install -r \"{req_path}\"")
        return False
    try:
        os.makedirs(os.path.dirname(stamp), exist_ok=True)
        with open(stamp, "w", encoding="utf-8") as f:
            f.write(digest + "\n")
    except OSError as e:
        # The install SUCCEEDED, so the run is fine - but say this out loud rather
        # than swallow it. An unwritable env root (a system-wide python, a shared
        # env) means the stamp never persists and pip is re-run on every single
        # launch. Silently that reads as "this is just slow to start".
        print(f"[setup] note: could not record the applied-deps stamp ({e}).\n"
              f"        '{app_id}' deps are installed, but this check will re-run "
              f"pip on every launch. A writable env - the one `--setup` creates - "
              f"avoids it.")
    return True


def _no_env_fallback(name):
    """Nothing suitable was found. Ask rather than pick something broken.

    Silently continuing under whatever interpreter happened to be current is how
    a machine ends up running the co-sim without pyyaml: everything starts, and
    the first symptom is a scenario setting quietly reading as its default."""
    print(f"[setup] the '{name}' env could not be created or found. The co-sim "
          f"needs: {', '.join(RUNTIME_MODULES)} (+ carla).")
    here = sys.executable
    lacks = missing_runtime(here)
    print(f"   [1] use this interpreter and pip-install what it lacks\n"
          f"       {here}\n"
          f"       missing: {', '.join(lacks) if lacks else 'nothing'}")
    print( "   [2] select a python.exe yourself")
    print( "   [3] quit and fix conda first")
    ans = (input("Enter 1, 2 or 3 (default 3): ").strip() or "3")
    if ans == "1":
        if lacks:
            # environment.yml is a conda spec, so there is no conda-free way to
            # replay it; these are its importable dependencies. NB it does not
            # list shapely at all, though the TL-table generator needs it (#221).
            pkgs = ["pyyaml", "pandas", "shapely", "eclipse-sumo", "traci", "sumolib"]
            # This is the riskiest install in the file: reached precisely when
            # conda is absent or broken, which is when `here` is most likely to
            # BE the OS python.
            if not _confirm_install(here, " ".join(pkgs)):
                sys.exit("[setup] nothing installed; create a dedicated env with:\n"
                         f"            conda env create -f {ENV_YML}")
            if not _pip_install(here, pkgs):
                sys.exit("[setup] pip install failed; fix the environment by hand.")
            still = missing_runtime(here)
            if still:
                sys.exit(f"[setup] still missing after install: {', '.join(still)}")
        return here
    if ans == "2":
        py = _pick_file(f"Select the python.exe of your '{name}' env (carla + SUMO)")
        if not py or not os.path.isfile(py):
            sys.exit("[setup] no python interpreter selected.")
        lacks = missing_runtime(py)
        if lacks:
            print(f"[setup] warning: {py} cannot import {', '.join(lacks)}; the "
                  f"co-sim will misbehave until that is fixed.")
        return py
    sys.exit(f"[setup] stopped. Create the env with:\n"
             f"            conda env create -f {ENV_YML}\n"
             f"        then re-run this setup.")


def find_source_wheel(carla_root, py_exe=None):
    """Auto-resolve the source build's carla wheel under PythonAPI/carla/dist,
    preferring one matching the interpreter's CPython tag. Returns a path or None."""
    dist = os.path.join(carla_root, "PythonAPI", "carla", "dist")
    if not os.path.isdir(dist):
        return None
    wheels = [os.path.join(dist, f) for f in os.listdir(dist) if f.endswith(".whl")]
    if not wheels:
        return None
    tag = _python_tag(py_exe) if py_exe else None
    if tag:
        tagged = [w for w in wheels if tag in os.path.basename(w)]
        if tagged:
            wheels = tagged
    return sorted(wheels)[-1]  # newest by name


def _pip_install(py_exe, args):
    cmd = [py_exe, "-m", "pip", "install", *args]
    print(f"[setup] {' '.join(cmd)}")
    return subprocess.call(cmd) == 0


def _confirm_install(py_exe, what, question=None):
    """Say what FIXS is about to install and into which interpreter; return True
    to go ahead.

    A FIXS-private conda env is written to without asking - that env exists for
    this. A SHARED interpreter is gated behind an explicit yes, because the
    install does not stay inside FIXS: on a system python it can replace
    packages the OS itself imports (and Debian/Ubuntu pip refuses outright under
    PEP 668, externally-managed-environment), and on a conda base env it changes
    the versions every other env on the machine solves against. Now that the
    ranked list offers system interpreters, this is the difference between
    'FIXS bound /usr/bin/python3' and 'FIXS rewrote /usr/bin/python3'.

    Pass `question` to ask whatever the interpreter."""
    label, shared = _interpreter_kind(py_exe)
    print(f"[setup] about to install: {what}")
    print(f"        into:             {py_exe}  [{label}]")
    if shared:
        print("        WARNING: that is not a FIXS-private env.")
        if label.startswith("SYSTEM"):
            print("          It belongs to the operating system. Installing here can")
            print("          overwrite packages your OS tools import, and on Debian/Ubuntu")
            print("          pip may refuse it (externally-managed-environment).")
        else:
            print("          It is the conda base env, shared by every other env and")
            print("          project on this machine; this can break their pinned versions.")
        print(f"        Safer: build the FIXS env instead ->")
        print(f"            conda env create -f {ENV_YML}")
    if question is None:
        if not shared:
            return True
        question = "install there anyway? [y/N]: "
    try:
        return input("        " + question).strip().lower() in ("y", "yes")
    except EOFError:
        # No console to answer on (piped run, CI). Declining is the only safe
        # default: the whole point is that this install is not reversible by
        # deleting an env.
        print("        (no console to confirm on - not installing.)")
        return False


_VERSION_PROBE = """\
import carla
v = ""
try:
    from importlib.metadata import version
    v = version("carla")
except Exception:
    v = getattr(carla, "__version__", "")
print(v)
"""


def _carla_version(py_exe):
    """The carla client version importable under py_exe, or '?'.

    Asked via importlib.metadata (stdlib since 3.8), not pkg_resources: the latter
    ships with setuptools, which a conda env is under no obligation to have and
    which setuptools>=81 deprecates outright. Probing for it made any env without
    it - i.e. any env that is not the one environment.yml built - dump a
    ModuleNotFoundError traceback into the middle of setup and then report the
    version as '?', which reads as a broken carla install when nothing is wrong.

    stderr is discarded for the same reason: this is a probe whose failure is
    already expressed by the return value, so its noise has no reader."""
    try:
        out = subprocess.check_output([py_exe, "-c", _VERSION_PROBE],
                                      text=True, stderr=subprocess.DEVNULL,
                                      timeout=30).strip()
        return out or "?"
    except Exception:
        return "?"


def ensure_carla(py_exe, mode, carla_root=None):
    """Make `import carla` work under py_exe, with the client matched to the
    chosen CARLA: PyPI wheel for packaged and client, the source build's wheel
    for source.

    'client' takes the PyPI wheel because there is no local build to take one
    from. run_cosim still needs `import carla` on this machine - it is what
    drives load_world, the readiness check and the spectator against the remote
    server - so the wheel is required even though nothing here ever launches
    CARLA. If that remote server is a source build with a patched PythonAPI,
    the version handshake is what catches the mismatch, not this."""
    has_carla = _python_can_import(py_exe, ("carla",))

    if mode in ("packaged", "client"):
        if has_carla:
            print(f"[setup] carla {_carla_version(py_exe)} already importable.")
            return
        print("[setup] carla missing in this env.")
        if not _confirm_install(py_exe, "carla==0.9.15 (PyPI wheel, with its deps)"):
            sys.exit("[setup] carla not installed; re-run and bind a dedicated env "
                     "(--update-python).")
        if not _pip_install(py_exe, ["carla==0.9.15"]):
            sys.exit("[setup] pip install carla==0.9.15 failed.")
        return None

    # source: client should match the custom server -> install the build's wheel
    wheel = find_source_wheel(carla_root, py_exe)
    if has_carla:
        print(f"[setup] carla {_carla_version(py_exe)} already importable.")
        if wheel:
            ok = _confirm_install(
                py_exe,
                f"{os.path.basename(wheel)} (this source build's wheel, "
                f"--force-reinstall --no-deps)",
                "reinstall it to guarantee client/server match? [y/N]: ")
            if not ok:
                # Return nothing: the caller records what came back as the config's
                # carla_wheel, i.e. as the wheel this env is running. Naming one that
                # was declined would describe an install that never happened.
                print(f"[setup] keeping the carla already in this env; "
                      f"{os.path.basename(wheel)} not installed.")
                return None
            if not _pip_install(py_exe, ["--force-reinstall", "--no-deps", wheel]):
                sys.exit("[setup] wheel reinstall failed.")
        return wheel
    # carla not importable -> must install the source wheel
    if not wheel:
        print("[setup] no wheel auto-found under "
              f"{os.path.join(carla_root, 'PythonAPI', 'carla', 'dist')}.")
        wheel = _pick_file("Select the source build's carla wheel (PythonAPI/carla/dist/*.whl)")
    if not wheel or not os.path.isfile(wheel):
        sys.exit("[setup] no carla wheel available; build CARLA's PythonAPI first "
                 "(make PythonAPI) or select the wheel manually.")
    if not _confirm_install(py_exe, f"{os.path.basename(wheel)} (this source "
                                    f"build's wheel, --no-deps)"):
        sys.exit("[setup] carla not installed; re-run and bind a dedicated env "
                 "(--update-python).")
    print(f"[setup] installing source carla wheel: {wheel}")
    if not _pip_install(py_exe, ["--no-deps", wheel]):
        sys.exit("[setup] wheel install failed.")
    return wheel


# ----------------------------------------------------------------- prompting

def _pick_dir(title):
    """Native file-explorer folder picker; falls back to a typed path."""
    try:
        import tkinter as tk
        from tkinter import filedialog
        root = tk.Tk()
        root.withdraw()
        root.update()
        path = filedialog.askdirectory(title=title)
        root.destroy()
        if path:
            return path
    except Exception as exc:  # no display / no tkinter
        print(f"[setup] folder picker unavailable ({exc}); type the path instead.")
    typed = input(f"{title}\n  path: ").strip().strip('"')
    return typed or None


def _pick_file(title):
    """Native file-explorer file picker; falls back to a typed path."""
    try:
        import tkinter as tk
        from tkinter import filedialog
        root = tk.Tk()
        root.withdraw()
        root.update()
        path = filedialog.askopenfilename(title=title)
        root.destroy()
        if path:
            return path
    except Exception as exc:  # no display / no tkinter
        print(f"[setup] file picker unavailable ({exc}); type the path instead.")
    typed = input(f"{title}\n  path: ").strip().strip('"')
    return typed or None


def run_setup(allow_packaged_windows=False):
    """Interactive setup; writes and returns the config."""
    print("=== CARLA environment setup ===")
    # On Windows, importing a *custom* map into a packaged CARLA is unsupported
    # by CARLA itself (map ingestion is Linux + Docker only - see
    # Util/ImportAssets.sh; there is no ImportAssets.bat). Custom-map apps on
    # Windows therefore need a source build. We skip the packaged option here to
    # avoid a dead end; pass allow_packaged_windows=True (--allow-packaged-windows)
    # if you only need stock maps (Town01, ...) from a packaged build.
    #
    # [3] client is offered EVERYWHERE, Windows included: the reasoning above is
    # about importing a map, and a client machine never imports one - the host
    # running CARLA does. It is how a workstation with no CARLA at all drives a
    # remote one.
    offer_packaged = platform.system() != "Windows" or allow_packaged_windows
    print("Which CARLA do you want to use?")
    if offer_packaged:
        print("  [1] Packaged CARLA  (a released build with CarlaUE4.exe / CarlaUE4.sh)")
    print("  [2] Source build    (run through the Unreal editor: UE4Editor -game)")
    print("  [3] None on this machine - CARLA runs on another host")
    print("      (SUMO + TrafficLayer + VirCarlaEnv run here; CARLA is reached over")
    print("       the network at CarlaSetup.CarlaServerIP)")
    if not offer_packaged:
        print("  (packaged is not offered on Windows: custom-map import is Linux+Docker")
        print("   only in CARLA. Only need stock maps? re-run with --allow-packaged-windows)")
    valid = ("1", "2", "3") if offer_packaged else ("2", "3")
    choice = input(f"Enter {' or '.join(valid)}: ").strip()
    if choice not in valid:
        sys.exit(f"[setup] invalid choice (expected {' or '.join(valid)}).")

    if choice == "1":
        root = _pick_dir("Select your PACKAGED CARLA folder (contains CarlaUE4.exe / .sh)")
        if not root:
            sys.exit("[setup] cancelled.")
        if not packaged_exe(root):
            sys.exit(f"[setup] no CarlaUE4 launcher found under {root}.")
        cfg = {"mode": "packaged", "carla_root": root}

    elif choice == "2":
        root = _pick_dir("Select your CARLA SOURCE folder (contains Unreal/CarlaUE4/CarlaUE4.uproject)")
        if not root:
            sys.exit("[setup] cancelled.")
        # Prefer $UE4_ROOT, but only if it actually contains the editor; otherwise
        # (unset OR wrong) fall back to the folder picker.
        ue4 = os.environ.get("UE4_ROOT")
        if ue4 and os.path.isfile(source_paths(root, ue4)[1]):
            print(f"[setup] using UE4_ROOT from environment: {ue4}")
        else:
            if ue4:
                print(f"[setup] UE4_ROOT={ue4} has no UE4Editor; please select it.")
            ue4 = _pick_dir("Select your Unreal Engine root (folder containing Engine/)")
        if not ue4:
            sys.exit("[setup] cancelled.")
        uproject, editor = source_paths(root, ue4)
        if not os.path.isfile(uproject):
            sys.exit(f"[setup] no CarlaUE4.uproject at {uproject}.")
        if not os.path.isfile(editor):
            sys.exit(f"[setup] no UE4Editor at {editor} (is this the engine root?).")
        # The uproject and the editor say this build can RUN. Cooking a map runs
        # CARLA's own Util/BuildTools/Import.py, which a partial checkout (or a
        # packaged tree that happens to carry a uproject) may not have - and
        # import_map only discovers that at cook time, several minutes and one
        # download later. Say it here instead. Not fatal: everything except the
        # cook works without it.
        import_py = os.path.join(root, "Util", "BuildTools", "Import.py")
        if not os.path.isfile(import_py):
            print(f"[setup] NOTE: {import_py} is missing, so this build cannot cook a\n"
                  f"        map (import_map will stop there). Running stock or "
                  f"already-cooked maps is unaffected.")
        cfg = {"mode": "source", "carla_root": root, "ue4_root": ue4}

    else:   # choice == "3"
        # No carla_root and no ue4_root on purpose: there is no local install to
        # validate, and inventing one would only give the cook/launch paths
        # something to half-succeed against. The server address is NOT stored
        # here either - it lives in the scenario yaml (CarlaSetup.CarlaServerIP),
        # which is already the one place every component reads it from.
        print("[setup] client mode: no CARLA on this machine. run_cosim will not")
        print("        launch or cook anything here; point CarlaSetup.CarlaServerIP")
        print("        at the host running CARLA, which must already have the map")
        print("        cooked with traffic lights and signs placed.")
        cfg = {"mode": "client"}

    # Resolve the interpreter (carla + SUMO) and match the carla client to the
    # chosen CARLA. Stored in the config so run_cosim re-execs under it on any
    # machine, regardless of the env's name.
    print("\n--- resolving the python env (carla + SUMO client) ---")
    cfg["python"] = resolve_python()
    wheel = ensure_carla(cfg["python"], cfg["mode"], cfg.get("carla_root"))
    if wheel:
        cfg["carla_wheel"] = wheel

    save_config(cfg)
    where = cfg.get("carla_root") or "on another host (see CarlaSetup.CarlaServerIP)"
    print(f"\n[setup] done: {cfg['mode']} CARLA @ {where}")
    print(f"[setup] python: {cfg['python']}")
    return cfg


def update_python():
    """--update-python: rebind the interpreter, keeping the CARLA choice.

    Deliberately not a full re-setup: the common reason to want this - the
    canonical env did not exist when setup ran, so the fallback bound a different
    one - has nothing to do with which CARLA is installed, and making the user
    re-pick their CARLA and UE4 folders to fix it is how a working config gets
    broken by an unrelated typo."""
    cfg = load_config()
    if cfg is None:
        print(f"[setup] nothing to update: no CARLA env configured ({CONFIG_PATH}).\n"
              f"        Run setup first - it resolves the python env as its last step.")
        return 1
    ensure_runtime(cfg, force=True)
    return 0


def main():
    ap = argparse.ArgumentParser(description="Configure which CARLA run_cosim.py uses.")
    ap.add_argument("--show", action="store_true", help="print the current config and exit")
    ap.add_argument("--update-python", action="store_true",
                    help="re-resolve ONLY the python env (carla + SUMO client) and save "
                         "it; the CARLA / UE4 paths are kept. Use after creating the env "
                         "setup asked for, or to move off one you picked by mistake")
    ap.add_argument("--allow-packaged-windows", action="store_true",
                    help="on Windows, also offer packaged CARLA (stock maps only; "
                         "custom-map import is unsupported in Windows packages)")
    args = ap.parse_args()
    if args.show:
        cfg = load_config()
        print(json.dumps(cfg, indent=2) if cfg else f"(no config at {CONFIG_PATH})")
        return 0
    if args.update_python:
        return update_python()
    run_setup(allow_packaged_windows=args.allow_packaged_windows)
    return 0


if __name__ == "__main__":
    sys.exit(main())
