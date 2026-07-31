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

    python carla_env_setup.py            # interactive
    python carla_env_setup.py --show     # print the current config
    setup_carla.bat / setup_carla.sh     # thin per-OS wrappers

The config is stored per-machine outside any repo, so every FIXS app on this
computer reuses it and it is never git-tracked.
"""
import argparse
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


# ------------------------------------------------ python interpreter / carla
# The CARLA + SUMO clients live in a conda env (built from environment.yml). The
# env name is NOT fixed (it may be `realsim`, `realsim_dev`, ...), so we resolve
# the interpreter by *capability* - we scan standard conda locations and the
# current interpreter, then test which one can actually import the modules. This
# is fully generic: it works on any machine / any cloner, with a manual picker
# fallback when auto-detection comes up empty.

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
            text=True, timeout=30).strip()
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


def _conda_candidates():
    """Candidate python executables: the current interpreter + every conda env
    under the discovered roots (and each root's base env). Existing files only."""
    cands = [sys.executable]
    for root in _conda_roots():
        cands.append(_env_python(root))  # base env
        envs = os.path.join(root, "envs")
        if os.path.isdir(envs):
            for name in sorted(os.listdir(envs)):
                cands.append(_env_python(os.path.join(envs, name)))
    seen, out = set(), []
    for c in cands:
        c = os.path.normpath(c)
        key = os.path.normcase(c)
        if key not in seen and os.path.isfile(c):
            seen.add(key)
            out.append(c)
    return out


def _canonical_env_name():
    """The env name from the shipped environment.yml (defaults to 'realsim')."""
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


def _conda_create_env(conda_exe, yml_path):
    cmd = [conda_exe, "env", "create", "-f", yml_path]
    print(f"[setup] {' '.join(cmd)}")
    print("[setup] creating the env can take several minutes ...")
    return subprocess.call(cmd) == 0


def resolve_python():
    """Resolve the interpreter that runs the co-sim.

    Order:
      1. the canonical env named in environment.yml ('realsim') if it exists;
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
            if _conda_create_env(conda, ENV_YML):
                py = _named_env_python(name)
                if py:
                    print(f"[setup] created '{name}': {py}")
                    return py
            print("[setup] env creation did not produce a usable interpreter; "
                  "falling back to detection.")
    elif not conda:
        print("[setup] conda/mamba not found on PATH.")

    # 3. fall back: any env that already imports the co-sim deps, else pick.
    cands = _conda_candidates()
    full = [p for p in cands if _python_can_import(p, ("carla",) + SUMO_MODULES)]
    sumo_only = [p for p in cands if p not in full and _python_can_import(p, SUMO_MODULES)]
    ranked = full + sumo_only
    if len(ranked) == 1:
        print(f"[setup] using python env: {ranked[0]}")
        return ranked[0]
    if len(ranked) > 1:
        print("[setup] found these python envs with the co-sim deps:")
        for i, p in enumerate(ranked):
            tag = " (carla+sumo)" if p in full else " (sumo only)"
            print(f"   [{i}] {p}{tag}")
        sel = input(f"pick 0-{len(ranked) - 1} (default 0): ").strip()
        return ranked[int(sel)] if sel.isdigit() and int(sel) < len(ranked) else ranked[0]

    print("[setup] no conda env with the co-sim deps found automatically.")
    py = _pick_file(f"Select the python.exe of your '{name}' env (has carla + SUMO)")
    if not py or not os.path.isfile(py):
        sys.exit("[setup] no python interpreter selected.")
    return py


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


def _carla_version(py_exe):
    try:
        return subprocess.check_output(
            [py_exe, "-c", "import carla,pkg_resources;"
                           "print(pkg_resources.get_distribution('carla').version)"],
            text=True, timeout=30).strip()
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
        print("[setup] carla missing in this env; installing carla==0.9.15 (PyPI) ...")
        if not _pip_install(py_exe, ["carla==0.9.15"]):
            sys.exit("[setup] pip install carla==0.9.15 failed.")
        return None

    # source: client should match the custom server -> install the build's wheel
    wheel = find_source_wheel(carla_root, py_exe)
    if has_carla:
        print(f"[setup] carla {_carla_version(py_exe)} already importable.")
        if wheel:
            ans = input(f"[setup] reinstall carla from this source build's wheel to guarantee\n"
                        f"        client/server match? {os.path.basename(wheel)} [y/N]: ").strip().lower()
            if ans == "y" and not _pip_install(py_exe, ["--force-reinstall", "--no-deps", wheel]):
                sys.exit("[setup] wheel reinstall failed.")
        return wheel
    # carla not importable -> must install the source wheel
    if not wheel:
        print(f"[setup] no wheel auto-found under {carla_root}\\PythonAPI\\carla\\dist.")
        wheel = _pick_file("Select the source build's carla wheel (PythonAPI/carla/dist/*.whl)")
    if not wheel or not os.path.isfile(wheel):
        sys.exit("[setup] no carla wheel available; build CARLA's PythonAPI first "
                 "(make PythonAPI) or select the wheel manually.")
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


def main():
    ap = argparse.ArgumentParser(description="Configure which CARLA run_cosim.py uses.")
    ap.add_argument("--show", action="store_true", help="print the current config and exit")
    ap.add_argument("--allow-packaged-windows", action="store_true",
                    help="on Windows, also offer packaged CARLA (stock maps only; "
                         "custom-map import is unsupported in Windows packages)")
    args = ap.parse_args()
    if args.show:
        cfg = load_config()
        print(json.dumps(cfg, indent=2) if cfg else f"(no config at {CONFIG_PATH})")
        return 0
    run_setup(allow_packaged_windows=args.allow_packaged_windows)
    return 0


if __name__ == "__main__":
    sys.exit(main())
