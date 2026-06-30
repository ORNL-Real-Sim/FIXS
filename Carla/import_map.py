"""
import_map.py - import a RoadRunner/OpenDRIVE map into a *source-build* CARLA.

Generalizes the proven one-click import (run CARLA's
`Util/BuildTools/Import.py --package=<name>` directly, with UE4_ROOT set) into a
shippable, parameterized helper. Used by run_cosim's --auto-import and by the
per-app one-click import_*.bat / .sh wrappers.

A map "package" is the `<name>.json` descriptor plus its fbx/xodr/fbm asset
folder. This helper makes sure that package is staged under <carla_root>/Import,
then runs the cook. The package is sourced, in order of preference, from:
  --package-dir <dir>   a local folder holding the package files, or
  --package-url <url>   a zip to download (e.g. a GitHub release asset), or
  already staged        files already present under <carla_root>/Import.

Importing requires a CARLA **source build** (the cook runs the Unreal editor);
packaged CARLA cannot import custom maps. carla_root / ue4_root default to the
saved env config (~/.fixs/carla.json) written by carla_env_setup.py.

Examples:
  python import_map.py --package RP_Ver0529 --package-dir C:/src_ext/Carla/Import
  python import_map.py --package RP_Ver0529 --package-url https://.../RP_Ver0529_import.zip
"""
import argparse
import os
import shutil
import subprocess
import sys
import tempfile
import urllib.request
import zipfile

import carla_env_setup as env


def cooked_map_path(carla_root, name):
    """Where the cooked .umap lands after a successful import."""
    return os.path.join(carla_root, "Unreal", "CarlaUE4", "Content",
                        name, "Maps", name, name + ".umap")


def map_is_imported(carla_root, name):
    return os.path.isfile(cooked_map_path(carla_root, name))


def _descriptor(carla_root, name):
    return os.path.join(carla_root, "Import", name + ".json")


def stage_package(carla_root, name, package_url=None, package_dir=None):
    """Ensure <carla_root>/Import has <name>.json (+ its assets). No-op if the
    descriptor is already present and no explicit source was given."""
    import_dir = os.path.join(carla_root, "Import")
    descriptor = _descriptor(carla_root, name)
    if os.path.isfile(descriptor) and not package_url and not package_dir:
        print(f"[import] package already staged: {descriptor}")
        return import_dir

    os.makedirs(import_dir, exist_ok=True)
    if package_dir:
        if not os.path.isdir(package_dir):
            sys.exit(f"[import] --package-dir not found: {package_dir}")
        print(f"[import] copying package from {package_dir} -> {import_dir}")
        for item in os.listdir(package_dir):
            src = os.path.join(package_dir, item)
            dst = os.path.join(import_dir, item)
            if os.path.isdir(src):
                shutil.copytree(src, dst, dirs_exist_ok=True)
            else:
                shutil.copy2(src, dst)
    elif package_url:
        tmp = tempfile.mkdtemp(prefix="fixs-map-")
        try:
            zpath = os.path.join(tmp, "package.zip")
            print(f"[import] downloading {package_url}")
            urllib.request.urlretrieve(package_url, zpath)
            print(f"[import] extracting into {import_dir}")
            with zipfile.ZipFile(zpath) as z:
                z.extractall(import_dir)
        finally:
            shutil.rmtree(tmp, ignore_errors=True)

    if not os.path.isfile(descriptor):
        sys.exit(f"[import] after staging, descriptor still missing: {descriptor}\n"
                 f"         (the package must contain {name}.json + its asset folder)")
    return import_dir


def run_import(carla_root, ue4_root, name):
    """Run CARLA's Import.py to cook the staged package. Returns its exit code."""
    import_py = os.path.join(carla_root, "Util", "BuildTools", "Import.py")
    if not os.path.isfile(import_py):
        sys.exit(f"[import] {import_py} not found - is {carla_root} a CARLA source build?")
    proc_env = dict(os.environ)
    if ue4_root:
        proc_env["UE4_ROOT"] = ue4_root
    if not proc_env.get("UE4_ROOT"):
        print("[import] WARNING: UE4_ROOT not set; the cook commandlet may fail.")
    cmd = [sys.executable, import_py, f"--package={name}"]
    print(f"[import] running: {' '.join(cmd)}  (cwd={carla_root})")
    print("[import] cooking the map can take several minutes ...")
    return subprocess.call(cmd, cwd=carla_root, env=proc_env)


def ensure_map(name, carla_root=None, ue4_root=None,
               package_url=None, package_dir=None, force=False, prompt_if_exists=False):
    """Make sure `name` is imported into the (source-build) CARLA, importing it
    if needed. Returns 0 on success; exits with a clear message otherwise.

    If the map already exists: `force` re-imports unconditionally; otherwise, when
    `prompt_if_exists` and the session is interactive, the user is asked whether to
    re-import (re-download + re-cook) - handy for updating a map or testing."""
    cfg = env.load_config() or {}
    carla_root = carla_root or cfg.get("carla_root")
    ue4_root = ue4_root or cfg.get("ue4_root")
    if not carla_root:
        sys.exit("[import] no CARLA configured - run setup_carla first.")
    if cfg.get("mode") == "packaged":
        sys.exit("[import] the configured CARLA is PACKAGED; importing a custom map "
                 "needs a SOURCE build (run setup_carla and pick source).")

    if map_is_imported(carla_root, name) and not force:
        print(f"[import] '{name}' already imported: {cooked_map_path(carla_root, name)}")
        if not (prompt_if_exists and sys.stdin.isatty()):
            return 0
        ans = input("[import] re-import it now (re-download + re-cook)? [y/N]: ").strip().lower()
        if ans != "y":
            print("[import] keeping the existing map.")
            return 0
        print("[import] re-importing ...")

    stage_package(carla_root, name, package_url, package_dir)
    rc = run_import(carla_root, ue4_root, name)
    if rc != 0:
        sys.exit(f"[import] Import.py failed (exit {rc}).")
    if not map_is_imported(carla_root, name):
        sys.exit(f"[import] import finished but {cooked_map_path(carla_root, name)} not found.")
    print(f"[import] done: '{name}' imported.")
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--package", required=True,
                    help="map/package name (matches Import/<name>.json)")
    ap.add_argument("--package-url", default=None,
                    help="URL of the package zip (e.g. a GitHub release asset)")
    ap.add_argument("--package-dir", default=None,
                    help="local folder holding the package files")
    ap.add_argument("--carla-root", default=None, help="override the saved carla_root")
    ap.add_argument("--ue4-root", default=None, help="override the saved ue4_root")
    ap.add_argument("--force", action="store_true",
                    help="re-import even if already present (no prompt)")
    args = ap.parse_args()
    # run standalone -> offer to re-import if the map already exists
    return ensure_map(args.package, args.carla_root, args.ue4_root,
                      args.package_url, args.package_dir, args.force, prompt_if_exists=True)


if __name__ == "__main__":
    sys.exit(main())
