"""
place_signs.py - place the RoadRunner road-sign meshes into a cooked CARLA map.

CARLA's import cooks the sign FBX meshes but never PLACES the ones whose
name/material contains "sign" (see unreal_place_signs.py), so custom RoadRunner
signs are imported yet invisible - and the auto-generated sign materials render
see-through. This driver mirrors place_tls.py: it runs unreal_place_signs.py
inside the FULL Unreal editor (-ExecutePythonScript) against the cooked level,
which both spawns the culled sign meshes and fixes their materials, then re-saves.

Map resolution matches place_tls: --map, else --map-config (a package= line), else
pick from the maps already cooked into this CARLA. Source-build only. A marker under
the map's Content folder records placement; a re-import wipes the content + marker,
so just re-run this afterwards.

Example:
  python place_signs.py --map Roosevelt_07142026
"""
import argparse
import os
import subprocess
import sys

import carla_env_setup as env
import import_map

HERE = os.path.dirname(os.path.abspath(__file__))
PLACER = os.path.join(HERE, "unreal_place_signs.py")


def content_map_path(name):
    """The /Game content path of the cooked level (what the editor opens)."""
    return f"/Game/{name}/Maps/{name}/{name}"


def signs_marker(carla_root, name):
    return os.path.join(import_map.cooked_content_dir(carla_root, name), ".fixs_signs_placed")


def signs_placed(carla_root, name):
    return os.path.isfile(signs_marker(carla_root, name))


def place_signs(name, carla_root=None, ue4_root=None, force=False):
    """Place the culled sign meshes into the cooked map `name` and fix their
    materials. Returns 0 on success; exits with a clear message otherwise. No-op if
    already placed (unless force)."""
    cfg = env.load_config() or {}
    carla_root = carla_root or cfg.get("carla_root")
    ue4_root = ue4_root or cfg.get("ue4_root")
    if not carla_root:
        sys.exit("[signs] no CARLA configured - run setup_carla first.")
    if cfg.get("mode") == "packaged":
        sys.exit("[signs] placing signs needs a SOURCE build (it saves the .umap).")
    if not import_map.map_is_imported(carla_root, name):
        sys.exit(f"[signs] map '{name}' is not imported yet; import it first.")
    if signs_placed(carla_root, name) and not force:
        print(f"[signs] signs already placed for '{name}' (marker present).")
        return 0

    uproject, editor = env.source_paths(carla_root, ue4_root)
    if not os.path.isfile(editor):
        sys.exit(f"[signs] UE4Editor not found: {editor}")
    proc_env = dict(os.environ)
    if ue4_root:
        proc_env["UE4_ROOT"] = ue4_root
    proc_env["SIGNS_MAP_PATH"] = content_map_path(name)
    proc_env["SIGNS_ASSET_ROOT"] = f"/Game/{name}"

    umap = import_map.cooked_map_path(carla_root, name)
    before = os.path.getmtime(umap) if os.path.isfile(umap) else None

    cmd = [editor, uproject, content_map_path(name), f"-ExecutePythonScript={PLACER}",
           *env.EDITOR_LAUNCH_FLAGS]
    print("[signs] placing road signs via the editor (a window opens briefly, "
          "no clicking needed) ...")
    print(f"[signs] {' '.join(cmd)}")
    rc = subprocess.call(cmd, env=proc_env)

    after = os.path.getmtime(umap) if os.path.isfile(umap) else None
    changed = before is not None and after is not None and after > before
    # Write the marker either way so we don't relaunch the editor next run: a map
    # with no sign meshes is a valid no-op, and UE4Editor exit codes are unreliable
    # (it often returns non-zero on a clean shutdown), so we trust "the pass ran"
    # over the code. Signs are cosmetic - never abort the caller (e.g. the co-sim).
    with open(signs_marker(carla_root, name), "w", encoding="utf-8") as f:
        f.write("placed\n" if changed else "none-found\n")
    if changed:
        print(f"[signs] done: road signs placed and level saved (editor exit {rc}).")
    else:
        print(f"[signs] no new signs placed - map has none, or already done "
              f"(editor exit {rc}); continuing.")
    return 0


def main():
    # Run under the interpreter carla.json names, whatever python this script was
    # started with. run_cosim imports this module rather than spawning it, so that
    # path is already on the right interpreter and this is a no-op there.
    env.reexec_under_configured(__file__, tag="signs")

    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--map", default=None, help="cooked map name (e.g. Roosevelt_07142026)")
    ap.add_argument("--map-config", default=None,
                    help="text file declaring the map (a package= line gives the name)")
    ap.add_argument("--carla-root", default=None, help="override the saved carla_root")
    ap.add_argument("--ue4-root", default=None, help="override the saved ue4_root")
    ap.add_argument("--force", action="store_true", help="re-place even if already done")
    args = ap.parse_args()

    name = args.map
    if not name and args.map_config:
        name = import_map.read_map_config(args.map_config).get("package")
    if not name:
        # No map given: pick from the maps already cooked into this CARLA.
        carla_root = args.carla_root or (env.load_config() or {}).get("carla_root")
        if not carla_root:
            ap.error("no CARLA configured (run setup_carla first), or pass --map <name>.")
        name = import_map.choose_imported_map(carla_root)

    return place_signs(name, args.carla_root, args.ue4_root, args.force)


if __name__ == "__main__":
    sys.exit(main())
