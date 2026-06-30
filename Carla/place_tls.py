"""
place_tls.py - place SUMO traffic-light actors into a cooked CARLA map.

For maps whose OpenDRIVE has no dynamic signals (so CARLA spawns no
traffic.traffic_light actors), the SUMO<->CARLA TL sync has nothing to drive.
This places the actors from a traffic_light_table.csv into the map's level and
saves it, by running sumo/auto_place_tls.py inside the FULL Unreal editor
(-ExecutePythonScript - the commandlet has no viewport and crashes).

Source-build only (it saves the .umap via the editor). carla_root / ue4_root come
from the saved env config. A marker file under the map's Content folder records
that placement is done, so run_cosim skips it on later runs and re-does it after a
re-import (which wipes the content + marker).

Examples:
  python place_tls.py --map RP_Ver0529 --tl-table path/to/traffic_light_table.csv
"""
import argparse
import os
import subprocess
import sys

import carla_env_setup as env
import import_map

HERE = os.path.dirname(os.path.abspath(__file__))
AUTO_PLACE = os.path.join(HERE, "sumo", "auto_place_tls.py")


def content_map_path(name):
    """The /Game content path of the cooked level (what the editor opens)."""
    return f"/Game/{name}/Maps/{name}/{name}"


def tls_marker(carla_root, name):
    return os.path.join(import_map.cooked_content_dir(carla_root, name), ".fixs_tls_placed")


def tls_placed(carla_root, name):
    return os.path.isfile(tls_marker(carla_root, name))


def place_tls(name, tl_table, carla_root=None, ue4_root=None, force=False):
    """Place traffic lights into the cooked map `name` from `tl_table`. Returns 0
    on success; exits with a clear message otherwise. No-op if already placed
    (unless force)."""
    cfg = env.load_config() or {}
    carla_root = carla_root or cfg.get("carla_root")
    ue4_root = ue4_root or cfg.get("ue4_root")
    if not carla_root:
        sys.exit("[tls] no CARLA configured - run setup_carla first.")
    if cfg.get("mode") == "packaged":
        sys.exit("[tls] placing traffic lights needs a SOURCE build (it saves the .umap).")
    if not tl_table or not os.path.isfile(tl_table):
        sys.exit(f"[tls] traffic-light table not found: {tl_table}")
    if not import_map.map_is_imported(carla_root, name):
        sys.exit(f"[tls] map '{name}' is not imported yet; import it first.")
    if tls_placed(carla_root, name) and not force:
        print(f"[tls] traffic lights already placed for '{name}' (marker present).")
        return 0

    uproject, editor = env.source_paths(carla_root, ue4_root)
    if not os.path.isfile(editor):
        sys.exit(f"[tls] UE4Editor not found: {editor}")
    proc_env = dict(os.environ)
    if ue4_root:
        proc_env["UE4_ROOT"] = ue4_root
    proc_env["SUMO_TLS_TABLE_PATH"] = os.path.abspath(tl_table)
    proc_env["SUMO_TLS_MAP_PATH"] = content_map_path(name)

    umap = import_map.cooked_map_path(carla_root, name)
    before = os.path.getmtime(umap) if os.path.isfile(umap) else None

    cmd = [editor, uproject, content_map_path(name), f"-ExecutePythonScript={AUTO_PLACE}"]
    print("[tls] placing traffic lights via the editor (a window opens briefly, "
          "no clicking needed) ...")
    print(f"[tls] {' '.join(cmd)}")
    rc = subprocess.call(cmd, env=proc_env)

    after = os.path.getmtime(umap) if os.path.isfile(umap) else None
    if before is None or after is None or after <= before:
        sys.exit(f"[tls] placement did not re-save the map (editor exit {rc}); "
                 f"check the editor log. Traffic lights were NOT placed.")
    with open(tls_marker(carla_root, name), "w", encoding="utf-8") as f:
        f.write("placed\n")
    print(f"[tls] done: traffic lights placed and level saved (editor exit {rc}).")
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--map", default=None, help="cooked map name (e.g. RP_Ver0529)")
    ap.add_argument("--map-config", default=None,
                    help="text file declaring the map (a package= line gives the name)")
    ap.add_argument("--tl-table", required=True, help="traffic_light_table.csv")
    ap.add_argument("--carla-root", default=None, help="override the saved carla_root")
    ap.add_argument("--ue4-root", default=None, help="override the saved ue4_root")
    ap.add_argument("--force", action="store_true", help="re-place even if already done")
    args = ap.parse_args()

    name = args.map
    if not name and args.map_config:
        name = import_map.read_map_config(args.map_config).get("package")
    if not name:
        ap.error("a map is required: pass --map, or --map-config with a package= line")

    return place_tls(name, args.tl_table, args.carla_root, args.ue4_root, args.force)


if __name__ == "__main__":
    sys.exit(main())
