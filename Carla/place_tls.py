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
import props

HERE = os.path.dirname(os.path.abspath(__file__))
AUTO_PLACE = os.path.join(HERE, "sumo", "auto_place_tls.py")


def content_map_path(name):
    """The /Game content path of the cooked level (what the editor opens)."""
    return f"/Game/{name}/Maps/{name}/{name}"


def tls_marker(carla_root, name):
    return os.path.join(import_map.cooked_content_dir(carla_root, name), ".fixs_tls_placed")


def marker_fields(carla_root, name):
    """The `key=value` lines the placement marker recorded, as a dict ({} if the map
    was never placed, or was placed by a version that wrote no detail).

    Lets a caller see what the saved .umap was built from without opening it - which
    is the whole reason the marker records more than the word "placed"."""
    marker = tls_marker(carla_root, name)
    if not os.path.isfile(marker):
        return {}
    fields = {}
    try:
        with open(marker, "r", encoding="utf-8") as fh:
            for line in fh:
                if "=" in line:
                    key, _, value = line.partition("=")
                    fields[key.strip()] = value.strip()
    except OSError:
        return {}
    return fields


def tls_placed(carla_root, name, fingerprint=None):
    """True if this map's lights are already placed FROM THE SAME INPUTS.

    The marker records the fingerprint of the manifest + props it was written for. It
    used to record only THAT placement had happened, so correcting z_offset_cm in the
    manifest changed nothing on a machine that had already placed once - the fix sat
    there looking applied. A fingerprint mismatch counts as not-placed, so a manifest
    edit re-places on the next run.

    A marker written before fingerprints existed holds just "placed" and says nothing
    about what produced it. That is exactly the state the maps with the ORIGINAL bug
    are in - placed with the stock pole blueprint and a +300 lift - so trusting it
    would mean the fix never reaches the maps it was written for. It counts as stale:
    one re-place per already-placed map, which IS the fix arriving.

    Callers that pass no fingerprint (nothing to compare against) still take the
    marker at face value.
    """
    marker = tls_marker(carla_root, name)
    if not os.path.isfile(marker):
        return False
    if not fingerprint:
        return True
    try:
        with open(marker, "r", encoding="utf-8") as fh:
            lines = fh.read().splitlines()
    except OSError:
        return True
    stamped = [ln.split("=", 1)[1].strip() for ln in lines if ln.startswith("fingerprint=")]
    if not stamped:
        return False
    return stamped[0] == fingerprint


def resolve_props(carla_root, ue4_root, bundle_dirs=(), props_dir=None):
    """Find the map bundle's placement manifest, install the props it names, and
    return (settings, manifest_path, fingerprint).

    A bundle that ships a manifest gets every number from it, and a missing value is
    fatal. A bundle that ships none keeps the pre-manifest behaviour - atlanta and
    mlk have no manifest yet, and failing them would be a flag day for no gain - but
    says so, because a silently-chosen stock asset is how this went wrong before.
    """
    manifest_path = props.find_manifest(*bundle_dirs)
    if not manifest_path:
        settings = props.legacy_settings()
        print("[props] no placement.yaml in this map bundle; using the built-in "
              "blueprint and z_offset. Ship a manifest to make these the map's own.")
        print(props.describe(settings, None))
        return settings, None, None

    manifest = props.load(manifest_path)
    settings = props.tl_settings(manifest, manifest_path)
    shared = import_map.props_cache_dir()
    assets_dir = props.resolve_assets_dir(manifest_path, manifest,
                                          explicit=props_dir, shared=shared)
    installed = props.install(carla_root, assets_dir, manifest, manifest_path,
                              ue4_root=ue4_root)
    print(props.describe(settings, manifest_path))
    return settings, manifest_path, props.fingerprint(settings, installed)


def place_tls(name, tl_table, carla_root=None, ue4_root=None, force=False,
              bundle_dirs=(), props_dir=None):
    """Place traffic lights into the cooked map `name` from `tl_table`. Returns 0
    on success; exits with a clear message otherwise. No-op if already placed from
    the same manifest + props (unless force)."""
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

    # Resolved before the "already placed?" check, because the manifest is part of
    # what that question is about: same map, different numbers, means not placed.
    try:
        settings, manifest_path, fingerprint = resolve_props(
            carla_root, ue4_root, bundle_dirs=bundle_dirs, props_dir=props_dir)
    except props.ManifestError as exc:
        sys.exit(f"[props] {exc}")

    if tls_placed(carla_root, name, fingerprint) and not force:
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
    proc_env.update(props.placer_env(settings))

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
        if fingerprint:
            # What it was placed FROM, so a later manifest edit is detectable.
            f.write(f"fingerprint={fingerprint}\n")
            f.write(f"manifest={os.path.abspath(manifest_path)}\n")
            f.write(f"blueprint={settings['blueprint']}\n")
            f.write(f"z_offset_cm={settings['z_offset_cm']:g}\n")
            f.write(f"flip_yaw_180={settings['flip_yaw_180']}\n")
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
    ap.add_argument("--bundle", default=None,
                    help="map bundle dir holding placement.yaml (default: the map's "
                         "cache under ~/.fixs/maps/<map>)")
    ap.add_argument("--props-dir", default=None,
                    help="directory holding the props' .uasset files, when they do "
                         "not sit beside placement.yaml (e.g. the map library's "
                         "shared props/ folder)")
    args = ap.parse_args()

    name = args.map
    if not name and args.map_config:
        name = import_map.read_map_config(args.map_config).get("package")
    if not name:
        # No map given: let the user pick from the maps already cooked into this
        # CARLA. place_tls operates on a cooked map, so this needs no repo/network.
        carla_root = args.carla_root or (env.load_config() or {}).get("carla_root")
        if not carla_root:
            ap.error("no CARLA configured (run setup_carla first), or pass --map <name>.")
        name = import_map.choose_imported_map(carla_root)

    # Where to look for placement.yaml: an explicit --bundle, else the map's own
    # cache dir (where run_cosim unpacks the bundle) and then the shared props
    # cache. Map first, so a bundle shipping its own manifest overrides the
    # library's shared defaults; find_manifest takes the first hit.
    if args.bundle:
        bundle_dirs = [args.bundle]
    else:
        bundle_dirs = [d for d in (import_map.map_cache_dir(name),
                                   import_map.props_cache_dir()) if d]

    return place_tls(name, args.tl_table, args.carla_root, args.ue4_root, args.force,
                     bundle_dirs=bundle_dirs, props_dir=args.props_dir)


if __name__ == "__main__":
    sys.exit(main())
