"""
import_vehicle.py - install a vehicle from the vehicle library into a CARLA source build.

The counterpart of import_map.py for vehicles (FIXS#470). The library stores one bundle
per vehicle (vehicle.yaml + the rig prep_vehicle.py made); this fetches it, builds the
blueprint in the Unreal editor, and registers it so `vehicle.<make>.<model>` spawns.

  python Carla/import_vehicle.py                          pick from the library catalog
  python Carla/import_vehicle.py --vehicle ev6            by id or model name
  python Carla/import_vehicle.py --bundle <zip|dir>       a bundle you have locally
  python Carla/import_vehicle.py --list                   catalog + what is installed here
  python Carla/import_vehicle.py --uninstall ev6          unregister + delete its assets
  python Carla/import_vehicle.py --verify ev6 --port 2000 spawn-and-drive check on a server

Writes /Game/FIXS/Vehicles/<Asset>/ and /Game/FIXS/Vehicles/_Shared/ (outside
/Game/Carla, which CARLA's updaters own), and edits ONE stock asset: the VehicleFactory
`Vehicles` array gets (or loses) this vehicle's entry, after a backup under
~/.fixs/vehicles/_factory_backup/. A CARLA update that replaces Content/Carla drops the
entry; re-run with --register-only to put it back.

Shared installs: other CARLA servers or editors running from the same install hold the
factory open and the save would fail. This refuses (listing them) unless --wait is
given, and never stops a process.

Packaged CARLA has no editor, so it installs a COOK instead: a source build makes it
with --cook, the library publishes it beside the bundle (<id>_cooked_windows.tar.gz), and
on a packaged build --vehicle fetches and extracts it (Carla/vehicles/cooked.py).

  python Carla/import_vehicle.py --cook out.tar.gz        source: cook every registered vehicle
  python Carla/import_vehicle.py --cooked <tar.gz>        packaged: install a cook you have
"""
import argparse
import datetime
import json
import os
import shutil
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import carla_env_setup as env  # noqa: E402
import props  # noqa: E402
from vehicles import catalog  # noqa: E402
from vehicles import cooked  # noqa: E402
from vehicles import install_guard  # noqa: E402
from vehicles import manifest as vmanifest  # noqa: E402

INSTALL_ROOT = "/Game/FIXS/Vehicles"
SHARED_DIR = INSTALL_ROOT + "/_Shared"
EDITOR_MAP = "/Game/Carla/Maps/OpenDriveMap"  # small level; the script never touches it
UNREAL_SCRIPT = os.path.join(HERE, "vehicles", "unreal_build.py")
FACTORY_GAME_PATH = "/Game/Carla/Blueprints/Vehicles/VehicleFactory"

# Stock CARLA 0.9.15 assets the build reads or duplicates. The paint parent sits on
# M_CarPaint_Master, whose colour is the "Base Color" parameter alone; the Tesla's
# M_CarExterior_Master multiplies it by a UV mask a foreign mesh does not carry.
STOCK = {
    "skeleton": "/Game/Carla/Static/Car/4Wheeled/Tesla/SM_TeslaM3_lights_body_Skeleton",
    "anim_class": "/Game/Carla/Static/Car/4Wheeled/Tesla/Tesla_Animation",
    "physics_asset": "/Game/Carla/Static/Car/4Wheeled/Tesla/SM_TeslaM3_PhysicsAsset",
    "template_vehicle": "/Game/Carla/Blueprints/Vehicles/Tesla/BP_TeslaM3",
    "front_wheel": "/Game/Carla/Blueprints/Vehicles/Tesla/BP_Tesla_FLW",
    "rear_wheel": "/Game/Carla/Blueprints/Vehicles/Tesla/BP_Tesla_RLW",
    "base_pawn": "/Game/Carla/Blueprints/Vehicles/BaseVehiclePawn",
    "vehicle_factory": FACTORY_GAME_PATH,
    "paint_parent": "/Game/Carla/Static/Car/4Wheeled/LincolnMKZ2020/Materials/MI_Lincoln2020_bodywork",
    "glass": "/Game/Carla/Static/Car/4Wheeled/Tesla/Materials/MI_CarGlass_TeslaM3",
    "light_glass": "/Game/Carla/Static/Car/4Wheeled/Tesla/Materials/MI_CarLightGlass_TeslaM3",
}


def log(msg):
    print("[vehicle] " + msg, flush=True)


# ---------------------------------------------------------------- pure helpers

def factory_file(carla_root):
    return props.content_dir(carla_root, FACTORY_GAME_PATH) + ".uasset"


def asset_dir(m):
    return "{}/{}".format(INSTALL_ROOT, m["asset_name"])


def class_path(m):
    return "{0}/BP_{1}.BP_{1}_C".format(asset_dir(m), m["asset_name"])


def collision_box_cm(m):
    """Body box of the physics asset, from the real car's dimensions (cm, UE full extents)."""
    d = m["dimensions"]
    length, width = d["length_m"]["value"], d["width_m"]["value"]
    height, clear = d["height_m"]["value"], d["ground_clearance_m"]["value"]
    return {"center": [0.0, 0.0, round((clear + (height - clear) / 2) * 100, 2)],
            "x": round(length * 100 - 5.0, 2), "y": round(width * 100, 2),
            "z": round((height - clear) * 100, 2)}


def unreal_job(m, slots, action, owned_ids, register, result_path, textures_dir, fbx):
    """Everything unreal_build.py needs, resolved from the manifest; no defaults of its own."""
    d, w = m["dimensions"], m["wheels"]
    return {
        "action": action, "id": m["id"], "asset_name": m["asset_name"],
        "dest": asset_dir(m), "shared": SHARED_DIR, "stock": dict(STOCK),
        "fbx": fbx, "textures_dir": textures_dir, "slots": slots,
        "paint_color": m["factory"]["recommended_colors"][0],
        "wheels": {"radius_cm": w["radius_m"] * 100.0, "width_cm": w["width_m"] * 100.0,
                   "max_steer_deg": w["max_steer_deg"], "rear_max_steer_deg": w["rear_max_steer_deg"]},
        "body": {"mass_kg": m["physics"]["mass_kg"], "drag_coefficient": m["physics"]["drag_coefficient"],
                 "chassis_width_cm": d["width_m"]["value"] * 100.0,
                 "chassis_height_cm": d["height_m"]["value"] * 100.0},
        "collision_box_cm": collision_box_cm(m),
        "drivetrain": m["drivetrain"],
        "factory_entry": vmanifest.factory_entry(m, class_path(m)),
        "owned_ids": sorted(set(owned_ids)),
        "register": bool(register),
        "result": result_path,
    }


def refusal(procs, locked, factory_path):
    lines = []
    if procs:
        lines.append("CARLA/Unreal processes are running from this install:\n" + install_guard.describe(procs))
    if locked:
        lines.append("{} is held open by another process.".format(factory_path))
    lines.append("Saving over a package another process has open fails, so nothing was changed. "
                 "Close those processes (this tool never stops them), or:\n"
                 "    --wait SECONDS     wait for them to finish, then continue\n"
                 "    --no-register      build the vehicle assets now; register later with --register-only")
    return "\n".join(lines)


def pick(vehicles, answer):
    """The vehicle a picker answer names: a list number, an id or a model name."""
    answer = (answer or "").strip()
    if answer.isdigit() and 1 <= int(answer) <= len(vehicles):
        return vehicles[int(answer) - 1]
    return catalog.entry(vehicles, answer)


# ---------------------------------------------------------------- bundle

def _load_bundle(bundle):
    path = vmanifest.find(bundle)
    if not path:
        sys.exit("[vehicle] {} has no vehicle.yaml".format(bundle))
    try:
        return vmanifest.load(path)
    except vmanifest.ManifestError as exc:
        sys.exit("[vehicle] {}".format(exc))


def bundle_from_local(src):
    """Stage a local bundle into the cache under its own id; returns the manifest."""
    try:
        staged = catalog.stage_bundle(src)
    except catalog.CatalogError as exc:
        sys.exit("[vehicle] {}".format(exc))
    m = _load_bundle(staged)
    dest = catalog.bundle_dir(m["id"])
    if os.path.normcase(os.path.abspath(staged)) != os.path.normcase(os.path.abspath(dest)):
        if os.path.isdir(dest):
            shutil.rmtree(dest)
        shutil.copytree(staged, dest)
    return _load_bundle(dest)


def bundle_from_library(repo, name):
    vehicles, where = catalog.fetch_catalog(repo)
    if not vehicles:
        sys.exit("[vehicle] no vehicles listed in {}'s catalog.json (gh auth / network?). "
                 "Pass --bundle <zip|dir> for a copy you have.".format(repo))
    if name:
        ent = catalog.entry(vehicles, name)
        if ent is None:
            sys.exit("[vehicle] {!r} is not in the catalog; it lists: {}".format(
                name, ", ".join(v["id"] for v in vehicles)))
    else:
        print("Vehicles in {} ({}):".format(repo, where))
        for i, v in enumerate(vehicles, 1):
            mark = " (installed)" if catalog.read_receipt(v["id"]) else ""
            print("  {}) {:<24} {}{}".format(i, v["id"], v["title"], mark))
        ent = pick(vehicles, input("Pick a vehicle [number or id]: "))
        if ent is None:
            sys.exit("[vehicle] nothing picked.")
    zip_path = catalog.download(repo, ent, os.path.join(catalog.vehicle_dir(ent["id"]), "download"))
    catalog.extract(zip_path, catalog.bundle_dir(ent["id"]))
    m = _load_bundle(catalog.bundle_dir(ent["id"]))
    if m["id"] != ent["id"]:
        sys.exit("[vehicle] the catalog lists {} but its bundle declares {}".format(ent["id"], m["id"]))
    return m


def rig_inputs(m):
    """(fbx, slots, textures_dir) from the bundle's rig/, or exit saying to run prep."""
    fbx = os.path.join(m["dir"], vmanifest.RIG_FBX)
    mats = os.path.join(m["dir"], vmanifest.RIG_MATERIALS)
    if not (os.path.isfile(fbx) and os.path.isfile(mats)):
        sys.exit("[vehicle] {} has no {} / {}; run Carla/prep_vehicle.py on the bundle first.".format(
            m["dir"], vmanifest.RIG_FBX, vmanifest.RIG_MATERIALS))
    with open(mats, "r", encoding="utf-8") as fh:
        slots = json.load(fh)["slots"]
    return fbx, slots, os.path.join(m["dir"], vmanifest.RIG_TEXTURES)


# ---------------------------------------------------------------- CARLA side

def is_packaged(carla_root):
    return bool(env.packaged_exe(carla_root)) and not os.path.isfile(
        os.path.join(carla_root, "Unreal", "CarlaUE4", "CarlaUE4.uproject"))


def resolve_carla(carla_root=None, ue4_root=None):
    """{mode, carla_root, ue4_root, uproject, editor, package_root} of the CARLA to install into."""
    cfg = env.load_config() or {}
    explicit = carla_root
    carla_root = carla_root or cfg.get("carla_root")
    ue4_root = ue4_root or cfg.get("ue4_root")
    mode = cfg.get("mode")
    if mode == "client" and not carla_root:
        sys.exit("[vehicle] this machine is a CARLA client; install the vehicle on the CARLA host.")
    if not carla_root:
        sys.exit("[vehicle] no CARLA configured - run setup_carla first, or pass --carla-root.")
    if (mode == "packaged" and not explicit) or is_packaged(carla_root):
        exe = env.packaged_exe(carla_root)
        if not exe:
            sys.exit("[vehicle] {} is not a packaged CARLA (no CarlaUE4 launcher).".format(carla_root))
        return {"mode": "packaged", "carla_root": carla_root, "package_root": os.path.dirname(exe),
                "ue4_root": None, "uproject": None, "editor": None}
    uproject, editor = env.source_paths(carla_root, ue4_root or "")
    if not os.path.isfile(uproject):
        sys.exit("[vehicle] {} is not a CARLA source build (no {}).".format(carla_root, uproject))
    if not ue4_root or not os.path.isfile(editor):
        sys.exit("[vehicle] UE4Editor not found (ue4_root={!r}); pass --ue4-root.".format(ue4_root))
    return {"mode": "source", "carla_root": carla_root, "package_root": None,
            "ue4_root": ue4_root, "uproject": uproject, "editor": editor}


def wait_until_free(carla_root, need_factory, wait_s):
    """(procs, locked) once nothing on this install blocks the edit, or after wait_s."""
    deadline = time.time() + max(0.0, wait_s)
    fpath = factory_file(carla_root)
    while True:
        procs = install_guard.instances_on_install(install_guard.list_processes(), carla_root)
        locked = need_factory and install_guard.file_in_use(fpath)
        if not procs and not locked:
            return [], False
        if time.time() >= deadline:
            return procs, locked
        log("waiting for {} process(es) on this install to exit ...".format(len(procs)))
        time.sleep(10)


def backup_factory(carla_root):
    """Copy VehicleFactory.uasset aside; the first copy ever taken is kept as the original."""
    src = factory_file(carla_root)
    d = os.path.join(catalog.home(), "_factory_backup")
    os.makedirs(d, exist_ok=True)
    original = os.path.join(d, "VehicleFactory.original.uasset")
    if not os.path.isfile(original):
        shutil.copy2(src, original)
    stamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
    copy = os.path.join(d, "VehicleFactory.{}.uasset".format(stamp))
    shutil.copy2(src, copy)
    return copy


def run_editor(uproject, editor, ue4_root, job, work, dry_run=False):
    os.makedirs(work, exist_ok=True)
    job_path = os.path.join(work, "job_{}.json".format(job["action"]))
    with open(job_path, "w", encoding="utf-8") as fh:
        json.dump(job, fh, indent=1)
    editor_log = os.path.join(work, "editor_{}.log".format(job["action"]))
    cmd = [editor, uproject, EDITOR_MAP, "-ExecutePythonScript={}".format(UNREAL_SCRIPT),
           *env.EDITOR_LAUNCH_FLAGS, "-abslog={}".format(editor_log)]
    log("job: {}".format(job_path))
    log("editor: {}".format(" ".join(cmd)))
    if dry_run:
        log("--dry-run: editor not launched.")
        return None
    if os.path.isfile(job["result"]):
        os.remove(job["result"])
    proc_env = dict(os.environ, FIXS_VEHICLE_JOB=job_path, UE4_ROOT=ue4_root)
    log("building in the Unreal editor (a window opens; no clicking needed) ...")
    rc = subprocess.call(cmd, env=proc_env)
    if not os.path.isfile(job["result"]):
        sys.exit("[vehicle] the editor wrote no result (exit {}); see {}".format(rc, editor_log))
    with open(job["result"], "r", encoding="utf-8") as fh:
        res = json.load(fh)
    if res.get("error"):
        sys.exit("[vehicle] the editor script failed; see {}:\n{}".format(editor_log, res["error"]))
    return res


# ---------------------------------------------------------------- commands

def install(m, carla_root, ue4_root, uproject, editor, register=True, register_only=False,
            wait_s=0.0, dry_run=False, adopt=False):
    try:
        props.check_engine(m, ue4_root, m["path"])
    except props.ManifestError as exc:
        sys.exit("[vehicle] {}".format(exc))
    need_factory = register or register_only
    fbx, slots, tex = rig_inputs(m)
    if not dry_run:
        procs, locked = wait_until_free(carla_root, need_factory, wait_s)
        held = [f for f in _files(props.content_dir(carla_root, asset_dir(m)))
                if install_guard.file_in_use(f)]
        if (procs and need_factory) or locked or held:
            sys.exit("[vehicle] " + refusal(procs, locked or bool(held), factory_file(carla_root)))
        if procs:
            log("note: other processes run from this install; they will not see the new vehicle "
                "until restarted:\n" + install_guard.describe(procs))
    work = catalog.vehicle_dir(m["id"])
    action = "register" if register_only else "install"
    owned = catalog.owned_ids(carla_root) + ([m["id"]] if adopt else [])
    job = unreal_job(m, slots, action, owned, need_factory,
                     os.path.join(work, "result_{}.json".format(action)), tex, fbx)
    backup = backup_factory(carla_root) if need_factory and not dry_run else None
    fpath = factory_file(carla_root)
    before = os.path.getmtime(fpath) if os.path.isfile(fpath) else None
    res = run_editor(uproject, editor, ue4_root, job, work, dry_run)
    if res is None:
        return 0

    fact = res.get("factory") or {}
    prior = catalog.read_receipt(m["id"], carla_root) or {}
    registered = prior.get("registered", False)
    if need_factory:
        registered = fact.get("action") in ("added", "updated", "unchanged") and fact.get("saved", True)
        if (fact.get("action") in ("added", "updated") and before is not None
                and os.path.getmtime(fpath) <= before):
            registered = False
    catalog.write_receipt(m["id"], {
        "id": m["id"], "title": m["title"], "carla_root": os.path.abspath(carla_root),
        "asset_dir": asset_dir(m), "class_path": class_path(m), "manifest": m["path"],
        "registered": bool(registered), "factory_backup": backup or prior.get("factory_backup"),
        "installed_at": datetime.datetime.now().isoformat(timespec="seconds"),
        "editor_result": job["result"]})
    if not res.get("ok"):
        sys.exit("[vehicle] the editor could not save everything (saved: {}); see {}".format(
            res.get("saved"), job["result"]))
    if need_factory and not registered:
        sys.exit("[vehicle] the VehicleFactory entry was not saved ({}); the assets are installed. "
                 "Re-run with --register-only once nothing holds the factory.".format(fact))
    log("{} {}: {} (VehicleFactory: {}).".format("registered" if register_only else "installed",
                                                 m["id"], asset_dir(m), fact.get("action", "not edited")))
    log("restart CARLA servers on this install to see it; check it with --verify {}".format(m["id"]))
    return 0


def _files(d):
    out = []
    for root, _, names in os.walk(d) if os.path.isdir(d) else ():
        out += [os.path.join(root, n) for n in names]
    return out


def uninstall(bp_id, carla_root, ue4_root, uproject, editor, keep_assets=False,
              restore_factory=False, wait_s=0.0, dry_run=False):
    rec = catalog.read_receipt(bp_id, carla_root)
    if rec is None:
        sys.exit("[vehicle] no install receipt for {}; nothing FIXS installed to remove.".format(bp_id))
    if not dry_run:
        procs, locked = wait_until_free(carla_root, True, wait_s)
        if procs or locked:
            sys.exit("[vehicle] " + refusal(procs, locked, factory_file(carla_root)))
    if restore_factory:
        original = os.path.join(catalog.home(), "_factory_backup", "VehicleFactory.original.uasset")
        if not os.path.isfile(original):
            sys.exit("[vehicle] no original VehicleFactory backup at {}".format(original))
        others = [i for i in catalog.installed_ids(carla_root) if i != bp_id]
        if others:
            log("restoring the original factory also unregisters: {}".format(", ".join(others)))
        if not dry_run:
            shutil.copy2(original, factory_file(carla_root))
            for i in others + [bp_id]:
                r = catalog.read_receipt(i, carla_root)
                if r:
                    r["registered"] = False
                    catalog.write_receipt(i, r)
    else:
        job = {"action": "unregister", "id": bp_id, "stock": dict(STOCK),
               "owned_ids": sorted(set(catalog.owned_ids(carla_root)) | {bp_id}),
               "result": os.path.join(catalog.vehicle_dir(bp_id), "result_unregister.json")}
        if not dry_run:
            backup_factory(carla_root)
        res = run_editor(uproject, editor, ue4_root, job, catalog.vehicle_dir(bp_id), dry_run)
        if res is not None and not res.get("ok"):
            sys.exit("[vehicle] could not save VehicleFactory; see {}".format(job["result"]))
    if not keep_assets and rec.get("asset_dir"):
        d = props.content_dir(carla_root, rec["asset_dir"])
        log("{} {}".format("would delete" if dry_run else "deleting", d))
        if not dry_run and os.path.isdir(d):
            shutil.rmtree(d)
    if not dry_run:
        catalog.remove_receipt(bp_id, carla_root)
    log("uninstalled {}.".format(bp_id))
    return 0


def carla_version(carla_root):
    """CARLA version of a source build, from its git tag; "" when unknown."""
    try:
        out = subprocess.run(["git", "-C", carla_root, "describe", "--tags"], capture_output=True,
                             text=True, timeout=10)
        return out.stdout.strip() if out.returncode == 0 else ""
    except (OSError, subprocess.SubprocessError):
        return ""


def cook_all(c, out_path):
    """Source build: cook every registered library vehicle into `out_path` for packaged CARLAs."""
    procs = install_guard.instances_on_install(install_guard.list_processes(), c["carla_root"])
    if procs:
        log("note: other processes run from this install; the cook only reads it:\n"
            + install_guard.describe(procs))
    recs = [r for _, r in catalog.registered_receipts(c["carla_root"])]
    editor_cmd = c["editor"].replace("UE4Editor.exe", "UE4Editor-Cmd.exe")
    work = os.path.join(catalog.home(), "_cook")
    os.makedirs(work, exist_ok=True)
    log("cooking {} for packaged CARLA (shaders take a while) ...".format(
        ", ".join(r["id"] for r in recs) or "nothing"))
    try:
        path = cooked.cook(c["carla_root"], editor_cmd, c["uproject"], recs, os.path.abspath(out_path),
                           work, carla_version(c["carla_root"]))
    except cooked.CookError as exc:
        sys.exit("[vehicle] {}".format(exc))
    log("wrote {} ({:.1f} MB, sha256 {})".format(path, os.path.getsize(path) / 1e6,
                                                 catalog.sha256_of(path)))
    return 0


def install_cooked(c, tar_path, wait_s=0.0):
    """Packaged build: install a cook. Refuses while a CARLA runs from this install."""
    procs, _ = wait_until_free(c["carla_root"], False, wait_s)
    if procs:
        sys.exit("[vehicle] CARLA runs from this install; close it first (this tool never stops "
                 "it):\n" + install_guard.describe(procs))
    try:
        ids = cooked.install(tar_path, c["package_root"], c["carla_root"], log=log)
    except cooked.CookError as exc:
        sys.exit("[vehicle] {}".format(exc))
    log("installed {} into {} (packaged).".format(", ".join(ids), c["package_root"]))
    return ids


def uninstall_cooked(c):
    """Packaged build: restore the stock factory and remove every cooked library vehicle."""
    procs, _ = wait_until_free(c["carla_root"], False, 0.0)
    if procs:
        sys.exit("[vehicle] CARLA runs from this install; close it first:\n"
                 + install_guard.describe(procs))
    try:
        gone = cooked.uninstall(c["package_root"], c["carla_root"], log=log)
    except cooked.CookError as exc:
        sys.exit("[vehicle] {}".format(exc))
    log("restored the stock VehicleFactory; removed {}.".format(", ".join(gone) or "nothing"))
    return 0


def cooked_from_library(repo, name):
    """Download the catalog's cook for `name` on this OS; returns its path."""
    vehicles, _ = catalog.fetch_catalog(repo)
    ent = catalog.entry(vehicles, name)
    if ent is None:
        sys.exit("[vehicle] {!r} is not in {}'s catalog.".format(name, repo))
    asset = cooked.catalog_cooked_asset(ent)
    if not asset:
        sys.exit("[vehicle] {} publishes no cook for this OS, so a packaged CARLA cannot load it; "
                 "use a source build.".format(ent["id"]))
    sha = ent.get("cooked_sha256_" + cooked.os_tag()) or ""
    try:
        return catalog.download(repo, dict(ent, asset=asset, sha256=sha),
                                os.path.join(catalog.vehicle_dir(ent["id"]), "download"))
    except catalog.CatalogError as exc:
        sys.exit("[vehicle] {}".format(exc))


def content_root(carla_root, mode):
    if mode == "packaged":
        return os.path.join(os.path.dirname(env.packaged_exe(carla_root)), "CarlaUE4", "Content")
    return os.path.join(carla_root, "Unreal", "CarlaUE4", "Content")


def install_state(bp_id, carla_root, mode):
    """'stock', 'installed' or 'missing' for blueprint `bp_id` in this CARLA."""
    if not bp_id or bp_id in catalog.STOCK_IDS:
        return "stock"
    rec = catalog.read_receipt(bp_id, carla_root)
    if not rec or not rec.get("registered") or not rec.get("asset_dir"):
        return "missing"
    d = os.path.join(content_root(carla_root, mode), rec["asset_dir"][len("/Game/"):].replace("/", os.sep))
    return "installed" if os.path.isdir(d) else "missing"


def ensure_for_run(bp_id, carla_root, mode, ue4_root=None, interactive=False, ask=input, repo=None):
    """run_cosim's preflight: make sure the configured ego blueprint exists in this CARLA.

    Stock and installed blueprints pass. A missing library vehicle is offered for import
    when someone is there to answer; otherwise the run stops with the command to run."""
    state = install_state(bp_id, carla_root, mode)
    if state != "missing":
        return state
    repo = catalog.resolve_repo(repo)
    how = "python FIXS/Carla/import_vehicle.py --vehicle {}".format(bp_id)
    vehicles, _ = catalog.fetch_catalog(repo)
    if catalog.entry(vehicles, bp_id) is None and not catalog.vehicle_receipts(bp_id):
        sys.exit("[cosim] CarlaSetup.EgoBlueprint {} is neither a stock CARLA 0.9.15 vehicle nor in "
                 "the vehicle library ({}). Check the spelling, or install a local bundle with:\n"
                 "    python FIXS/Carla/import_vehicle.py --bundle <zip|dir>".format(bp_id, repo))
    print("[cosim] the ego vehicle {} is not installed in this {} CARLA ({}).".format(
        bp_id, mode, carla_root))
    if not interactive:
        sys.exit("[cosim] install it first:\n    {}".format(how))
    answer = (ask("[cosim] import it now from {}? [Y/n] ".format(repo)) or "y").strip().lower()
    if answer not in ("y", "yes"):
        sys.exit("[cosim] not imported. To do it later:\n    {}".format(how))
    c = resolve_carla(carla_root, ue4_root)
    if c["mode"] == "packaged":
        install_cooked(c, cooked_from_library(repo, bp_id))
    else:
        install(bundle_from_library(repo, bp_id), c["carla_root"], c["ue4_root"], c["uproject"],
                c["editor"])
    if install_state(bp_id, carla_root, c["mode"]) != "installed":
        sys.exit("[cosim] {} still is not installed; see the lines above.".format(bp_id))
    return "imported"


def list_vehicles(repo):
    vehicles, where = catalog.fetch_catalog(repo)
    print("Library {} ({}):".format(repo, where or "unreachable"))
    for v in vehicles:
        r = catalog.read_receipt(v["id"])
        state = ("registered" if r and r.get("registered") else "assets only") if r else "-"
        print("  {:<24} {:<12} {}".format(v["id"], state, v["title"]))
    local = [i for i in os.listdir(catalog.home())] if os.path.isdir(catalog.home()) else []
    extra = [i for i in local if i.startswith("vehicle.") and i not in {v["id"] for v in vehicles}]
    for i in extra:
        r = catalog.read_receipt(i) or {}
        print("  {:<24} {:<12} (local bundle)".format(i, "registered" if r.get("registered") else "-"))
    return 0


def main():
    env.reexec_under_configured(__file__, tag="vehicle")
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--vehicle", help="catalog id (vehicle.kia.ev6) or model name (ev6)")
    ap.add_argument("--bundle", help="local bundle zip or folder instead of the library")
    ap.add_argument("--repo", help="vehicle library repo (default: vehicle_repo= in fixs_sources.txt, "
                                   "else {})".format(catalog.DEFAULT_REPO))
    ap.add_argument("--list", action="store_true", help="list the catalog and what is installed")
    ap.add_argument("--uninstall", metavar="ID", help="unregister a vehicle and delete its assets")
    ap.add_argument("--keep-assets", action="store_true", help="with --uninstall: only unregister")
    ap.add_argument("--restore-factory", action="store_true",
                    help="with --uninstall: put back the VehicleFactory backed up before FIXS first "
                         "edited it (drops every FIXS vehicle)")
    ap.add_argument("--no-register", action="store_true", help="build the assets, leave VehicleFactory alone")
    ap.add_argument("--register-only", action="store_true", help="only (re)add the VehicleFactory entry")
    ap.add_argument("--adopt", action="store_true",
                    help="replace a VehicleFactory entry with this id that FIXS did not add (e.g. one "
                         "added by hand); without it such an entry is left alone and the install refused")
    ap.add_argument("--wait", type=float, default=0.0, metavar="S",
                    help="wait up to S seconds for other processes on this install to exit")
    ap.add_argument("--dry-run", action="store_true", help="write the editor job and print the command only")
    ap.add_argument("--verify", metavar="ID", help="spawn-and-drive check against a running CARLA server")
    ap.add_argument("--host", default="localhost", help="--verify: CARLA host")
    ap.add_argument("--port", type=int, default=2000, help="--verify: CARLA RPC port")
    ap.add_argument("--tm-port", type=int, default=8000, help="--verify: Traffic Manager port")
    ap.add_argument("--cook", metavar="OUT", help="source build: cook every registered library vehicle "
                                                 "into OUT (.tar.gz) for packaged CARLAs")
    ap.add_argument("--cooked", metavar="TAR", help="packaged build: install a cook you have")
    ap.add_argument("--carla-root", help="override the saved carla_root")
    ap.add_argument("--ue4-root", help="override the saved ue4_root")
    args = ap.parse_args()
    repo = catalog.resolve_repo(args.repo)

    if args.list:
        return list_vehicles(repo)
    if args.verify:
        from vehicles import verify
        return verify.main_check(catalog.resolve_id(args.verify), args.host, args.port, args.tm_port)

    c = resolve_carla(args.carla_root, args.ue4_root)
    if c["mode"] == "packaged":
        if args.cook:
            sys.exit("[vehicle] --cook needs a source build; this CARLA is packaged.")
        if args.uninstall:
            return uninstall_cooked(c)
        install_cooked(c, args.cooked or cooked_from_library(repo, args.vehicle or ""), args.wait)
        return 0
    if args.cooked:
        sys.exit("[vehicle] --cooked is for a packaged CARLA; a source build installs the bundle.")
    if args.cook:
        return cook_all(c, args.cook)
    carla_root, ue4_root, uproject, editor = c["carla_root"], c["ue4_root"], c["uproject"], c["editor"]
    if args.uninstall:
        return uninstall(catalog.resolve_id(args.uninstall), carla_root, ue4_root, uproject, editor,
                         args.keep_assets, args.restore_factory, args.wait, args.dry_run)
    m = bundle_from_local(args.bundle) if args.bundle else bundle_from_library(repo, args.vehicle)
    return install(m, carla_root, ue4_root, uproject, editor, register=not args.no_register,
                   register_only=args.register_only, wait_s=args.wait, dry_run=args.dry_run,
                   adopt=args.adopt)


if __name__ == "__main__":
    sys.exit(main())
