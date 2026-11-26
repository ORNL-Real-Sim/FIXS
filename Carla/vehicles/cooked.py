"""Library vehicles for a PACKAGED CARLA: cook them on a source build, install the cook elsewhere.

A packaged CARLA has no editor, so it cannot build a vehicle; it can load one that was
already cooked by the same CARLA version. The cook is a .tar.gz laid out like the
package root:

    fixs_vehicle_cook.json                              what it holds (COOK_INFO)
    manifests/<id>/vehicle.yaml                         the bridge reads physics from here
    CarlaUE4/Content/FIXS/Vehicles/<Asset>/...          each vehicle's cooked assets
    CarlaUE4/Content/FIXS/Vehicles/_Shared/...
    CarlaUE4/Content/Carla/Blueprints/Vehicles/VehicleFactory.uasset/.uexp

The cooked VehicleFactory is the stock list plus every vehicle in the cook, so a cook
replaces the whole set of library vehicles on a packaged install, never adds to it.
"""

import datetime
import io
import json
import os
import platform
import shutil
import subprocess
import tarfile

from . import catalog

COOK_INFO = "fixs_vehicle_cook.json"
COOK_SCHEMA = 1
CONTENT = "CarlaUE4/Content/"
VEHICLES_PREFIX = CONTENT + "FIXS/Vehicles/"
FACTORY_GAME_PATH = "/Game/Carla/Blueprints/Vehicles/VehicleFactory"
FACTORY_PREFIX = CONTENT + "Carla/Blueprints/Vehicles/VehicleFactory."
FACTORY_EXTS = (".uasset", ".uexp")


class CookError(Exception):
    pass


def os_tag(system=None):
    return "windows" if (system or platform.system()) == "Windows" else "linux"


def asset_name(bp_id, system=None):
    """Release asset of a vehicle's cook, beside its bundle zip (maps use the same suffix)."""
    return "{}_cooked_{}.tar.gz".format(bp_id, os_tag(system))


def catalog_cooked_asset(ent, system=None):
    """The catalog entry's cook for this OS: `cooked_asset_<os>`, else the naming rule."""
    key = "cooked_asset_" + os_tag(system)
    if key in ent:
        return ent[key] or None
    return asset_name(ent["id"], system)


def game_to_rel(game_path):
    """`/Game/FIXS/Vehicles/KiaEV6` -> `CarlaUE4/Content/FIXS/Vehicles/KiaEV6`."""
    if not game_path.startswith("/Game/"):
        raise CookError("not a /Game path: {!r}".format(game_path))
    return CONTENT + game_path[len("/Game/"):]


def allowed_member(name):
    """Whether a cook may write `name` into a packaged CARLA."""
    n = name.replace("\\", "/")
    if n.startswith("/") or ".." in n.split("/"):
        return False
    if n == COOK_INFO or (n.startswith("manifests/") and n.endswith("/vehicle.yaml")):
        return True
    if n.startswith(VEHICLES_PREFIX):
        return True
    return any(n == FACTORY_PREFIX[:-1] + ext for ext in FACTORY_EXTS)


def package_packages(asset_dirs, content_root):
    """/Game package names to cook: every asset under `asset_dirs`, plus the factory."""
    out = []
    for game_dir in asset_dirs:
        d = os.path.join(content_root, game_dir[len("/Game/"):].replace("/", os.sep))
        for root, _, names in os.walk(d):
            for n in sorted(names):
                if n.endswith(".uasset"):
                    rel = os.path.relpath(os.path.join(root, n[:-len(".uasset")]), content_root)
                    out.append("/Game/" + rel.replace(os.sep, "/"))
    return sorted(out) + [FACTORY_GAME_PATH]


def cook_command(editor_cmd, uproject, packages, out_dir, target="WindowsNoEditor"):
    return [editor_cmd, uproject, "-run=cook", "-targetplatform=" + target, "-cooksinglepackage",
            "-map=" + "+".join(packages), "-OutputDir=" + out_dir, "-unattended", "-nop4",
            "-NoLogTimes"]


def collect(out_dir, asset_dirs):
    """{archive name: file} for the cook's vehicle assets and factory under `out_dir`."""
    files = {}
    for game_dir in list(asset_dirs):
        rel = game_to_rel(game_dir)
        d = os.path.join(out_dir, rel.replace("/", os.sep))
        for root, _, names in os.walk(d):
            for n in names:
                p = os.path.join(root, n)
                files[os.path.relpath(p, out_dir).replace(os.sep, "/")] = p
    for ext in FACTORY_EXTS:
        rel = FACTORY_PREFIX[:-1] + ext
        p = os.path.join(out_dir, rel.replace("/", os.sep))
        if not os.path.isfile(p):
            raise CookError("the cook produced no {}".format(rel))
        files[rel] = p
    return files


def write_archive(path, files, info, manifests):
    """Pack `files` ({name: path}), COOK_INFO and each id's vehicle.yaml into a .tar.gz."""
    tmp = path + ".tmp"
    with tarfile.open(tmp, "w:gz") as tar:
        data = json.dumps(info, indent=2, sort_keys=True).encode("utf-8")
        ti = tarfile.TarInfo(COOK_INFO)
        ti.size = len(data)
        tar.addfile(ti, io.BytesIO(data))
        for bp_id, mpath in sorted(manifests.items()):
            tar.add(mpath, arcname="manifests/{}/vehicle.yaml".format(bp_id))
        for name in sorted(files):
            tar.add(files[name], arcname=name)
    os.replace(tmp, path)
    return path


def cook(carla_root, editor_cmd, uproject, receipts, out_path, work, carla_version, run=None):
    """Cook every vehicle FIXS registered in this SOURCE build into one archive.

    `receipts` are the registered install receipts; all of them go in, because the cooked
    factory references each one and a packaged CARLA cannot load a class it lacks."""
    if not receipts:
        raise CookError("no library vehicle is registered in {}; install one first".format(carla_root))
    content_root = os.path.join(carla_root, "Unreal", "CarlaUE4", "Content")
    asset_dirs = sorted({r["asset_dir"] for r in receipts}) + ["/Game/FIXS/Vehicles/_Shared"]
    out_dir = os.path.join(work, "cook_out")
    if os.path.isdir(out_dir):
        shutil.rmtree(out_dir)
    cmd = cook_command(editor_cmd, uproject, package_packages(asset_dirs, content_root), out_dir)
    log_path = os.path.join(work, "cook.log")
    with open(log_path, "w", encoding="utf-8") as fh:
        rc = (run or subprocess.call)(cmd, stdout=fh, stderr=subprocess.STDOUT)
    if rc != 0:
        raise CookError("the cook exited {}; see {}".format(rc, log_path))
    files = collect(out_dir, asset_dirs)
    info = {"schema": COOK_SCHEMA, "carla": carla_version, "platform": os_tag(),
            "ids": sorted(r["id"] for r in receipts),
            "asset_dirs": {r["id"]: r["asset_dir"] for r in receipts},
            "shared_dir": "/Game/FIXS/Vehicles/_Shared",
            "cooked_at": datetime.datetime.now().isoformat(timespec="seconds")}
    manifests = {r["id"]: r["manifest"] for r in receipts if r.get("manifest")}
    return write_archive(out_path, files, info, manifests)


def read_info(tar):
    try:
        info = json.load(tar.extractfile(COOK_INFO))
    except (KeyError, ValueError, AttributeError):
        raise CookError("not a FIXS vehicle cook: no readable {}".format(COOK_INFO))
    if info.get("schema") != COOK_SCHEMA or not info.get("ids"):
        raise CookError("unsupported {} (schema {!r})".format(COOK_INFO, info.get("schema")))
    return info


def factory_backup_dir(carla_root):
    return os.path.join(catalog.home(), "_factory_backup", catalog.root_key(carla_root))


def backup_factory(package_root):
    """Keep the packaged factory's original files once, then a stamped copy per install."""
    d = factory_backup_dir(package_root)
    os.makedirs(d, exist_ok=True)
    stamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
    for ext in FACTORY_EXTS:
        src = os.path.join(package_root, (FACTORY_PREFIX[:-1] + ext).replace("/", os.sep))
        if not os.path.isfile(src):
            raise CookError("{} is not a packaged CARLA (no {})".format(package_root, src))
        original = os.path.join(d, "VehicleFactory.original" + ext)
        if not os.path.isfile(original):
            shutil.copy2(src, original)
        shutil.copy2(src, os.path.join(d, "VehicleFactory.{}{}".format(stamp, ext)))
    return d


def restore_factory(package_root):
    d = factory_backup_dir(package_root)
    for ext in FACTORY_EXTS:
        original = os.path.join(d, "VehicleFactory.original" + ext)
        if not os.path.isfile(original):
            raise CookError("no original VehicleFactory backup in {}".format(d))
    for ext in FACTORY_EXTS:
        shutil.copy2(os.path.join(d, "VehicleFactory.original" + ext),
                     os.path.join(package_root, (FACTORY_PREFIX[:-1] + ext).replace("/", os.sep)))


def install(tar_path, package_root, carla_root, log=print):
    """Install a cook into a packaged CARLA. Returns the ids it registered.

    Every library vehicle registered here before and absent from this cook is
    unregistered: the cook's factory does not list it."""
    with tarfile.open(tar_path, "r:*") as tar:
        info = read_info(tar)
        members = tar.getmembers()
        bad = [m.name for m in members if not allowed_member(m.name) or not (m.isfile() or m.isdir())]
        if bad:
            raise CookError("{} holds files a vehicle cook may not write: {}".format(
                tar_path, ", ".join(bad[:5])))
        backup_factory(package_root)
        for bp_id, game_dir in info["asset_dirs"].items():
            d = os.path.join(package_root, game_to_rel(game_dir).replace("/", os.sep))
            if os.path.isdir(d):
                shutil.rmtree(d)
        content = [m for m in members if m.name.startswith(CONTENT)]
        tar.extractall(package_root, members=content)
        for bp_id in info["ids"]:
            mf = "manifests/{}/vehicle.yaml".format(bp_id)
            dest = catalog.bundle_dir(bp_id)
            names = {m.name for m in members}
            if mf in names and not os.path.isfile(os.path.join(dest, "vehicle.yaml")):
                os.makedirs(dest, exist_ok=True)
                with open(os.path.join(dest, "vehicle.yaml"), "wb") as fh:
                    fh.write(tar.extractfile(mf).read())
    for bp_id, r in catalog.registered_receipts(carla_root):
        if bp_id not in info["ids"]:
            r["registered"] = False
            catalog.write_receipt(bp_id, r)
            log("{} is not in this cook; it is no longer registered here".format(bp_id))
    for bp_id in info["ids"]:
        manifest = os.path.join(catalog.bundle_dir(bp_id), "vehicle.yaml")
        catalog.write_receipt(bp_id, {
            "id": bp_id, "carla_root": os.path.abspath(carla_root), "mode": "packaged",
            "asset_dir": info["asset_dirs"][bp_id],
            "manifest": manifest if os.path.isfile(manifest) else None,
            "registered": True, "cook": os.path.abspath(tar_path), "cook_carla": info.get("carla"),
            "installed_at": datetime.datetime.now().isoformat(timespec="seconds")})
    return list(info["ids"])


def uninstall(package_root, carla_root, log=print):
    """Put the stock factory back and delete every library vehicle cooked into this install."""
    restore_factory(package_root)
    gone = []
    for bp_id, r in catalog.registered_receipts(carla_root, include_unregistered=True):
        if r.get("asset_dir"):
            d = os.path.join(package_root, game_to_rel(r["asset_dir"]).replace("/", os.sep))
            if os.path.isdir(d):
                shutil.rmtree(d)
        catalog.remove_receipt(bp_id, carla_root)
        gone.append(bp_id)
    shared = os.path.join(package_root, (VEHICLES_PREFIX + "_Shared").replace("/", os.sep))
    if os.path.isdir(shared):
        shutil.rmtree(shared)
    vehicles = os.path.dirname(shared)
    if os.path.isdir(vehicles) and not os.listdir(vehicles):
        os.rmdir(vehicles)
    return gone
