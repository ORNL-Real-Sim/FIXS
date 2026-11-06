"""The vehicle library's catalog, its release assets, and what this machine has installed.

The library repo's root catalog.json carries a `vehicles` list beside the map library's
`maps`:

    {"schema": 1,
     "vehicles": [{"id": "vehicle.kia.ev6", "title": "...", "release": "ev6",
                   "asset": "vehicle.kia.ev6.zip", "sha256": "<optional>"}]}

Machine state lives in ~/.fixs/vehicles/ ($FIXS_VEHICLE_CACHE), outside any checkout:

    <id>/bundle/          the extracted bundle (vehicle.yaml, rig/, ...)
    <id>/installs/*.json  one receipt per CARLA install, written by import_vehicle.py
    _factory_backup/      VehicleFactory.uasset copies taken before the first edit
    catalog.json          last fetched catalog, for offline use

Stdlib only at import time: the CARLA bridge imports installed_manifest() from here.
"""

import hashlib
import json
import os
import shutil
import subprocess
import zipfile

DEFAULT_REPO = "ORNL-Real-Sim/VehicleDynamicsLibrary"
CATALOG_SCHEMA = 1

# The 41 vehicles CARLA 0.9.15's own VehicleFactory lists: never a library install.
STOCK_IDS = frozenset((
    "vehicle.audi.a2", "vehicle.audi.etron", "vehicle.audi.tt", "vehicle.bh.crossbike",
    "vehicle.bmw.grandtourer", "vehicle.carlamotors.carlacola", "vehicle.carlamotors.european_hgv",
    "vehicle.carlamotors.firetruck", "vehicle.chevrolet.impala", "vehicle.citroen.c3",
    "vehicle.diamondback.century", "vehicle.dodge.charger_2020", "vehicle.dodge.charger_police",
    "vehicle.dodge.charger_police_2020", "vehicle.ford.ambulance", "vehicle.ford.crown",
    "vehicle.ford.mustang", "vehicle.gazelle.omafiets", "vehicle.harley-davidson.low_rider",
    "vehicle.jeep.wrangler_rubicon", "vehicle.kawasaki.ninja", "vehicle.lincoln.mkz_2017",
    "vehicle.lincoln.mkz_2020", "vehicle.mercedes.coupe", "vehicle.mercedes.coupe_2020",
    "vehicle.mercedes.sprinter", "vehicle.micro.microlino", "vehicle.mini.cooper_s",
    "vehicle.mini.cooper_s_2021", "vehicle.mitsubishi.fusorosa", "vehicle.nissan.micra",
    "vehicle.nissan.patrol", "vehicle.nissan.patrol_2021", "vehicle.seat.leon",
    "vehicle.tesla.cybertruck", "vehicle.tesla.model3", "vehicle.toyota.prius",
    "vehicle.vespa.zx125", "vehicle.volkswagen.t2", "vehicle.volkswagen.t2_2021",
    "vehicle.yamaha.yzf"))


class CatalogError(Exception):
    pass


# ---------------------------------------------------------------- local state

def home():
    d = os.environ.get("FIXS_VEHICLE_CACHE") or os.path.join(os.path.expanduser("~"), ".fixs", "vehicles")
    return d


def vehicle_dir(bp_id):
    return os.path.join(home(), bp_id)


def bundle_dir(bp_id):
    return os.path.join(vehicle_dir(bp_id), "bundle")


def _same_root(a, b):
    return os.path.normcase(os.path.abspath(a or "")) == os.path.normcase(os.path.abspath(b or ""))


def root_key(carla_root):
    """Short stable name for one CARLA install."""
    return hashlib.sha1(os.path.normcase(os.path.abspath(carla_root)).encode("utf-8")).hexdigest()[:10]


def receipt_path(bp_id, carla_root):
    """One receipt per CARLA install, so a source and a packaged build each keep their own."""
    return os.path.join(vehicle_dir(bp_id), "installs", root_key(carla_root) + ".json")


def _legacy_receipt_path(bp_id):
    return os.path.join(vehicle_dir(bp_id), "install.json")


def _load_json(path):
    try:
        with open(path, "r", encoding="utf-8") as fh:
            return json.load(fh)
    except (OSError, ValueError):
        return None


def vehicle_receipts(bp_id):
    """Every install receipt of `bp_id`, one per CARLA install."""
    d = os.path.join(vehicle_dir(bp_id), "installs")
    out = [_load_json(os.path.join(d, n)) for n in sorted(os.listdir(d))
           if n.endswith(".json")] if os.path.isdir(d) else []
    out = [r for r in out if r]
    legacy = _load_json(_legacy_receipt_path(bp_id))
    if legacy and not any(_same_root(r.get("carla_root"), legacy.get("carla_root")) for r in out):
        out.append(legacy)
    return out


def read_receipt(bp_id, carla_root=None):
    """The receipt for `bp_id` in `carla_root`; without a root, the first one found."""
    for r in vehicle_receipts(bp_id):
        if carla_root is None or _same_root(r.get("carla_root"), carla_root):
            return r
    return None


def write_receipt(bp_id, receipt):
    path = receipt_path(bp_id, receipt["carla_root"])
    os.makedirs(os.path.dirname(path), exist_ok=True)
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as fh:
        json.dump(receipt, fh, indent=2, sort_keys=True)
    os.replace(tmp, path)
    legacy = _load_json(_legacy_receipt_path(bp_id))
    if legacy and _same_root(legacy.get("carla_root"), receipt["carla_root"]):
        os.remove(_legacy_receipt_path(bp_id))


def remove_receipt(bp_id, carla_root):
    for path in (receipt_path(bp_id, carla_root), _legacy_receipt_path(bp_id)):
        r = _load_json(path)
        if r is not None and _same_root(r.get("carla_root"), carla_root):
            os.remove(path)


def _receipts(carla_root=None):
    if not os.path.isdir(home()):
        return []
    out = []
    for name in sorted(os.listdir(home())):
        for r in vehicle_receipts(name):
            if carla_root and not _same_root(r.get("carla_root"), carla_root):
                continue
            out.append((name, r))
    return out


def registered_receipts(carla_root, include_unregistered=False):
    """[(id, receipt)] for the library vehicles registered in `carla_root`."""
    return [(n, r) for n, r in _receipts(carla_root) if include_unregistered or r.get("registered")]


def installed_ids(carla_root=None):
    """Ids whose receipt (for `carla_root`, when given) says the factory entry is in place."""
    return [n for n, r in _receipts(carla_root) if r.get("registered")]


def owned_ids(carla_root=None):
    """Ids FIXS installed into `carla_root`: the only factory entries it may replace or remove."""
    return [n for n, _ in _receipts(carla_root)]


def resolve_id(name):
    """`vehicle.kia.ev6` as given, or the cached vehicle whose id ends in `.ev6`."""
    if name.startswith("vehicle."):
        return name
    local = os.listdir(home()) if os.path.isdir(home()) else []
    hits = [d for d in local if d.startswith("vehicle.") and d.lower().endswith("." + name.lower())]
    return hits[0] if len(hits) == 1 else name


def installed_manifest(bp_id):
    """Path of the vehicle.yaml installed for blueprint `bp_id`, or None (stock blueprint)."""
    if not bp_id or not bp_id.startswith("vehicle."):
        return None
    p = os.path.join(bundle_dir(bp_id), "vehicle.yaml")
    return p if os.path.isfile(p) else None


# ---------------------------------------------------------------- catalog

def parse_catalog(doc):
    """The validated `vehicles` list of a catalog document."""
    if not isinstance(doc, dict):
        raise CatalogError("catalog.json is not a JSON object")
    vehicles = doc.get("vehicles")
    if vehicles is None:
        return []
    if not isinstance(vehicles, list):
        raise CatalogError("catalog.json `vehicles` must be a list")
    out, seen = [], set()
    for i, v in enumerate(vehicles):
        if not isinstance(v, dict):
            raise CatalogError("vehicles[{}] is not an object".format(i))
        missing = [k for k in ("id", "release", "asset") if not v.get(k)]
        if missing:
            raise CatalogError("vehicles[{}] ({}) lacks {}".format(i, v.get("id", "?"), ", ".join(missing)))
        if not str(v["id"]).startswith("vehicle.") or v["id"].count(".") != 2:
            raise CatalogError("vehicles[{}].id {!r} is not vehicle.<make>.<model>".format(i, v["id"]))
        if v["id"] in seen:
            raise CatalogError("vehicles: {} is listed twice".format(v["id"]))
        seen.add(v["id"])
        out.append(dict(v, title=v.get("title") or v["id"]))
    return out


def entry(vehicles, name):
    """The entry known by `name`: its full id, or the id's model part (`ev6`)."""
    if not name:
        return None
    name = name.lower()
    for v in vehicles:
        if name == v["id"].lower():
            return v
    hits = [v for v in vehicles if v["id"].lower().rsplit(".", 1)[-1] == name]
    return hits[0] if len(hits) == 1 else None


def fetch_catalog(repo):
    """(vehicles, source) from `repo`'s catalog.json via gh, else the cached copy."""
    cache = os.path.join(home(), "catalog.json")
    gh = shutil.which("gh")
    if gh:
        try:
            out = subprocess.run([gh, "api", "repos/{}/contents/catalog.json".format(repo),
                                  "-H", "Accept: application/vnd.github.raw+json"],
                                 capture_output=True, text=True, timeout=20)
            if out.returncode == 0 and out.stdout.strip():
                doc = json.loads(out.stdout)
                vehicles = parse_catalog(doc)
                os.makedirs(home(), exist_ok=True)
                with open(cache, "w", encoding="utf-8") as fh:
                    fh.write(out.stdout)
                return vehicles, repo
        except (OSError, ValueError, subprocess.SubprocessError):
            pass
    if os.path.isfile(cache):
        with open(cache, "r", encoding="utf-8") as fh:
            return parse_catalog(json.load(fh)), cache
    return [], None


def resolve_repo(repo=None, app_root=None):
    """Explicit repo, else `vehicle_repo=` in the app's fixs_sources.txt, else DEFAULT_REPO."""
    if repo:
        return repo
    root = app_root or os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.dirname(os.path.abspath(__file__)))))
    src = os.path.join(root, "fixs_sources.txt")
    if os.path.isfile(src):
        with open(src, "r", encoding="utf-8") as fh:
            for line in fh:
                key, sep, val = line.strip().partition("=")
                if sep and key.strip().lower() == "vehicle_repo" and val.strip():
                    return val.strip()
    return DEFAULT_REPO


# ---------------------------------------------------------------- fetching

def sha256_of(path):
    h = hashlib.sha256()
    with open(path, "rb") as fh:
        for chunk in iter(lambda: fh.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def download(repo, ent, dest_dir):
    """Download the entry's release asset into `dest_dir` with gh; returns the zip path."""
    gh = shutil.which("gh")
    if not gh:
        raise CatalogError("downloading a library vehicle needs the GitHub CLI (gh) with "
                           "access to {}; or pass --bundle <zip|dir> for a copy you have.".format(repo))
    os.makedirs(dest_dir, exist_ok=True)
    rc = subprocess.call([gh, "release", "download", ent["release"], "-R", repo,
                          "-p", ent["asset"], "-D", dest_dir, "--clobber"])
    path = os.path.join(dest_dir, ent["asset"])
    if rc != 0 or not os.path.isfile(path):
        raise CatalogError("`gh release download {} -p {}` from {} failed (rc={})".format(
            ent["release"], ent["asset"], repo, rc))
    want = (ent.get("sha256") or "").lower().replace("sha256:", "")
    if want and sha256_of(path) != want:
        raise CatalogError("{} does not match the catalog's sha256; refusing it".format(path))
    return path


def extract(zip_path, dest):
    """Extract a bundle zip into `dest` (replaced), refusing members that escape it.

    A bundle zipped with one top-level folder is unwrapped, so vehicle.yaml ends up at
    `dest/vehicle.yaml` either way."""
    root = os.path.realpath(dest)
    if os.path.isdir(dest):
        shutil.rmtree(dest)
    os.makedirs(dest)
    with zipfile.ZipFile(zip_path) as z:
        for name in z.namelist():
            target = os.path.realpath(os.path.join(dest, name))
            if target != root and not target.startswith(root + os.sep):
                raise CatalogError("refusing to extract {!r}: it escapes {}".format(name, dest))
        z.extractall(dest)
    if not os.path.isfile(os.path.join(dest, "vehicle.yaml")):
        inner = [d for d in os.listdir(dest) if os.path.isdir(os.path.join(dest, d))]
        if len(inner) == 1 and os.path.isfile(os.path.join(dest, inner[0], "vehicle.yaml")):
            tmp = dest + ".unwrap"
            os.replace(os.path.join(dest, inner[0]), tmp)
            shutil.rmtree(dest)
            os.replace(tmp, dest)
    if not os.path.isfile(os.path.join(dest, "vehicle.yaml")):
        raise CatalogError("{} has no vehicle.yaml at its root".format(zip_path))
    return dest


def stage_bundle(src, bp_id=None):
    """Copy/extract a local bundle (zip or folder) into the cache; returns the bundle dir."""
    if os.path.isfile(src) and src.lower().endswith(".zip"):
        tmp = os.path.join(home(), "_incoming")
        extract(src, tmp)
        src_dir = tmp
    elif os.path.isdir(src):
        src_dir = src
    else:
        raise CatalogError("--bundle {} is neither a .zip nor a folder".format(src))
    if not os.path.isfile(os.path.join(src_dir, "vehicle.yaml")):
        raise CatalogError("{} has no vehicle.yaml".format(src_dir))
    return src_dir
