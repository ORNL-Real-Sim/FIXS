"""vehicle.yaml: the manifest a vehicle bundle ships, its schema and validation.

The library stores vehicles; FIXS reads them. Every value that describes the car comes
from here, and a missing required value is a hard error naming the key, never a FIXS
default. All units are SI (m, kg, N*m, deg, km/h for the steering curve); conversion to
CARLA's units happens in physics_plan / unreal_job.

Bundle layout (the release zip, or a folder):

    vehicle.yaml          this manifest
    README.md             credit / licence notice
    source/...            the source model prep reads (optional in a published bundle)
    rig/vehicle.fbx       rigged skeletal mesh written by prep_vehicle.py
    rig/materials.json    material slot table written by prep_vehicle.py
    rig/textures/         textures the slots reference
    rig/prep_report.json  what prep measured, for review
"""

import os
import re

SCHEMA = 1
MANIFEST_NAME = "vehicle.yaml"
RIG_DIR = "rig"
RIG_FBX = "rig/vehicle.fbx"
RIG_MATERIALS = "rig/materials.json"
RIG_TEXTURES = "rig/textures"
RIG_REPORT = "rig/prep_report.json"

WHEEL_BONES = ("Wheel_Front_Left", "Wheel_Front_Right", "Wheel_Rear_Left", "Wheel_Rear_Right")
# CARLA's VehiclePhysicsControl.wheels order on a 4-wheeled PhysX vehicle.
CARLA_WHEEL_ORDER = ("front_left", "front_right", "rear_left", "rear_right")

AXES = ("+x", "-x", "+y", "-y", "+z", "-z")
FORMATS = {".dae": "collada", ".fbx": "fbx", ".glb": "gltf", ".gltf": "gltf", ".obj": "obj"}
MATERIAL_ROLES = ("paint", "glass", "light_glass")
BASE_TYPES = ("car", "truck", "van", "motorcycle", "bicycle", "bus")
DIFFERENTIALS = ("limited_slip_4w", "limited_slip_front_drive", "limited_slip_rear_drive",
                 "open_4w", "open_front_drive", "open_rear_drive")
# Dimensions prep measures on the mesh and checks; width_m (body, no mirrors) is not one.
CHECKED_DIMENSIONS = ("length_m", "width_mirrors_m", "height_m", "wheelbase_m",
                      "track_front_m", "track_rear_m")

VEHICLE_SCALARS = {  # manifest key -> VehiclePhysicsControl attribute
    "mass_kg": "mass",
    "drag_coefficient": "drag_coefficient",
    "max_rpm": "max_rpm",
    "moi": "moi",
    "damping_rate_full_throttle": "damping_rate_full_throttle",
    "damping_rate_zero_throttle_clutch_engaged": "damping_rate_zero_throttle_clutch_engaged",
    "damping_rate_zero_throttle_clutch_disengaged": "damping_rate_zero_throttle_clutch_disengaged",
    "gear_switch_time": "gear_switch_time",
    "clutch_strength": "clutch_strength",
    "final_ratio": "final_ratio",
}
TIRE_SCALARS = ("tire_friction", "damping_rate", "max_brake_torque", "max_handbrake_torque",
                "lat_stiff_max_load", "lat_stiff_value", "long_stiff_value")


class ManifestError(Exception):
    """The manifest is unreadable, or names a value wrongly or not at all."""


def find(bundle_dir):
    """Path of the bundle's vehicle.yaml, or None."""
    p = os.path.join(bundle_dir, MANIFEST_NAME)
    return p if os.path.isfile(p) else None


def load(path):
    """Parse and validate `path`; returns the normalized manifest (see validate)."""
    try:
        import yaml
    except ImportError as exc:  # pragma: no cover - environment.yml declares pyyaml
        raise ManifestError("cannot read {}: pyyaml is not installed ({})".format(path, exc))
    try:
        with open(path, "r", encoding="utf-8") as fh:
            raw = yaml.safe_load(fh)
    except Exception as exc:
        raise ManifestError("cannot parse {}: {}".format(path, exc))
    return validate(raw, path)


def for_blueprint(bp_id, explicit=""):
    """(manifest, path) for blueprint `bp_id`, or (None, None) when it has none (a stock car).

    `explicit` is CarlaSetup.EgoVehicleManifest (relative to the working directory); without
    it the manifest import_vehicle.py installed for `bp_id` is used."""
    from . import catalog
    path = explicit or catalog.installed_manifest(bp_id)
    if not path:
        return None, None
    path = os.path.abspath(path)
    if not os.path.isfile(path):
        raise ManifestError("CarlaSetup.EgoVehicleManifest {} does not exist".format(path))
    m = load(path)
    if m["id"] != bp_id:
        raise ManifestError("{} describes {}, but CarlaSetup.EgoBlueprint is {}".format(
            path, m["id"], bp_id))
    return m, path


# ---------------------------------------------------------------- validation

class _Checker:
    def __init__(self, path):
        self.path = path
        self.errors = []

    def err(self, key, msg):
        self.errors.append("{}: {}".format(key, msg))

    def section(self, raw, key, required=True):
        v = raw.get(key) if isinstance(raw, dict) else None
        if v is None:
            if required:
                self.err(key, "required section is missing")
            return None
        if not isinstance(v, dict):
            self.err(key, "must be a mapping")
            return None
        return v

    def value(self, block, prefix, key, required=True):
        if not isinstance(block, dict) or block.get(key) is None:
            if required:
                self.err(prefix + key, "required")
            return None
        return block[key]

    def string(self, block, prefix, key, required=True, pattern=None):
        v = self.value(block, prefix, key, required)
        if v is None:
            return None
        if not isinstance(v, str) or not v.strip():
            self.err(prefix + key, "must be a non-empty string")
            return None
        if pattern and not re.fullmatch(pattern, v):
            self.err(prefix + key, "{!r} does not match {}".format(v, pattern))
            return None
        return v.strip()

    def number(self, block, prefix, key, required=True, lo=None, hi=None, lo_open=False):
        v = self.value(block, prefix, key, required)
        if v is None:
            return None
        if isinstance(v, bool) or not isinstance(v, (int, float)):
            self.err(prefix + key, "must be a number, got {!r}".format(v))
            return None
        v = float(v)
        if lo is not None and (v < lo or (lo_open and v == lo)):
            self.err(prefix + key, "must be {} {}, got {}".format(">" if lo_open else ">=", lo, v))
            return None
        if hi is not None and v > hi:
            self.err(prefix + key, "must be <= {}, got {}".format(hi, v))
            return None
        return v

    def integer(self, block, prefix, key, required=True, lo=None):
        v = self.value(block, prefix, key, required)
        if v is None:
            return None
        if isinstance(v, bool) or not isinstance(v, int):
            self.err(prefix + key, "must be an integer, got {!r}".format(v))
            return None
        if lo is not None and v < lo:
            self.err(prefix + key, "must be >= {}, got {}".format(lo, v))
            return None
        return v

    def flag(self, block, prefix, key, required=True):
        v = self.value(block, prefix, key, required)
        if v is None:
            return None
        if not isinstance(v, bool):
            self.err(prefix + key, "must be true or false, got {!r}".format(v))
            return None
        return v

    def strings(self, block, prefix, key, required=True, non_empty=True):
        v = self.value(block, prefix, key, required)
        if v is None:
            return [] if not required else None
        if isinstance(v, str):
            v = [v]
        if not isinstance(v, list) or not all(isinstance(s, str) and s for s in v):
            self.err(prefix + key, "must be a list of non-empty strings")
            return None
        if non_empty and not v:
            self.err(prefix + key, "must not be empty")
            return None
        return list(v)

    def vec(self, block, prefix, key, n, required=True):
        v = self.value(block, prefix, key, required)
        if v is None:
            return None
        if (not isinstance(v, (list, tuple)) or len(v) != n
                or not all(isinstance(x, (int, float)) and not isinstance(x, bool) for x in v)):
            self.err(prefix + key, "must be a list of {} numbers, got {!r}".format(n, v))
            return None
        return [float(x) for x in v]

    def range_(self, block, prefix, key, required=True):
        v = self.vec(block, prefix, key, 2, required)
        if v is not None and not 0 < v[0] < v[1]:
            self.err(prefix + key, "must be [low, high] with 0 < low < high, got {}".format(v))
            return None
        return v

    def curve(self, block, prefix, key, required=False):
        v = self.value(block, prefix, key, required)
        if v is None:
            return None
        ok = (isinstance(v, list) and len(v) >= 2 and all(
            isinstance(p, (list, tuple)) and len(p) == 2 and all(
                isinstance(x, (int, float)) and not isinstance(x, bool) for x in p) for p in v))
        if not ok:
            self.err(prefix + key, "must be a list of at least two [x, y] pairs")
            return None
        pts = [(float(a), float(b)) for a, b in v]
        if any(b[0] <= a[0] for a, b in zip(pts, pts[1:])):
            self.err(prefix + key, "x values must increase strictly")
            return None
        return pts

    def color(self, prefix, c):
        if (not isinstance(c, (list, tuple)) or len(c) not in (3, 4)
                or not all(isinstance(x, int) and not isinstance(x, bool) and 0 <= x <= 255 for x in c)):
            self.err(prefix, "colour must be [r, g, b] with integers 0..255, got {!r}".format(c))
            return None
        return [int(x) for x in c[:3]]


def axis_vector(axis):
    sign = -1.0 if axis[0] == "-" else 1.0
    v = [0.0, 0.0, 0.0]
    v[{"x": 0, "y": 1, "z": 2}[axis[1]]] = sign
    return v


def validate(raw, path="vehicle.yaml"):
    """Normalized manifest dict, or ManifestError listing every problem found."""
    c = _Checker(path)
    if not isinstance(raw, dict):
        raise ManifestError("{} is not a mapping".format(path))
    m = {"path": os.path.abspath(path), "dir": os.path.dirname(os.path.abspath(path))}

    schema = raw.get("schema")
    if schema != SCHEMA:
        c.err("schema", "must be {} (this FIXS reads schema {}), got {!r}".format(SCHEMA, SCHEMA, schema))
    m["schema"] = schema

    name_re = r"[A-Za-z0-9_]+"
    m["make"] = c.string(raw, "", "make", pattern=name_re)
    m["model"] = c.string(raw, "", "model", pattern=name_re)
    m["id"] = c.string(raw, "", "id")
    if m["make"] and m["model"] and m["id"]:
        want = "vehicle.{}.{}".format(m["make"], m["model"]).lower()
        if m["id"] != want:
            c.err("id", "CARLA derives {!r} from make/model, but id says {!r}".format(want, m["id"]))
    m["title"] = c.string(raw, "", "title", required=False) or m["id"]
    m["asset_name"] = c.string(raw, "", "asset_name", required=False, pattern=r"[A-Za-z][A-Za-z0-9_]*")
    if not m["asset_name"] and m["make"] and m["model"]:
        m["asset_name"] = re.sub(r"[^A-Za-z0-9]", "", m["make"].title() + m["model"])

    eng = c.section(raw, "engine")
    m["engine"] = {"unreal": str(eng["unreal"]) if eng and eng.get("unreal") is not None else None,
                   "carla": str(eng["carla"]) if eng and eng.get("carla") is not None else None}
    if eng is not None:
        for k in ("unreal", "carla"):
            if m["engine"][k] is None:
                c.err("engine." + k, "required")

    cr = c.section(raw, "credit")
    m["credit"] = {k: c.string(cr, "credit.", k, required=k in ("author", "license", "source_url"))
                   for k in ("title", "author", "source_url", "license", "license_url", "notice")}

    m["source"] = _source(c, raw)
    m["materials"] = _materials(c, raw)
    m["dimensions"], m["tolerance_m"] = _dimensions(c, raw)

    wh = c.section(raw, "wheels")
    m["wheels"] = {
        "radius_m": c.number(wh, "wheels.", "radius_m", lo=0.1, hi=1.5),
        "width_m": c.number(wh, "wheels.", "width_m", lo=0.05, hi=1.0),
        "max_steer_deg": c.number(wh, "wheels.", "max_steer_deg", lo=0.0, hi=89.0, lo_open=True),
        "rear_max_steer_deg": c.number(wh, "wheels.", "rear_max_steer_deg", required=False,
                                       lo=0.0, hi=89.0) or 0.0,
        "radius_tolerance_m": c.number(wh, "wheels.", "radius_tolerance_m", required=False,
                                       lo=0.0, lo_open=True),
    }

    m["physics"] = _physics(c, raw)
    m["drivetrain"] = _drivetrain(c, raw)
    m["factory"] = _factory(c, raw)

    if c.errors:
        raise ManifestError("{} is not a valid vehicle manifest:\n  - {}".format(
            path, "\n  - ".join(c.errors)))
    return m


def _source(c, raw):
    src = c.section(raw, "source", required=False)
    if src is None:
        return None
    p = "source."
    out = {"file": c.string(src, p, "file")}
    fmt = src.get("format")
    if fmt is None and out["file"]:
        fmt = FORMATS.get(os.path.splitext(out["file"])[1].lower())
    if fmt not in FORMATS.values():
        c.err(p + "format", "must be one of {} (or a file extension that implies one), got {!r}"
              .format(sorted(set(FORMATS.values())), fmt))
    out["format"] = fmt
    out["scale"] = c.number(src, p, "scale", lo=0.0, lo_open=True)
    out["forward"] = c.value(src, p, "forward")
    out["up"] = c.value(src, p, "up")
    for k in ("forward", "up"):
        if out[k] is not None and out[k] not in AXES:
            c.err(p + k, "must be one of {}, got {!r}".format(AXES, out[k]))
            out[k] = None
    if out["forward"] and out["up"] and out["forward"][1] == out["up"][1]:
        c.err(p + "up", "must be perpendicular to forward")
    out["dedupe_twins"] = c.flag(src, p, "dedupe_twins", required=False) or False
    out["decimate"] = c.number(src, p, "decimate", required=False, lo=0.0, hi=1.0, lo_open=True) or 1.0

    wd = c.section(src, "wheels", required=False) if isinstance(src, dict) else None
    out["wheels"] = None
    if wd is not None:
        if "nodes" in wd:
            nodes = wd["nodes"]
            if not isinstance(nodes, dict) or set(nodes) != set(CARLA_WHEEL_ORDER) or not all(
                    isinstance(v, str) and v for v in nodes.values()):
                c.err(p + "wheels.nodes", "must map each of {} to a node name".format(CARLA_WHEEL_ORDER))
            else:
                out["wheels"] = {"nodes": dict(nodes)}
        elif "detect" in wd:
            det = wd["detect"] if isinstance(wd["detect"], dict) else None
            if det is None:
                c.err(p + "wheels.detect", "must be a mapping")
            else:
                out["wheels"] = {"detect": {
                    "diameter_m": c.range_(det, p + "wheels.detect.", "diameter_m"),
                    "width_m": c.range_(det, p + "wheels.detect.", "width_m")}}
        else:
            c.err(p + "wheels", "needs either nodes: {...} or detect: {diameter_m, width_m}")
    elif isinstance(src, dict):
        c.err(p + "wheels", "required: how prep finds the four wheels (nodes or detect)")
    return out


LOOK_KEYS = (("metallic", 1.0), ("roughness", 1.0), ("specular", 1.0), ("emissive", 100.0))


def _look(c, item, q):
    look = {}
    for k, hi in LOOK_KEYS:
        look[k] = c.number(item, q, k, required=False, lo=0.0, hi=hi)
    if "color" in item:
        look["color"] = c.color(q + "color", item["color"])
    return look


def _materials(c, raw):
    mat = c.section(raw, "materials")
    p = "materials."
    out = {"paint": c.strings(mat, p, "paint") or [],
           "glass": c.strings(mat, p, "glass", required=False, non_empty=False) or [],
           "light_glass": c.strings(mat, p, "light_glass", required=False, non_empty=False) or [],
           "strip": c.strings(mat, p, "strip", required=False, non_empty=False) or [],
           "look": [], "wheel_slots": [], "plate": None}
    for i, item in enumerate((mat or {}).get("look") or []):
        q = "{}look[{}].".format(p, i)
        if not isinstance(item, dict):
            c.err(q[:-1], "must be a mapping")
            continue
        out["look"].append(dict(_look(c, item, q), match=c.string(item, q, "match")))
    # a source material's triangles on the wheels split into their own slot (e.g. rims)
    for i, item in enumerate((mat or {}).get("wheel_slots") or []):
        q = "{}wheel_slots[{}].".format(p, i)
        if not isinstance(item, dict):
            c.err(q[:-1], "must be a mapping")
            continue
        out["wheel_slots"].append(dict(_look(c, item, q), match=c.string(item, q, "match"),
                                       slot=c.string(item, q, "slot", pattern=r"[A-Za-z0-9_]+")))
    plate = (mat or {}).get("plate")
    if plate is not None:
        q = p + "plate."
        if not isinstance(plate, dict):
            c.err(q[:-1], "must be a mapping")
        else:
            # match is the exact source material name, so a generic word cannot catch other slots
            size = c.vec(plate, q, "size_m", 2, required=False)
            if size is not None and not (0.05 <= min(size) and max(size) <= 1.0):
                c.err(q + "size_m", "must be [width, height] in metres, each 0.05-1.0, got {}".format(size))
            out["plate"] = dict(_look(c, plate, q), match=c.string(plate, q, "match"),
                                texture=c.string(plate, q, "texture", required=False), size_m=size)
    return out


def _dimensions(c, raw):
    d = c.section(raw, "dimensions")
    p = "dimensions."
    tol = c.number(d, p, "tolerance_m", lo=0.0, lo_open=True)
    out = {}
    required = ("length_m", "width_m", "height_m", "wheelbase_m", "track_front_m",
                "track_rear_m", "ground_clearance_m")
    for key in required + ("width_mirrors_m",):
        v = (d or {}).get(key)
        if v is None:
            if key in required:
                c.err(p + key, "required")
            continue
        if isinstance(v, dict):
            val = c.number(v, p + key + ".", "value", lo=0.0, lo_open=True)
            t = c.number(v, p + key + ".", "tolerance_m", lo=0.0, lo_open=True)
        else:
            val = c.number(d, p, key, lo=0.0, lo_open=True)
            t = tol
        if val is not None:
            out[key] = {"value": val, "tolerance_m": t}
    if "ground_clearance_m" in out and "height_m" in out and \
            out["ground_clearance_m"]["value"] >= out["height_m"]["value"]:
        c.err(p + "ground_clearance_m", "must be below height_m")
    return out, tol


def _physics(c, raw):
    ph = c.section(raw, "physics")
    p = "physics."
    out = {}
    for key in VEHICLE_SCALARS:
        req = key in ("mass_kg", "drag_coefficient")
        lo = 0.0 if key != "mass_kg" else 1.0
        out[key] = c.number(ph, p, key, required=req, lo=lo)
    out["use_gear_autobox"] = c.flag(ph, p, "use_gear_autobox", required=False)
    out["center_of_mass_m"] = c.vec(ph, p, "center_of_mass_m", 3, required=False)
    out["torque_curve"] = c.curve(ph, p, "torque_curve")
    out["steering_curve"] = c.curve(ph, p, "steering_curve")
    gears = (ph or {}).get("forward_gears")
    out["forward_gears"] = None
    if gears is not None:
        parsed = []
        if not isinstance(gears, list) or not gears:
            c.err(p + "forward_gears", "must be a non-empty list of {ratio, down_ratio, up_ratio}")
        else:
            for i, g in enumerate(gears):
                q = "{}forward_gears[{}].".format(p, i)
                parsed.append({k: c.number(g if isinstance(g, dict) else None, q, k, lo=0.0)
                               for k in ("ratio", "down_ratio", "up_ratio")})
        out["forward_gears"] = parsed
    tires = (ph or {}).get("tires")
    out["tires"] = {"front": {}, "rear": {}}
    if tires is not None:
        if not isinstance(tires, dict) or not set(tires) <= {"front", "rear", "all"}:
            c.err(p + "tires", "must be a mapping with keys from all / front / rear")
        else:
            for axle in ("all", "front", "rear"):
                blk = tires.get(axle)
                if blk is None:
                    continue
                if not isinstance(blk, dict) or not set(blk) <= set(TIRE_SCALARS):
                    c.err(p + "tires." + axle, "keys must be from {}".format(TIRE_SCALARS))
                    continue
                vals = {k: c.number(blk, p + "tires." + axle + ".", k, lo=0.0) for k in blk}
                targets = ("front", "rear") if axle == "all" else (axle,)
                for t in targets:
                    out["tires"][t].update(vals)
    return out


def _drivetrain(c, raw):
    dt = c.section(raw, "drivetrain", required=False)
    if dt is None:
        return None
    p = "drivetrain."
    diff = c.string(dt, p, "differential")
    if diff and diff not in DIFFERENTIALS:
        c.err(p + "differential", "must be one of {}, got {!r}".format(DIFFERENTIALS, diff))
    # PhysX FrontRearSplit is the front axle's share of drive torque (>0.5 = more to the front)
    return {"differential": diff,
            "front_rear_split": c.number(dt, p, "front_torque_fraction", lo=0.0, hi=1.0)}


def _factory(c, raw):
    f = c.section(raw, "factory")
    p = "factory."
    out = {"base_type": c.string(f, p, "base_type"),
           "special_type": c.string(f, p, "special_type", required=False) or "",
           "object_type": c.string(f, p, "object_type", required=False) or "",
           "generation": c.integer(f, p, "generation", lo=0),
           "recommended_colors": []}
    if out["base_type"] and out["base_type"] not in BASE_TYPES:
        c.err(p + "base_type", "must be one of {}, got {!r}".format(BASE_TYPES, out["base_type"]))
    for key in ("has_lights", "has_dynamic_doors"):
        if (f or {}).get(key):
            c.err(p + key, "this importer builds no light or door rig; it must be false or absent")
    colors = (f or {}).get("recommended_colors")
    if not isinstance(colors, list) or not colors:
        c.err(p + "recommended_colors",
              "required, at least one [r, g, b]: without it CARLA gives the blueprint no "
              "`color` attribute and CarlaSetup.EgoColor cannot be applied")
    else:
        for i, col in enumerate(colors):
            v = c.color("{}recommended_colors[{}]".format(p, i), col)
            if v:
                out["recommended_colors"].append(v)
    return out


# ---------------------------------------------------------------- checks

def check_dimensions(measured, m):
    """[(key, want, got, tolerance, ok)] for every manifest dimension prep can measure.

    `measured` holds the same keys (metres) plus `wheel_radius_m`."""
    rows = []
    for key in CHECKED_DIMENSIONS:
        spec = m["dimensions"].get(key)
        if spec is None or measured.get(key) is None:
            continue
        got = float(measured[key])
        rows.append((key, spec["value"], got, spec["tolerance_m"],
                     abs(got - spec["value"]) <= spec["tolerance_m"]))
    if measured.get("wheel_radius_m") is not None:
        tol = m["wheels"]["radius_tolerance_m"] or m["tolerance_m"]
        got = float(measured["wheel_radius_m"])
        rows.append(("wheel_radius_m", m["wheels"]["radius_m"], got, tol,
                     abs(got - m["wheels"]["radius_m"]) <= tol))
    return rows


def format_checks(rows):
    lines = []
    for key, want, got, tol, ok in rows:
        lines.append("  {:<16} manifest {:7.3f}  model {:7.3f}  diff {:+.3f}  tol {:.3f}  {}".format(
            key, want, got, got - want, tol, "ok" if ok else "OUT OF TOLERANCE"))
    return "\n".join(lines)


# ---------------------------------------------------------------- runtime physics

def physics_plan(m):
    """What the bridge applies after spawning, in CARLA units, from the manifest only.

    {"vehicle": {attr: value}, "wheels": [{attr: value} x4 in CARLA_WHEEL_ORDER]}.
    Only declared keys appear; everything else keeps the blueprint's value.
    """
    ph = m["physics"]
    veh = {attr: ph[key] for key, attr in VEHICLE_SCALARS.items() if ph.get(key) is not None}
    if ph.get("use_gear_autobox") is not None:
        veh["use_gear_autobox"] = ph["use_gear_autobox"]
    if ph.get("center_of_mass_m") is not None:
        veh["center_of_mass"] = tuple(ph["center_of_mass_m"])
    for key in ("torque_curve", "steering_curve"):
        if ph.get(key) is not None:
            veh[key] = [tuple(pt) for pt in ph[key]]
    if ph.get("forward_gears"):
        veh["forward_gears"] = [dict(g) for g in ph["forward_gears"]]
    wheels = []
    w = m["wheels"]
    for pos in CARLA_WHEEL_ORDER:
        front = pos.startswith("front")
        d = {"radius": w["radius_m"] * 100.0,  # WheelPhysicsControl.radius is in cm
             "max_steer_angle": w["max_steer_deg"] if front else w["rear_max_steer_deg"]}
        d.update(ph["tires"]["front" if front else "rear"])
        wheels.append(d)
    return {"vehicle": veh, "wheels": wheels}


def apply_physics(pc, plan, carla):
    """Write `plan` into the VehiclePhysicsControl `pc` (in place) using the `carla` module.

    Returns the list of attributes set. Kept free of any CARLA connection so it can be
    tested with stand-in objects."""
    done = []
    for attr, v in plan["vehicle"].items():
        if attr in ("torque_curve", "steering_curve"):
            v = [carla.Vector2D(float(x), float(y)) for x, y in v]
        elif attr == "center_of_mass":
            v = carla.Location(*[float(x) for x in v])
        elif attr == "forward_gears":
            v = [carla.GearPhysicsControl(ratio=g["ratio"], down_ratio=g["down_ratio"],
                                          up_ratio=g["up_ratio"]) for g in v]
        setattr(pc, attr, v)
        done.append(attr)
    wheels = list(pc.wheels)
    if len(wheels) != len(plan["wheels"]):
        raise ManifestError("blueprint has {} wheels, the manifest describes {}".format(
            len(wheels), len(plan["wheels"])))
    for wc, want in zip(wheels, plan["wheels"]):
        for attr, v in want.items():
            setattr(wc, attr, float(v))
    pc.wheels = wheels
    done.append("wheels")
    return done


# ---------------------------------------------------------------- registration

def factory_entry(m, class_path):
    """The VehicleFactory entry (factory.FIELDS) for this vehicle."""
    f = m["factory"]
    return {"make": m["make"], "model": m["model"], "class_": class_path, "number_of_wheels": 4,
            "generation": f["generation"], "object_type": f["object_type"],
            "base_type": f["base_type"], "special_type": f["special_type"],
            "has_dynamic_doors": False, "has_lights": False,
            "recommended_colors": [list(col) + [255] for col in f["recommended_colors"]],
            "supported_drivers": []}
