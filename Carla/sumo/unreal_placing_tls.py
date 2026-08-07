import unreal
import sys
import os
import csv
from collections import defaultdict
import importlib

script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(script_dir)
sys.path.append(os.path.join(script_dir, ".."))
# co-sim helpers live alongside in Carla/utils
sys.path.append(os.path.normpath(os.path.join(script_dir, "..", "utils")))

import trafficlight_helper
importlib.reload(trafficlight_helper)
TrafficLightHelper = trafficlight_helper.TrafficLightHelper

# TrafficLightHelper.set_offset((0.06, 328.61)) # this is the offset for the CARLA town01

# ----------------------------
# Blueprints and placement numbers
# ----------------------------
# These used to be literals here. They are now supplied by place_tls.py from the map
# bundle's placement.yaml, because the same hardcoded content path resolves to
# DIFFERENT assets on different installs (a 3-lens head on a pole on stock Windows
# CARLA, a bare head where someone stripped the pole by hand) and the +300 cm lift
# below is only correct for one of them. See FIXS#223 / Digital-Twin-Library#2.
#
# The manifest is read outside Unreal - UE's embedded Python has no pyyaml - so the
# values arrive as environment variables. Nothing is defaulted here: a value that
# failed to cross the boundary must stop the run, not silently become the stock
# asset again, which is the failure this change exists to remove.

def _required_env(name, what):
    value = os.environ.get(name)
    if value is None or not value.strip():
        raise RuntimeError(
            f"{name} is not set, so there is no {what} to place. Launch this through "
            f"place_tls.py, which reads it from the map bundle's placement.yaml."
        )
    return value.strip()


TRAFFICLIGHT_GROUP_BLUEPRINT_PATH = _required_env(
    "FIXS_TL_GROUP_BLUEPRINT", "traffic-light group blueprint")
TRAFFICLIGHT_HEAD_ONLY_BLUEPRINT_PATH = _required_env(
    "FIXS_TL_BLUEPRINT", "traffic-light blueprint")

# Vertical lift applied to every head above the table's z. Belongs next to the
# blueprint it was measured against - the two drifting apart is the bug.
try:
    TRAFFICLIGHT_Z_OFFSET_CM = float(_required_env("FIXS_TL_Z_OFFSET_CM", "z offset"))
except ValueError as exc:
    raise RuntimeError(f"FIXS_TL_Z_OFFSET_CM is not a number: {exc}")

# False keeps the original rotation from traffic_light_table.csv.
# True flips every signal head 180 degrees around yaw.
FLIP_SIGNAL_HEADS_180 = os.environ.get(
    "FLIP_SIGNAL_HEADS_180", "").strip().lower() in {"1", "true", "yes", "on"}

unreal.log_warning(
    f"placing_tls.py: blueprint={TRAFFICLIGHT_HEAD_ONLY_BLUEPRINT_PATH} "
    f"z_offset_cm={TRAFFICLIGHT_Z_OFFSET_CM:g} flip_yaw_180={FLIP_SIGNAL_HEADS_180}"
)

def load_bp_class(path: str):
    cls = unreal.EditorAssetLibrary.load_blueprint_class(path)
    if not cls:
        raise RuntimeError(f"Failed to load blueprint class: {path}")
    return cls


# Labels this script gives what it spawns. Used to find a previous run's actors -
# see clear_previously_placed().
GROUP_LABEL_PREFIX = "TrafficLightGroup"
HEAD_LABEL_PREFIX = "TrafficLight"


def clear_previously_placed():
    """Delete the traffic lights a previous run of THIS script left behind.

    Placement used to be purely additive: every run spawned a fresh set on top of
    the old one, so re-placing a map doubled its lights (roosevelt_full was found
    holding 134 stock heads AND 134 new ones, with 14 groups for 7 junctions). That
    was survivable only while re-placement was rare - a marker suppressed it after
    the first run. Now that a changed manifest deliberately triggers re-placement,
    additive behaviour would corrupt the map on every manifest edit.

    Matching is by actor LABEL, not class: the previous run may well have used a
    different blueprint (that is the whole point of making the blueprint
    manifest-driven), so class is not a stable identity across runs. The labels
    below are exactly the ones this script assigns, which keeps it from touching
    lights that were authored into the map by hand or shipped with it.
    """
    removed_heads = removed_groups = 0
    for actor in unreal.EditorLevelLibrary.get_all_level_actors():
        try:
            label = actor.get_actor_label()
        except Exception:
            continue
        if not label.startswith(HEAD_LABEL_PREFIX):
            continue
        is_group = label.startswith(GROUP_LABEL_PREFIX)
        # Only remove things that are actually traffic lights, so a same-named
        # actor of an unrelated class is left alone.
        if not is_group and not isinstance(actor, unreal.TrafficLightBase):
            continue
        if unreal.EditorLevelLibrary.destroy_actor(actor):
            if is_group:
                removed_groups += 1
            else:
                removed_heads += 1

    if removed_heads or removed_groups:
        unreal.log_warning(
            f"placing_tls.py: removed {removed_heads} head(s) and {removed_groups} "
            f"group(s) from a previous placement before re-placing."
        )
    return removed_heads, removed_groups

def normalize_degrees(angle):
    return ((angle + 180.0) % 360.0) - 180.0

def make_rotator_like(rot, yaw):
    out_rot = unreal.Rotator(0.0, 0.0, 0.0)
    out_rot.pitch = rot.pitch
    out_rot.yaw = normalize_degrees(yaw)
    out_rot.roll = rot.roll
    return out_rot

def get_signal_head_rotation(rot):
    if not FLIP_SIGNAL_HEADS_180:
        return rot
    return make_rotator_like(rot, rot.yaw + 180.0)

# ----------------------------
# Input CSV
# ----------------------------
TLS_TABLE_PATH = os.environ.get("SUMO_TLS_TABLE_PATH")
if not TLS_TABLE_PATH:
    raise RuntimeError(
        "SUMO_TLS_TABLE_PATH is not set. "
        "Set it in your shell before launching Unreal, or in the Output Log Python console."
    )

if not os.path.isfile(TLS_TABLE_PATH):
    raise RuntimeError(f"SUMO_TLS_TABLE_PATH does not exist: {TLS_TABLE_PATH}")

unreal.log_warning(f"placing_tls.py START: reading {TLS_TABLE_PATH}")

# ----------------------------
# Build groups: junction_id -> list[(unreal_location, unreal_rotation)]
# ----------------------------
trafficlight_groups = defaultdict(list)
seen_link_keys = set()  # (junction_id, link_id)

required_cols = {"junction_id", "link_id", "x", "y", "z", "heading"}

with open(TLS_TABLE_PATH, newline="") as f:
    reader = csv.DictReader(f)
    if not reader.fieldnames:
        raise RuntimeError("CSV has no header row / could not read fieldnames")

    missing = required_cols.difference(set(reader.fieldnames))
    if missing:
        raise RuntimeError(f"CSV is missing required columns: {sorted(missing)}. Found: {reader.fieldnames}")

    for row in reader:
        junction_id = str(row["junction_id"]).strip()
        link_id = str(row["link_id"]).strip()

        # Only take the first row per (junction_id, link_id) like values[0]
        key = (junction_id, link_id)
        if key in seen_link_keys:
            continue
        seen_link_keys.add(key)

        try:
            position_x = float(row["x"])
            position_y = float(row["y"])
            position_z = float(row["z"])
            heading = float(row["heading"])
        except ValueError as e:
            unreal.log_warning(f"Skipping row with bad numeric values: {row} ({e})")
            continue

        traffic_sumo_location, traffic_sumo_rotation = TrafficLightHelper.create_sumo_transform(
            position_x, position_y, position_z, heading
        )
        trafficlight_carla_location, trafficlight_carla_rotation = TrafficLightHelper.sumo_transform_to_carla_transform(
            traffic_sumo_location, traffic_sumo_rotation
        )
        trafficlight_unreal_location, trafficlight_unreal_rotation = TrafficLightHelper.carla_transform_to_unreal_transform(
            trafficlight_carla_location, trafficlight_carla_rotation
        )

        trafficlight_groups[junction_id].append((trafficlight_unreal_location, trafficlight_unreal_rotation))

unreal.log_warning(
    f"Parsed {len(seen_link_keys)} unique (junction_id, link_id) entries across "
    f"{len(trafficlight_groups)} junctions."
)

# ----------------------------
# Spawn actors
# ----------------------------
group_cls = load_bp_class(TRAFFICLIGHT_GROUP_BLUEPRINT_PATH)
head_cls = load_bp_class(TRAFFICLIGHT_HEAD_ONLY_BLUEPRINT_PATH)

# Placement is a replace, not an append: whatever a previous run put here goes
# first, so re-placing a map (a changed manifest, or --force) cannot double it.
clear_previously_placed()

spawned_groups = 0
spawned_heads = 0

for junction_id, trafficlight_transforms in trafficlight_groups.items():
    if not trafficlight_transforms:
        continue

    group_loc, group_rot = TrafficLightHelper.get_trafficlight_group_transform_from_trafficlight_unreal_transforms(
        trafficlight_transforms
    )

    group_actor = unreal.EditorLevelLibrary.spawn_actor_from_class(group_cls, group_loc, group_rot)
    if not group_actor:
        unreal.log_warning(f"Failed to spawn TrafficLightGroup{junction_id}")
        continue

    group_actor.set_actor_label(f"TrafficLightGroup{junction_id}", mark_dirty=True)
    unreal.log(f"Spawned TrafficLightGroup{junction_id} at {group_loc}")
    spawned_groups += 1

    # Always work with a real list copy
    current_list = list(group_actor.get_editor_property("TrafficLights"))

    for (loc, rot) in trafficlight_transforms:
        spawn_loc = unreal.Vector(loc.x, loc.y, loc.z + TRAFFICLIGHT_Z_OFFSET_CM)

        spawn_rot = get_signal_head_rotation(rot)
        light_actor = unreal.EditorLevelLibrary.spawn_actor_from_class(head_cls, spawn_loc, spawn_rot)
        if not light_actor:
            unreal.log_warning(f"Failed to spawn head for junction {junction_id} at {spawn_loc}")
            continue

        light_actor.set_actor_label(f"TrafficLight{int(spawn_loc.x)}_{int(spawn_loc.y)}", mark_dirty=True)
        unreal.log(f"Spawned TrafficLight{int(spawn_loc.x)}_{int(spawn_loc.y)} at {spawn_loc}")
        spawned_heads += 1

        current_list.append(light_actor)

    group_actor.set_editor_property("TrafficLights", current_list)

unreal.log_warning(f"placing_tls.py DONE: spawned {spawned_groups} groups and {spawned_heads} heads.")

# Optional: save all dirty packages, comment out if not needed
unreal.EditorLoadingAndSavingUtils.save_dirty_packages(True, True)
