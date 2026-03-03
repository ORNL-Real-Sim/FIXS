import unreal
import sys
import os
import csv
from collections import defaultdict
import importlib

script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(script_dir)
sys.path.append(os.path.join(script_dir, ".."))
sys.path.append(os.path.join(script_dir, "..", "carla_scripts"))
sys.path.append(os.path.join(script_dir, "..", "utils"))
sys.path.append(os.path.join(script_dir, "..", "test_scenarios"))

import utils.trafficlight_helper as trafficlight_helper
importlib.reload(trafficlight_helper)
TrafficLightHelper = trafficlight_helper.TrafficLightHelper

# TrafficLightHelper.set_offset((0.06, 328.61)) # this is the offset for the CARLA town01

# ----------------------------
# Blueprints
# ----------------------------
TRAFFICLIGHT_GROUP_BLUEPRINT_PATH = "/Game/Carla/Static/TrafficLight/Streetlights_01/BP_TrafficLightGroup"
TRAFFICLIGHT_HEAD_ONLY_BLUEPRINT_PATH = "/Game/Carla/Static/TrafficLight/Streetlights_01/BP_TrafficLight"

def load_bp_class(path: str):
    cls = unreal.EditorAssetLibrary.load_blueprint_class(path)
    if not cls:
        raise RuntimeError(f"Failed to load blueprint class: {path}")
    return cls

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
        spawn_loc = unreal.Vector(loc.x, loc.y, loc.z + 300.0)  # +300cm lift

        light_actor = unreal.EditorLevelLibrary.spawn_actor_from_class(head_cls, spawn_loc, rot)
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