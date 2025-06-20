import unreal
import sys
import os

# Add the script directories to sys.path
script_dir = os.path.dirname(__file__)
sys.path.append(script_dir)
sys.path.append(os.path.join(script_dir, ".."))
sys.path.append(os.path.join(script_dir, "..", "carla_scripts"))
sys.path.append(os.path.join(script_dir, "..", "utils"))
# Asset path (relative to /Game)
blueprint_path = "/Game/Carla/Static/TrafficLight/Streetlights_01/BP_TrafficLight"

# Load the Blueprint class
bp_class = unreal.EditorAssetLibrary.load_blueprint_class(blueprint_path)

if not bp_class:
    unreal.log_error(f"Failed to load blueprint class at: {blueprint_path}")
else:
    # Define spawn location and rotation
    location = unreal.Vector(32341.009766, 20203.007812, 100.0)
    rotation = unreal.Rotator(0.0, 0.0, 90.0)

    # Spawn the actor in the level
    actor = unreal.EditorLevelLibrary.spawn_actor_from_class(bp_class, location, rotation)

    if actor:
        # Assign a custom label
        custom_label = "TrafficLight37"
        actor.set_actor_label(custom_label, mark_dirty=True)
        unreal.log(f"✅ Spawned and labeled as: {custom_label}")
    else:
        unreal.log_error("❌ Failed to spawn actor.")