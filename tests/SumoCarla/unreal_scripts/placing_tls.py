import unreal
import sys
import os
import pandas as pd
import typing
import importlib
import utils.trafficlight_helper as trafficlight_helper
importlib.reload(trafficlight_helper)

TrafficLightHelper = trafficlight_helper.TrafficLightHelper
TrafficLightHelper.set_offset((0.06, 328.61))
# Add the script directories to sys.path
script_dir = os.path.dirname(__file__)
sys.path.append(script_dir)
sys.path.append(os.path.join(script_dir, ".."))
sys.path.append(os.path.join(script_dir, "..", "carla_scripts"))
sys.path.append(os.path.join(script_dir, "..", "utils"))


TRAFFICLIGHT_GROUP_BLUEPRINT_PATH = "/Game/Carla/Static/TrafficLight/Streetlights_01/BP_TrafficLightGroup"
TRAFFICLIGHT_BLUEPRINT_PATH = "/Game/Carla/Static/TrafficLight/Streetlights_01/BP_TrafficLight"
TRAFFICLIGHT_HEAD_ONLY_BLUEPRINT_PATH = "/Game/Carla/Static/TrafficLight/Streetlights_01/BP_TrafficLight_Head_Only"
# load the tls table
tls_table_path = 'test_scenarios/Town01_with_ego_type_as_blueprint/traffic_light_table.csv'
tls_table = pd.read_csv(tls_table_path)
# junction_id,link_id,x,y,z,heading

# this stores the trafficlight groups, key is the junction_id, value is a list of trafficlight_unreal_transforms
trafficlight_groups = typing.DefaultDict(list)

for junction_id, group in tls_table.groupby('junction_id'):
    for link_id, link_group in group.groupby('link_id'):
        position_x = link_group['x'].values[0]
        position_y = link_group['y'].values[0]
        position_z = link_group['z'].values[0]
        heading = link_group['heading'].values[0]
        traffic_sumo_location, traffic_sumo_rotation = TrafficLightHelper.create_sumo_transform(position_x, position_y, position_z, heading)
        trafficlight_carla_location, trafficlight_carla_rotation = TrafficLightHelper.sumo_transform_to_carla_transform(traffic_sumo_location, traffic_sumo_rotation)
        trafficlight_unreal_location, trafficlight_unreal_rotation = TrafficLightHelper.carla_transform_to_unreal_transform(trafficlight_carla_location, trafficlight_carla_rotation)
        trafficlight_groups[junction_id].append((trafficlight_unreal_location, trafficlight_unreal_rotation))

for junction_id, trafficlight_transforms in trafficlight_groups.items():
    trafficlight_group_location, trafficlight_group_rotation = TrafficLightHelper.get_trafficlight_group_transform_from_trafficlight_unreal_transforms(trafficlight_transforms)
    
    # spawn the trafficlight group
    trafficlight_class = unreal.EditorAssetLibrary.load_blueprint_class(TRAFFICLIGHT_GROUP_BLUEPRINT_PATH)
    trafficlight_group_actor = unreal.EditorLevelLibrary.spawn_actor_from_class(trafficlight_class, 
                                                                                trafficlight_group_location, 
                                                                                trafficlight_group_rotation
                                                                                )
    trafficlight_group_actor.set_actor_label(f"TrafficLightGroup{junction_id}", mark_dirty=True)
    unreal.log(f"✅ Spawned TrafficLightGroup{junction_id} at {trafficlight_group_location}")
    
    # spawn the trafficlights
    for (trafficlight_unreal_location, trafficlight_unreal_rotation) in trafficlight_transforms:
        trafficlight_unreal_location.z = 300
        trafficlight_class = unreal.EditorAssetLibrary.load_blueprint_class(TRAFFICLIGHT_HEAD_ONLY_BLUEPRINT_PATH)
        trafficlight_actor = unreal.EditorLevelLibrary.spawn_actor_from_class(trafficlight_class, 
                                                                              trafficlight_unreal_location, 
                                                                              trafficlight_unreal_rotation
                                                                              )
        trafficlight_actor.set_actor_label(f"TrafficLight{trafficlight_unreal_location.x}_{trafficlight_unreal_location.y}", mark_dirty=True)
        unreal.log(f"✅ Spawned TrafficLight{trafficlight_unreal_location.x}_{trafficlight_unreal_location.y} at {trafficlight_unreal_location}")
        # # attach the trafficlight to the trafficlight group
        # trafficlight_group_actor.attach_to_actor(trafficlight_actor)
        # add the trafficlight to the trafficlight group's trafficlights list
        current_list = trafficlight_group_actor.get_editor_property("TrafficLights")
        current_list.append(trafficlight_actor)
        trafficlight_group_actor.set_editor_property("TrafficLights", current_list)
        # show the available properties of the trafficlight_group_actor






















# Asset path (relative to /Game)
# blueprint_path = "/Game/Carla/Static/TrafficLight/Streetlights_01/BP_TrafficLight"

# # Load the Blueprint class
# bp_class = unreal.EditorAssetLibrary.load_blueprint_class(blueprint_path)

# if not bp_class:
#     unreal.log_error(f"Failed to load blueprint class at: {blueprint_path}")
# else:
#     # Define spawn location and rotation
#     location = unreal.Vector(-6209.930664, 12375.550781, 100.0)
#     rotation = unreal.Rotator(0.0, 0.0, 0.0)

#     # Spawn the actor in the level
#     actor = unreal.EditorLevelLibrary.spawn_actor_from_class(bp_class, location, rotation)

#     if actor:
#         unreal.log(f"✅ Spawned {actor.get_name()} at {location}")
#     else:
#         unreal.log_error("❌ Failed to spawn actor.")