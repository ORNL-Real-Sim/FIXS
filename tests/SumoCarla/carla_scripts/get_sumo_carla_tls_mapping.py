from re import T
import carla
import os
import pandas as pd
from CommonLib.ConfigHelper import ConfigHelper
from utils.trafficlight_helper import TrafficLightHelper

TrafficLightHelper.set_offset((0.06, 328.61))
config_path = os.path.join(os.getcwd(), 'defaultConfig.yaml')
config_helper = ConfigHelper()
config_helper.getConfig(config_path)
carla_server_ip = config_helper.Carla_setup["CarlaServerIP"]
carla_server_port = config_helper.Carla_setup["CarlaServerPort"]

carla_client = carla.Client(carla_server_ip, carla_server_port)
world = carla_client.get_world()

sumo_traffic_light_table_path = "test_scenarios/Town01_with_ego_type_as_blueprint/traffic_light_table.csv"

sumo_traffic_light_table = pd.read_csv(sumo_traffic_light_table_path)
# junction_id,link_id,x,y,z,heading
# build a new table with one extra column: carla_actor_id
sumo_carla_traffic_light_table = sumo_traffic_light_table.copy()
sumo_carla_traffic_light_table['carla_actor_id'] = None

for traffic_light_actor in world.get_actors().filter("traffic.traffic_light"):
    carla_traffic_light_actor_id = traffic_light_actor.id
    # get the traffic light actor location in carla coordinates
    carla_traffic_light_actor_location = traffic_light_actor.get_location()
    sumo_traffic_light_actor_location = TrafficLightHelper.carla_location_to_sumo_location(carla_traffic_light_actor_location)
    # find the closest sumo traffic light in the table
    position_x = sumo_traffic_light_actor_location.x
    position_y = sumo_traffic_light_actor_location.y
    print(f"position_x: {position_x}, position_y: {position_y}")
    def find_closest_tls_index(tls_table: pd.DataFrame, x: float, y: float) -> int:
        """
        Returns the index of the closest traffic light (row) in tls_table to the given (x, y) point.
        """
        # Compute squared Euclidean distance to each (x, y) in the table
        distances = (tls_table['x'] - x)**2 + (tls_table['y'] - y)**2

        # Return the index of the closest row
        closest_index = distances.idxmin()
        return closest_index
    
    closest_index = find_closest_tls_index(sumo_traffic_light_table, position_x, position_y)
    print(f"closest_index: {closest_index}")
    sumo_carla_traffic_light_table.loc[closest_index, 'carla_actor_id'] = carla_traffic_light_actor_id
    
sumo_carla_traffic_light_table.to_csv("test_scenarios/Town01_with_ego_type_as_blueprint/sumo_carla_traffic_light_table.csv", index=False)



















# settings = world.get_settings()
# settings.synchronous_mode = False
# world.apply_settings(settings)


# draw_axes_at_location(world, carla.Location(334.887, 18.39, 0), length=100.0)
# move_spectator_to_location(world, carla.Location(334.887, 18.39, 100))
# try_to_spawn_vehicle(world, carla.Location(334.887, 18.39, 0.0), carla.Rotation(pitch=0, yaw=90.0346, roll=0))



