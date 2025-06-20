from re import T
import carla
import os
from CommonLib.ConfigHelper import ConfigHelper

config_path = os.path.join(os.getcwd(), 'defaultConfig.yaml')
config_helper = ConfigHelper()
config_helper.getConfig(config_path)
carla_server_ip = config_helper.Carla_setup["CarlaServerIP"]
carla_server_port = config_helper.Carla_setup["CarlaServerPort"]


carla_client = carla.Client(carla_server_ip, carla_server_port)

import carla

def draw_axes_at_location(world, location, length=5.0, thickness=0.1, life_time=0.0, persistent=True):
    """
    Draw X (red), Y (green), Z (blue) axes at the world origin (0,0,0).
    """
    debug = world.debug

    # Red X-axis
    debug.draw_line(location, carla.Location(x=length, y=0, z=100),
                    thickness=thickness, color=carla.Color(255, 0, 0), life_time=life_time, persistent_lines=persistent)

    # Green Y-axis  
    debug.draw_line(location, carla.Location(x=0, y=length, z=100),
                    thickness=thickness, color=carla.Color(0, 255, 0), life_time=life_time, persistent_lines=persistent)

    # Blue Z-axis
    debug.draw_line(location, carla.Location(x=0, y=0, z=length),
                    thickness=thickness, color=carla.Color(0, 0, 255), life_time=life_time, persistent_lines=persistent)

def move_spectator_to_location(world, location, pitch=-90.0):
    """
    Move the spectator directly above the origin, looking down.
    """
    spectator = world.get_spectator()
    rotation = carla.Rotation(pitch=pitch, yaw=0)
    spectator.set_transform(carla.Transform(location, rotation))
def try_to_spawn_vehicle(world, location, rotation):
    vehicle_bp = world.get_blueprint_library().find('vehicle.mini.cooper_s')
    vehicle = world.try_spawn_actor(vehicle_bp, carla.Transform(location, rotation))
    return vehicle

world = carla_client.get_world()
settings = world.get_settings()
settings.synchronous_mode = False
world.apply_settings(settings)


# draw_axes_at_location(world, carla.Location(334.887, 18.39, 0), length=100.0)
# move_spectator_to_location(world, carla.Location(334.887, 18.39, 100))
# try_to_spawn_vehicle(world, carla.Location(334.887, 18.39, 0.0), carla.Rotation(pitch=0, yaw=90.0346, roll=0))



