import carla
import os
from CommonLib.ConfigHelper import ConfigHelper
from dotenv import load_dotenv
import cv2
import numpy as np
import random
def process_image(image):
    array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
    array = np.reshape(array, (image.height, image.width, 4))  # RGBA format
    rgb_array = array[:, :, :3]  # Drop the alpha channel
    cv2.imshow("Front Camera", rgb_array)
    cv2.waitKey(1)



if __name__ == "__main__":
    load_dotenv()
    config_path = os.environ["CONFIG_PATH"]
    config_helper = ConfigHelper()
    config_helper.getConfig(config_path)
    
    carla_server_ip = config_helper.Carla_setup["CarlaServerIP"]
    carla_server_port = config_helper.Carla_setup["CarlaServerPort"]
    carla_client = carla.Client(carla_server_ip, carla_server_port)

    carla_world = carla_client.get_world()
    carla_blueprint_library = carla_world.get_blueprint_library()
    camera_bp = carla_blueprint_library.find('sensor.camera.rgb')
    camera_bp.set_attribute('image_size_x', '800')
    camera_bp.set_attribute('image_size_y', '600')
    camera_bp.set_attribute('fov', '90')
    camera_transform = carla.Transform(carla.Location(x=1.5, z=2.4))  # x is forward, z is up
    # Wait until the ego vehicle is spawned in the carla world
    RANDOM_SPAWN = True
    ego_vehicle_role_name = 'ego'
    ego_vehicle_carla_actor: carla.Vehicle = None
    ego_vehicle_carla_actor_id = ''
    while True:
        try:
            carla_vehicle_actors_in_world = carla_world.get_actors().filter('vehicle.*')
            carla_vehicle_actor: carla.Vehicle
            # if the ego vehicle is not spawned in carla
            if RANDOM_SPAWN:
                spawn_points = carla_world.get_map().get_spawn_points()
                spawn_point = random.choice(spawn_points)
                ego_vehicle_carla_actor = carla_world.spawn_actor(carla_blueprint_library.find('vehicle.tesla.model3'), spawn_point)
                # set as auto pilot
                ego_vehicle_carla_actor.set_autopilot(True)
            if ego_vehicle_carla_actor is None or carla_world.get_actor(ego_vehicle_carla_actor_id) is None:
                camera_actor.destroy()
                for carla_vehicle_actor in carla_vehicle_actors_in_world:
                    if 'role_name' in carla_vehicle_actor.attributes.keys():
                        carla_actor_role_name = carla_vehicle_actor.attributes.get('role_name', None)
                        if carla_actor_role_name == ego_vehicle_role_name:
                            ego_vehicle_carla_actor = carla_vehicle_actor
                            ego_vehicle_carla_actor_id = carla_vehicle_actor.id
                            # if the ego vehicle is in carla
                            camera_actor: carla.Sensor
                            camera_actor = carla_world.spawn_actor(camera_bp, camera_transform, attach_to=ego_vehicle_carla_actor)
            camera_actor.listen(lambda image: process_image(image))
        except Exception as e:
            print(f"Error occurred: {str(e)}")

        finally:
            # Clean up: stop camera and vehicle
            camera_actor.stop()
            ego_vehicle_carla_actor.destroy()
            camera_actor.destroy()
            cv2.destroyAllWindows()
            print("Cleaning up...")