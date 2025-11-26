import carla
import os
from CommonLib.ConfigHelper import ConfigHelper
from dotenv import load_dotenv
if __name__ == "__main__":
    load_dotenv()
    config_path = os.environ["CONFIG_PATH"]
    config_helper = ConfigHelper()
    config_helper.getConfig(config_path)
    
    carla_server_ip = config_helper.Carla_setup["CarlaServerIP"]
    carla_server_port = config_helper.Carla_setup["CarlaServerPort"]
    carla_client = carla.Client(carla_server_ip, carla_server_port)

    world = carla_client.get_world()
    settings = world.get_settings()
    settings.synchronous_mode = False
    world.apply_settings(settings)


