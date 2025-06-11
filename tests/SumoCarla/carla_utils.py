import carla

carla_server_ip = '127.0.0.1'
carla_server_port = 2000

carla_client = carla.Client(carla_server_ip, carla_server_port)

# set carla to async mode
world = carla_client.get_world()
settings = world.get_settings()
settings.synchronous_mode = False
world.apply_settings(settings)
