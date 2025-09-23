import carla
import random
import time

def main():
    # Connect to CARLA
    client = carla.Client("localhost", 2000)
    client.set_timeout(10.0)

    world = client.get_world()
    blueprint_library = world.get_blueprint_library()

    # Pick a vehicle blueprint (Tesla as an example)
    ego_bp = blueprint_library.filter("model3")[0]

    # Choose a random spawn point
    spawn_points = world.get_map().get_spawn_points()
    spawn_point = random.choice(spawn_points)

    # Spawn the ego vehicle
    ego_vehicle = world.try_spawn_actor(ego_bp, spawn_point)
    if ego_vehicle is None:
        print("Could not spawn ego vehicle, try again")
        return

    print(f"Ego vehicle spawned at {spawn_point.location}")

    # Enable autopilot with Traffic Manager
    traffic_manager = client.get_trafficmanager()
    traffic_manager.set_random_device_seed(random.randint(0, 1000))

    ego_vehicle.set_autopilot(True, traffic_manager.get_port())

    # Configure random route behavior
    traffic_manager.ignore_lights_percentage(ego_vehicle, 0)   # obey lights
    traffic_manager.random_left_lanechange_percentage(ego_vehicle, 30)
    traffic_manager.random_right_lanechange_percentage(ego_vehicle, 30)

    print("Vehicle is now following random routes...")
    try:
        time.sleep(60)  # let it run for 1 minute
    finally:
        print("Cleaning up actors")
        ego_vehicle.destroy()

if __name__ == '__main__':
    main()
