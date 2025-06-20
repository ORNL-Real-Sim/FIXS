import carla
import time
import random

def main():
    # Connect to the CARLA server
    client = carla.Client('localhost', 2002)
    client.set_timeout(10.0)

    # Load the world and enable synchronous mode
    world = client.get_world()

    settings = world.get_settings()
    settings.synchronous_mode = True         # Enables synchronous mode
    settings.fixed_delta_seconds = 0.05      # Simulation step size
    world.apply_settings(settings)

    blueprint_library = world.get_blueprint_library()
    vehicle_bp = blueprint_library.filter('vehicle.tesla.model3')[0]
    vehicle_bp.set_attribute('role_name', 'ego')

    spawn_points = world.get_map().get_spawn_points()
    spawn_point = random.choice(spawn_points)

    # Spawn the vehicle
    vehicle = world.spawn_actor(vehicle_bp, spawn_point)

    # Move the spectator to a top-down view
    spectator = world.get_spectator()
    def update_spectator():
        
        location = carla.Location(x=0, y=0, z=1000)
        rotation = carla.Rotation(pitch=-90.0, yaw=0.0, roll=0.0)
        spectator.set_transform(carla.Transform(location, rotation))

    try:
        update_spectator()
        for frame in range(200):
            
            world.tick()  # This replaces simulationStep()
            print(f"Frame {frame}: vehicle at {vehicle.get_location()}")
            time.sleep(0.05)

    finally:
        print('Cleaning up...')
        vehicle.destroy()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        world.apply_settings(settings)

if __name__ == '__main__':
    main()
