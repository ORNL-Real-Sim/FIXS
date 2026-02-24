import carla


if __name__ == "__main__":
    client = carla.Client("localhost", 2000)
    client.set_timeout(10.0)
    client.load_world("MLK_noped1002_final_debug")
