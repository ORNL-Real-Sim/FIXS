import os

SIMULATION_FOLDER = os.path.join(os.getcwd(), "Experiments_Sumo\\Shallowford_after_calibration_banleftturn_AdjustedFixedTime_V3")
CONFIG_PATH = os.path.join(SIMULATION_FOLDER, "config_Sumo.yaml")
SUMO_NET_PATH = os.path.join(SIMULATION_FOLDER, "chatt.net.xml")


def setup_simulation_env():
    # write to dot env file
    with open('.env', 'w') as f:
        f.write(f'SIMULATION_FOLDER={SIMULATION_FOLDER}\n')
        f.write(f'CONFIG_PATH={CONFIG_PATH}\n')
        f.write(f'SUMO_NET_PATH={SUMO_NET_PATH}\n')

if __name__ == "__main__":
    setup_simulation_env()