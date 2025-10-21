import os
from utils.extract_sumo_tls_as_table import parse_sumo_tls_from_netxml, tls_groups_to_df

SIMULATION_FOLDER = os.path.join(os.getcwd(), "MLK_Sumo_Scenario")
CONFIG_PATH = os.path.join(SIMULATION_FOLDER, "config_Sumo_Carla_dSPACE.yaml")
SUMO_NET_PATH = os.path.join(SIMULATION_FOLDER, "MLK_final_elevation_20251009.net.xml")
SUMO_TLS_TABLE_PATH = os.path.join(SIMULATION_FOLDER, "traffic_light_table.csv")

def extract_sumo_tls_as_table(sumo_net_file, sumo_tls_table_path):
    traffic_light_groups, _ = parse_sumo_tls_from_netxml(
        sumo_net_file,
        offset_forward=0.0,
        apply_linkwise_offset=True,
        extend_back=50.0,
        default_lane_width=3.2
    )
    traffic_light_groups_df = tls_groups_to_df(traffic_light_groups)
    traffic_light_groups_df.to_csv(sumo_tls_table_path, index=False)

def setup_simulation_env():
    if not os.path.exists(SUMO_TLS_TABLE_PATH):
        extract_sumo_tls_as_table(SUMO_NET_PATH, SUMO_TLS_TABLE_PATH)
    # write to dot env file
    with open('.env', 'w') as f:
        f.write(f'SIMULATION_FOLDER={SIMULATION_FOLDER}\n')
        f.write(f'CONFIG_PATH={CONFIG_PATH}\n')
        f.write(f'SUMO_NET_PATH={SUMO_NET_PATH}\n')
        f.write(f'SUMO_TLS_TABLE_PATH={SUMO_TLS_TABLE_PATH}\n')

if __name__ == "__main__":
    setup_simulation_env()