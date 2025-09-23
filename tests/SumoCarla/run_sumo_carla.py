import os
from CommonLib.ConfigHelper import ConfigHelper
from dotenv import load_dotenv

def run_sumo(simulation_folder_path, simulation_net_file, port, gui=False):
    simulation_net_name = os.path.basename(simulation_net_file)
    if gui:
        os.system(f'start sumo-gui -c {simulation_folder_path}\{simulation_net_name} --remote-port {port} --step-length 0.1 --netstate-dump {simulation_folder_path}\{simulation_net_name}.xml --netstate-dump.precision 10 --num-clients 2')
    else:
        os.system(f'start sumo -c {simulation_folder_path}\{simulation_net_name} --remote-port {port} --step-length 0.1 --netstate-dump {simulation_folder_path}\{simulation_net_name}.xml --netstate-dump.precision 10 --num-clients 2')

def run_traffic_layer(traffic_layer_path, config_path):
    # start cmd /k ..\..\Trafficlayer\x64\Debug\TrafficLayer.exe -f '.\ecodrivingConfig.yaml'\
 
    os.system(f'start cmd /k {traffic_layer_path} -f {config_path}')
    
def run_carla(carla_path, config_path, traffic_light_table_path):
    os.system(f'start cmd /k {carla_path} -f {config_path} -t {traffic_light_table_path}')

def run_controller(config_path, sumo_port, traffic_layer_port):
    os.system(f'start cmd /k python .\\controller_template.py -c {config_path} --sumoPort {sumo_port} --trafficlayerPort {traffic_layer_port}')

if __name__ == "__main__":
    load_dotenv()
    simulation_folder_path = os.environ["SIMULATION_FOLDER"]
    config_path = os.environ["CONFIG_PATH"]
    simulation_net_file = os.environ["SUMO_NET_PATH"]
    tls_table_path = os.environ["SUMO_TLS_TABLE_PATH"]
    config_helper = ConfigHelper()
    config_helper.getConfig(config_path)
    sumo_port = config_helper.simulation_setup['TrafficSimulatorPort']
    traffic_layer_path = os.path.join(os.getcwd(), 'TrafficLayer.exe')
    carla_path = os.path.join(os.getcwd(), 'VirCarlaEnv.exe')
    run_sumo(simulation_folder_path, simulation_net_file, sumo_port, gui=True)
    run_traffic_layer(traffic_layer_path, config_path)
    run_carla(carla_path, config_path, tls_table_path)
    