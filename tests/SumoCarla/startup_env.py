import os
from CommonLib.ConfigHelper import ConfigHelper
import time

def run_sumo(simulation_folder_path, simulation_net_file, port, gui=False):
    simulation_net_name = os.path.basename(simulation_net_file)
    print(f'start sumo-gui -c {simulation_folder_path}\{simulation_net_name}.sumocfg --remote-port {port} --step-length 0.1 --netstate-dump {simulation_folder_path}\{simulation_net_name}.xml --netstate-dump.precision 10 --num-clients 1  --begin 0 --end 300 --collision.action warn')
    if gui:
        os.system(f'start sumo-gui -c {simulation_folder_path}\{simulation_net_name}.sumocfg --remote-port {port} --step-length 0.1 --netstate-dump {simulation_folder_path}\{simulation_net_name}.xml --netstate-dump.precision 10 --num-clients 1  --begin 0 --end 300 --collision.action warn')
    else:
        os.system(f'start sumo -c {simulation_folder_path}\{simulation_net_name}.sumocfg --remote-port {port} --step-length 0.1 --netstate-dump {simulation_folder_path}\{simulation_net_name}.xml --netstate-dump.precision 10 --num-clients 1  --begin 0 --end 300 --collision.action warn')

def run_traffic_layer(traffic_layer_path, config_path):
    # start cmd /k ..\..\Trafficlayer\x64\Debug\TrafficLayer.exe -f '.\ecodrivingConfig.yaml'\
 
    os.system(f'start cmd /k {traffic_layer_path} -f {config_path}')

if __name__ == "__main__":
    config_path = os.path.join(os.getcwd(), 'defaultConfig.yaml')
    simulation_folder = 'test_scenarios\Town01_with_ego_type_as_blueprint'
    simulation_folder_path = os.path.join(os.getcwd(), simulation_folder)
    simulation_net_file = 'Town01'
    
    config_helper = ConfigHelper()
    config_helper.getConfig(config_path)
    sumo_port = config_helper.simulation_setup['TrafficSimulatorPort']
    traffic_layer_path = os.path.join(os.getcwd(), 'TrafficLayer.exe')
    run_sumo(simulation_folder_path, simulation_net_file, sumo_port, gui=True)
    # run_traffic_layer(traffic_layer_path, config_path)
    