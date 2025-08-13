import os
from CommonLib.ConfigHelper import ConfigHelper
import time

def run_sumo(simulation_folder_path, simulation_net_file, port, gui=False):
    simulation_net_name = os.path.basename(simulation_net_file)
    if gui:
        os.system(f'start sumo-gui -c {simulation_folder_path}\{simulation_net_name}.sumocfg --remote-port {port} --step-length 0.1 --netstate-dump {simulation_folder_path}\{simulation_net_name}.xml --netstate-dump.precision 10 --num-clients 2  --begin 0 --end 300 --collision.action warn')
    else:
        os.system(f'start sumo -c {simulation_folder_path}\{simulation_net_name}.sumocfg --remote-port {port} --step-length 0.1 --netstate-dump {simulation_folder_path}\{simulation_net_name}.xml --netstate-dump.precision 10 --num-clients 2  --begin 0 --end 300 --collision.action warn')

def run_traffic_layer(traffic_layer_path, config_path):
    # start cmd /k ..\..\Trafficlayer\x64\Debug\TrafficLayer.exe -f '.\ecodrivingConfig.yaml'\
 
    os.system(f'start cmd /k {traffic_layer_path} -f {config_path}')
    
def run_carla(carla_path, config_path, traffic_light_table_path):
    os.system(f'start cmd /k {carla_path} -f {config_path} -t {traffic_light_table_path}')

def run_controller(config_path, sumo_port, traffic_layer_port):
    os.system(f'start cmd /k python .\\controller_template.py -c {config_path} --sumoPort {sumo_port} --trafficlayerPort {traffic_layer_port}')

if __name__ == "__main__":
    config_path = os.path.join(os.getcwd(), 'defaultConfig.yaml')
    simulation_folder = 'test_scenarios\Town01_with_ego_type_as_blueprint'
    simulation_folder_path = os.path.join(os.getcwd(), simulation_folder)
    simulation_net_file = 'Town01'
    tls_table_path = os.path.join(simulation_folder_path, 'traffic_light_table.csv')
    config_helper = ConfigHelper()
    config_helper.getConfig(config_path)
    sumo_port = config_helper.simulation_setup['TrafficSimulatorPort']
    traffic_layer_path = os.path.join(os.getcwd(), 'TrafficLayer.exe')
    carla_path = os.path.join(os.getcwd(), 'VirCarlaEnv.exe')
    run_sumo(simulation_folder_path, simulation_net_file, sumo_port, gui=True)
    # run_traffic_layer(traffic_layer_path, config_path)
    # run_carla(carla_path, config_path, tls_table_path)
    # run_controller(config_path, sumo_port, 440)
    