import os


def run_sumo(simulation_folder_path, simulation_net_file, port, num_clients, gui=False):
    simulation_net_name = os.path.basename(simulation_net_file)
    if gui:
        os.system(f'start sumo-gui -c {simulation_folder_path}\{simulation_net_name} --remote-port {port} --step-length 0.1 --netstate-dump {simulation_folder_path}\{simulation_net_name}.xml --netstate-dump.precision 10 --num-clients {num_clients}')
    else:
        os.system(f'start sumo -c {simulation_folder_path}\{simulation_net_name} --remote-port {port} --step-length 0.1 --netstate-dump {simulation_folder_path}\{simulation_net_name}.xml --netstate-dump.precision 10 --num-clients {num_clients}')

def run_traffic_layer(traffic_layer_path, config_path):
    # start cmd /k ..\..\Trafficlayer\x64\Debug\TrafficLayer.exe -f '.\ecodrivingConfig.yaml'\
 
    os.system(f'start cmd /k {traffic_layer_path} -f {config_path}')
    
def run_carla(carla_path, config_path, traffic_light_table_path):
    os.system(f'start cmd /k {carla_path} -f {config_path} -t {traffic_light_table_path}')
