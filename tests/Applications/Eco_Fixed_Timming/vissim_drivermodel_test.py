import os


def run_traffic_layer(traffic_layer_path, config_path):
    
    os.system(f'start cmd /k {traffic_layer_path} -f {config_path}')
    
if __name__ == '__main__':
    traffic_layer_path = r"C:\\Users\\hg25079\\Documents\\GitHub\\FIXS\\tests\\Applications\\Eco_Fixed_Timming\\TrafficLayer.exe"
    config_path = r"C:\\Users\\hg25079\\Documents\\GitHub\\FIXS\\tests\\Applications\\Eco_Fixed_Timming\\Experiments_Vissim\\ecodrivingConfig_Vissim_speed.yaml"
    run_traffic_layer(traffic_layer_path, config_path)