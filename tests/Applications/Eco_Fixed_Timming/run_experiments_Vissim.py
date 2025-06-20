import easydict as edict
import os
import yaml
from ruamel.yaml import YAML
import time
import sys
import shutil
import xml.etree.ElementTree as ET

def change_config_directory(vissim_setting, source_dir, output_dir):

    shutil.copytree(os.path.join(source_dir, vissim_setting['src_path']), output_dir, dirs_exist_ok=True)


def run_traffic_layer(setting_dir, port):
    # start cmd /k ..\..\Trafficlayer\x64\Debug\TrafficLayer.exe -f '.\ecodrivingConfig.yaml'\

    os.system(f'start cmd /k {setting_dir}\\TrafficLayer.exe -f {setting_dir}\\ecodriving_config_Vissim.yaml')

def run_controller(setting_dir, vissim_port, traffic_layer_port, simulink_port, penetration_rate, simulation_file_path, eco_driving=True, vehicle_dynamics=True):
    command = f'start cmd /k python .\\eco_driving_mpr_Vissim.py -c {setting_dir}\\ecodriving_config_Vissim.yaml --vissimPort {vissim_port} --trafficlayerPort {traffic_layer_port} --simulinkPort {simulink_port} --pathToNet {simulation_file_path} --penetrationRate {penetration_rate}'
    if eco_driving:
        command += ' --ecoDriving'
    if vehicle_dynamics:
        command += ' --vehicleDynamics'
    os.system(command)

def run_simulink(setting_dir, model_name):
    
    # start cmd /c matlab -nodesktop -nosplash -r "configFilename = '.\config_SUMO.yaml'; simModelName= 'EV_longitude'; ecodrivingMain; "
    matlab_command = f"cmd /c matlab -nodesktop -nosplash -r \"realsim_script_Vissim('{setting_dir}', '{model_name}');\""
    print(matlab_command)
    # Execute the MATLAB command
    os.system(matlab_command)


def run_one_setting(setting_dir, source_dir, experiment_setting):

    penetration_rate = experiment_setting['penetration_rate']
    vissim_port = experiment_setting['Vissim']['port']
    simulink_port = experiment_setting['Simulink']['port']
    model_name = experiment_setting['Simulink']['model_name']
    fixs_port = experiment_setting['FIXS']['port']
    ecodriving = experiment_setting['eco_driving']
    vehicle_dynamics = experiment_setting['vehicle_dynamics']
    os.chdir(setting_dir)
    run_traffic_layer(setting_dir, fixs_port)
    # wait until initialization is done
    time.sleep(2)
    os.chdir('..\\..\\..\\')
    run_controller(setting_dir, vissim_port, fixs_port, simulink_port, penetration_rate, setting_dir, ecodriving, vehicle_dynamics)
    time.sleep(2)
    os.chdir(setting_dir)
    # print('Running Simulink model...')
    # print(f'setting_dir: {setting_dir}')
    # run_simulink(setting_dir, model_name)

def run_settings(config):
    root_dir = config['root_dir']
    for experiment_idx, experiment_setting in config['Experiments'].items():
            
        working_dir = os.path.join(root_dir, experiment_setting['working_dir'])
        source_dir = os.path.join(root_dir, experiment_setting['source_dir'])
        
        if not os.path.exists(working_dir):
            os.makedirs(working_dir)
        os.chdir(working_dir)
        step_length = experiment_setting['step_length']
        penetration_rate = experiment_setting['penetration_rate']
        ecodriving = experiment_setting['eco_driving']
        vehicle_dynamics = experiment_setting['vehicle_dynamics']
        model_name = experiment_setting['Simulink']['model_name']
        output_dir = os.path.join(working_dir, '{}%_{}Hz'.format(int(penetration_rate*100), int(1/step_length)))
        
        if ecodriving:
            output_dir += '_E'
        else:
            output_dir += ''
        if vehicle_dynamics:
            output_dir += '_D'
            output_dir += '_' + model_name
        else:
            output_dir += ''

        vissim_setting = experiment_setting['Vissim']
        change_config_directory(vissim_setting, source_dir=source_dir, output_dir=output_dir)


        # copy the .\TrafficLayer.exe to the experiment directory
        shutil.copy(os.path.join(source_dir, 'TrafficLayer.exe'), output_dir)
        # copy the realsim script to the experiment directory
        shutil.copy(os.path.join(source_dir, 'realsim_script_Vissim.m'), output_dir)
        # copy the sumo signal control file to the experiment directory
        vissim_port = vissim_setting['port']
        simulink_port = experiment_setting['Simulink']['port']
        fixs_port = experiment_setting['FIXS']['port']
        config_file_path = os.path.join(source_dir, experiment_setting['config_file'])

        yaml = YAML()
        yaml.preserve_quotes = True  # Optional: Preserve quotes if present in the original YAML
        with open(os.path.join(config_file_path), 'r') as f:
            config = yaml.load(f)

        # Modify the YAML data
        config['SimulationSetup']['TrafficSimulatorPort'] = vissim_port
        config['ApplicationSetup']['VehicleSubscription'][0]['port'] = [fixs_port]
        config['XilSetup']['VehicleSubscription'][0]['port'] = [simulink_port]

        # Write back to the file while preserving the original format
        with open(os.path.join(output_dir, os.path.basename(config_file_path)), 'w') as f:
            yaml.dump(config, f)


        run_one_setting(output_dir, source_dir, experiment_setting)
        os.chdir(working_dir)
        break
# main function
if __name__ == '__main__':
    config_file = './Experiments_Vissim/experiment_config_Vissim.yaml'
    with open(config_file, 'r') as f:
        config = yaml.load(f, Loader=yaml.FullLoader)
    
    run_settings(config)