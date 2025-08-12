import easydict as edict
import os
import yaml
from ruamel.yaml import YAML
import time
import sys
import shutil
import xml.etree.ElementTree as ET

def change_cav_mpr(mpr, src_dir, output_dir):
    xmlTree = ET.parse(src_dir+ r"chattCavMpr.rou.xml")
    xmlRoot = xmlTree.getroot()
    # Cycle through all vehicles defined in file.
    for vehicle_class in xmlRoot.findall("vType"):
        # Looks for the car ID.
        car_id = vehicle_class.get("id")

        # Assign probabilities to vehicle classes based on argument
        if (car_id == "CAV"):
            vehicle_class.set("probability", str(mpr))
        elif (car_id == "HDV"):
            vehicle_class.set("probability", str(1 - mpr))
        else:
            print("Error in assigning probabilities")
    xmlTree.write(os.path.join(output_dir, "chattCavMpr.rou.xml"))

def config_change(root, new_dir):

    # Using Iterator to quickly search sub trees
    for net_file in root.iter("net-file"):
        # Alters the output file to the new directory for the simulation statistics.
        net_file.set("value", os.path.join(new_dir, 'chatt.net.xml'))
    # Using Iterator to quickly search sub trees
    # for route_file in root.iter("route-files"):
    #     # Alters the output file to the new directory for the simulation statistics.
    #     route_file.set("value", new_dir + r"\chattCavMpr.rou.xml")
    # Using Iterator to quickly search sub trees
    for add_file in root.iter("additional-files"):
        # Alters the output file to the new directory for the simulation statistics.
        add_file.set("value", "updated_signal.xml" + ', ' +  os.path.join(new_dir, "Edge.add.xml"))
    # # Using Iterator to quickly search sub trees
    # for edge_out in root.iter("edgedata-output"):
    #     # Alters the output file to the new directory for the simulation statistics.
    #     edge_out.set("value", new_dir + r"\EdgeData.xml")
    #
    # # Using Iterator to quickly search sub trees
    # for fcd_out in root.iter("fcd-output"):
    #     # Alters the output file to the new directory for the tripinfo results.
    #     fcd_out.set("value", new_dir + r"\fcd.xml")
    #
    # # Using Iterator to quickly search sub trees
    # for signal_out in root.iter("timedEvent"):
    #     # Alters the output file to the new directory for the tripinfo results.
    #     signal_out.set("dest", new_dir + r"\signal_result.xml")
    print("New edge_out & fcd_out & signal_out outputs: " + new_dir )
    # Returns the edited root.
    return root
def change_config_directory(sumo_setting, penetration_rate, step_length, source_dir, output_dir):

    # shutil.copytree(sumo_setting['src_path'], output_dir)
    # THE sumo_setting['src_path']  is a relative path to the root directory
    # Copy the sumo files to the output directory
    print(source_dir)
    shutil.copytree(os.path.join(source_dir, sumo_setting['src_path']), output_dir, dirs_exist_ok=True)
    # NEMA signal.xml
    xmlTree_signal = ET.parse(os.path.join(source_dir, sumo_setting['src_path'], r"updated_signal.xml"))
    xmlRoot_signal = xmlTree_signal.getroot()
    # xmlRoot_signal = self.config_change(xmlRoot_signal, output_dir)
    xmlTree_signal.write(output_dir + r"\updated_signal.xml")
    # MPR
    change_cav_mpr(penetration_rate, os.path.join(source_dir, sumo_setting['src_path']), output_dir)
    # sumo config
    xmlTree = ET.parse(os.path.join(source_dir, sumo_setting['src_path'], r"chattCavMpr.sumocfg"))
    xmlRoot = xmlTree.getroot()
    xmlRoot = config_change(xmlRoot, output_dir)
    # Using Iterator to quickly search sub trees
    for step_len in xmlRoot.iter("step-length"):
        # Alters the output file to the new directory for the simulation statistics.
        step_len.set("value", str(step_length))
    sumo_config_file = os.path.join(output_dir, r"chattCavMpr.sumocfg")

    xmlTree.write(sumo_config_file)
    os.environ['PATH'] += os.pathsep + '..\\..\\..\\..\\CommonLib\\libsumo'

def run_sumo(setting_dir, port, step_length, num_clients, gui=False):
    # start sumo-gui -c .\chattCavMpr.sumocfg --remote-port 1337 --step-length 1 --netstate-dump chatt.xml --netstate-dump.precision 10 --num-clients 2  --begin 28800 --end 33000
    # the files are in setting_dir
    if gui:
        os.system(f'start sumo-gui -c {setting_dir}/chattCavMpr.sumocfg --remote-port {port} --step-length {step_length} --netstate-dump {setting_dir}/chatt.xml --netstate-dump.precision 10 --num-clients {num_clients}  --begin 28800 --end 32400')
    else:
        os.system(f'start sumo -c {setting_dir}/chattCavMpr.sumocfg --remote-port {port} --step-length {step_length} --netstate-dump {setting_dir}/chatt.xml --netstate-dump.precision 10 --num-clients {num_clients}  --begin 28800 --end 32400')

def run_traffic_layer(setting_dir, config_file_name):
    # start cmd /k ..\..\Trafficlayer\x64\Debug\TrafficLayer.exe -f '.\ecodrivingConfig.yaml'\
    
    os.system(f'start cmd /k {setting_dir}\\TrafficLayer.exe -f {os.path.join(setting_dir, config_file_name)}')

def run_controller(setting_dir, config_file_name, sumo_port, traffic_layer_port, simulink_port, penetration_rate, step_length, eco_driving=True, vehicle_dynamics=True, use_simulink_for_energy_evaluation=False, vanila=True):
    controller_script = 'eco_driving_mpr_SUMO_vanila.py' if vanila else 'eco_driving_mpr_SUMO.py'
    command = f'start cmd /k python .\\{controller_script} -c {os.path.join(setting_dir, config_file_name)} --sumoPort {sumo_port} --trafficlayerPort {traffic_layer_port} --simulinkPort {simulink_port} --pathToNet {setting_dir} --penetrationRate {penetration_rate} --stepLength {step_length}'
    if eco_driving:
        command += ' --ecoDriving'
    if vehicle_dynamics:
        command += ' --vehicleDynamics'
    if use_simulink_for_energy_evaluation:
        command += ' --useSimulinkForEnergyEvaluation'
    os.system(command)

def run_simulink(setting_dir, model_name, config_file_name):
    # start cmd /c matlab -nodesktop -nosplash -r "configFilename = '.\config_SUMO.yaml'; simModelName= 'EV_longitude'; ecodrivingMain; "
    matlab_command = f"cmd /c matlab -nodesktop -nosplash -r \"realsim_script_Sumo('{setting_dir}', '{model_name}', '{config_file_name}');\""
    print(matlab_command)
    # Execute the MATLAB command
    os.system(matlab_command)


def run_one_setting(setting_dir, source_dir, experiment_setting, vanila=False):

    penetration_rate = experiment_setting['penetration_rate']
    step_length = experiment_setting['step_length']
    sumo_port = experiment_setting['SUMO']['port']
    simulink_port = experiment_setting['Simulink']['port']
    model_name = experiment_setting['Simulink']['model_name']
    fixs_port = experiment_setting['FIXS']['port']
    ecodriving = experiment_setting['eco_driving']
    vehicle_dynamics = experiment_setting['vehicle_dynamics']
    use_simulink_for_energy_evaluation = experiment_setting['use_simulink_for_energy_evaluation']
    os.chdir(setting_dir)
    run_sumo(setting_dir, sumo_port, step_length, num_clients=2 if not vanila else 1, gui=True)
    config_file_name = os.path.basename(experiment_setting['config_file'])
    run_traffic_layer(setting_dir, config_file_name)
    # wait until initialization is done
    # if not vanila:
    #     time.sleep(5)
    #     os.chdir('..\\..\\..\\')
    #     run_controller(setting_dir, 
    #                    config_file_name, 
    #                    sumo_port, 
    #                    fixs_port, 
    #                    simulink_port, 
    #                    penetration_rate, 
    #                    step_length, 
    #                    ecodriving, 
    #                    vehicle_dynamics, 
    #                    use_simulink_for_energy_evaluation, 
    #                    vanila=vanila)
    #     time.sleep(2)
    #     os.chdir(setting_dir)
    #     if vehicle_dynamics or use_simulink_for_energy_evaluation:
    #         print('Running Simulink model...')
    #         print(f'setting_dir: {setting_dir}')
    #         run_simulink(setting_dir, model_name, config_file_name)

def run_settings(config, vanila=False):
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
        use_simulink_for_energy_evaluation = experiment_setting['use_simulink_for_energy_evaluation']
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

        # if os.path.exists(output_dir):
        #     shutil.rmtree(output_dir)
        # Copy the Sumo files to the experiment directory
        sumo_setting = experiment_setting['SUMO']
        change_config_directory(sumo_setting, penetration_rate, step_length, source_dir=source_dir, output_dir=output_dir)


        # copy the .\TrafficLayer.exe to the experiment directory
        shutil.copy(os.path.join(source_dir, 'TrafficLayer.exe'), output_dir)
        # copy the realsim script to the experiment directory
        shutil.copy(os.path.join(source_dir, 'realsim_script_Sumo.m'), output_dir)
        # copy the sumo signal control file to the experiment directory
        sumo_port = sumo_setting['port']
        simulink_port = experiment_setting['Simulink']['port']
        fixs_port = experiment_setting['FIXS']['port']
        config_file_name = os.path.basename(experiment_setting['config_file'])
        config_file_path = os.path.join(source_dir, config_file_name)

        yaml = YAML()
        yaml.preserve_quotes = True  # Optional: Preserve quotes if present in the original YAML
        with open(os.path.join(config_file_path), 'r') as f:
            config = yaml.load(f)

        # Modify the YAML data
        config['SimulationSetup']['TrafficSimulatorPort'] = sumo_port
        config['ApplicationSetup']['VehicleSubscription'][0]['port'] = [fixs_port]
        config['XilSetup']['VehicleSubscription'][0]['port'] = [simulink_port]

        # Write back to the file while preserving the original format
        with open(os.path.join(output_dir, config_file_name), 'w') as f:
            yaml.dump(config, f)


        run_one_setting(output_dir, source_dir, experiment_setting, vanila=vanila)
        os.chdir(working_dir)
        
if __name__ == '__main__':
    VANILA = False
    config_file = './Experiments_Sumo/experiment_config_Sumo_debug.yaml'
    with open(config_file, 'r') as f:
        config = yaml.load(f, Loader=yaml.FullLoader)
    
    run_settings(config, vanila=VANILA)