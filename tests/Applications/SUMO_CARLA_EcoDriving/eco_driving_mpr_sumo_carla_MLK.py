import os
from dotenv import load_dotenv
import time
import traci
import shutil
import socket
import argparse
import numpy as np
import pandas as pd
import traci.constants as tc
import xml.etree.ElementTree as ET
from CommonLib.SocketHelper import SocketHelper
from CommonLib.ConfigHelper import ConfigHelper
from CommonLib.MsgHelper import MsgHelper
from CommonLib.VehDataMsgDefs import VehData
from collections import defaultdict
from mlk_xil.traffic_light_utils import TrafficLight
from mlk_xil.vehicle_eco_pilot import EcoVehicle
from mlk_xil.speed_control_utils import *
from mlk_xil.utils1 import process_spat_for_gen_speed_update


class SumoEnvMultiAgent:
    def __init__(self, 
                 sumo_folder, # path to the folder containing the sumo files
                 sumo_config, # file name of the sumo config file
                 sumo_net, # file name of the sumo net file 
                 sumo_route, # file name of the sumo route file
                 step_length : float, # step length of the simulation
                 penetration_rate : float, # market penetration rate of cav
                 working_directory, # root of where the updated sumo files are stored
                 eb_links, # eastbound links
                 wb_links, # westbound links
                 tl_ids, # traffic light ids
                 sumo_signal_config, # sumo signal config dataframe
                 movement_indices_dict, # movement indices dictionary
                 traffic_layer_config, # traffic layer config
                 sumo_ip: str='127.0.0.1', # ip address of the sumo
                 sumo_port: int=1337, # port of the sumo
                 traffic_layer_ip: str ='127.0.0.1', # ip address of the traffic layer
                 traffic_layer_port: int=430,  # port of the traffic layer
                 enable_vehicle_dynamics: bool=False,
                 vehicle_dynamics_ip: str='127.0.0.1',
                 vehicle_dynamics_port: int=420):
        # sumo startup utils
        self.sumo_folder = sumo_folder
        self.working_directory = working_directory
        self.sumo_config = sumo_config
        self.sumo_net = sumo_net
        self.sumo_route = sumo_route
        self.penetration_rate = penetration_rate
        self.step_length = step_length
        self.enable_vehicle_dynamics = enable_vehicle_dynamics
        self.change_config_directory() 
        self.wb = wb_links
        self.eb = eb_links
        self.edges = self.eb + self.wb + [self.eb[0]] # a loop route for the ego vehicle
        


        # traffic light ids
        self.tl_ids = tl_ids
        self.sumo_signal_config = sumo_signal_config
        self.movement_indices_dict = movement_indices_dict
        self.phase_tracking_dict = defaultdict(lambda: TrafficLight(None))
        self.spat_statuses = {}
        self.color_dict = {'green': 0, 'red': 1}
        self.cav_id_list = []
        self.cav_object_dict = defaultdict(lambda: EcoVehicle(None))
        # Subscribe to specific variables (e.g., position, speed)
        self.subscription_vars = [tc.VAR_SPEED, tc.VAR_POSITION, tc.VAR_TYPE, tc.VAR_ROAD_ID, tc.VAR_ACCELERATION, 
                                  tc.VAR_ROUTE_INDEX, tc.VAR_SPEED_WITHOUT_TRACI, tc.VAR_ALLOWED_SPEED]
        self.subscribed_vehicles = []
        self.speed_min = 0
        self.speed_max = 21
        self.max_acc = 2.0
        self.prev_acc = 0.01

        # initialize the socket connections
        config_helper = ConfigHelper()
        config_helper.getConfig(traffic_layer_config)
        msg_helper = MsgHelper()
        msg_helper.set_vehicle_message_field(config_helper.simulation_setup['VehicleMessageField'])
        self.socket_helper = SocketHelper(config_helper=config_helper, msg_helper=msg_helper)
        
    
        self.sumo_ip = sumo_ip
        self.sumo_port = sumo_port

        # IP to connect to the FIXS server
        self.traffic_layer_ip = traffic_layer_ip
        self.traffic_layer_port = traffic_layer_port
        self.vehicle_dynamics_ip = vehicle_dynamics_ip
        self.vehicle_dynamics_port = vehicle_dynamics_port

            
    def change_cav_mpr(self, output_dir):

        xmlTree = ET.parse(os.path.join(self.sumo_folder, self.sumo_route))
        xmlRoot = xmlTree.getroot()

        # Cycle through all vehicles defined in file.
        for vehicle_class in xmlRoot.findall("vType"):

            # Looks for the car ID.
            car_id = vehicle_class.get("id")
            print(car_id)

            # Assign probabilities to vehicle classes based on argument
            if (car_id == "CAV"):
                vehicle_class.set("probability", str(self.penetration_rate))
            elif (car_id == "HDV"):
                vehicle_class.set("probability", str(1 - self.penetration_rate))
            else:
                print("Error in assigning probabilities")

        xmlTree.write(os.path.join(output_dir, self.sumo_route))

    def change_config_directory(self):
        file_name = f'{int(self.penetration_rate*100)}%_{int(1/self.step_length)}Hz' + f'{"_w" if self.enable_vehicle_dynamics else "_wo"}_vehDyn'
        output_dir = os.path.join(self.sumo_folder, self.working_directory, file_name)

        # Making Results Directory
        try:
            os.mkdir(output_dir)
            print("Created Directory:" + output_dir)
        except OSError as error:
            print("Directory already exists, will overwrite  upon simulation.")
        shutil.copy(os.path.join(self.sumo_folder, "updated_signal.xml"), output_dir)
        shutil.copy(os.path.join(self.sumo_folder, self.sumo_net), output_dir)

        # MPR
        self.change_cav_mpr(output_dir)

        # sumo config
        xmlTree = ET.parse(os.path.join(self.sumo_folder, self.sumo_config))
        xmlRoot = xmlTree.getroot()

        # Using Iterator to quickly search sub trees
        for step_len in xmlRoot.iter("step-length"):
            # Alters the output file to the new directory for the simulation statistics.
            step_len.set("value", str(step_length))

        self.sumo_config = os.path.join(output_dir, self.sumo_config)
        xmlTree.write(self.sumo_config)
        print("New SUMO Config File Created")
    
    def get_phases(self, sumo_signal_config):
        for tl_id in self.tl_ids:
            self.phase_tracking_dict[tl_id] = TrafficLight(tl_id, sumo_signal_config)

    def phase_tracker(self):
        # Phase trackers
        for tl_id in self.tl_ids:
            self.phase_tracking_dict[tl_id].get_remaining_green()
            self.spat_statuses[tl_id] = self.phase_tracking_dict[tl_id].spat_status
    
    def insert_ego_safely(self):

        # IMPORTANT: give SUMO freedom to place the car
        # (best lane, free position, max allowed speed)
        traci.vehicle.add(
            vehID='ego',
            routeID='route1',
            typeID='CAV',  # '' uses DEFAULT_VEHTYPE (or change to an existing vType, e.g. 'CAV')
            depart="now",  # explicit time in seconds (string or int is fine)
            departPos='free',
            departLane='best',
            departSpeed='max',
        )
        traci.vehicle.setColor('ego', (255, 0, 0, 255))
        traci.vehicle.setSpeedMode('ego', 31)
        traci.vehicle.setDecel('ego', self.max_acc)
        traci.vehicle.setAccel('ego', self.max_acc)
    
    def reset(self):
        # If ego is currently in the network, remove it and step once so removal takes effect
        if 'ego' in traci.vehicle.getIDList():
            traci.vehicle.remove('ego')
            
        self.insert_ego_safely()

    def subscribe_departed_veh(self):

        # Get newly departed vehicles, tuple
        departed_vehicles = traci.simulation.getDepartedIDList()
        # Subscribe to all departed vehicles
        for veh_id in departed_vehicles:
            try:
                if veh_id not in self.subscribed_vehicles:
                    traci.vehicle.subscribe(veh_id, self.subscription_vars)
                    # traci.vehicle.subscribeLeader(veh_id, dist=200.0)
                    self.subscribed_vehicles.append(veh_id)
                    # print(f"Subscribed to {veh_id}")

                    veh_type = traci.vehicle.getTypeID(veh_id)
                    # print(f"Vehicle {veh_id} type: {veh_type}")
                    if veh_type == 'CAV' or veh_id == 'ego':
                        
                        self.cav_id_list.append(veh_id)
                        self.cav_object_dict[veh_id] = EcoVehicle(veh_id)
                        # print(f"Create CAV agent {veh_id}")

            except:
                print('Subscription Error')
                continue

    def get_travel_direction(self, veh_id, veh_type, road_id, route_index, route_edges, next_tls):
        # get next edge
        if (route_index + 1) <= (len(route_edges) - 1):
            next_edge = route_edges[route_index + 1]
        else:
            next_edge = 'None'

        # get travel_direction
        if road_id in self.wb:
            travel_direction = 'WB'
        elif road_id in self.eb:
            travel_direction = 'EB'
        elif ':' in road_id:
            if next_edge in self.wb:
                travel_direction = 'WB'
            elif next_edge in self.eb:
                travel_direction = 'EB'
            else:
                travel_direction = 'NSB'
        else:
            travel_direction = 'NSB'

        # determine if control or not
        # still have two or more intersections to go
        if veh_type == 'CAV' or veh_id == 'ego':
            if travel_direction in ['EB', 'WB'] and road_id not in ['pos_1216083266', ':202695973_2', 'pos_19505254', 'E2', ':1939230364_4', ':1939230364_3', '111563727#2']:
                next_light = next_tls[0][0] if next_tls else None
                if next_light is not None:
                    next_tls_index = next_tls[0][1]
                    if next_tls_index in self.movement_indices_dict[next_light]:
                        control = 'True'
                    else:
                        control = 'False'
                    # do something with next_light
                else:
                    control = 'False'
            else:
                control = 'False'
        else:
            control = 'False'

        return travel_direction, control
    
    def get_spat(self, phase_tracking_dict, spat_statuses, tl_light, control):

        if control == 'True':
            if tl_light:
                t1s, t1e, t2s, t2e, r1s, curr_status = process_spat_for_gen_speed_update(tl_light, phase_tracking_dict, spat_statuses)
            else:
                t1s, t1e, t2s, t2e, r1s, curr_status = 0, 50, 100, 150, 50, 'green'
        else:
            t1s, t1e, t2s, t2e, r1s, curr_status = 0, 50, 100, 150, 50, 'green'

        # print(t1s, t1e, t2s, t2e, r1s, curr_status)

        return t1s, t1e, t2s, t2e, r1s, curr_status
    
    def run_sumo(self, num_clients, gui=False):
        # start sumo-gui -c .\chattCavMpr.sumocfg --remote-port 1337 --step-length 1 --netstate-dump chatt.xml --netstate-dump.precision 10 --num-clients 2  --begin 28800 --end 33000
        # the files are in setting_dir
        if gui:
            os.system(f'start C:\\Users\\RVDP\\Desktop\\sumo-1.24.0\\bin\\sumo-gui -c {self.sumo_config} --remote-port {self.sumo_port} --step-length {self.step_length} --num-clients {num_clients} --time-to-teleport -1 --collision.action warn --collision.check-junctions false')
        else:
            os.system(f'start C:\\Users\\RVDP\\Desktop\\sumo-1.24.0\\bin\\sumo -c {self.sumo_config} --remote-port {self.sumo_port} --step-length {self.step_length} --num-clients {num_clients}')
    
    def setup_connections(self):
        
        self.socket2FIXS = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket2FIXS.connect((self.traffic_layer_ip, int(self.traffic_layer_port)))
        print('Connected to FIXS server')

        if self.enable_vehicle_dynamics:
            socket2simulink = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            print('Waiting for vehicle dynamics client to connect...')
            # bind the socket to the port and listen for incoming connections
            print(f'Binding to ip {self.vehicle_dynamics_ip} port {self.vehicle_dynamics_port}')
            socket2simulink.bind((self.vehicle_dynamics_ip, int(self.vehicle_dynamics_port)))
            socket2simulink.listen(1)
            # if a connection is established, accept it
            self.socket2simulink, addr = socket2simulink.accept()
            print('Connected by vehicle dynamics client')
            
    def start_subscription(self, vehicle_dynamics=False, eco_driving=False):

        veh_ids_controlled_by_FIXS = ['ego']
        traci.init(port=int(self.sumo_port), host=self.sumo_ip)
        traci.setOrder(2)
        

        traci.route.add(routeID='route1', edges=self.edges)

        self.get_phases(self.sumo_signal_config)
        sim_time = traci.simulation.getTime()

        start_time_1 = time.time()
        while sim_time < 28850:
            sim_time = traci.simulation.getTime()
            # Phase trackers
            self.phase_tracker()
            self.subscribe_departed_veh()
            if sim_time == 28850:
                self.reset()
            traci.simulationStep()
            self.apply_vehicle_control_FIXS({}, vehicle_dynamics=vehicle_dynamics, eco_driving=eco_driving, control_veh_ids=veh_ids_controlled_by_FIXS)
            
            
        print('Total time spent for the first 28985: ', time.time() - start_time_1)
         
        while sim_time <= 32400:
            
            sim_time = traci.simulation.getTime()
            if sim_time % 100 == 0:
                print(sim_time)
            # Phase trackers
            self.phase_tracker()
            self.subscribe_departed_veh()
            # get subscription results
            results = traci.vehicle.getAllSubscriptionResults()
            # convert results to a dataframe
            results_df = pd.DataFrame.from_dict(results, orient="index")
            results_df['veh_id'] = results_df.index.astype(str)
            # https://sumo.dlr.de/docs/TraCI/Vehicle_Value_Retrieval.html, https://www.rapidtables.com/convert/number/hex-to-decimal.html?x=70
            results_df = results_df.rename(columns={64: 'speed', 66: 'position', 79: 'veh_type', 80: 'road_id',
                                                    114: 'acceleration', 112: 'next_tls', 177: 'speed_wo_traci', 183: 'allowed_speed'})
            results_df['edges'] = results_df['veh_id'].apply(lambda x: traci.vehicle.getRoute(x) if x is not None else None)
            results_df['route_index'] = results_df['veh_id'].apply(lambda x: traci.vehicle.getRouteIndex(x) if x is not None else None)
            results_df['next_tls'] = results_df['veh_id'].apply(lambda x: traci.vehicle.getNextTLS(x) if x is not None else None)
            results_df[['travel_direction', 'control']] = results_df.apply(lambda row: self.get_travel_direction(row['veh_id'], row['veh_type'],row['road_id'], row['route_index'], row['edges'], row['next_tls']), axis=1, result_type='expand')
            
            results_df['orginal_desire_spd'] = results_df['allowed_speed'] * 2.23694
            # get lead vehicle's speed
            results_df['speed'] = results_df['speed'] * 2.23694
            results_df['leader'] = results_df['veh_id'].apply(lambda x: traci.vehicle.getLeader(x, dist=200.0) if x is not None else None)
            results_df['leader_id'] = np.where(results_df['leader'].notnull(), results_df['leader'].str[0], 'None')
            results_df = results_df.join(results_df['speed'], on='leader_id', rsuffix='_lead')
            results_df['lead_dist'] = np.where(results_df['leader'].notnull(), results_df['leader'].str[1] * 3.28084, 500 * 3.28084)
            # if lead dist < 0, set it to 0
            results_df['lead_dist'] = np.where(results_df['lead_dist'] < 0.5, 0.5, results_df['lead_dist'])
            results_df['speed_lead'] = np.where(results_df['leader'].notnull(), results_df['speed_lead'], self.speed_max * 2.23694)
            # get the distance to the stop bar
            results_df['dist2Stop'] = np.where(results_df['next_tls'].notnull(), results_df['next_tls'].str[0].str[2] * 3.28084, 10000.0)

            results_df[['t1s', 't1e', 't2s', 't2e', 'r1s', 'curr_status']] = results_df.apply(lambda row: self.get_spat(self.phase_tracking_dict,
                                                                                                                        self.spat_statuses,
                                                                                                                        row['next_tls'],
                                                                                                                        row['control']), axis=1, result_type='expand')
            list_cav_back_to_sumo = results_df.loc[(results_df['veh_type'] == 'CAV') & (results_df['control'] == 'False')].index.tolist()
            list_cav_control = results_df.loc[results_df['control'] == 'True'].index.tolist()
            # set indefinite route for the 'ego' vehicle
            if 'ego' in results_df.index.tolist():
                if results_df.loc['ego', 'road_id'] == "-2801":
                    traci.vehicle.setRoute('ego', self.edges)
                    
            eco_speed_dic = {key: veh.get_eco_speed_subscribe(results_df.loc[key, 'orginal_desire_spd'],
                                                         results_df.loc[key, 'speed'],
                                                         results_df.loc[key, 'acceleration'],
                                                         results_df.loc[key, 'lead_dist'],
                                                         results_df.loc[key, 'speed_lead'],
                                                         results_df.loc[key, 'control'],
                                                         results_df.loc[key, 'dist2Stop'],
                                                         results_df.loc[key, 't1s'],
                                                         results_df.loc[key, 't1e'],
                                                         results_df.loc[key, 't2s'],
                                                         results_df.loc[key, 't2e'],
                                                         results_df.loc[key, 'r1s'],
                                                         results_df.loc[key, 'curr_status']) for index, (key, veh) in enumerate(self.cav_object_dict.items()) if key in (list_cav_back_to_sumo + list_cav_control)}

            self.apply_vehicle_control(eco_speed_dic, smooth=True, exclude_veh_ids=veh_ids_controlled_by_FIXS)
            traci.simulationStep()
            self.apply_vehicle_control_FIXS(eco_speed_dic, vehicle_dynamics=vehicle_dynamics, eco_driving=eco_driving, control_veh_ids=veh_ids_controlled_by_FIXS)
            
            self.cav_object_dict = {key: value for key, value in self.cav_object_dict.items() if key in list(set(list_cav_back_to_sumo + list_cav_control + ["ego"]))}

        self.close()

    
    def apply_vehicle_control_FIXS(self, eco_speed_dic, vehicle_dynamics=False, eco_driving=False, control_veh_ids = ['ego']):
        """
        Apply the vehicle control to the vehicles
        :param eco_speed_dic: dictionary of vehicle id and eco speed
        :param vehicle_dynamics: apply vehicle dynamics
        :param eco_driving: apply eco driving
        :return:
        """
        sim_state, sim_time = self.socket_helper.recv_data(self.socket2FIXS)
        ori_speed = {veh_data.id:veh_data.speed for veh_data in self.socket_helper.vehicle_data_receive_list}
        for veh_id, eco_speed in eco_speed_dic.items():
            if eco_speed is not None and veh_id in control_veh_ids:
                if eco_driving:
                    if eco_speed > self.speed_max:
                        eco_speed = self.speed_max
                    veh_data = VehData(id=veh_id, speedDesired=eco_speed)
                else:
                    veh_data = VehData(id=veh_id, speedDesired=ori_speed[veh_id])
                self.socket_helper.vehicle_data_send_list.append(veh_data)
                
        if self.enable_vehicle_dynamics:
            self.socket_helper.sendData(sim_state, sim_time, self.socket2simulink)
            self.socket_helper.clear_data()
            # receive data from the client (the actual vehicle data after the vehidle dynamics model)
            self.socket_helper.recv_data(self.socket2simulink)
            self.socket_helper.vehicle_data_send_list.clear()
            self.socket_helper.vehicle_data_send_list.extend(self.socket_helper.vehicle_data_receive_list)
        
        for idx in range(len(self.socket_helper.vehicle_data_send_list)):

            if not vehicle_dynamics:
                # if not applying vehicle dynamics, set the speedDesired to the eco_speed
                veh_id = self.socket_helper.vehicle_data_receive_list[idx].id
                if veh_id not in eco_speed_dic.keys():
                    continue
                
                speed_desired_eco = eco_speed_dic[veh_id]
                if speed_desired_eco is not None:
                    self.socket_helper.vehicle_data_send_list[idx].speedDesired = speed_desired_eco
                else:
                    self.socket_helper.vehicle_data_send_list[idx].speedDesired = ori_speed[veh_id]

            else:
                
                speed_desired_simulink = self.socket_helper.vehicle_data_receive_list[idx].speedDesired
                speed_desired_simulink = max(self.speed_min, min(self.speed_max, speed_desired_simulink))
                # if applying vehicle dynamics, set the speedDesired to the speedDesired from the simulink
                self.socket_helper.vehicle_data_send_list[idx].speedDesired = speed_desired_simulink
                
        if len(self.socket_helper.vehicle_data_send_list) == 0:
            veh_data = VehData()
            self.socket_helper.vehicle_data_send_list.append(veh_data)
        
        self.socket_helper.sendData(sim_state, sim_time, self.socket2FIXS)
        self.socket_helper.clear_data()


    def apply_vehicle_control(self, eco_speed_dic, smooth=False, exclude_veh_ids=None):
        # to handle the case of a single vehicle
        for veh_id, eco_speed in eco_speed_dic.items():
            if eco_speed is not None and eco_speed >= 0 and veh_id not in (exclude_veh_ids if exclude_veh_ids else []):
                    if smooth:
                        traci.vehicle.slowDown(veh_id, eco_speed, self.step_length)
                    else:
                        traci.vehicle.setSpeed(veh_id, eco_speed)


    def close(self):
        traci.close()

    @classmethod
    def configToDict(cls, data):
        """
        Parses config contents into a dictionary.

        Parameters
        ----------
        data : ConfigParser.read() output
            content of config file in ConfigParser format

        Returns
        -------
        dict
            dictionary of config file
        """
        config = {}
        for section in data.sections():
            config[section] = {}
            for item in data[section]:
                config[section][item] = data[section][item]
        return config
    
def run_traffic_layer(traffic_layer_path, traffic_layer_config):
    os.system(f'start cmd /k {traffic_layer_path} -f {traffic_layer_config}')
    
def run_carla(carla_path, config_path, traffic_light_table_path):
    os.system(f'start cmd /k {carla_path} -f {config_path} -t {traffic_light_table_path}')

if __name__ == "__main__":
    load_dotenv()
    parser = argparse.ArgumentParser()
    parser.add_argument("--trafficlayerConfig", type=str, help="Path to the Configuration file", default=os.environ["CONFIG_PATH"])
    parser.add_argument("--trafficlayerIp", type=str, help="Specify Ip of traffic layer", default='127.0.0.1')
    parser.add_argument("--trafficlayerPort", type=str, help="Specify port of traffic layer", default=430)
    parser.add_argument("--vehicleDynamics", action="store_true", help="use the vehicle dynamics", default=True)
    parser.add_argument("--enableVehicleDynamics", action="store_true", help="use the vehicle dynamics", default=True)
    parser.add_argument("--vehicleDynamicsIp", type=str, help="Specify Ip of vehicle dynamics", default='192.168.140.11')
    parser.add_argument("--vehicleDynamicsPort", type=str, help="Specify port of vehicle dynamics", default=420)

    # the following parameters are for CAV settings
    parser.add_argument("--penetrationRate", type=float, help="the market penetration rate of cav", default=0.1)
    parser.add_argument("--stepLength", type=float, help="the step length of the simulation", default=0.1)
    parser.add_argument("--ecoDriving", action="store_true", help="Use the eco driving controller", default=True)
    
    # the following parameters are for SUMO 
    parser.add_argument("--sumoIp", type=str, help="Specify Ip of sumo", default='127.0.0.1')
    parser.add_argument("--sumoPort", type=str, help="Specify port of sumo", default=1337)
    ## sumo files
    parser.add_argument("--sumoFolder", type=str, help="Specify folder of sumo", default=os.environ["SIMULATION_FOLDER"])
    parser.add_argument("--sumoConfig", type=str, help="Specify sumo config file", default='mlk_simulation.sumocfg')
    parser.add_argument("--sumoNet", type=str, help="Specify sumo net file", default=os.environ["SUMO_NET_PATH"])
    parser.add_argument("--sumoRoute", type=str, help="Specify sumo route file", default='routes_8_9AM_v3.rou.xml')
    parser.add_argument("--workingDirectory", type=str, help="Specify working directory", default='MPR')
    args = parser.parse_args()
    traffic_layer_config = args.trafficlayerConfig
    traffic_layer_ip = args.trafficlayerIp
    traffic_layer_port = args.trafficlayerPort
    enable_vehicle_dynamics = args.enableVehicleDynamics
    vehicle_dynamics_ip = args.vehicleDynamicsIp
    vehicle_dynamics_port = args.vehicleDynamicsPort
    penetration_rate = args.penetrationRate
    step_length = args.stepLength
    eco_driving = args.ecoDriving
    vehicle_dynamics = args.vehicleDynamics
    sumo_ip = args.sumoIp
    sumo_port = args.sumoPort
    sumo_folder = args.sumoFolder
    sumo_config = args.sumoConfig
    sumo_net = args.sumoNet
    sumo_route = args.sumoRoute
    working_directory = args.workingDirectory


    df_eb_edges = pd.read_csv(os.path.join(sumo_folder, "connected_edge_sequence_EB.csv"))
    df_wb_edges = pd.read_csv(os.path.join(sumo_folder, "connected_edge_sequence_WB.csv"))
    wb_links = df_wb_edges["edge_id"].astype(str).tolist()
    eb_links = df_eb_edges["edge_id"].astype(str).tolist()

    # read sumo_signal_config
    sumo_signal_config = pd.read_csv(os.path.join(sumo_folder, 'sumoSignalConfig_26.csv'), index_col=0)
    sumo_signal_config['id'] = sumo_signal_config['id'].astype(str)
    sumo_signal_config['name'] = sumo_signal_config['name'].astype(str)

    # generate a dictionary for the controlled tls index for each intersection
    grouped = sumo_signal_config.groupby('id')['movement_index'].apply(lambda x: sorted(set([int(i) for i in ','.join(str(val).replace('[', '').replace(']', '') for val in x).split(',') if i.strip()]))).reset_index()
    # Convert to a clean dictionary mapping intersection ID to movement indices
    movement_indices_dict = dict(zip(grouped['id'], grouped['movement_index']))
    
    # list of signalized intersections
    tl_ids = ['202587081', '202605864', '651276685', '202596786', '202612556', '202645406', '202633693',
              '202608349', '202692612', '202639124', '202695971']
    print(f'running with penetration rate: {penetration_rate}, vehicle dynamics: {vehicle_dynamics}, eco driving: {eco_driving}')
    senv = SumoEnvMultiAgent(sumo_folder = sumo_folder, # path to the folder containing the sumo files
                             sumo_config = sumo_config, # file name of the sumo config file
                             sumo_net = sumo_net, # file name of the sumo net file
                             sumo_route = sumo_route, # file name of the sumo route file
                             step_length = step_length, # step length of the simulation
                             penetration_rate = penetration_rate, # market penetration rate of cav
                             working_directory = working_directory, # root of where the updated sumo files are stored
                             eb_links = eb_links, # eastbound links
                             wb_links = wb_links, # westbound links
                             tl_ids = tl_ids, # traffic light ids
                             sumo_signal_config = sumo_signal_config, # sumo signal config dataframe
                             movement_indices_dict = movement_indices_dict, # movement indices dictionary
                             traffic_layer_config = traffic_layer_config, # traffic layer config
                             sumo_ip = sumo_ip, # ip address of the sumo
                             sumo_port = sumo_port, # port of the sumo
                             traffic_layer_ip = traffic_layer_ip, # ip address of the traffic layer
                             traffic_layer_port = traffic_layer_port,  # port of the traffic layer
                             enable_vehicle_dynamics = enable_vehicle_dynamics,
                             vehicle_dynamics_ip = vehicle_dynamics_ip,
                             vehicle_dynamics_port = vehicle_dynamics_port)

    senv.run_sumo(num_clients=2, gui=True)
    time.sleep(2)
    run_traffic_layer('TrafficLayer.exe', os.environ["CONFIG_PATH"])
    time.sleep(2)
    # note: the traffic lights are disabled for now
    run_carla('VirCarlaEnv.exe', os.environ["CONFIG_PATH"], os.environ["SUMO_TLS_TABLE_PATH"])
    time.sleep(2)
    senv.setup_connections()
    print('Starting subscription')
    print('Enable vehicle dynamics: ', enable_vehicle_dynamics)
    print('Use eco driving controller: ', eco_driving)
    senv.start_subscription(eco_driving=eco_driving, vehicle_dynamics=vehicle_dynamics)
