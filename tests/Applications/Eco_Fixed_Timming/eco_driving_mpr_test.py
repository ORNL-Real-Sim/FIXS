import numpy as np
import traci
import os
import sys
import sumolib
from cav_casestudy.SUMO.traffic_light_utils import TrafficLight
from cav_casestudy.SUMO.vehicle_eco_pilot import EcoVehicle
from cav_casestudy.SUMO.speed_control_utils import *
from cav_casestudy.SUMO.utils1 import process_spat_for_gen_speed_update
from sumolib import checkBinary
import xml.etree.ElementTree as ET
import traci.constants as tc
import socket
import argparse
import pandas as pd
import time
from CommonLib.SocketHelper import SocketHelper
from CommonLib.ConfigHelper import ConfigHelper
from CommonLib.MsgHelper import MsgHelper
from CommonLib.VehDataMsgDefs import VehData
if 'SUMO_HOME' in os.environ:
    sys.path.append(os.path.join(os.environ['SUMO_HOME'], 'tools'))


class SumoEnvMultiAgent:
    def __init__(self, sumoSignalConfig, traffic_layer_config, sumo_port=1337, traffic_layer_port=430, simulink_port=420, path_to_net=''):
        # sumo startup utils
        self.sumoBinary = checkBinary('sumo-gui')  # or 'sumo-gui' for graphical interface
        self.sumo_config_file = os.path.join(path_to_net, 'chattCavMpr.sumocfg')
        self.sumo_net_file = os.path.join(path_to_net, 'chatt.net.xml')
        self.settings_dir = path_to_net
        self.sumoCmd = [self.sumoBinary, "-c", self.sumo_config_file]
        self.graph = sumolib.net.readNet(self.sumo_net_file, withInternal=True)  # internal edges are edges inside intersections or connections
        self.vertex = self.graph.getNodes()
        self.edge = self.graph.getEdges(withInternal=True)
        self.edges = ['-2801', '-280', '-307', '-327', '-281', '-315', '-321', '-300', '-2851', '-285', '-290', '-298',
                      '-293', '-297', '-288', '-286', '-302', '-3221', '-322', '-313', '-284', '-328', '-304', '-2801']
        self.wb = ['-2801', '-280', '-307', '-327', '-281', '-315', '-321', '-300', '-2851', '-285', '-290', '-298', '-295']
        self.eb = ['-312', '-293', '-297', '-288', '-286', '-302', '-3221', '-322', '-313', '-284', '-328', '-304',]
        self.sumoSignalConfig = sumoSignalConfig


        # traffic light ids
        self.tl_ids = ['2', '3', '10', '8', '9', '12']
        self.phase_tracking_dict = {}
        self.spat_statuses = {}
        self.color_dict = {'green': 0, 'red': 1}
        self.cav_id_list = []
        self.cav_object_dict = {}
        # Subscribe to specific variables (e.g., position, speed)
        # self.subscription_vars = [tc.VAR_POSITION, tc.VAR_SPEED]
        self.subscription_vars = [tc.VAR_SPEED, tc.VAR_POSITION, tc.VAR_TYPE, tc.VAR_ROAD_ID, tc.VAR_ACCELERATION, tc.VAR_ROUTE_INDEX]
        self.subscribed_vehicles = []
        self.speed_min = 0
        self.speed_max = 21
        self.max_acc = 4.0
        self.prev_acc = 0.01

        # initialize the socket connections
        config_helper = ConfigHelper()
        config_helper.getConfig(traffic_layer_config)
        msg_helper = MsgHelper()
        msg_helper.set_vehicle_message_field(config_helper.simulation_setup['VehicleMessageField'])
        self.socket_helper = SocketHelper(config_helper=config_helper, msg_helper=msg_helper)
        # IP to connect to the FIXS server
        
        self.sumo_port = sumo_port
        self.traffic_layer_port = traffic_layer_port
        self.simulink_port = simulink_port
        self.socket2FIXS = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket2FIXS.connect(('127.0.0.1', int(self.traffic_layer_port)))
        print('Connected to FIXS server')

        socket2simulink = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        print('Waiting for Simulink client to connect...')
        # bind the socket to the port and listen for incoming connections
        print('Binding to port: ', self.simulink_port)
        socket2simulink.bind(('127.0.0.1', int(self.simulink_port)))
        socket2simulink.listen(1)

        # if a connection is established, accept it
        self.socket2simulink, addr = socket2simulink.accept()
        print('Connected by Simulink client')


    def get_phases(self, sumoSignalConfig):
        for tl_id in self.tl_ids:
            self.phase_tracking_dict[tl_id] = TrafficLight(tl_id, sumoSignalConfig)

    def phase_tracker(self):
        # Phase trackers
        for tl_id in self.tl_ids:
            self.phase_tracking_dict[tl_id].get_remaining_green()
            self.spat_statuses[tl_id] = self.phase_tracking_dict[tl_id].spat_status

    def reset(self):
        if 'ego' in traci.vehicle.getIDList():
            traci.vehicle.remove('ego')
            traci.vehicle.add('ego', 'route1', departPos=str(1), departSpeed=str(0.1), departLane='0')
            traci.vehicle.setColor('ego', color=(255, 0, 0, 255))
            traci.vehicle.setSpeedMode('ego', 31)  # https://sumo.dlr.de/docs/TraCI/Change_Vehicle_State.html#speed_mode_0xb3
            traci.vehicle.setDecel('ego', self.max_acc)
            traci.vehicle.setAccel('ego', self.max_acc)
        else:
            # traci.simulationStep()
            traci.vehicle.add('ego', 'route1', departPos=str(1), departSpeed=str(0.1), departLane='0')
            traci.vehicle.setColor('ego', color=(255, 0, 0, 255))
            traci.vehicle.setSpeedMode('ego', 31)  # https://sumo.dlr.de/docs/TraCI/Change_Vehicle_State.html#speed_mode_0xb3
            traci.vehicle.setDecel('ego', self.max_acc)
            traci.vehicle.setAccel('ego', self.max_acc)



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
            if travel_direction in ['EB', 'WB']:
                if len(next_tls) >= 2:
                    control = 'True'
                # only has one intersection to go but the route edges contains the last segment on either EB or WB
                elif '-304' in route_edges or '-295' in route_edges:
                    control = 'True'
                else:
                    control = 'False'
            else:
                control = 'False'
        else:
            control = 'False'

        return travel_direction, control
    
    def get_spat(self, phase_tracking_dict, spat_statuses, tl_light, travel_direction, control):

        if control == 'True':
            if tl_light:
                t1s, t1e, t2s, t2e, r1s, curr_status = process_spat_for_gen_speed_update(tl_light, travel_direction,
                                                                                         phase_tracking_dict, spat_statuses)
            else:
                t1s, t1e, t2s, t2e, r1s, curr_status = 0, 50, 100, 150, 50, 'green'
        else:
            t1s, t1e, t2s, t2e, r1s, curr_status = 0, 50, 100, 150, 50, 'green'

        return t1s, t1e, t2s, t2e, r1s, curr_status

    def start_subscription(self, mpr, step_length, vehicle_dynamics=True, eco_driving=True):
        """
        What needs to be subscribed
            1. speed, yes
            2. acceleration, yes
            3. getNextTLS
            4. getRoadID, yes
            5. getLeader, yes
        :return:
        """

        # traci.start(self.sumoCmd)
        traci.init(port=int(self.sumo_port), host='127.0.0.1')
        traci.setOrder(2)
        traci.route.add(routeID='route1', edges=self.edges)

        self.get_phases(self.sumoSignalConfig)

        sim_time = traci.simulation.getTime()

        start_time_1 = time.time()
        while sim_time < 28985:
            sim_time = traci.simulation.getTime()
            # Phase trackers
            self.phase_tracker()
            self.subscribe_departed_veh()
            if sim_time < 28985:
                traci.simulationStep()
            elif sim_time == 28985:
                self.reset()
                traci.simulationStep()
            self.apply_vehicle_control_FIXS({}, vehicle_dynamics=vehicle_dynamics, eco_driving=eco_driving)
        print('Total time spent for the first 28985: ', time.time() - start_time_1)
        print('test')

        print(self.cav_object_dict.keys())
        while sim_time <= 33000:
            # try:
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
                                                    114: 'acceleration', 112: 'next_tls'})
            results_df['edges'] = results_df['veh_id'].apply(lambda x: traci.vehicle.getRoute(x) if x is not None else None)
            results_df['route_index'] = results_df['veh_id'].apply(lambda x: traci.vehicle.getRouteIndex(x) if x is not None else None)
            results_df['next_tls'] = results_df['veh_id'].apply(lambda x: traci.vehicle.getNextTLS(x) if x is not None else None)
            results_df[['travel_direction', 'control']] = results_df.apply(lambda row: self.get_travel_direction(row['veh_id'], row['veh_type'],row['road_id'], row['route_index'], row['edges'], row['next_tls']), axis=1, result_type='expand')
            
            # get lead vehicle's speed
            results_df['speed'] = results_df['speed'] * 2.23694
            results_df['leader'] = results_df['veh_id'].apply(lambda x: traci.vehicle.getLeader(x) if x is not None else None)
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
                                                                                                                    row['travel_direction'],
                                                                                                                    row['control']), axis=1, result_type='expand')
            list_cav_back_to_sumo = results_df.loc[(results_df['veh_type'] == 'CAV') & (results_df['control'] == 'False')].index.tolist()
            list_cav_control = results_df.loc[results_df['control'] == 'True'].index.tolist()
            # set indefinite route for the 'ego' vehicle
            if 'ego' in results_df.index.tolist():
                if results_df.loc['ego', 'road_id'] == "-2801":
                    traci.vehicle.setRoute('ego', self.edges)
            # print(list_cav_back_to_sumo + list_cav_control)
            # print(self.cav_object_dict.keys())
            eco_speed_dic = {key: veh.get_eco_speed_subscribe(
                                                        self.phase_tracking_dict,
                                                        self.spat_statuses,
                                                        results_df.loc[key, 'next_tls'],
                                                        results_df.loc[key, 'travel_direction'],
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
            # self.apply_vehicle_control(eco_speed_dic, smooth=False)
            # for index, (key, veh) in enumerate(self.cav_object_dict.items()):
            #     if key in (list_cav_back_to_sumo):
            #         veh.gain_back_sumo_control()
            # print('eco_speed_dic: ', eco_speed_dic)
            
            self.cav_object_dict = {key: value for key, value in self.cav_object_dict.items() if key in (list_cav_back_to_sumo + list_cav_control)}

            traci.simulationStep()
            self.apply_vehicle_control_FIXS(eco_speed_dic, vehicle_dynamics=vehicle_dynamics, eco_driving=eco_driving)
            


            # except:
            #     ego_travel_direction_df.to_csv(output_dir + '\ego_travel_direction.csv', index=False)
            #     break
        # ego_travel_direction_df.to_csv(output_dir + '\ego_travel_direction.csv', index=False)
        self.close()

    
    def apply_vehicle_control_FIXS(self, eco_speed_dic, vehicle_dynamics=True, eco_driving=True):
        """
        Apply the vehicle control to the vehicles
        :param eco_speed_dic: dictionary of vehicle id and eco speed
        :param vehicle_dynamics: apply vehicle dynamics
        :param eco_driving: apply eco driving
        :return:
        """
        sim_state, sim_time = self.socket_helper.recv_data(self.socket2FIXS)
        ori_speed = {''.join([chr(c) for c in (veh_data.id[:veh_data.idLength])])
                     :veh_data.speed for veh_data in self.socket_helper.vehicle_data_receive_list}
        for veh_id, eco_speed in eco_speed_dic.items():
            if eco_speed is not None and veh_id == 'ego':
                # veh_id to uint8Arr
                veh_id_uint8Arr = [ord(c) for c in veh_id]
                veh_id_length = len(veh_id_uint8Arr)
                if eco_driving:
                    veh_data = VehData(id=veh_id_uint8Arr, idLength=veh_id_length, speedDesired=eco_speed)
                else:
                    veh_data = VehData(id=veh_id_uint8Arr, idLength=veh_id_length, speedDesired=ori_speed[veh_id])
                self.socket_helper.vehicle_data_send_list.append(veh_data)
        
        self.socket_helper.sendData(sim_state, sim_time, self.socket2simulink)
        self.socket_helper.clear_data()
        # receive data from the client (the actual vehicle data after the vehidle dynamics model)
        self.socket_helper.recv_data(self.socket2simulink)
        
        self.socket_helper.vehicle_data_send_list.extend(self.socket_helper.vehicle_data_receive_list)
        for idx in range(len(self.socket_helper.vehicle_data_send_list)):
            speed_desired_simulink = self.socket_helper.vehicle_data_receive_list[idx].speedDesired
            if not vehicle_dynamics:
                # if not applying vehicle dynamics, set the speedDesired to the eco_speed
                veh_id = ''.join([chr(c) for c in (self.socket_helper.vehicle_data_receive_list[idx].id[:self.socket_helper.vehicle_data_receive_list[idx].idLength])])
                if veh_id not in eco_speed_dic.keys():
                    continue
                
                speed_desired_eco = eco_speed_dic[veh_id]
                self.socket_helper.vehicle_data_send_list[idx].speedDesired = speed_desired_eco

            else:
                # if applying vehicle dynamics, set the speedDesired to the speedDesired from the simulink
                self.socket_helper.vehicle_data_send_list[idx].speedDesired = speed_desired_simulink

        if len(self.socket_helper.vehicle_data_send_list) == 0:
            veh_data = VehData()
            self.socket_helper.vehicle_data_send_list.append(veh_data)
        self.socket_helper.sendData(sim_state, sim_time, self.socket2FIXS)



    def apply_vehicle_control(self, eco_speed_dic, smooth=False):
        # to handle the case of a single vehicle
       for veh_id, eco_speed in eco_speed_dic.items():
            if eco_speed is not None:
                if smooth:
                    traci.vehicle.slowDown(veh_id, eco_speed, 1)
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


if __name__ == "__main__":
    start_time = time.time()
    
    
    
    parser = argparse.ArgumentParser()
    parser.add_argument("-c", "--config", type=str, help="Path to the Configuration file", default=None)
    parser.add_argument("--sumoPort", type=str, help="Specify port of sumo", default=1333)
    parser.add_argument("--trafficlayerPort", type=str, help="Specify port of traffic layer", default=433)
    parser.add_argument("--simulinkPort", type=str, help="Specify port of simulink", default=423)
    parser.add_argument("--ecoDriving", action="store_true", help="Use the eco driving controller", default=False)
    parser.add_argument("--vehicleDynamics", action="store_true", help="use the vehicle dynamis", default=False)
    parser.add_argument("--penetrationRate", type=float, help="the penetration rate of cav", default=1.0)
    parser.add_argument("--pathToNet", type=str, help="the path to the net file", default=None)
    
    args = parser.parse_args()
    traffic_layer_config = args.config
    sumo_port = args.sumoPort
    traffic_layer_port = args.trafficlayerPort
    simulink_port = args.simulinkPort
    penetration_rate = float(args.penetrationRate)
    path_to_net = args.pathToNet
    vehicle_dynamics = args.vehicleDynamics
    eco_driving = args.ecoDriving
    # read sumoSignalConfig_26
    sumoSignalConfig = pd.read_csv(os.path.join(path_to_net, 'sumoSignalConfig_26.csv'), index_col=0)
    sumoSignalConfig['id'] = sumoSignalConfig['id'].astype(str)
    sumoSignalConfig['name'] = sumoSignalConfig['name'].astype(str)
    print(f'running with penetration rate: {penetration_rate}, vehicle dynamics: {vehicle_dynamics}, eco driving: {eco_driving}')
    senv = SumoEnvMultiAgent(sumoSignalConfig, traffic_layer_config=traffic_layer_config, sumo_port=sumo_port, traffic_layer_port=traffic_layer_port, simulink_port=simulink_port, path_to_net=path_to_net)
    step_length = 1
    senv.start_subscription(penetration_rate, step_length, vehicle_dynamics=vehicle_dynamics, eco_driving=eco_driving)
    total_time = time.time() - start_time
    print('Total time spent: ', total_time)
