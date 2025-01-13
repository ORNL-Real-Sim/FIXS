import numpy as np
import traci
import os
import sys
import sumolib
from traffic_light_utils import TrafficLight
from vehicle_eco_pilot import EcoVehicle
from speed_control_utils import *
from sumolib import checkBinary
import xml.etree.ElementTree as ET
import traci.constants as tc
import inspect
import socket
import argparse
from CommonLib.SocketHelper import SocketHelper
from CommonLib.ConfigHelper import ConfigHelper
from CommonLib.MsgHelper import MsgHelper
from CommonLib.VehDataMsgDefs import VehData
import typing

if 'SUMO_HOME' in os.environ:
    sys.path.append(os.path.join(os.environ['SUMO_HOME'], 'tools'))


class SumoEnvMultiAgent:
    def __init__(self, sumoSignalConfig, traffic_layer_config, working_directory='MPR'):
        # sumo startup utils
        self.sumoBinary = checkBinary('sumo-gui')  # or 'sumo-gui' for graphical interface
        self.sumo_config_file = os.path.join(os.getcwd(), 'chattCavMpr.sumocfg')
        self.sumo_net_file = os.path.join(os.getcwd(), 'chatt.net.xml')
        self.sumoCmd = [self.sumoBinary, "-c", self.sumo_config_file]
        self.graph = sumolib.net.readNet(self.sumo_net_file, withInternal=True)  # internal edges are edges inside intersections or connections
        self.vertex = self.graph.getNodes()
        self.edge = self.graph.getEdges(withInternal=True)
        self.edges = ['-2801', '-280', '-307', '-327', '-281', '-315', '-321', '-300', '-2851', '-285', '-290', '-298',
                      '-293', '-297', '-288', '-286', '-302', '-3221', '-322', '-313', '-284', '-328', '-304', '-2801']
        self.wb = ['-2801', '-280', '-307', '-327', '-281', '-315', '-321', '-300', '-2851',
                                              '-285', '-290', '-298', '-295']
        self.sumoSignalConfig = sumoSignalConfig
        self.working_directory = working_directory

        # traffic light ids
        self.tl_ids = ['2', '3', '10', '8', '9', '12']
        self.phase_tracking_dict = {}
        self.spat_statuses = {}
        self.color_dict = {'green': 0, 'red': 1}
        self.cav_id_list = []
        self.cav_object_dict = {}

        # Desired vehicle type to subscribe to
        self.target_vehicle_type = "CAV"
        # Subscribe to specific variables (e.g., position, speed)
        # self.subscription_vars = [tc.VAR_POSITION, tc.VAR_SPEED]
        self.subscription_vars = [tc.VAR_SPEED, tc.VAR_POSITION, tc.VAR_TYPE, tc.VAR_ROAD_ID, tc.VAR_ACCELERATION]

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
        _FIXS_IP = '127.0.0.1'
        _FIXS_PORT = '430'
        # IP for the simulink client to connect to
        _SIMULINK_IP='127.0.0.1'
        _SIMULINK_PORT='420'
        self.socket2FIXS = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket2FIXS.connect((_FIXS_IP, int(_FIXS_PORT)))
        print('Connected to FIXS server')

        socket2simulink = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        socket2simulink.bind((_SIMULINK_IP, int(_SIMULINK_PORT)))
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

    # Changes the directory of the (unique tags) stats and tripinfo device output file.
    def config_change(self, root, new_dir):

        # Using Iterator to quickly search sub trees
        for net_file in root.iter("net-file"):
            # Alters the output file to the new directory for the simulation statistics.
            net_file.set("value", os.path.join(os.getcwd(), 'chatt.net.xml'))

        # Using Iterator to quickly search sub trees
        # for route_file in root.iter("route-files"):
        #     # Alters the output file to the new directory for the simulation statistics.
        #     route_file.set("value", new_dir + r"\chattCavMpr.rou.xml")

        # Using Iterator to quickly search sub trees
        for add_file in root.iter("additional-files"):
            # Alters the output file to the new directory for the simulation statistics.
            add_file.set("value", "updated_signal.xml" + ', ' +  os.path.join(os.getcwd(), "Edge.add.xml"))

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

    def change_cav_mpr(self, mpr, output_dir):

        xmlTree = ET.parse(r"chattCavMpr.rou.xml")
        xmlRoot = xmlTree.getroot()

        # Cycle through all vehicles defined in file.
        for vehicle_class in xmlRoot.findall("vType"):

            # Looks for the car ID.
            car_id = vehicle_class.get("id")
            print(car_id)

            # Assign probabilities to vehicle classes based on argument
            if (car_id == "CAV"):
                vehicle_class.set("probability", str(mpr))
            elif (car_id == "HDV"):
                vehicle_class.set("probability", str(1 - mpr))
            else:
                print("Error in assigning probabilities")

        xmlTree.write(output_dir + r"\chattCavMpr.rou.xml")

    def change_config_directory(self, mpr, step_length, subscribe):
        output_dir = os.path.join(self.working_directory, '{}%_{}_{}Hz'.format(int(mpr*100), subscribe, int(1/step_length)))

        # Making Results Directory
        try:
            os.mkdir(output_dir)
            print()
            print("Created Directory:" + output_dir)
        except OSError as error:
            print("Directory already exists, will overwrite  upon simulation.")

        # NEMA signal.xml
        xmlTree_signal = ET.parse(r"updated_signal.xml")
        xmlRoot_signal = xmlTree_signal.getroot()
        # xmlRoot_signal = self.config_change(xmlRoot_signal, output_dir)
        xmlTree_signal.write(output_dir + r"\updated_signal.xml")

        # MPR
        self.change_cav_mpr(mpr, output_dir)

        # sumo config
        xmlTree = ET.parse(r"chattCavMpr.sumocfg")
        xmlRoot = xmlTree.getroot()
        xmlRoot = self.config_change(xmlRoot, output_dir)

        # Using Iterator to quickly search sub trees
        for step_len in xmlRoot.iter("step-length"):
            # Alters the output file to the new directory for the simulation statistics.
            step_len.set("value", str(step_length))

        self.sumo_config_file = output_dir + r"\chattCavMpr.sumocfg"
        xmlTree.write(self.sumo_config_file)

    

    def subscribe_departed_veh(self):
        self.subscribed_vehicles = set()

        # Get newly departed vehicles
        departed_vehicles = traci.simulation.getDepartedIDList()
        # print(departed_vehicles)

        # Subscribe to all departed vehicles
        for veh_id in departed_vehicles:
            if veh_id not in self.subscribed_vehicles:
                traci.vehicle.subscribe(veh_id, self.subscription_vars)
                # traci.vehicle.subscribeLeader(veh_id, dist=200.0)
                self.subscribed_vehicles.add(veh_id)
                print(f"Subscribed to {veh_id}")

                veh_type = traci.vehicle.getTypeID(veh_id)
                # print(veh_id, ' IS ', veh_type)
                if veh_type == 'CAV' or veh_id == 'ego':
                    self.cav_id_list.append(veh_id)
                    self.cav_object_dict[veh_id] = EcoVehicle(veh_id)

    def start_subscription(self, mpr, step_length):
        """
        What needs to be subscribed
            1. speed, yes
            2. acceleration, yes
            3. getNextTLS
            4. getRoadID, yes
            5. getLeader, yes
        :return:
        """
        # modify the mpr (e.g., 0.2, 0.4, 0.6, 0.8, 0.999)
        self.change_config_directory(mpr, step_length, 'Subscription')
        # self.sumoCmd = [self.sumoBinary, "-c", self.sumo_config_file]

        # traci.start(self.sumoCmd)
        traci.init(port=1337, host='127.0.0.1')
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
            self.apply_vehicle_control_FIXS({}, vehicle_dynamics=True, eco_driving=True)
        print('Total time spent for the first 28985: ', time.time() - start_time_1)
        print('test')

        
        while sim_time <= 33000:
            sim_time = traci.simulation.getTime()
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
                                                    114: 'acceleration', 112: 'next_tls', 104: 'leader'})

            results_df['travel_direction'] = np.where(results_df['road_id'].isin(self.wb), 'WB', 'EB')
            results_df['next_tls'] = results_df['veh_id'].apply(lambda x: traci.vehicle.getNextTLS(x) if x is not None else None)
            # get lead vehicle's speed
            results_df['leader'] = results_df['veh_id'].apply(lambda x: traci.vehicle.getLeader(x, dist=200.0) if x is not None else None)
            results_df['leader_id'] = np.where(results_df['leader'].notnull(), results_df['leader'].str[0], 'None')
            results_df = results_df.join(results_df['speed'], on='leader_id', rsuffix='_lead')
            
            # set indefinite route for the 'ego' vehicle
            if 'ego' in results_df.index.tolist():
                if results_df.loc['ego', 'road_id'] == "-2801":
                    traci.vehicle.setRoute('ego', self.edges)

            # only working on the CAVs that are currently in the network
            self.cav_object_dict = {key: value for key, value in self.cav_object_dict.items() if key in results_df.index.tolist()}
            
            # call the function and control the speed
            # key: veh_id, value: eco_speed
            eco_speed_dic = {key: veh.get_eco_speed_subscribe(self.phase_tracking_dict,
                                                     self.spat_statuses,
                                                     results_df.loc[key, 'next_tls'],
                                                     results_df.loc[key, 'travel_direction'],
                                                     results_df.loc[key, 'speed'],
                                                     results_df.loc[key, 'acceleration'],
                                                     results_df.loc[key, 'leader'],
                                                     results_df.loc[key, 'speed_lead']) for index, (key, veh) in enumerate(self.cav_object_dict.items())}
            

            traci.simulationStep()
            self.apply_vehicle_control_FIXS(eco_speed_dic, vehicle_dynamics=True, eco_driving=True)

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
        
        for veh_id, eco_speed in eco_speed_dic.items():
            if eco_speed is not None and veh_id == 'ego':
                # veh_id to uint8Arr
                veh_id_uint8Arr = [ord(c) for c in veh_id]
                veh_id_length = len(veh_id_uint8Arr)
                veh_data = VehData(id=veh_id_uint8Arr, idLength=veh_id_length, speedDesired=eco_speed)
                self.socket_helper.vehicle_data_send_list.append(veh_data)

        self.socket_helper.sendData(sim_state, sim_time, self.socket2simulink)
        self.socket_helper.clear_data()
        # receive data from the client (the actual vehicle data after the vehidle dynamics model)
        self.socket_helper.recv_data(self.socket2simulink)
        ### Simulink data to FIXS Layer
        self.socket_helper.vehicle_data_send_list.extend(self.socket_helper.vehicle_data_receive_list)

        for idx in range(len(self.socket_helper.vehicle_data_send_list)):
            veh_id = ''.join([chr(c) for c in (self.socket_helper.vehicle_data_receive_list[idx].id[:self.socket_helper.vehicle_data_receive_list[idx].idLength])])
            if not vehicle_dynamics:
                self.socket_helper.vehicle_data_send_list[idx].speedDesired = eco_speed_dic[veh_id]

        if len(self.socket_helper.vehicle_data_send_list) == 0:
            veh_data = VehData()
            self.socket_helper.vehicle_data_send_list.append(veh_data)
        self.socket_helper.sendData(sim_state, sim_time, self.socket2FIXS)



    def apply_vehicle_control(self, cav_id_list, eco_speed, smooth=False):
        # to handle the case of a single vehicle
        if type(cav_id_list) == str:
            cav_id_list = [cav_id_list]
            eco_speed = [eco_speed]

        for i, vid in enumerate(cav_id_list):
            if eco_speed[i] is not None:
                if smooth:
                    traci.vehicle.slowDown(vid, eco_speed[i] * 0.44704, 1)
                else:
                    traci.vehicle.setSpeed(vid, eco_speed[i] * 0.44704)

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
    # read sumoSignalConfig_26
    sumoSignalConfig = pd.read_csv('sumoSignalConfig_26.csv', index_col=0)
    sumoSignalConfig['id'] = sumoSignalConfig['id'].astype(str)
    sumoSignalConfig['name'] = sumoSignalConfig['name'].astype(str)
    parser = argparse.ArgumentParser()
    parser.add_argument("-c", "--config", type=str, help="Path to the Configuration file", default='./ecodrivingConfig.yaml')
    traffic_layer_config = parser.parse_args().config
    senv = SumoEnvMultiAgent(sumoSignalConfig, traffic_layer_config=traffic_layer_config)
    mpr = 0.2
    step_length = 1
    # no subscription
    # senv.start(mpr, step_length)
    # with subscription
    # for mpr in []
    senv.start_subscription(mpr, step_length)
    total_time = time.time() - start_time
    print('Total time spent: ', total_time)
