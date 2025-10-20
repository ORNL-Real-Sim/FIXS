import numpy as np
import traci
import os
import sys
import sumolib
from traffic_light_utils import TrafficLight
from vehicle_eco_pilot import EcoVehicle
from speed_control_utils import *
from utils1 import *
from sumolib import checkBinary
import xml.etree.ElementTree as ET
import traci.constants as tc
import inspect
import shutil


if 'SUMO_HOME' in os.environ:
    sys.path.append(os.path.join(os.environ['SUMO_HOME'], 'tools'))


class SumoEnvMultiAgent:
    def __init__(self, sumoSignalConfig, sumoFolder, working_directory='MPR'):
        # sumo startup utils
        self.sumoFolder = sumoFolder
        self.sumoBinary = checkBinary('sumo-gui')  # or 'sumo-gui' for graphical interface
        self.sumo_config_file = os.path.join(os.getcwd(), sumoFolder, 'chattCavMpr.sumocfg')
        self.sumo_net_file = os.path.join(os.getcwd(), sumoFolder, 'chatt.net.xml')
        self.sumoCmd = [self.sumoBinary, "-c", self.sumo_config_file]
        self.graph = sumolib.net.readNet(self.sumo_net_file, withInternal=True)  # internal edges are edges inside intersections or connections
        self.vertex = self.graph.getNodes()
        self.edge = self.graph.getEdges(withInternal=True)
        self.edges = ['-2801', '-280', '-307', '-327', '-3271', '-281', '-315', '-3151', '-321', '-300', '-2851', '-285', '-290', '-298', '-295', '-312',
                      '-293', '-297', '-288', '-2881', '-286', '-302', '-3221', '-322', '-313', '-284', '-2841', '-328', '-304', '-2801']
        self.wb = ['-2801', '-280', '-307', '-327', '-3271', '-281', '-315', '-3151', '-321', '-300', '-2851', '-285', '-290', '-298', '-295']
        self.eb = ['-312', '-293', '-297', '-288', '-2881', '-286', '-302', '-3221', '-322', '-313', '-284', '-2841', '-328', '-304']

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

        # https://sumo.dlr.de/pydoc/traci.constants.html, , tc.VAR_NEXT_LINKS is not working well, VAR_ROUTE = 87, VAR_ROUTE_INDEX = 105, VAR_SPEED_WITHOUT_TRACI = 177
        self.subscription_vars = [tc.VAR_SPEED, tc.VAR_POSITION, tc.VAR_TYPE, tc.VAR_ROAD_ID, tc.VAR_ACCELERATION,
                                  tc.VAR_NEXT_TLS, tc.VAR_EDGES, tc.VAR_ROUTE_INDEX, tc.VAR_SPEED_WITHOUT_TRACI,
                                  tc.VAR_ALLOWED_SPEED]
        #tc.VAR_ROUTE is not working for subscription, using VAR_EDGES = 84 instead.
        self.subscribed_vehicles = []

        self.speed_min = 0
        self.speed_max = 21
        self.max_acc = 4.0
        self.prev_acc = 0.01

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
    # def config_change(self, root, new_dir):
    #
    #     # Using Iterator to quickly search sub trees
    #     # for net_file in root.iter("net-file"):
    #     #     # Alters the output file to the new directory for the simulation statistics.
    #     #     net_file.set("value", os.path.join(self.sumoFolder, 'chatt.net.xml'))
    #
    #     # Using Iterator to quickly search sub trees
    #     # for route_file in root.iter("route-files"):
    #     #     # Alters the output file to the new directory for the simulation statistics.
    #     #     route_file.set("value", new_dir + r"\chattCavMpr.rou.xml")
    #
    #     # Using Iterator to quickly search sub trees
    #     # for add_file in root.iter("additional-files"):
    #     #     # Alters the output file to the new directory for the simulation statistics.
    #     #     add_file.set("value", "updated_signal.xml" + ', ' +  os.path.join(self.sumoFolder, "Edge.add.xml"))
    #
    #     # # Using Iterator to quickly search sub trees
    #     # for edge_out in root.iter("edgedata-output"):
    #     #     # Alters the output file to the new directory for the simulation statistics.
    #     #     edge_out.set("value", new_dir + r"\EdgeData.xml")
    #     #
    #     # # Using Iterator to quickly search sub trees
    #     # for fcd_out in root.iter("fcd-output"):
    #     #     # Alters the output file to the new directory for the tripinfo results.
    #     #     fcd_out.set("value", new_dir + r"\fcd.xml")
    #     #
    #     # # Using Iterator to quickly search sub trees
    #     # for signal_out in root.iter("timedEvent"):
    #     #     # Alters the output file to the new directory for the tripinfo results.
    #     #     signal_out.set("dest", new_dir + r"\signal_result.xml")
    #
    #     print("New edge_out & fcd_out & signal_out outputs: " + new_dir )
    #     # Returns the edited root.
    #     return root

    def change_cav_mpr(self, mpr, output_dir):

        xmlTree = ET.parse(os.path.join(self.sumoFolder, "chattCavMpr.rou.xml"))
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

    def change_config_directory(self, mpr, seed, step_length, subscribe):
        output_dir = os.path.join(self.sumoFolder, self.working_directory, '{}%_Seed{}_{}_{}Hz'.format(int(mpr*100), seed, subscribe, int(1/step_length)))

        # Making Results Directory
        try:
            os.mkdir(output_dir)
            print()
            print("Created Directory:" + output_dir)
        except OSError as error:
            print("Directory already exists, will overwrite  upon simulation.")

        # # NEMA signal.xml
        # xmlTree_signal = ET.parse(os.path.join(self.sumoFolder, "updated_signal.xml"))
        # xmlRoot_signal = xmlTree_signal.getroot()
        # # xmlRoot_signal = self.config_change(xmlRoot_signal, output_dir)
        # xmlTree_signal.write(output_dir + r"\updated_signal.xml")

        # Copy file to the destination directory
        shutil.copy(os.path.join(self.sumoFolder, "updated_signal.xml"), output_dir)
        shutil.copy(os.path.join(self.sumoFolder, "chatt.net.xml"), output_dir)
        shutil.copy(os.path.join(self.sumoFolder, "Edge.add.xml"), output_dir)

        # MPR
        self.change_cav_mpr(mpr, output_dir)

        # sumo config
        xmlTree = ET.parse(os.path.join(self.sumoFolder, "chattCavMpr.sumocfg"))
        xmlRoot = xmlTree.getroot()
        # xmlRoot = self.config_change(xmlRoot, output_dir)

        # Using Iterator to quickly search sub trees
        for step_len in xmlRoot.iter("step-length"):
            # Alters the output file to the new directory for the simulation statistics.
            step_len.set("value", str(step_length))

        self.sumo_config_file = output_dir + r"\chattCavMpr.sumocfg"
        xmlTree.write(self.sumo_config_file)



    def subscribe_departed_veh(self):

        # Get newly departed vehicles, tuple
        departed_vehicles = traci.simulation.getDepartedIDList()
        # print(departed_vehicles)
        # all_vehicles = traci.vehicle.getIDList()
        # all_vehicles_unsubscribed = tuple(set(all_vehicles) - set(self.subscribed_vehicles))
        # all_vehicles_unsubscribed = tuple(set(all_vehicles) - set(list(self.cav_object_dict.keys())))

        # Subscribe to all departed vehicles
        for veh_id in departed_vehicles:
            try:
                if veh_id not in self.subscribed_vehicles:
                    traci.vehicle.subscribe(veh_id, self.subscription_vars)
                    traci.vehicle.subscribeLeader(veh_id, dist=200.0)
                    self.subscribed_vehicles.append(veh_id)
                    # print(f"Subscribed to {veh_id}")

                    veh_type = traci.vehicle.getTypeID(veh_id)
                    # print(veh_id, ' IS ', veh_type)
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



    def start_subscription(self, mpr, seed, step_length, with_ego_veh):
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
        self.change_config_directory(mpr, seed, step_length, 'Subscription')
        self.sumoCmd = [self.sumoBinary, "--seed", str(seed), "-c", self.sumo_config_file]

        traci.start(self.sumoCmd)
        traci.route.add(routeID='route1', edges=self.edges)

        self.get_phases(self.sumoSignalConfig)

        sim_time = traci.simulation.getTime()

        start_time_1 = time.time()
        while sim_time <= 28985:
            sim_time = traci.simulation.getTime()
            # Phase trackers
            self.phase_tracker()
            self.subscribe_departed_veh()
            traci.simulationStep()

        print('Total time spent for the first 28985: ', time.time() - start_time_1)
        print('test')

        if with_ego_veh:
            self.reset()

        while sim_time <= 29500:
            sim_time = traci.simulation.getTime()
            print(sim_time)
            # Phase trackers
            self.phase_tracker()
            self.subscribe_departed_veh()

            # get subscription results
            results = traci.vehicle.getAllSubscriptionResults()
            # convert results to a dataframe
            results_df = pd.DataFrame.from_dict(results, orient="index")
            # https://sumo.dlr.de/docs/TraCI/Vehicle_Value_Retrieval.html, https://www.rapidtables.com/convert/number/hex-to-decimal.html?x=70
            results_df = results_df.rename(columns={64: 'speed', 66: 'position', 79: 'veh_type', 80: 'road_id',
                                                    114: 'acceleration', 112: 'next_tls', 104: 'leader',
                                                    84: 'edges', 105: 'route_index', 177: 'speed_wo_traci',
                                                    183: 'allowed_speed'})

            results_df[['travel_direction', 'control']] = results_df.apply(lambda row: self.get_travel_direction(row.name, row['veh_type'], row['road_id'], row['route_index'], row['edges'], row['next_tls']), axis=1, result_type='expand')

            # get lead vehicle's speed
            results_df['speed'] = results_df['speed'] * 2.23694
            results_df['leader_id'] = np.where(results_df['leader'].notnull(), results_df['leader'].str[0], 'None')
            results_df = results_df.join(results_df['speed'], on='leader_id', rsuffix='_lead')
            results_df['lead_dist'] = np.where(results_df['leader'].notnull(), results_df['leader'].str[1] * 3.28084, 500 * 3.28084)
            results_df['speed_lead'] = np.where(results_df['leader'].notnull(), results_df['speed_lead'], self.speed_max * 2.23694)

            results_df['dist2Stop'] = np.where(results_df['next_tls'].notnull(), results_df['next_tls'].str[0].str[2] * 3.28084, 10000.0)
            results_df['orginal_desire_spd'] = results_df['allowed_speed'] * 2.23694
            results_df[['t1s', 't1e', 't2s', 't2e', 'r1s', 'curr_status']] = results_df.apply(lambda row: self.get_spat(self.phase_tracking_dict,
                                                                                                                        self.spat_statuses,
                                                                                                                        row['next_tls'],
                                                                                                                        row['travel_direction'],
                                                                                                                        row['control']), axis=1, result_type='expand')
            # determine whether we will control the CAV
            # results_df['control'] = results_df.apply(lambda row: self.get_cav_control_noncontrol(row['edges'], row['next_tls'], row['travel_direction']), axis=1)

            # results_df = results_df[(results_df['travel_direction'].isin(['EB', 'WB'])) & (results_df['next_edge'].isin(self.wb + self.eb))]
            list_cav_back_to_sumo = results_df.loc[(results_df['veh_type'] == 'CAV') & (results_df['control'] == 'False')].index.tolist()
            list_cav_control = results_df.loc[results_df['control'] == 'True'].index.tolist()

            # results_df = results_df[results_df['control'] == 'True']

            # set indefinite route for the 'ego' vehicle
            if 'ego' in results_df.index.tolist():
                if results_df.loc['ego', 'road_id'] == "-2801":
                    traci.vehicle.setRoute('ego', self.edges)

            # give back the non-controlled vehicle to SUMO
            # if len(list_cav_back_to_sumo) > 0:
            #     [veh.gain_back_sumo_control() for index, (key, veh) in enumerate(self.cav_object_dict.items()) if key in list_cav_back_to_sumo]

            # call the function and control the speed
            if len(self.cav_object_dict) > 0:
                [veh.get_eco_speed_subscribe(self.phase_tracking_dict,
                                                         self.spat_statuses,
                                                         results_df.loc[key, 'next_tls'],
                                                         results_df.loc[key, 'travel_direction'],
                                                         results_df.loc[key, 'orginal_desire_spd'],
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
                                                         results_df.loc[key, 'curr_status'])
                 for index, (key, veh) in enumerate(self.cav_object_dict.items()) if key in (list_cav_back_to_sumo + list_cav_control)]

            # only working on the CAVs that are currently in the network
            self.cav_object_dict = {key: value for key, value in self.cav_object_dict.items() if key in (list_cav_back_to_sumo + list_cav_control)}

            traci.simulationStep()

        self.close()

    # def apply_vehicle_control(self, cav_id_list, eco_speed, smooth=False):
    #     # to handle the case of a single vehicle
    #     if type(cav_id_list) == str:
    #         cav_id_list = [cav_id_list]
    #         eco_speed = [eco_speed]
    #
    #     for i, vid in enumerate(cav_id_list):
    #         if eco_speed[i] is not None:
    #             if smooth:
    #                 traci.vehicle.slowDown(vid, eco_speed[i] * 0.44704, 1)
    #             else:
    #                 traci.vehicle.setSpeed(vid, eco_speed[i] * 0.44704)

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
    sumoFolder = 'Shallowford_after_calibration_banleftturn_AdjustedFixedTime_V3'
    # sumoFolder = 'Shallowford_before_calibration_V3'

    # read sumoSignalConfig_26
    sumoSignalConfig = pd.read_csv(os.path.join(sumoFolder, 'sumoSignalConfig_26.csv'), index_col=0)
    sumoSignalConfig['id'] = sumoSignalConfig['id'].astype(str)
    sumoSignalConfig['name'] = sumoSignalConfig['name'].astype(str)

    # sumoFolder = 'Shallowford_before_calibration_AdjustedFixedTime'

    # mpr = 0.1
    # step_length = 1
    step_length = 0.1

    # this is variable is to disable and enable the added ego vehicle
    with_ego_veh = True
    # no subscription
    # senv.start(mpr, step_length)
    # with subscription
    # for mpr in [0.1, 0.2, 0.5, 1.0]:
    ego_depart_time_replace_vehicle = []
    for mpr in [0.0]:
        
        start_time = time.time()
        for seed in range(100, 101):
            senv = SumoEnvMultiAgent(sumoSignalConfig, sumoFolder)

            senv.start_subscription(mpr, seed, step_length, with_ego_veh)
            total_time = time.time() - start_time
            print('Seed ({}) Total time spent: '.format(seed), total_time)
