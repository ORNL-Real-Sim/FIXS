# -*- coding: utf-8 -*-
# import pdb  # pdb.set_trace()
from __future__ import print_function
from fileinput import filename
from isort import file
import pandas as pd
import win32com.client as com # COM-Server
import os
import shutil
import subprocess
import socket
import numpy as np
from cav_casestudy.SUMO.speed_control_utils import *
from CommonLib.SocketHelper import SocketHelper
from CommonLib.ConfigHelper import ConfigHelper
from CommonLib.MsgHelper import MsgHelper
from CommonLib.VehDataMsgDefs import VehData
import argparse
def get_start_index(fname, char_to_search, nocc):
    """
    find first line of data in VISSIM output files
    char_to_search: the character to be searched; ";" for LSA files and "$" for FZP files
    nocc: the number of occurance of the detected string before break the search; 1 for LSA files and 2 for FZP files
    :param fname:
    :param char_to_search:
    :param nocc:
    :return:
    """
    lineno = 0
    startidx = 0
    occ = 0
    with open(fname, 'r') as read_obj:
        for line in read_obj:
            if char_to_search in line:
                startidx = lineno
                occ += 1
                if occ == nocc:
                    break
            lineno += 1
    return startidx

def lsa_sub(df, sloc, direction):
    """
    subset LSA to one direction
    :param df:
    :param sloc:
    :param direction:
    :return:
    """
    loc_d = sloc[sloc['Direction'] == direction]
    keys = ['SC','SG']
    i1 = df.set_index(keys).index
    i2 = loc_d.set_index(keys).index
    lsa1=df[i1.isin(i2)]
    lsa1=lsa1.sort_values(by=['SC', 'SG'])
    return lsa1, loc_d


# coasting parameters based on tractive power
A = 0.083698
B = 0.00385
C = 0.000278
M = 1.6443
# reading sample coasting profile
example_coasting_profile = pd.read_csv(r'example_coasting_profile.csv', index_col=0)
def gen_desire_speed_dir_r_(CAV_curr, simsec, lsa, A, B, C, M, example_coasting_profile, static_routes_eb_wb_th):

    # select only the controllable vehicles
    CAV_curr = CAV_curr[~((CAV_curr['Direction'].isnull())
                                      | (CAV_curr['RouteNo'].isnull())
                                      | (~CAV_curr['VehRoutSta'].isin(static_routes_eb_wb_th['VehRoutSta'].values.tolist()))
                                      | (CAV_curr['nextSC'] == 0)
                                      | (CAV_curr['Pos'].isnull()))]

    if len(CAV_curr) > 0:
        # get the signal state of all the intersections at the current simsec
        lsa = lsa[(lsa['SimSec'] <= simsec) & (lsa['status_end'] > simsec)].reset_index(drop=True)

        # join SPaT for nextSC
        CAV_curr_control = CAV_curr.join(lsa[["SimSec", "SC", "SigState", 'Direction', 'status_end', 't1s', 't1e', 't2s', 't2e', 'r1s']].set_index(['SC', 'Direction']), on=['nextSC', 'Direction'])

        try:
            if len(CAV_curr_control) > 0:
                CAV_curr_control['instant_desired_speed'], CAV_curr_control['mode'], CAV_curr_control['a_out'], CAV_curr_control['max_desired_speed'], CAV_curr_control['minimum_desired_speed'] = np.vectorize(
                    gen_desired_spd, excluded=['example_coasting_profile'])(
                    step_length=1,
                    example_coasting_profile=example_coasting_profile,
                    A=A, B=B, C=C, M=M,
                    orginal_desire_spd=CAV_curr_control['OrgDesSpeed'],
                    next_movement='TH',
                    vehicle_current_speed=CAV_curr_control['Speed'],
                    current_acceleration=CAV_curr_control['Acceleration'],
                    simsec=simsec,
                    distance2stopbar=CAV_curr_control['d'],
                    acc_speed_base=CAV_curr_control['Speed_base'],
                    acc_dist=CAV_curr_control['Clear'],
                    num_through_lanes=2,
                    upstream_flow_rate=300,
                    upstream_flow_speed=40,
                    SigState=CAV_curr_control['SigState'],
                    t1s=CAV_curr_control['t1s'],
                    t1e=CAV_curr_control['t1e'],
                    t2s=CAV_curr_control['t2s'],
                    t2e=CAV_curr_control['t2e'],
                    r1s=CAV_curr_control['r1s'])

                CAV_curr_control['instant_desired_speed'] = np.where(CAV_curr_control['instant_desired_speed'].isnull(), CAV_curr_control['DesSpeed'], CAV_curr_control['instant_desired_speed'])

        except:
            print('Test')
            return CAV_curr_control

        return CAV_curr_control
    
def get_two_green_window(lsa, scloc):
    # get the current signal status and the green start time (for speed control with queue)
    lsa = lsa[lsa['SG'].isin([2, 6])].reset_index(drop=True)
    lsa = lsa.join(scloc[['SC', 'SG', 'Direction']].set_index(['SC', 'SG']), on=['SC', 'SG'])
    lsa = lsa.sort_values(by=['SC', 'Direction', 'SimSec']).reset_index(drop=True)

    lsa = lsa[~lsa['SigState'].str.contains('amber')].reset_index(drop=True)
    lsa['signal_duration'] = lsa['SimSec'].shift(-1) - lsa['SimSec']
    lsa = lsa[(lsa['signal_duration'] > 0) & (lsa['signal_duration'].notnull())].reset_index(drop=True)
    lsa['status_end'] = lsa['SimSec'] + lsa['signal_duration']

    # get the next signal status information
    lsa['next_status_end'] = lsa['status_end'].shift(-1)
    lsa['next_signal_duration'] = lsa['next_status_end'] - lsa['status_end']
    lsa = lsa[(lsa['next_signal_duration'] > 0) & (lsa['next_signal_duration'].notnull())].reset_index(drop=True)

    # get third signal status information
    lsa['third_status_end'] = lsa['next_status_end'].shift(-1)
    lsa['third_signal_duration'] = lsa['third_status_end'] - lsa['next_status_end']
    lsa = lsa[(lsa['third_signal_duration'] > 0) & (lsa['third_signal_duration'].notnull())].reset_index(drop=True)

    # get forth signal status information
    lsa['forth_status_end'] = lsa['third_status_end'].shift(-1)
    lsa['forth_signal_duration'] = lsa['forth_status_end'] - lsa['third_status_end']
    lsa = lsa[(lsa['forth_signal_duration'] > 0) & (lsa['forth_signal_duration'].notnull())].reset_index(drop=True)

    # get t1s, t1e, t2s, t2e
    lsa['t1s'] = np.where(lsa['SigState'].str.contains('green'), lsa['SimSec'], lsa['status_end'])
    lsa['t1e'] = np.where(lsa['SigState'].str.contains('green'), lsa['status_end'], lsa['next_status_end'])
    lsa['t2s'] = np.where(lsa['SigState'].str.contains('green'), lsa['next_status_end'], lsa['third_status_end'])
    lsa['t2e'] = np.where(lsa['SigState'].str.contains('green'), lsa['third_status_end'], lsa['forth_status_end'])

    lsa['r1s'] = np.where(lsa['SigState'].str.contains('green'), lsa['status_end'], lsa['SimSec'])

    # only keep important variables
    lsa = lsa[["SimSec", "SC", "SG", "SigState", 'Direction', 'status_end', 't1s', 't1e', 't2s', 't2e', 'r1s']]

    return lsa

class VisSimEnvMultiAgent:
    def __init__(self, traffic_layer_config, vissim_port=1337, traffic_layer_port=430, simulink_port=420, simulation_file_path='', with_ego_veh=True):
        # sumo startup utils
        # remember to enable the Evaluation --> Configuration -->Direct Output --> Signal changes
        self.with_ego_veh = with_ego_veh
        # initialize the socket connections
        config_helper = ConfigHelper()
        config_helper.getConfig(traffic_layer_config)
        msg_helper = MsgHelper()
        msg_helper.set_vehicle_message_field(config_helper.simulation_setup['VehicleMessageField'])
        self.socket_helper = SocketHelper(config_helper=config_helper, msg_helper=msg_helper)
        # IP to connect to the FIXS server
        
        self.vissim_port = vissim_port
        self.simulation_file_path = simulation_file_path

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


    def setup_vissim(self, end_of_simulation=3600, seed=42):
        """
        Set up the simulation environment
        :param Vissim:
        :param End_of_simulation:
        :param seed:
        :param simulation_file_path:
        :return:
        """
        self.end_of_simulation = end_of_simulation
        self.vissim = com.DispatchEx("Vissim.Vissim", f"127.0.0.1:{self.vissim_port}")
        # Load a Vissim Network:
        # remember to enable the Evaluation --> Configuration -->Direct Output --> Signal changes
        Filename = os.path.join(self.simulation_file_path, 'calibrated_turnsSA_behaviorTS_fin_leftban_nooverlap.inpx')
        flag_read_additionally = False  # you can read network(elements) additionally, in this case set "flag_read_additionally" to true
        self.vissim.LoadNet(Filename, flag_read_additionally)

        ## Load a Layout:
        Filename = os.path.join(self.simulation_file_path, 'calibrated_turnsSA_behaviorTS_fin_leftban_nooverlap.layx')
        self.vissim.LoadLayout(Filename)

        # End_of_simulation = 3600   # run long enough to record any signal changes within a cycle after 1200 s
        self.vissim.Simulation.SetAttValue('SimPeriod', self.end_of_simulation)
        # Set up simulation to use all cores
        self.vissim.Simulation.SetAttValue('UseAllCores', True)
        # Set a random seed
        self.vissim.Simulation.SetAttValue('RandSeed', seed)
        # Activate QuickMode:
        self.vissim.Graphics.CurrentNetworkWindow.SetAttValue("QuickMode",1) # use 0 if want to see vehicles in simulation
        #Vissim.SuspendUpdateGUI();

        self.vehComps = self.vissim.Net.VehicleCompositions.GetAll()

    def runGreenWave(self, cavpr, scloc, linkinfo, file_name, static_routes_eb_wb_th, vehicle_dynamics=True, eco_driving=True):
        """
    
        :param timeofday:
        :param cycLen:
        :param cavpr: cav penetration rate
        :param seed:
        :param scloc: signal controller location
        :param linkinfo:
        :return:
        """
        carpr = 1 - cavpr
    
        ## load and parse VISSIM signal changes (LSA file)
        flsa = self.simulation_file_path + '/{}_{}.lsa'.format(file_name, '001')
        lsa_start = get_start_index(flsa, ';', 1)
        lsa = pd.read_csv(flsa, skiprows=lsa_start, header=None, usecols=range(0, 5), sep=';')
        lsa.columns = ["SimSec", "CycleTime", "SC", "SG", "SigState"]  # ,"RuntimeState","SC_type","SG_caused"]
        # get the next two green 
        lsa = get_two_green_window(lsa, scloc)
        ## subset lsa to eastbound/westbound direction
        # elsa, eloc = lsa_sub(lsa, scloc, 'EB')
        # e_greens = elsa[elsa['SigState'].str.contains(pat='green')]
        # e_reds = elsa[elsa['SigState'].str.contains(pat='red')]
        #
        # wlsa, wloc = lsa_sub(lsa, scloc, 'WB')
        # w_greens = wlsa[wlsa['SigState'].str.contains(pat='green')]
        # w_reds = wlsa[wlsa['SigState'].str.contains(pat='red')]
        for vc in range(len(self.vehComps)):
            # get the DesSpeedDistributions in this approach
            vehComps_tem = self.vehComps[vc].VehCompRelFlows.GetMultipleAttributes(('VehType', 'DesSpeedDistr', 'RelFlow'))
            vehComps_tem = pd.DataFrame(list(vehComps_tem), columns=['VehType', 'DesSpeedDistr', 'RelFlow'])
            DesSpeedDistributions = vehComps_tem.DesSpeedDistr.unique()[0]
            # add AddVehicleCompositionRelativeFlow for cav vehicle type
            self.vehComps[vc].VehCompRelFlows.AddVehicleCompositionRelativeFlow(Vissim.Net.VehicleTypes.ItemByKey(10001),
                                                                           Vissim.Net.DesSpeedDistributions.ItemByKey(DesSpeedDistributions))
            relflows = self.vehComps[vc].VehCompRelFlows.GetAll()
            # if len(relflows) == 3:
            relflows[0].SetAttValue('VehType', 100) # Changing the vehicle type to ensure setting relative flow for the right type
            relflows[0].SetAttValue('RelFlow', carpr) # Changing the relative flow of Car Relative Flow.
            relflows[1].SetAttValue('VehType', 10001) # Changing the vehicle type to ensure setting relative flow for the right type
            relflows[1].SetAttValue('RelFlow', cavpr) # Changing the relative flow of the CAV Relative Flow.
        # create an empty cav_all to store all the CAV information
        # CAV_all = pd.DataFrame(columns=['No', 'VehType', 'Speed', 'Pos', 'Lane', 'DesSpeed', 'RouteNo', 'OrgDesSpeed'])
        # Run first 300 seconds as warm up
        self.vissim.Simulation.SetAttValue('SimBreakAt', 300)
        # Set maximum speed:
        self.vissim.Simulation.SetAttValue('UseMaxSimSpeed', True)
        self.vissim.Simulation.RunContinuous()
        simsec = self.vissim.Simulation.SimulationSecond
        if self.with_ego_veh:
            # Add an Ego Vehicle at a Specific Position
            # Parameters: Vehicle Type ID, Link Number, Lane Number, Position, Desired Speed
            vehicle_type_id = 10002  # Use an existing vehicle type ID
            link_id = 1  # Replace with your target link ID
            lane = 1  # Lane index (1-based)
            position = 10  # Distance from start of link (in feet)
            desired_speed = 50  # Speed in m/s
            
            ego_vehicle = self.vissim.Net.Vehicles.AddVehicleAtLinkPosition(vehicle_type_id, link_id, lane, position, desired_speed)
            # ego_vehicle.SetAttValue("Route", infinite_route.AttValue("No"))
            print(f"Ego vehicle added with ID: {ego_vehicle.AttValue('No')}")
        while simsec <= self.end_of_simulation - 300:
            print(simsec)
            # start_control_time = time.time()
            # get vehicle information
            v = self.vissim.Net.Vehicles.GetMultipleAttributes(('No', 'VehType', 'Speed', 'Pos', 'Lane', 'DesSpeed',
                                                           'RouteNo', 'RoutDecNo', 'VehRoutSta', 'DesSpeedFrac',
                                                           'Acceleration', 'FollowDistNet', 'Clear', 'InteractState',
                                                           'InteractTargNo', 'InteractTargType', 'SpeedDiff'))
            vehs = pd.DataFrame(list(v), columns=['No', 'VehType', 'Speed', 'Pos', 'Lane', 'DesSpeed', 'RouteNo', 'RoutDecNo',
                                         'VehRoutSta', 'DesSpeedFrac', 'Acceleration', 'FollowDistNet', 'Clear', 'InteractState',
                                                           'InteractTargNo', 'InteractTargType', 'SpeedDiff'])
            # Check the InteractTargType, make the value of Clear only meaningful when “InteractTargType” == “VEHICLE”
            vehs['Clear'] = np.where(vehs['InteractTargType'] == 'VEHICLE', vehs['Clear'], 10000)
            vehs['Acceleration'] = vehs['Acceleration'] * 0.3048  # convert the unit from feet/s2 to meter/s2.
            vehs['Speed_base'] = vehs['Speed'] - vehs['SpeedDiff']
            # veh_desspeeds = Vissim.Net.Vehicles.GetMultiAttValues('DesSpeed')
            # get initial CAVs information and set up CAV_all to store all CAVS info for cross reference
            CAV_curr = vehs[vehs.VehType.isin(['10001', '10002'])].copy()
            # get the current link information
            CAV_curr['CurrentLink'] = CAV_curr['Lane'].str.split('-').str[0].astype(int)
            # join the linkinfo based on link id
            CAV_curr = CAV_curr.join(linkinfo[['Link', 'Direction', 'nextSC', 'Length2D', 'cumLength2D', 'cumLength2D_max']].set_index('Link'), on='CurrentLink')
            CAV_curr['d'] = CAV_curr['cumLength2D_max'] - CAV_curr['cumLength2D'] + CAV_curr['Length2D'] - CAV_curr['Pos']
            # get original desired speed based on link information and DesSpeedFrac (given we know all the DesSpeedDistr on the major road are No. 64)
            CAV_curr['OrgDesSpeed'] = 50 + CAV_curr['DesSpeedFrac'] * (54 - 50)
            # For those vehicles that are not on the major road, which we are not controlling. Set the OrgDesSpeed equals to DesSpeed
            CAV_curr['OrgDesSpeed'] = np.where(CAV_curr['DesSpeedFrac'].isna(), CAV_curr['DesSpeed'], CAV_curr['OrgDesSpeed'])
            # update the desire speed for controllable vehicles on EB and WB
            CAV_curr_control_eb = gen_desire_speed_dir_r_(CAV_curr, simsec, lsa, A, B, C, M, example_coasting_profile, static_routes_eb_wb_th)
            
            # vihicles controllable by Vissim (exclude ego vehicle type)
            CAV_VISSIM = CAV_curr[CAV_curr['VehType'] != 10002]
            CAV_FIXS = CAV_curr[CAV_curr['VehType'] == 10002]
            ################################################################################################
            # map to all the cavs
            ################################################################################################
            try:
                if CAV_curr_control_eb is not None:
                    # set the controlled CAV desired speed
                    CAV_curr.loc[CAV_curr['No'].isin(CAV_curr_control_eb['No'].values), 'DesSpeed'] = CAV_curr['No'].map(CAV_curr_control_eb.set_index('No')['instant_desired_speed'])
                    # reset the uncontroller CAV's desired speed as the original desired speed
                    CAV_curr.loc[~CAV_curr['No'].isin(CAV_curr_control_eb['No'].values), 'DesSpeed'] = CAV_curr.loc[~CAV_curr['No'].isin(CAV_curr_control_eb['No'].values), 'OrgDesSpeed']
                    # update the desire speed of all CAVs to overall vehicles
                    vehs.loc[vehs['No'].isin(CAV_curr['No'].values), 'DesSpeed'] = vehs['No'].map(CAV_curr.set_index('No')['DesSpeed'])
                else:
                    CAV_curr['DesSpeed'] = CAV_curr['OrgDesSpeed']
                # add numbering to prepare set speeds data
                vehs['ID'] = vehs.index + 1
                setSpeeds = vehs[["ID", "DesSpeed"]]
                # set desire speeds
                self.vissim.Net.Vehicles.SetMultiAttValues(('DesSpeed'), tuple(list(setSpeeds.itertuples(index=False, name=None))))
            except:
                print('ERROR')
            # print('Time cost for speed control: {} seconds'.format(time.time() - start_control_time))
            # Run n second with the setting
            interval = 1
            # interval = 0.5
            # interval = time.time() - start_control_time + 0.1
            # print('Interval: {} seconds'.format(interval))
            self.vissim.Simulation.SetAttValue('SimBreakAt', simsec + interval)
            # Set maximum speed:
            self.vissim.Simulation.SetAttValue('UseMaxSimSpeed', True)
            self.vissim.Simulation.RunContinuous()
            # Vissim.Simulation.RunSingleStep()
            simsec = self.vissim.Simulation.SimulationSecond
            # print(simsec)
    
        Vissim = None







def start_vissim(port=5000, vissim_path=r"C:\Program Files\PTV Vision\PTV Vissim 2022\Exe\Vissim220.exe"):
    """
    Launches PTV Vissim 2022 in Automation mode on the specified port.
    
    Parameters:
    - port (int): TCP port for the COM Automation server (default: 5000)
    - vissim_path (str): Full path to Vissim220.exe
    """
    try:
        subprocess.Popen([vissim_path, "-Automation", f"-Port={port}"], shell=False)
        print(f"Vissim launched with Automation server on port {port}.")
    except FileNotFoundError:
        print(f"Error: Vissim executable not found at {vissim_path}")
    except Exception as e:
        print(f"Failed to launch Vissim: {e}")






if __name__ == '__main__':

    start_time = time.time()
    
    
    
    # parser = argparse.ArgumentParser()
    # parser.add_argument("-c", "--config", type=str, help="Path to the Configuration file", default=None)
    # parser.add_argument("--sumoPort", type=str, help="Specify port of sumo", default=1333)
    # parser.add_argument("--trafficlayerPort", type=str, help="Specify port of traffic layer", default=433)
    # parser.add_argument("--simulinkPort", type=str, help="Specify port of simulink", default=423)
    # parser.add_argument("--ecoDriving", action="store_true", help="Use the eco driving controller", default=False)
    # parser.add_argument("--vehicleDynamics", action="store_true", help="use the vehicle dynamis", default=False)
    # parser.add_argument("--penetrationRate", type=float, help="the penetration rate of cav", default=1.0)
    # parser.add_argument("--pathToNet", type=str, help="the path to the net file", default=None)
    
    # args = parser.parse_args()
    # traffic_layer_config = args.config
    # sumo_port = args.sumoPort
    # traffic_layer_port = args.trafficlayerPort
    # simulink_port = args.simulinkPort
    # penetration_rate = float(args.penetrationRate)
    # path_to_net = args.pathToNet
    # vehicle_dynamics = args.vehicleDynamics
    # eco_driving = args.ecoDriving


    # print('path_to_net: ', path_to_net)
    # print(f'running with penetration rate: {penetration_rate}, vehicle dynamics: {vehicle_dynamics}, eco driving: {eco_driving}')
    # senv = SumoEnvMultiAgent(sumoSignalConfig, traffic_layer_config=traffic_layer_config, sumo_port=sumo_port, traffic_layer_port=traffic_layer_port, simulink_port=simulink_port, path_to_net=path_to_net)
    # step_length = 1
    # senv.start_subscription(penetration_rate, step_length, vehicle_dynamics=vehicle_dynamics, eco_driving=eco_driving)
    # total_time = time.time() - start_time
    # print('Total time spent: ', total_time)
    
    # scloc = pd.read_csv("sc_loc.csv")
    # # load shallowford corridor geometry
    # linkinfo = pd.read_csv("linkInfo.csv")
    # linkinfo = linkinfo[(linkinfo['Direction'] == 'EB') | (linkinfo['Direction'] == 'WB')]
    # linkinfo['cumLength2D_max'] = linkinfo.groupby(by=['nextSC', 'Direction'])['cumLength2D'].transform(max)

    # # read static routes
    # static_routes_eb_wb_th = pd.read_csv("static_routes_eb_wb.csv", index_col=0)
    # # absolute path of the VISSIM Network folder
    # absolute_folder_path = os.path.join(os.getcwd(), r'Experiments\banMinorLeftTurnFixedTimingV2_WithEgo')


    

    # # this is variable is to disable and enable the added ego vehicle
    # with_ego_veh = True
    # runGreenWave(cavpr=0.2, seed=seed, scloc=scloc, linkinfo=linkinfo, absolute_folder_path=absolute_folder_path, filename='calibrated_turnsSA_behaviorTS_fin_leftban_nooverlap', static_routes_eb_wb_th=static_routes_eb_wb_th)
    start_vissim(port=1337)