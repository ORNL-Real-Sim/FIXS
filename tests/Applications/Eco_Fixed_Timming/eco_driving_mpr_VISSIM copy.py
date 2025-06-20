# -*- coding: utf-8 -*-
# import pdb  # pdb.set_trace()
from __future__ import print_function
from fileinput import filename
from tabnanny import verbose
#from cycler import V
import pandas as pd
#from regex import F
import win32com.client as com # COM-Server
import os
import socket
import numpy as np
from cav_casestudy.SUMO.speed_control_utils import *
from CommonLib.SocketHelper import SocketHelper
from CommonLib.ConfigHelper import ConfigHelper
from CommonLib.MsgHelper import MsgHelper
from CommonLib.VehDataMsgDefs import VehData
import argparse

ENABLE_SOCKET = True
WARMUP_SECONDS = 20
INIT_SPEED = 22.35
LOOKAHEAD_STEP = 0.5 # look ahead speed time step

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
    print(fname)
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




def gen_desire_speed_dir_r_(CAV_curr, simsec, step_length, lsa, A, B, C, M, example_coasting_profile, static_routes_eb_wb_th):

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
                    step_length=step_length,
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

class VissimEnvMultiAgent:
    def __init__(self,
                 driver_model_path, 
                 vissim_port=1337,
                 traffic_layer_ip='127.0.0.1',
                 traffic_layer_port=430, 
                 simulink_ip='127.0.0.1',
                 simulink_port=420, 
                 simulation_file_path='', 
                 with_ego_veh=True,
                 with_ext_driver=True,
                 traffic_layer_config_path='',
                 with_vehicle_dynamics=False,
                 use_simulink_for_energy_evaluation=False,
                 eco_driving=False,
                 step_length=0.1,
                 verbose=False):
        # sumo startup utils
        # remember to enable the Evaluation --> Configuration -->Direct Output --> Signal changes
        self.with_ego_veh = with_ego_veh
        self.with_ext_driver = with_ext_driver
        self.driver_model_path = driver_model_path
        self.traffic_layer_config_path = traffic_layer_config_path
        self.use_simulink_for_energy_evaluation = use_simulink_for_energy_evaluation
        # initialize the socket connections
        config_helper = ConfigHelper()
        config_helper.getConfig(self.traffic_layer_config_path)
        msg_helper = MsgHelper()
        msg_helper.set_vehicle_message_field(config_helper.simulation_setup['VehicleMessageField'])
        self.use_accel = msg_helper.vehicle_msg_field_valid['accelerationDesired']
        assert msg_helper.vehicle_msg_field_valid['accelerationDesired'] or msg_helper.vehicle_msg_field_valid['speedDesired'] , "Must have either accelerationDesired or speedDesired in the VehicleMessageField"
        self.socket_helper = SocketHelper(config_helper=config_helper, msg_helper=msg_helper)
        # IP to connect to the FIXS server
        
        self.vissim_port = vissim_port
        self.simulation_file_path = simulation_file_path
        # IP to connect to the traffic layer
        self.traffic_layer_ip = traffic_layer_ip
        self.traffic_layer_port = traffic_layer_port
        # IP to connect to the Simulink server
        self.simulink_ip = simulink_ip
        self.simulink_port = simulink_port

        self.with_vehicle_dynamics = with_vehicle_dynamics
        self.eco_driving = eco_driving
        self.step_length = step_length
        
        # whether print our the controller information
        self.verbose = verbose
        self.socket2FIXS = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket2FIXS.connect((self.traffic_layer_ip, int(self.traffic_layer_port)))
        print('Connected to FIXS server')

        if self.with_vehicle_dynamics or self.use_simulink_for_energy_evaluation:
            self.socket2simulink = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            print('Waiting for Simulink client to connect...')      
            # bind the socket to the port and listen for incoming connections       
            print('Binding to port: ', self.simulink_port)      
            self.socket2simulink.bind((self.simulink_ip, int(self.simulink_port)))        
            self.socket2simulink.listen(1)      
            # if a connection is established, accept it     
            self.socket2simulink, addr = self.socket2simulink.accept()      
            print('Connected by Simulink client')
            
    def _load_scloc(self, sloc_file_path='sc_loc.csv'):
        scloc = pd.read_csv(sloc_file_path)
        self.scloc = scloc

    def _load_lsa(self, lsa_file_name='calibrated_turnsSA_behaviorTS_fin_leftban_nooverlap'):
        if not os.path.exists(self.simulation_file_path):
            raise FileNotFoundError(f"Simulation file path {self.simulation_file_path} does not exist.")
        assert self.scloc is not None, "scloc is not loaded"

        ## load and parse VISSIM signal changes (LSA file)
        ## [ ] Note: We need to run the simulation once to get the LSA file
        ## In here, we change the name of reference to the LSA file to vanilla
        flsa = self.simulation_file_path + '/{}_{}.lsa'.format(lsa_file_name, 'vanilla')
        lsa_start = get_start_index(flsa, ';', 1)
        lsa = pd.read_csv(flsa, skiprows=lsa_start, header=None, usecols=range(0, 5), sep=';')
        lsa.columns = ["SimSec", "CycleTime", "SC", "SG", "SigState"]  # ,"RuntimeState","SC_type","SG_caused"]
        # get the next two green 
        lsa = get_two_green_window(lsa, self.scloc)
        self.lsa = lsa  
    
    def _load_linkinfo(self, linkinfo_file_path='linkInfo.csv'):
        # load shallowford corridor geometry
        linkinfo = pd.read_csv(linkinfo_file_path)
        linkinfo = linkinfo[(linkinfo['Direction'] == 'EB') | (linkinfo['Direction'] == 'WB')]
        linkinfo['cumLength2D_max'] = linkinfo.groupby(by=['nextSC', 'Direction'])['cumLength2D'].transform(max)
        self.linkinfo = linkinfo
    
    def _load_static_routes(self, static_routes_file_path='static_routes_eb_wb.csv'):
        # read static routes
        static_routes_eb_wb_th = pd.read_csv(static_routes_file_path, index_col=0)
        self.static_routes_eb_wb_th = static_routes_eb_wb_th

    def _load_example_coasting_profile(self, example_coasting_profile_file_path='example_coasting_profile.csv'):
        example_coasting_profile = pd.read_csv(example_coasting_profile_file_path, index_col=0)
        self.example_coasting_profile = example_coasting_profile
        
    def init_simulation_environment(self, cavpr=0.1, end_of_simulation=3600, seed=42):

        self._load_scloc()
        self._load_lsa()
        self._load_linkinfo()
        self._load_static_routes()
        self._load_example_coasting_profile()
        self.end_of_simulation = end_of_simulation
        self.vissim = com.Dispatch("Vissim.Vissim-64.2200") # Vissim
        # self.vissim = com.DispatchEx("Vissim.Vissim")
        # Load a Vissim Network:
        # remember to enable the Evaluation --> Configuration -->Direct Output --> Signal changes
        Filename = os.path.join(self.simulation_file_path, 'calibrated_turnsSA_behaviorTS_fin_leftban_nooverlap.inpx')
        flag_read_additionally = False  # you can read network(elements) additionally, in this case set "flag_read_additionally" to true
        
        self.vissim.LoadNet(Filename, flag_read_additionally)
        
        ## Load a Layout:
        Filename = os.path.join(self.simulation_file_path, 'calibrated_turnsSA_behaviorTS_fin_leftban_nooverlap.layx')
        self.vissim.LoadLayout(Filename)
        
        if self.with_ego_veh and self.with_ext_driver:
            self.init_ego_veh()
        # End_of_simulation = 3600   # run long enough to record any signal changes within a cycle after 1200 s
        self.vissim.Simulation.SetAttValue('SimPeriod', self.end_of_simulation)
        # Set up simulation to use all cores
        # self.vissim.Simulation.SetAttValue('UseAllCores', True)
        self.vissim.Simulation.SetAttValue('NumCores', 1)
        # Set a random seed
        self.vissim.Simulation.SetAttValue('RandSeed', seed)
        # Activate QuickMode:
        self.vissim.Graphics.CurrentNetworkWindow.SetAttValue("QuickMode",1) # use 0 if want to see vehicles in simulation

        carpr = 1 - cavpr
        # ! The simulation resolution should be consistent with the signal control resolution
        # self.vissim.Simulation.SetAttValue('SimRes', 1)
        #Vissim.SuspendUpdateGUI();

        print('Configure VISSIM simulation environment')
        self.vehComps = self.vissim.Net.VehicleCompositions.GetAll()

        for vc in range(len(self.vehComps)):
            # get the DesSpeedDistributions in this approach
            vehComps_tem = self.vehComps[vc].VehCompRelFlows.GetMultipleAttributes(('VehType', 'DesSpeedDistr', 'RelFlow'))
            vehComps_tem = pd.DataFrame(list(vehComps_tem), columns=['VehType', 'DesSpeedDistr', 'RelFlow'])
            DesSpeedDistributions = vehComps_tem.DesSpeedDistr.unique()[0]
            # add AddVehicleCompositionRelativeFlow for cav vehicle type
            self.vehComps[vc].VehCompRelFlows.AddVehicleCompositionRelativeFlow(self.vissim.Net.VehicleTypes.ItemByKey(10001),
                                                                           self.vissim.Net.DesSpeedDistributions.ItemByKey(DesSpeedDistributions))
            relflows = self.vehComps[vc].VehCompRelFlows.GetAll()
            # if len(relflows) == 3:
            relflows[0].SetAttValue('VehType', 100) # Changing the vehicle type to ensure setting relative flow for the right type
            relflows[0].SetAttValue('RelFlow', carpr) # Changing the relative flow of Car Relative Flow.
            relflows[1].SetAttValue('VehType', 10001) # Changing the vehicle type to ensure setting relative flow for the right type
            relflows[1].SetAttValue('RelFlow', cavpr) # Changing the relative flow of the CAV Relative Flow.
            print(f"Set relative flow for vehicle composition to CAR: {carpr} and CAV: {cavpr}")
    def close_vissim(self):
        self.vissim = None

    def init_ego_veh(self):
        # Initialize the ego vehicle
        # Set the ego vehicle to use the external driver model, which sends the vehicle state to the traffic layer
        ego_vt_no = 10002  # Use an existing vehicle type ID
        ego_vt = self.vissim.Net.VehicleTypes.ItemByKey(ego_vt_no)
        ego_vt.SetAttValue("ExtDriver", True)
        ego_vt.SetAttValue("ExtDriverDLLFile", self.driver_model_path)
        ego_vt.SetAttValue("ExtDriverParFile", self.traffic_layer_config_path)

    def add_ego_veh(self):
        ego_vt_no = 10002  # Use an existing vehicle type ID
        link_id = 1  # Replace with your target link ID
        lane = 1  # Lane index (1-based)
        position = 10  # Distance from start of link (in feet)
        desired_speed = 50  # Speed in m/s
        ego_vehicle = self.vissim.Net.Vehicles.AddVehicleAtLinkPosition(ego_vt_no, link_id, lane, position, desired_speed)
        if self.verbose:
            print(f"Ego vehicle added with ID: {ego_vehicle.AttValue('No')}")
        self.ego_vehicle_id = ego_vehicle.AttValue('No') 
    
    def warmup_vissim(self):
        """
        Set up the simulation environment
        :param cavpr:
        :param end_of_simulation:
        :param seed:
        :return:
        """
        
        # self.vissim.Simulation.SetAttValue('SimBreakAt', WARMUP_SECONDS)
        # self.vissim.Simulation.RunContinuous()
        self.vissim.Simulation.RunSingleStep()
        simsec = self.vissim.Simulation.SimulationSecond
        is_very_first_step = True
        # Run first WARMUP_SECONDS seconds as warm up
        while simsec < WARMUP_SECONDS:
            simsec = self.vissim.Simulation.SimulationSecond

            if is_very_first_step:
                is_very_first_step = False
                sim_state = 1
                self.socket_helper.vehicle_data_send_list.append(VehData(id='20', 
                                               speedDesired=22.0,
                                               accelerationDesired=0.0
                                               ))

            if self.with_vehicle_dynamics:
                self.socket_helper.sendData(sim_state, simsec, self.socket2simulink)    
                self.socket_helper.clear_data() 
                # receive data from the client (the actual vehicle data after the vehidle dynamics model)   
                self.socket_helper.recv_data(self.socket2simulink)  
                sim_state = 1
                #self.socket_helper.vehicle_data_send_list.extend(self.socket_helper.vehicle_data_receive_list)
                self.socket_helper.vehicle_data_send_list.append(VehData(id='20', 
                                               speedDesired=22.0,
                                               accelerationDesired=0.0
                                               ))
            self.vissim.Simulation.RunSingleStep()

        if self.with_ego_veh:
            self.add_ego_veh()
        print('Warmup Finished')

    def start_subscription(self, control_ego_only=False):
        """
    
        :param vehicle_dynamics:
        :param eco_driving:
        :return:
        """
        self.socket_helper.clear_data()
        simsec = self.vissim.Simulation.SimulationSecond
        is_very_first_step = True
        while simsec <= self.end_of_simulation - 300:
            if is_very_first_step:
                is_very_first_step = False
                sim_state = 1
                self.socket_helper.vehicle_data_send_list.append(VehData(id=str(self.ego_vehicle_id),
                                                                        speedDesired=22.0,
                                                                        accelerationDesired=0.0))

            try:
                # send the previous state to the FIXS server
                if ENABLE_SOCKET:
                    self.socket_helper.sendData(sim_state, simsec, self.socket2FIXS)
                    self.socket_helper.clear_data()
                if self.verbose:
                    print(f'Current Simulation Time Step: {simsec}')
            except Exception as e:
                print('receive FIXS done')

            try:
                # get vehicle information
                if control_ego_only:
                    filtering_criteria = f"[No] = {self.ego_vehicle_id}"

                else:
                    filtering_criteria = f"[VehType] = 10001 OR [VehType] = 10002"
                   
                 
                filtered_vehicles = self.vissim.Net.Vehicles.GetFilteredSet(filtering_criteria)
                v = filtered_vehicles.GetMultipleAttributes(('No', 'VehType', 'Speed', 'Pos', 'Lane', 'DesSpeed',
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
                CAV_curr = vehs
                
                # get the current link information
                CAV_curr['CurrentLink'] = CAV_curr['Lane'].str.split('-').str[0].astype(int)
                # join the linkinfo based on link id
                CAV_curr = CAV_curr.join(self.linkinfo[['Link', 'Direction', 'nextSC', 'Length2D', 'cumLength2D', 'cumLength2D_max']].set_index('Link'), on='CurrentLink')
                CAV_curr['d'] = CAV_curr['cumLength2D_max'] - CAV_curr['cumLength2D'] + CAV_curr['Length2D'] - CAV_curr['Pos']
                # get original desired speed based on link information and DesSpeedFrac (given we know all the DesSpeedDistr on the major road are No. 64)
                CAV_curr['OrgDesSpeed'] = 50 + CAV_curr['DesSpeedFrac'] * (54 - 50)
                # For those vehicles that are not on the major road, which we are not controlling. Set the OrgDesSpeed equals to DesSpeed
                CAV_curr['OrgDesSpeed'] = np.where(CAV_curr['DesSpeedFrac'].isna(), CAV_curr['DesSpeed'], CAV_curr['OrgDesSpeed'])
                # coasting parameters based on tractive power
                A = 0.083698
                B = 0.00385
                C = 0.000278
                M = 1.6443
                # update the desire speed for controllable vehicles on EB and WB
                CAV_curr_control_eb = gen_desire_speed_dir_r_(CAV_curr, simsec, LOOKAHEAD_STEP, self.lsa, A, B, C, M, self.example_coasting_profile, self.static_routes_eb_wb_th)
                
                # vihicles controllable by Vissim (CAV)
                CAV_VISSIM = CAV_curr[CAV_curr['VehType'] == '10001']
                # vihicles controllable by FIXS (egoCAV)
                CAV_FIXS = CAV_curr[CAV_curr['VehType'] == '10002']
                if not control_ego_only:
                    ################################################################################################
                    # map to all the cavs
                    ################################################################################################
                    if CAV_curr_control_eb is not None:
                        # set the controlled CAV desired speed
                        CAV_curr.loc[CAV_curr['No'].isin(CAV_curr_control_eb['No'].values), 'DesSpeed'] = CAV_curr['No'].map(CAV_curr_control_eb.set_index('No')['instant_desired_speed'])
                        # reset the uncontroller CAV's desired speed as the original desired speed
                        CAV_curr.loc[~CAV_curr['No'].isin(CAV_curr_control_eb['No'].values), 'DesSpeed'] = CAV_curr.loc[~CAV_curr['No'].isin(CAV_curr_control_eb['No'].values), 'OrgDesSpeed']
                        # update the desire speed of all CAVs to overall vehicles
                        vehs.loc[vehs['No'].isin(CAV_curr['No'].values), 'DesSpeed'] = vehs['No'].map(CAV_curr.set_index('No')['DesSpeed'])

                        # # remove the egoCAV from the overall vehicles, to be later sent to FIXS
                        # vehs = vehs[~vehs['No'].isin(CAV_FIXS['No'].values)]
                        vehs = vehs[vehs['No'].isin(CAV_VISSIM['No'].values)]
                    else:
                        CAV_curr['DesSpeed'] = CAV_curr['OrgDesSpeed']
                    # add numbering to prepare set speeds data
                    vehs['ID'] = vehs.index + 1
                    setSpeeds = vehs[["ID", "DesSpeed"]]
                    # set desire speeds
                    self.vissim.Net.Vehicles.SetMultiAttValues(('DesSpeed'), tuple(list(setSpeeds.itertuples(index=False, name=None))))
                    if self.verbose:
                        print(setSpeeds)
                if CAV_curr_control_eb is not None:
                    # get the egoCAV speed, maybe multiple egoCAV
                    egoCAV_curr = CAV_curr_control_eb[CAV_curr_control_eb['VehType'] == '10002']
                    egoCAV_curr['DesAcceleration'] = egoCAV_curr['a_out'] # m/s2
                    egoCAV_curr['No'] = egoCAV_curr['No'].astype(str)
                    egoCAV_curr['DesSpeed'] = egoCAV_curr['instant_desired_speed'].astype(float) * 0.44704 # mph to m/s
                    eco_speed_dic = egoCAV_curr.set_index('No')['DesSpeed'].to_dict()
                    eco_accel_dic = egoCAV_curr.set_index('No')['DesAcceleration'].to_dict()
                else: 
                    eco_speed_dic = vehs.set_index('No')['Speed'].to_dict()
                    eco_accel_dic = vehs.set_index('No')['Acceleration'].to_dict()

            except Exception as e:
                print('set surrounding vehicle desired speed done')
            
            try:
                self.vissim.Simulation.RunSingleStep()
                simsec = self.vissim.Simulation.SimulationSecond
                # Should apply the FIXS control to the ego vehicle at this step
                if ENABLE_SOCKET:
                    sim_state, sim_time = self.socket_helper.recv_data(self.socket2FIXS)
                    ori_speed_dic = {veh_data.id:veh_data.speed for veh_data in self.socket_helper.vehicle_data_receive_list}
                    self.socket_helper.clear_data()
                    # set the disired speed or acceleration for the vehicles to control
                    for veh_id in ori_speed_dic.keys():
                        # if using eco driving, and set the accelerationDesired
                        if self.use_accel and veh_id in eco_accel_dic.keys() and eco_driving:
                            # veh_data = VehData(id=veh_id, accelerationDesired=eco_accel_dic[veh_id])
                            veh_data = VehData(id=veh_id, 
                                            #    speedDesired=(eco_accel_dic[veh_id] * self.step_length) + ori_speed_dic[veh_id],
                                               speedDesired=eco_speed_dic[veh_id],
                                               accelerationDesired=eco_accel_dic[veh_id]
                                               )
                            if self.verbose:
                                print(f"(Controller) Trying to set {veh_id} from {ori_speed_dic[veh_id]} to  {eco_speed_dic[veh_id]} with acc {eco_accel_dic[veh_id]}")
                        # if using eco driving, and set the speedDesired
                        elif (not self.use_accel) and veh_id in eco_speed_dic.keys() and eco_driving:
                            veh_data = VehData(id=veh_id, speedDesired=eco_speed_dic[veh_id])
                        else:
                            veh_data = VehData(id=veh_id, speedDesired=ori_speed_dic[veh_id])
                        
                        self.socket_helper.vehicle_data_send_list.append(veh_data)
                    
                    if self.with_vehicle_dynamics or self.use_simulink_for_energy_evaluation:
                        self.socket_helper.sendData(sim_state, simsec, self.socket2simulink)    
                        self.socket_helper.clear_data() 
                        # receive data from the client (the actual vehicle data after the vehidle dynamics model)   
                        self.socket_helper.recv_data(self.socket2simulink)  
                        # vehicle_data_send_tmp = [].extend(self.socket_helper.vehicle_data_receive_list)
                        # self.socket_helper.vehicle_data_send_list.extend(vehicle_data_send_tmp)
                        # self.socket_helper.vehicle_data_send_list = self.socket_helper.vehicle_data_receive_list
                        simulink_speed_dic = {veh_data.id:veh_data.speedDesired for veh_data in self.socket_helper.vehicle_data_receive_list}

                        for veh_id in ori_speed_dic.keys():
                            # if using eco driving, and set the accelerationDesired
                            if self.use_accel and veh_id in simulink_speed_dic.keys() and eco_driving and self.with_vehicle_dynamics:
                                # veh_data = VehData(id=veh_id, accelerationDesired=eco_accel_dic[veh_id])
                                veh_data = VehData(id=veh_id, 
                                                #    speedDesired=(eco_accel_dic[veh_id] * self.step_length) + ori_speed_dic[veh_id],
                                                   speedDesired=simulink_speed_dic[veh_id],
                                                   accelerationDesired=(simulink_speed_dic[veh_id] - ori_speed_dic[veh_id])/ self.step_length
                                                   )
                                if self.verbose:
                                    print(f"(Simulink) Trying to set {veh_id} from {ori_speed_dic[veh_id]} to  {simulink_speed_dic[veh_id]} with acc {(simulink_speed_dic[veh_id] - ori_speed_dic[veh_id])/ self.step_length}")
                            # if using eco driving, and set the speedDesired
                            elif (not self.use_accel) and veh_id in simulink_speed_dic.keys() and eco_driving and self.with_vehicle_dynamics:
                                veh_data = VehData(id=veh_id, speedDesired=simulink_speed_dic[veh_id])
                            elif self.use_simulink_for_energy_evaluation and veh_id in simulink_speed_dic.keys() and eco_driving:
                                veh_data = VehData(id=veh_id, speedDesired=eco_speed_dic[veh_id])
                            else:
                                veh_data = VehData(id=veh_id, speedDesired=ori_speed_dic[veh_id])

                            self.socket_helper.vehicle_data_send_list.append(veh_data)
                    
            except Exception as e:
                print('\nERROR: FIXS simulink exception\n')
                pass

        self.close_vissim()



def run_traffic_layer(traffic_layer_path, config_path):
    # start cmd /k ..\..\Trafficlayer\x64\Debug\TrafficLayer.exe -f '.\ecodrivingConfig.yaml'\
    
    os.system(f'start cmd /k {traffic_layer_path} -f {config_path}')




if __name__ == '__main__':
    experiment_config = {
        'driver_model_path': r"DriverModel_RealSim_v2021.dll", #r"DriverModel_RealSim_v2021_vanilla.dll", #r"DriverModel_RealSim.dll",
        'config_path': r"ecodrivingConfig_VISSIM_speed.yaml",
        'simulation_file_path':r'Experiments\\banMinorLeftTurnFixedTimingV2_WithEgo'
    }
    experiment_config = {path_key:os.path.join(os.getcwd(), relative_path) for path_key, relative_path in experiment_config.items()}
    print(experiment_config)
    parser = argparse.ArgumentParser()
    parser.add_argument("-c", "--config", type=str, help="Path to the Configuration file", default=r'C:\Users\hg25079\Documents\GitHub\FIXS\tests\Applications\Eco_Fixed_Timming\Experiments_Vissim\ecodriving_config_Vissim.yaml')
    parser.add_argument("--vissimPort", type=str, help="Specify port of vissim", default=1337)
    parser.add_argument("--trafficLayerIP", type=str, help="Specify IP of traffic layer", default="127.0.0.1")
    parser.add_argument("--trafficlayerPort", type=str, help="Specify port of traffic layer", default=430)
    parser.add_argument("--simulinkIP", type=str, help="Specify IP of simulink", default="127.0.0.1")
    parser.add_argument("--simulinkPort", type=str, help="Specify port of simulink", default=420)
    parser.add_argument("--ecoDriving", action="store_true", help="Use the eco driving controller", default=True)
    parser.add_argument("--vehicleDynamics", action="store_true", help="use the vehicle dynamis", default=False)
    parser.add_argument("--penetrationRate", type=float, help="the penetration rate of cav", default=0.1)
    parser.add_argument("--pathToNet", type=str, help="the path to the net file", default=r'C:\Users\hg25079\Documents\GitHub\FIXS\tests\Applications\Eco_Fixed_Timming\Experiments_Vissim\banMinorLeftTurnFixedTimingV2_WithEgo')
    parser.add_argument("--verbose", action="store_true", help="print verbose output", default=False)
    
    args = parser.parse_args()
    traffic_layer_config = args.config
    vissim_port = args.vissimPort
    traffic_layer_ip = args.trafficLayerIP
    traffic_layer_port = args.trafficlayerPort
    simulink_ip = args.simulinkIP
    simulink_port = args.simulinkPort
    penetration_rate = float(args.penetrationRate)
    path_to_net = args.pathToNet
    print(path_to_net)
    vehicle_dynamics = args.vehicleDynamics
    eco_driving = args.ecoDriving
    verbose = args.verbose
    run_traffic_layer('TrafficLayer.exe',traffic_layer_config)
    vissim_env = VissimEnvMultiAgent(driver_model_path=r"C:\Users\hg25079\Documents\GitHub\FIXS\tests\Applications\Eco_Fixed_Timming\DriverModel_RealSim_v2021.dll",
                                    vissim_port=vissim_port,
                                    traffic_layer_ip=traffic_layer_ip,
                                    traffic_layer_port=traffic_layer_port,
                                    simulink_ip=simulink_ip,
                                    simulink_port=simulink_port,
                                    simulation_file_path=path_to_net,
                                    with_ego_veh=True,
                                    with_ext_driver=True,
                                    traffic_layer_config_path=traffic_layer_config, 
                                    eco_driving=eco_driving,
                                    with_vehicle_dynamics=vehicle_dynamics,
                                    use_simulink_for_energy_evaluation=False,
                                    step_length=0.1,
                                    verbose=verbose)
    
    vissim_env.init_simulation_environment(cavpr=penetration_rate)
    vissim_env.warmup_vissim()
    vissim_env.start_subscription(control_ego_only=True)
    
