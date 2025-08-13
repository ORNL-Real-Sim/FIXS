import traci
import socket
import argparse
import numpy as np
from CommonLib.SocketHelper import SocketHelper
from CommonLib.ConfigHelper import ConfigHelper
from CommonLib.MsgHelper import MsgHelper
from CommonLib.VehDataMsgDefs import VehData


class SumoEnvMultiAgent:
    def __init__(self, traffic_layer_config, sumo_port=1337, traffic_layer_port=430):

        # initialize the socket connections
        config_helper = ConfigHelper()
        config_helper.getConfig(traffic_layer_config)
        msg_helper = MsgHelper()
        msg_helper.set_vehicle_message_field(config_helper.simulation_setup['VehicleMessageField'])
        self.socket_helper = SocketHelper(config_helper=config_helper, msg_helper=msg_helper)
        # IP to connect to the FIXS server
        
        self.sumo_port = sumo_port
        self.traffic_layer_port = traffic_layer_port
        self.socket2FIXS = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        print(self.traffic_layer_port)
        self.socket2FIXS.connect(('127.0.0.1', int(self.traffic_layer_port)))
        print('Connected to FIXS server')

    def _get_simulate_speed(self, sim_time, speed_max=20.0, speed_min=8.0, period=10.0):
        """
        Get the speed of the vehicle at the given simulation time
        """
        return speed_max * (1 - np.exp(-sim_time / period)) + speed_min

    def start_subscription(self):
        
        traci.init(port=int(self.sumo_port), host='127.0.0.1')
        traci.setOrder(2)
        while True:
            try:
                traci.simulationStep()
                sim_time = traci.simulation.getTime()
                desired_speed = self._get_simulate_speed(sim_time)
                self.apply_vehicle_control_FIXS({'ego': desired_speed}, control_veh_ids = ['ego'])
            except Exception as e:
                print(e)
                self.close()
                break

            
    def apply_vehicle_control_FIXS(self, desired_speed_dic, control_veh_ids = ['ego']):
        """
        Apply the vehicle control to the vehicles
        :param desired_speed_dic: dictionary of vehicle id and desired speed
        :param control_veh_ids: list of vehicle ids to control
        :return:
        """
        sim_state, sim_time = self.socket_helper.recv_data(self.socket2FIXS)
        ori_speed = {veh_data.id:veh_data.speed for veh_data in self.socket_helper.vehicle_data_receive_list}
        for veh_id, desired_speed in desired_speed_dic.items():
            if desired_speed is not None and veh_id in control_veh_ids:
                veh_data = VehData(id=veh_id, speedDesired=desired_speed)
                self.socket_helper.vehicle_data_send_list.append(veh_data)
       
        
        self.socket_helper.sendData(sim_state, sim_time, self.socket2FIXS)
        self.socket_helper.clear_data()



    def close(self):
        traci.close()



if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-c", "--config", type=str, help="Path to the Configuration file", default='defaultConfig.yaml')
    parser.add_argument("--sumoPort", type=str, help="Specify port of sumo", default=1337)
    parser.add_argument("--trafficlayerPort", type=str, help="Specify port of traffic layer", default=431)
    args = parser.parse_args()
    traffic_layer_config = args.config
    sumo_port = args.sumoPort
    traffic_layer_port = args.trafficlayerPort

    senv = SumoEnvMultiAgent(traffic_layer_config=traffic_layer_config,
                             sumo_port=sumo_port, 
                             traffic_layer_port=traffic_layer_port)
    senv.start_subscription()
