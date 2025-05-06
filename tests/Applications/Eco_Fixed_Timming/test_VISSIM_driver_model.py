from CommonLib.ConfigHelper import ConfigHelper
from CommonLib.MsgHelper import MsgHelper
from CommonLib.SocketHelper import SocketHelper
import socket
import subprocess
def start_vissim(ip='127.0.0.1', port=5000, vissim_path=r"C:\Program Files\PTV Vision\PTV Vissim 2022\Exe\Vissim220.exe"):
    """
    Launches PTV Vissim 2022 in Automation mode on the specified port.
    
    Parameters:
    - port (int): TCP port for the COM Automation server (default: 5000)
    - vissim_path (str): Full path to Vissim220.exe
    """
    try:
        print(f"Launching Vissim with Automation server on port {ip}:{port}.")
        subprocess.Popen([vissim_path, "-Automation", f"-Port={port}"], shell=False)
    except FileNotFoundError:
        print(f"Error: Vissim executable not found at {vissim_path}")
    except Exception as e:
        print(f"Failed to launch Vissim: {e}")

if __name__ == '__main__':
    
    config_helper = ConfigHelper()
    config_helper.getConfig('ecodrivingConfig.yaml')

    msg_helper = MsgHelper()
    msg_helper.set_vehicle_message_field(config_helper.simulation_setup['VehicleMessageField'])
    socket_helper = SocketHelper(config_helper=config_helper, msg_helper=msg_helper)

    vissim_ip = config_helper.simulation_setup['TrafficSimulatorIP']
    vissim_port = config_helper.simulation_setup['TrafficSimulatorPort']
    start_vissim(ip=vissim_ip, port=vissim_port, vissim_path=r"C:\Program Files\PTV Vision\PTV Vissim 2022\Exe\Vissim220.exe")



    # socket2FIXS = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # socket2FIXS.connect((vissim_ip, int(vissim_port)))
    # print('Connected to FIXS server')

