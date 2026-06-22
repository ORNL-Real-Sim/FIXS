"""
Simple Echo Client with Simulink XIL
Python client acts as middleware between TrafficLayer, SUMO, and Simulink:
- Connects to SUMO as second client via TraCI
- Receives ego + traffic vehicles from TrafficLayer
- Sends ego commands to Simulink
- Receives ego actual from Simulink and sends back to TrafficLayer
- Controls traffic vehicles (traffic_0, traffic_1, traffic_2) via SUMO setSpeed
"""

import socket
import sys
import os
import traci

# Add parent directory to path to import CommonLib
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..')))

from CommonLib.SocketHelper import SocketHelper
from CommonLib.MsgHelper import MsgHelper
from CommonLib.ConfigHelper import ConfigHelper


def main():
    # Load configuration
    config_file = os.path.join(os.path.dirname(__file__), 'config.yaml')
    config_helper = ConfigHelper()
    config_helper.getConfig(config_file)

    # Initialize message helper for TrafficLayer communication
    msg_helper = MsgHelper()
    vehicle_msg_fields = config_helper.simulation_setup.get('VehicleMessageField', ['id', 'speed'])
    msg_helper.set_vehicle_message_field(vehicle_msg_fields)

    # Initialize socket helper for TrafficLayer
    socket_helper = SocketHelper(config_helper, msg_helper)

    # Initialize message helper for Simulink communication
    msg_helper_simulink = MsgHelper()
    msg_helper_simulink.set_vehicle_message_field(vehicle_msg_fields)
    socket_helper_simulink = SocketHelper(config_helper, msg_helper_simulink)

    # Get TrafficLayer connection parameters from ApplicationSetup
    vehicle_subscription = config_helper.application_setup.get('VehicleSubscription', [])
    if not vehicle_subscription:
        print('Error: No VehicleSubscription found in ApplicationSetup', file=sys.stderr)
        return

    traffic_layer_ip = vehicle_subscription[0]['ip'][0]
    traffic_layer_port = vehicle_subscription[0]['port'][0]

    # Get SUMO connection parameters from SimulationSetup
    sumo_ip = config_helper.simulation_setup.get('TrafficSimulatorIP', '127.0.0.1')
    sumo_port = config_helper.simulation_setup.get('TrafficSimulatorPort', 1337)

    # Get Simulink connection parameters from XilSetup
    xil_subscription = config_helper.xil_setup.get('VehicleSubscription', [])
    if not xil_subscription:
        print('Error: No XIL VehicleSubscription found in XilSetup', file=sys.stderr)
        return

    simulink_ip = xil_subscription[0]['ip'][0]
    simulink_port = xil_subscription[0]['port'][0]

    # Connect to TrafficLayer
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    print(f'Connecting to TrafficLayer at {traffic_layer_ip}:{traffic_layer_port}', file=sys.stderr)

    try:
        client_socket.connect((traffic_layer_ip, traffic_layer_port))
        print('Connected to TrafficLayer successfully!', file=sys.stderr)
    except Exception as e:
        print(f'Failed to connect to TrafficLayer: {e}', file=sys.stderr)
        return

    # Connect to SUMO as second client
    print(f'Connecting to SUMO at {sumo_ip}:{sumo_port}', file=sys.stderr)

    try:
        traci.init(sumo_port, host=sumo_ip)
        print('Connected to SUMO successfully!', file=sys.stderr)
    except Exception as e:
        print(f'Failed to connect to SUMO: {e}', file=sys.stderr)
        client_socket.close()
        return

    # Setup server socket for Simulink connection
    simulink_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    simulink_socket.bind((simulink_ip, simulink_port))
    simulink_socket.listen(1)
    print(f'Waiting for Simulink to connect on {simulink_ip}:{simulink_port}...', file=sys.stderr)

    simulink_conn, simulink_addr = simulink_socket.accept()
    print(f'Simulink connected from {simulink_addr}', file=sys.stderr)

    # Get verbose log flag from config
    verbose_log = config_helper.simulation_setup.get('EnableVerboseLog', False)

    # Main loop
    step_count = 0
    controlled_traffic = ['traffic_0', 'traffic_1', 'traffic_2']

    try:
        while True:
            # Clear previous data
            socket_helper.clear_data()
            socket_helper_simulink.clear_data()

            # Step 1: Receive data from TrafficLayer (ego + traffic vehicles)
            sim_state, sim_time = socket_helper.recv_data(client_socket)

            # Check for shutdown signal (state == 0)
            if sim_state == 0:
                print('\nReceived shutdown signal from TrafficLayer. Exiting gracefully...', file=sys.stderr)
                # Send shutdown signal to Simulink
                socket_helper_simulink.sendData(0, sim_time, simulink_conn)
                break

            step_count += 1

            # Step 2: Find ego vehicle data from TrafficLayer
            ego_data = None
            traffic_data = []

            for veh_data in socket_helper.vehicle_data_receive_list:
                veh_id = veh_data.id.strip()
                if veh_id == 'ego':
                    ego_data = veh_data
                elif veh_id in controlled_traffic:
                    traffic_data.append(veh_data)

            # Step 3: Send ego commands to Simulink
            if ego_data:
                socket_helper_simulink.vehicle_data_send_list.append(ego_data)
                socket_helper_simulink.sendData(sim_state, sim_time, simulink_conn)

                if verbose_log:
                    print(f'\n--- Step {step_count} | Time: {sim_time:.2f}s ---')
                    print(f'Sent ego cmd to Simulink: Speed={ego_data.speed:.2f} m/s')

            # Step 4: Receive ego actual from Simulink
            socket_helper_simulink.clear_data()
            sim_state_simulink, sim_time_simulink = socket_helper_simulink.recv_data(simulink_conn)

            ego_actual = None
            if len(socket_helper_simulink.vehicle_data_receive_list) > 0:
                ego_actual = socket_helper_simulink.vehicle_data_receive_list[0]

                if verbose_log:
                    print(f'Received ego actual from Simulink: Speed={ego_actual.speed:.2f} m/s')

            # Step 5: Control traffic vehicles via SUMO setSpeed
            for veh_data in traffic_data:
                veh_id = veh_data.id.strip()
                try:
                    if veh_id in traci.vehicle.getIDList():
                        # Simple control: maintain desired speed
                        target_speed = veh_data.speedDesired
                        traci.vehicle.setSpeed(veh_id, target_speed)

                        if verbose_log:
                            print(f'Set {veh_id} speed to {target_speed:.2f} m/s via SUMO')
                except Exception as e:
                    if verbose_log:
                        print(f'Error controlling {veh_id}: {e}')

            # Step 6: Send ego actual back to TrafficLayer
            if ego_actual:
                socket_helper.vehicle_data_send_list.append(ego_actual)

            socket_helper.sendData(sim_state, sim_time, client_socket)

            # Step 7: Advance SUMO simulation (as second client)
            traci.simulationStep()

            # Print basic info
            if not verbose_log and step_count % 100 == 0:
                print(f'Step {step_count} | Time: {sim_time:.2f}s | Vehicles: {len(socket_helper.vehicle_data_receive_list)}')

    except KeyboardInterrupt:
        print('\nShutting down client...', file=sys.stderr)
    except ConnectionResetError:
        print('\nConnection closed.', file=sys.stderr)
    except Exception as e:
        print(f'\nError occurred: {e}', file=sys.stderr)
        import traceback
        traceback.print_exc()
    finally:
        print('Closing connections...', file=sys.stderr)
        client_socket.close()
        simulink_conn.close()
        simulink_socket.close()
        traci.close()


if __name__ == '__main__':
    main()
