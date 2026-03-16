"""
Simple Echo Client for TrafficLayer.exe
Receives VehicleData messages and echoes them back to the server
"""

import socket
import sys
import os

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

    # Initialize message helper
    msg_helper = MsgHelper()
    vehicle_msg_fields = config_helper.simulation_setup.get('VehicleMessageField', ['id', 'speed'])
    msg_helper.set_vehicle_message_field(vehicle_msg_fields)

    # Initialize socket helper
    socket_helper = SocketHelper(config_helper, msg_helper)

    # Get connection parameters from ApplicationSetup VehicleSubscription
    vehicle_subscription = config_helper.application_setup.get('VehicleSubscription', [])
    if not vehicle_subscription:
        print('Error: No VehicleSubscription found in ApplicationSetup', file=sys.stderr)
        return

    server_ip = vehicle_subscription[0]['ip'][0]
    server_port = vehicle_subscription[0]['port'][0]

    # Create a TCP/IP socket
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    # Connect to TrafficLayer.exe
    server_address = (server_ip, server_port)
    print(f'Connecting to {server_ip} port {server_port}', file=sys.stderr)

    try:
        client_socket.connect(server_address)
        print('Connected successfully!', file=sys.stderr)
    except Exception as e:
        print(f'Failed to connect: {e}', file=sys.stderr)
        return

    # Get verbose log flag from config
    verbose_log = config_helper.simulation_setup.get('EnableVerboseLog', False)

    # Main loop: receive and echo back
    step_count = 0
    try:
        while True:
            # Clear previous data
            socket_helper.clear_data()

            # Receive data from TrafficLayer
            sim_state, sim_time = socket_helper.recv_data(client_socket)

            # Check for shutdown signal (state == 0)
            if sim_state == 0:
                print('\nReceived shutdown signal from server. Exiting gracefully...', file=sys.stderr)
                break

            # Print received vehicle data
            step_count += 1

            if verbose_log:
                print(f'\n--- Step {step_count} | Time: {sim_time:.2f}s | State: {sim_state} ---')
                print(f'Received {len(socket_helper.vehicle_data_receive_list)} vehicles:')

                for veh_data in socket_helper.vehicle_data_receive_list:
                    print(f'  Vehicle ID: {veh_data.id.strip()}, Speed: {veh_data.speed:.2f} m/s, '
                          f'Pos: ({veh_data.positionX:.2f}, {veh_data.positionY:.2f})')

                    # Echo back: add received vehicle to send list
                    socket_helper.vehicle_data_send_list.append(veh_data)

                # Send data back to TrafficLayer
                socket_helper.sendData(sim_state, sim_time, client_socket)
                print(f'Echoed {len(socket_helper.vehicle_data_send_list)} vehicles back to server')
            else:
                # Echo back: add received vehicles to send list
                for veh_data in socket_helper.vehicle_data_receive_list:
                    socket_helper.vehicle_data_send_list.append(veh_data)

                # Send data back to TrafficLayer
                socket_helper.sendData(sim_state, sim_time, client_socket)

                # Print basic info every 100 steps
                if step_count % 100 == 0:
                    print(f'Step {step_count} | Time: {sim_time:.2f}s | Vehicles: {len(socket_helper.vehicle_data_receive_list)}')

    except KeyboardInterrupt:
        print('\nShutting down client...', file=sys.stderr)
    except ConnectionResetError:
        print('\nServer closed the connection.', file=sys.stderr)
    except Exception as e:
        print(f'\nError occurred: {e}', file=sys.stderr)
        import traceback
        traceback.print_exc()
    finally:
        print('Closing connection...', file=sys.stderr)
        client_socket.close()


if __name__ == '__main__':
    main()
