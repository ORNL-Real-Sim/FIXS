"""
Simple Echo Client for TrafficLayer.exe
Receives VehicleData messages and echoes them back to the server
"""

import argparse
import socket
import sys
import os
import pathlib
import time

# Add repo root to path to import CommonLib
sys.path.insert(0, str(pathlib.Path(__file__).parents[3]))

from CommonLib.SocketHelper import SocketHelper
from CommonLib.MsgHelper import MsgHelper
from CommonLib.ConfigHelper import ConfigHelper


def parse_args():
    p = argparse.ArgumentParser(description='Simple echo client / subscription repro for TrafficLayer.exe')
    p.add_argument('--config', default=os.path.join(os.path.dirname(__file__), 'config.yaml'),
                   help='Config YAML to read connection params from (must match the one TrafficLayer uses).')
    p.add_argument('--steps', type=int, default=0,
                   help='Run this many simulation steps then exit cleanly. 0 = run until server shuts down.')
    p.add_argument('--warmup', type=int, default=0,
                   help='Ignore the first N steps before counting (lets background traffic finish inserting).')
    p.add_argument('--max-echo', type=int, default=0,
                   help='Echo back at most this many vehicles (0 = all). Use 1 to mirror XIL '
                        '(client returns only ego) and avoid stressing the receive-many path.')
    p.add_argument('--report', action='store_true',
                   help='On exit, print a machine-readable summary line: RESULT max_vehicles=<N> distinct_total=<M>.')
    p.add_argument('--expect-min', type=int, default=None,
                   help='Exit non-zero unless max observed vehicle count >= this value.')
    p.add_argument('--expect-max', type=int, default=None,
                   help='Exit non-zero unless max observed vehicle count <= this value.')
    return p.parse_args()


def main():
    args = parse_args()

    # Load configuration
    config_file = args.config
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

    # Connect to TrafficLayer.exe (retry: TrafficLayer may still be opening its listen socket)
    server_address = (server_ip, server_port)
    print(f'Connecting to {server_ip} port {server_port}', file=sys.stderr)

    client_socket = None
    connect_deadline = time.time() + 15
    while True:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            s.connect(server_address)
            client_socket = s
            print('Connected successfully!', file=sys.stderr)
            break
        except OSError as e:
            s.close()
            if time.time() >= connect_deadline:
                print(f'Failed to connect within timeout: {e}', file=sys.stderr)
                return 1
            time.sleep(0.5)

    # Get verbose log flag from config
    verbose_log = config_helper.simulation_setup.get('EnableVerboseLog', False)

    # Counters for the subscription repro (#176): peak vehicle count and all distinct ids seen
    max_vehicles = 0
    distinct_ids = set()

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

            # Track vehicle visibility (after warmup) for the subscription repro
            if step_count > args.warmup:
                n = len(socket_helper.vehicle_data_receive_list)
                if n > max_vehicles:
                    max_vehicles = n
                for veh_data in socket_helper.vehicle_data_receive_list:
                    distinct_ids.add(veh_data.id.strip())

            # Decide how many vehicles to echo back. In real XIL the client (CarMaker)
            # returns only the ego pose, so TrafficLayer's RECEIVE path is only ever
            # exercised with ~1 vehicle. --max-echo caps the echo to mirror that; echoing
            # ALL received vehicles (max_echo=0) stresses a receive-many path that
            # production never uses and can deadlock the round-trip at higher counts.
            echo_list = socket_helper.vehicle_data_receive_list
            if args.max_echo > 0:
                echo_list = echo_list[:args.max_echo]

            if verbose_log:
                print(f'\n--- Step {step_count} | Time: {sim_time:.2f}s | State: {sim_state} ---')
                print(f'Received {len(socket_helper.vehicle_data_receive_list)} vehicles:')
                for veh_data in socket_helper.vehicle_data_receive_list:
                    print(f'  Vehicle ID: {veh_data.id.strip()}, Speed: {veh_data.speed:.2f} m/s, '
                          f'Pos: ({veh_data.positionX:.2f}, {veh_data.positionY:.2f})')

            for veh_data in echo_list:
                socket_helper.vehicle_data_send_list.append(veh_data)
            socket_helper.sendData(sim_state, sim_time, client_socket)

            if verbose_log:
                print(f'Echoed {len(socket_helper.vehicle_data_send_list)} vehicles back to server')
            elif step_count % 100 == 0:
                print(f'Step {step_count} | Time: {sim_time:.2f}s | Vehicles: {len(socket_helper.vehicle_data_receive_list)}')

            # Stop after the requested number of steps (used by the automated repro)
            if args.steps and step_count >= args.steps:
                print(f'\nReached requested {args.steps} steps. Exiting.', file=sys.stderr)
                break

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

    # Machine-readable summary for the automated subscription comparison (#176)
    if args.report:
        print(f'RESULT max_vehicles={max_vehicles} distinct_total={len(distinct_ids)} '
              f'ids={",".join(sorted(distinct_ids))}')

    # Optional pass/fail gating
    rc = 0
    if args.expect_min is not None and max_vehicles < args.expect_min:
        print(f'FAIL: max_vehicles={max_vehicles} < expected min {args.expect_min}', file=sys.stderr)
        rc = 1
    if args.expect_max is not None and max_vehicles > args.expect_max:
        print(f'FAIL: max_vehicles={max_vehicles} > expected max {args.expect_max}', file=sys.stderr)
        rc = 1
    return rc


if __name__ == '__main__':
    sys.exit(main() or 0)
