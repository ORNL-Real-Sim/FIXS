"""
Python client for SpeedLimitLite — subscribes to ego vehicle '6' on the
SpeedLimit network and logs received vehicle data to CSV. No XIL, no
Simulink, no .mat comparison.

This is the ugly-but-functional twin of speedLimitClient.slx. Same
network and config as ../SpeedLimit/, but the speed-limit assertions
are reduced to "log it and let the user eyeball the trace" — golden
regression stays in ../SpeedLimit/.
"""

import csv
import os
import socket
import sys
import pathlib

sys.path.insert(0, str(pathlib.Path(__file__).parents[3]))

from CommonLib.SocketHelper import SocketHelper
from CommonLib.MsgHelper import MsgHelper
from CommonLib.ConfigHelper import ConfigHelper


HERE = pathlib.Path(__file__).parent.resolve()
LOG_CSV = HERE / 'speed_limit_lite_trace.csv'
LOG_PARQUET = HERE / 'speed_limit_lite_trace.parquet'


def main():
    config_file = HERE / 'config.yaml'
    config_helper = ConfigHelper()
    config_helper.getConfig(str(config_file))

    msg_helper = MsgHelper()
    fields = config_helper.simulation_setup.get('VehicleMessageField', ['id', 'speed'])
    msg_helper.set_vehicle_message_field(fields)

    socket_helper = SocketHelper(config_helper, msg_helper)

    vehicle_subscription = config_helper.application_setup.get('VehicleSubscription', [])
    if not vehicle_subscription:
        print('Error: No VehicleSubscription found in ApplicationSetup', file=sys.stderr)
        return

    server_ip = vehicle_subscription[0]['ip'][0]
    server_port = vehicle_subscription[0]['port'][0]

    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    print(f'Connecting to {server_ip}:{server_port}', file=sys.stderr)
    try:
        client_socket.connect((server_ip, server_port))
    except Exception as e:
        print(f'Failed to connect: {e}', file=sys.stderr)
        return
    print('Connected.', file=sys.stderr)

    log_fields = ['simTime', 'id', 'speed', 'speedLimit', 'speedLimitNext',
                  'speedLimitChangeDistance', 'signalLightId', 'signalLightHeadId',
                  'precedingVehicleDistance', 'precedingVehicleSpeed']

    csv_file = open(LOG_CSV, 'w', newline='')
    writer = csv.DictWriter(csv_file, fieldnames=log_fields)
    writer.writeheader()

    step_count = 0
    try:
        while True:
            socket_helper.clear_data()
            sim_state, sim_time = socket_helper.recv_data(client_socket)

            if sim_state == 0:
                print('\nShutdown signal received.', file=sys.stderr)
                break

            step_count += 1

            for veh in socket_helper.vehicle_data_receive_list:
                writer.writerow({
                    'simTime': f'{sim_time:.3f}',
                    'id': veh.id.strip(),
                    'speed': f'{veh.speed:.3f}',
                    'speedLimit': f'{getattr(veh, "speedLimit", -1):.3f}',
                    'speedLimitNext': f'{getattr(veh, "speedLimitNext", -1):.3f}',
                    'speedLimitChangeDistance': f'{getattr(veh, "speedLimitChangeDistance", -1):.3f}',
                    'signalLightId': getattr(veh, 'signalLightId', '').strip() if isinstance(getattr(veh, 'signalLightId', ''), str) else getattr(veh, 'signalLightId', ''),
                    'signalLightHeadId': getattr(veh, 'signalLightHeadId', ''),
                    'precedingVehicleDistance': f'{getattr(veh, "precedingVehicleDistance", -1):.3f}',
                    'precedingVehicleSpeed': f'{getattr(veh, "precedingVehicleSpeed", -1):.3f}',
                })

                # Echo back so TrafficLayer's send/recv handshake stays balanced
                socket_helper.vehicle_data_send_list.append(veh)

            socket_helper.sendData(sim_state, sim_time, client_socket)

            if step_count % 50 == 0:
                print(f'Step {step_count} | t={sim_time:.2f}s | vehicles={len(socket_helper.vehicle_data_receive_list)}')

    except KeyboardInterrupt:
        print('\nInterrupted.', file=sys.stderr)
    except ConnectionResetError:
        print('\nServer closed the connection.', file=sys.stderr)
    except Exception as e:
        print(f'\nError: {e}', file=sys.stderr)
        import traceback
        traceback.print_exc()
    finally:
        csv_file.close()
        client_socket.close()
        print(f'Trace written to {LOG_CSV}', file=sys.stderr)

        # Also write a Parquet copy for compact/typed storage. ~10x smaller
        # than CSV and the canonical format for compare_traces.py.
        try:
            import pandas as pd
            df = pd.read_csv(LOG_CSV)
            df.rename(columns={'simTime': 'Time'}, inplace=True)
            df.to_parquet(LOG_PARQUET, compression='zstd')
            print(f'Trace also written to {LOG_PARQUET}', file=sys.stderr)
        except ImportError:
            print(f'(pandas/pyarrow not installed; Parquet skipped)', file=sys.stderr)


if __name__ == '__main__':
    main()
