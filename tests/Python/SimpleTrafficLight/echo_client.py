"""
Simple Echo Client for TrafficLayer.exe - Debug Test
Receives VehicleData messages and echoes them back to the server
"""

import argparse
import os
import pathlib
import sys

# One line of bootstrap, then one import. The bootstrap exists only because this
# script lives inside the FIXS checkout, where nothing has put the repo root on
# the path; a deployed application gets `import fixs` with no bootstrap at all
# (#316, and #313 for where the package lands in the bundle).
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[3]))

from CommonLib import fixs

# #177 verification: always print the ego's NEXT signal each step so you can
# eyeball it live against sumo-gui -- the id, the colour, the distance counting
# down, and the lane-specific head index. This is what Phase 2's cached/topology
# reconstruction must keep byte-identical to getNextTLS.
_COLOR = {-1: '(none)', 0: '?', 1: 'RED', 2: 'YELLOW', 3: 'GREEN',
          4: 'RED-YELLOW', 5: 'off-blink', 6: 'OFF', 7: 'stop'}

EGO_ID = 'egoCm123'


def parse_args():
    p = argparse.ArgumentParser(description='Traffic-light echo client for TrafficLayer.exe')
    p.add_argument('--verbose', action='store_true',
                   help='Print full detail every step. Separate from the config\'s '
                        'EnableVerboseLog, which controls FIXS\'s own hex-buffer dumps.')
    return p.parse_args()


def main():
    args = parse_args()
    fixs.connect(os.path.join(os.path.dirname(__file__), 'config.yaml'))

    step_count = 0
    try:
        # recv() takes the next tick, send() answers it. recv() returns None at
        # shutdown -- which this client previously had no way to notice.
        while True:
            sim_time = fixs.recv()
            if sim_time is None:
                break
            step_count += 1

            veh_ids = fixs.vehicle.getIDList()
            ego = fixs.vehicle.get(EGO_ID)

            if ego is not None:
                print(f't={sim_time:7.2f}s  ego next TLS: {ego.signalLightId or "(none)":>10}'
                      f'  {_COLOR.get(ego.signalLightColor, ego.signalLightColor):>10}'
                      f'  dist={ego.signalLightDistance:8.2f} m  head={ego.signalLightHeadId}')

            if args.verbose:
                print(f'\n--- Step {step_count} | Time: {sim_time:.2f}s ---')
                print(f'Received {len(veh_ids)} vehicles:')
                if ego is not None:
                    print(f'  Vehicle ID: {EGO_ID}, Speed: {ego.speed:.2f} m/s, '
                          f'Pos: ({ego.positionX:.2f}, {ego.positionY:.2f})')
                    print(f'    Traffic Light - ID: {ego.signalLightId}, '
                          f'Head: {ego.signalLightHeadId}, '
                          f'Distance: {ego.signalLightDistance:.2f} m, '
                          f'Color: {ego.signalLightColor}')
                print(f'Echoed {len(veh_ids)} vehicles back to server')
            elif step_count % 100 == 0:
                print(f'Step {step_count} | Time: {sim_time:.2f}s | Vehicles: {len(veh_ids)}')

            # Echo the whole feed back, which is what this client is for.
            fixs.send(veh_ids)

    except KeyboardInterrupt:
        print('\nShutting down client...', file=sys.stderr)
    except Exception as e:
        print(f'\nError occurred: {e}', file=sys.stderr)
        import traceback
        traceback.print_exc()
    finally:
        print('Closing connection...', file=sys.stderr)
        fixs.close()


if __name__ == '__main__':
    main()
