"""
Simple Echo Client for TrafficLayer.exe - Debug Test
Receives VehicleData messages and echoes them back to the server
"""

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


def main():
    fixs.connect(os.path.join(os.path.dirname(__file__), 'config.yaml'), ego=EGO_ID)

    step_count = 0
    try:
        # One iteration is one tick. The reply is sent for us at the end of each
        # iteration, and the loop ends when TrafficLayer signals shutdown --
        # which this client previously had no way to notice.
        for sim_time in fixs.steps():
            step_count += 1

            if fixs.ego is not None:
                print(f't={sim_time:7.2f}s  ego next TLS: {fixs.ego.signalLightId or "(none)":>10}'
                      f'  {_COLOR.get(fixs.ego.signalLightColor, fixs.ego.signalLightColor):>10}'
                      f'  dist={fixs.ego.signalLightDistance:8.2f} m  head={fixs.ego.signalLightHeadId}')

            # Echo the whole feed back, which is what this client is for.
            fixs.echo_all()

            if fixs.verbose:
                print(f'\n--- Step {step_count} | Time: {sim_time:.2f}s | State: {fixs.state} ---')
                print(f'Received {len(fixs.vehicle)} vehicles:')
                if fixs.ego is not None:
                    print(f'  Vehicle ID: {fixs.ego.id.strip()}, Speed: {fixs.ego.speed:.2f} m/s, '
                          f'Pos: ({fixs.ego.positionX:.2f}, {fixs.ego.positionY:.2f})')
                    print(f'    Traffic Light - ID: {fixs.ego.signalLightId}, '
                          f'Head: {fixs.ego.signalLightHeadId}, '
                          f'Distance: {fixs.ego.signalLightDistance:.2f} m, '
                          f'Color: {fixs.ego.signalLightColor}')
                print(f'Echoed {len(fixs.vehicle)} vehicles back to server')
            elif step_count % 100 == 0:
                print(f'Step {step_count} | Time: {sim_time:.2f}s | Vehicles: {len(fixs.vehicle)}')

    except KeyboardInterrupt:
        print('\nShutting down client...', file=sys.stderr)
    except Exception as e:
        print(f'\nError occurred: {e}', file=sys.stderr)
        import traceback
        traceback.print_exc()
    finally:
        print('Closing connection...', file=sys.stderr)


if __name__ == '__main__':
    main()
