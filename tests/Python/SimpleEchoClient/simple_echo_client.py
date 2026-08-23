"""
Simple Echo Client for TrafficLayer.exe
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

    # Connect. The endpoint, the message fields and the verbose flag all come
    # from the same config TrafficLayer is running, so there is nothing here to
    # keep in sync by hand.
    fixs.connect(args.config)

    # Counters for the subscription repro (#176): peak vehicle count and all distinct ids seen
    max_vehicles = 0
    distinct_ids = set()

    step_count = 0
    try:
        # recv() takes the next tick, send() answers it. TrafficLayer does not
        # advance until every subscriber has answered, so every tick received
        # gets exactly one send().
        while True:
            sim_time = fixs.recv()
            if sim_time is None:
                break
            step_count += 1

            vehicles = fixs.vehicle.getAll()

            # Track vehicle visibility (after warmup) for the subscription repro
            if step_count > args.warmup:
                if len(vehicles) > max_vehicles:
                    max_vehicles = len(vehicles)
                distinct_ids.update(vehicles)

            # Decide how many vehicles to echo back. In real XIL the client (CarMaker)
            # returns only the ego pose, so TrafficLayer's RECEIVE path is only ever
            # exercised with ~1 vehicle. --max-echo caps the echo to mirror that; echoing
            # ALL received vehicles (max_echo=0) stresses a receive-many path that
            # production never uses and can deadlock the round-trip at higher counts.
            echo_ids = fixs.vehicle.getIDList()
            if args.max_echo > 0:
                echo_ids = echo_ids[:args.max_echo]

            if fixs.verbose:
                print(f'\n--- Step {step_count} | Time: {sim_time:.2f}s ---')
                print(f'Received {len(vehicles)} vehicles:')
                for veh_id, veh_data in vehicles.items():
                    print(f'  Vehicle ID: {veh_id}, Speed: {veh_data.speed:.2f} m/s, '
                          f'Pos: ({veh_data.positionX:.2f}, {veh_data.positionY:.2f})')
            elif step_count % 100 == 0:
                print(f'Step {step_count} | Time: {sim_time:.2f}s | Vehicles: {len(vehicles)}')

            fixs.send(echo_ids)

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
        fixs.close()

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
