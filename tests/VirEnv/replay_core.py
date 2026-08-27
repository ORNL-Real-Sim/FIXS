"""replay_core -- drive VirEnvCore with a hand-authored FIXS trace, no simulator.

Python peer of ``tests/VirEnvCore/replay_core.cpp``: the SAME two scenarios, the
same assertions, and a transcript in the same format, so ``test_core_parity.py``
can diff the two implementations directly. Runs with NO CARLA, NO CarMaker, NO
server, NO TrafficLayer -- it calls :meth:`VirEnvCore.processStep` directly with
``Msg_c.VehDataRecv_um`` pre-filled, which is exactly why the core keeps that
method split out from ``runStep``.

Run it standalone::

    python tests/VirEnv/replay_core.py            # prints the transcripts, exits 0/1
    python tests/VirEnv/replay_core.py --json     # machine-readable, for the parity test
"""

import argparse
import json
import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

from CommonLib.VehDataMsgDefs import VehData                          # noqa: E402
from CommonLib.VirEnv.IVirEnvBackend import EgoState                  # noqa: E402
from CommonLib.VirEnv.VirEnvCore import VirEnvCore                    # noqa: E402
from tests.VirEnv.MockVirEnvBackend import MockVirEnvBackend          # noqa: E402


def veh(vid, vclass, x, y, z, hdg, grade=0.0, lights=0):
    """Build one received record, the peer of replay_core.cpp's ``veh()``."""
    return VehData(id=vid, type='car', vehicleClass=vclass,
                   positionX=x, positionY=y, positionZ=z,
                   heading=hdg, grade=grade, lightIndicators=lights)


def step(core, mock, simTime, onUpdate, vs):
    """Feed one exchange (or one sub-step) and assert the core did not error."""
    mock.mark('step t=%.2f onUpdate=%d' % (simTime, int(onUpdate)))
    core.Msg_c.clearRecvStorage()
    for v in vs:
        core.Msg_c.VehDataRecv_um[v.id] = v
    rc, err = core.processStep(simTime, onUpdate, 1, simTime)
    assert rc == 0, 'processStep returned an error: %s' % err


def hasInOrder(events, needles):
    """Is ``needles`` a subsequence of ``events`` by substring match?

    The same tolerant check ``replay_core.cpp`` uses: it pins the ORDER of the
    decisions that matter without pinning every incidental verb call in between.
    """
    i = 0
    for e in events:
        if i < len(needles) and needles[i] in e:
            i += 1
    return i == len(needles)


def dump(title, events):
    print('--- %s transcript (%d) ---' % (title, len(events)))
    for e in events:
        print('  ' + e)
    sys.stdout.flush()


def scenario1():
    """Carla-style: interpolate=False, 1:1 at the 0.1 s feed."""
    mock = MockVirEnvBackend()
    core = VirEnvCore()
    core.setBackend(mock)
    core.ENABLE_REALSIM = False
    core.SYNCHRONIZE_TRAFFIC_SIGNAL = False
    core.ENABLE_SEPARATE_EGO_TRAFFIC = False
    core.sendEgoFromCore = True
    core.egoId_ = '__none__'
    core.trafficRefreshRate_ = 0.1
    core.interpolateTraffic = False
    core.Msg_c.set_vehicle_message_field(
        ['vehicleClass', 'heading', 'grade', 'lightIndicators'])
    mock.setMockEgo(EgoState(), False)      # readEgoState -> False (clean transcript)

    step(core, mock, 0.00, False, [])                                      # initTrafficPool
    step(core, mock, 0.10, True, [veh('v1', 'passenger', 10, 0, 0.1, 90)])  # v1 appears @ A
    step(core, mock, 0.20, True, [veh('v1', 'passenger', 20, 0, 0.1, 90),
                            veh('v2', 'truck', 5, 5, 0.1, 0)])        # v1 @ B, v2 appears
    step(core, mock, 0.30, True, [veh('v2', 'truck', 6, 5, 0.1, 0)])        # v1 gone

    dump('Carla-style', mock.events())
    assert hasInOrder(mock.events(), [
        'initTrafficPool',
        'spawnVehicle Car(car/passenger) -> 0',
        'setVehiclePose 0 (10.000,0.000,0.100)',   # direct @ A (step 0.1 refresh)
        'spawnVehicle Truck(car/truck)',           # v2 appears (step 0.2 map)
        'setVehiclePose 0 (20.000,0.000,0.100)',   # direct @ B, NO interpolation
        'despawnVehicle 0',                        # v1 destroyed the first absent step
    ]), 'Scenario 1 decisions changed'
    print('Scenario 1 (Carla direct-set, immediate despawn) PASS\n')
    return mock.events()


def scenario2():
    """CarMaker-style: interpolate=True, sub-steps between exchanges."""
    mock = MockVirEnvBackend()
    core = VirEnvCore()
    core.setBackend(mock)
    core.ENABLE_REALSIM = False
    core.SYNCHRONIZE_TRAFFIC_SIGNAL = False
    core.sendEgoFromCore = True
    core.egoId_ = '__none__'
    core.trafficRefreshRate_ = 0.05
    core.interpolateTraffic = True
    core.Msg_c.set_vehicle_message_field(['vehicleClass', 'heading', 'grade'])
    mock.setMockEgo(EgoState(), False)

    step(core, mock, 0.00, False, [])
    step(core, mock, 0.10, True, [veh('v1', 'passenger', 10, 0, 0.1, 90)])  # first sight -> A
    step(core, mock, 0.20, True, [veh('v1', 'passenger', 20, 0, 0.1, 90)])  # prev=A@0.2, next=B@0.3
    step(core, mock, 0.25, False, [])                                       # sub-step -> midpoint

    dump('CarMaker-style', mock.events())
    # f = (0.25 - 0.2) / (0.3 - 0.2) = 0.5 -> midpoint (15, 0, 0.1)
    assert hasInOrder(mock.events(), ['setVehiclePose 0 (15.000,0.000,0.100)']), \
        'Scenario 2 interpolation changed'
    print('Scenario 2 (CarMaker interpolation) PASS\n')
    return mock.events()


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--json', action='store_true',
                    help='emit the transcripts as JSON on stdout (for the parity test)')
    args = ap.parse_args()

    if args.json:
        # Assertions still run; only the human output is suppressed, so --json can
        # never report a transcript from a run that failed its own checks.
        import contextlib
        import io
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            transcripts = {'scenario1': scenario1(), 'scenario2': scenario2()}
        json.dump(transcripts, sys.stdout, indent=1)
        return 0

    scenario1()
    scenario2()
    print('REPLAY PASS: VirEnvCore drives the verbs correctly, SDK-free, no sockets.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
