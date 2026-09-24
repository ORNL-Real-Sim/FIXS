"""replay_core -- drive VirEnvCore with a FIXS trace, no simulator.

Python peer of ``tests/VirEnvCore/replay_core.cpp``: the SAME two scripted
scenarios, the same assertions, and a transcript in the same format, so
``test_core_parity.py`` can diff the two implementations directly. Runs with NO
CARLA, NO CarMaker, NO server, NO TrafficLayer -- it calls
:meth:`VirEnvCore.processStep` directly with ``Msg_c.VehDataRecv_um`` pre-filled,
which is exactly why the core keeps that method split out from ``runStep``.

It also replays a RECORDED feed (``tests/VirEnv/record_feed.py``). The scripted
scenarios pin four steps and two vehicles; a recorded corridor drives the same
core through thousands of real appear / disappear / skip events, which is where a
bridge that disagrees about who owns the ego actually diverges.

Run it standalone::

    python tests/VirEnv/replay_core.py                    # scripted, human output
    python tests/VirEnv/replay_core.py --json             # scripted, for the parity test
    python tests/VirEnv/replay_core.py --trace mlk.trace --substeps 4 \\
           --digest-out mlk.py.json                       # recorded corridor
"""

import argparse
import json
import struct
import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

from CommonLib.VehDataMsgDefs import TrafficLightData, VehData          # noqa: E402
from CommonLib.VirEnv.FixsProtocol import kFeedPeriodS                  # noqa: E402
from CommonLib.VirEnv.IVirEnvBackend import EgoState                    # noqa: E402
from CommonLib.VirEnv.VirEnvCore import VirEnvCore                      # noqa: E402
from tests.VirEnv.MockVirEnvBackend import MockVirEnvBackend            # noqa: E402

NL = chr(10)


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


# ---------------------------------------------------------------------------
# The two scripted scenarios (peers of replay_core.cpp)
# ---------------------------------------------------------------------------

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

    step(core, mock, 0.00, False, [])                                       # initTrafficPool
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
    print('Scenario 1 (Carla direct-set, immediate despawn) PASS' + NL)
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
    print('Scenario 2 (CarMaker interpolation) PASS' + NL)
    return mock.events()


# ---------------------------------------------------------------------------
# Recorded-trace replay
# ---------------------------------------------------------------------------

def f32(text):
    """(string) -> float -- the trace decimal as the float32 the wire carried.

    A plain float() would give the nearest DOUBLE to the decimal, which is not
    the float32 the record actually held: the trace prints %.9g, which uniquely
    identifies a float32 but is not its exact binary value. The C++ reader uses
    stof and gets the float32; reading it as a double here would feed the two
    cores inputs that differ in the low bits, and the digests would then differ
    for a reason that has nothing to do with either core.

    This is also what the real pipeline does -- MsgHelper unpacks these fields
    with struct "f" -- so a replay feeds the core exactly what a live run does.
    """
    return struct.unpack("f", struct.pack("f", float(text)))[0]


def readTrace(path):
    """(string) -> list -- the recorded exchanges; see record_feed.py for the format."""
    steps = []
    with open(path, encoding='utf-8') as f:
        for line in f:
            line = line.rstrip(chr(13) + chr(10))
            if not line:
                continue
            parts = line.split(',')
            if parts[0] == 'S':
                steps.append({'t': float(parts[1]), 'state': int(parts[2]),
                              'veh': [], 'tls': []})
            elif parts[0] == 'V':
                steps[-1]['veh'].append(
                    VehData(id=parts[1], type=parts[2], vehicleClass=parts[3],
                            positionX=f32(parts[4]), positionY=f32(parts[5]),
                            positionZ=f32(parts[6]), heading=f32(parts[7]),
                            grade=f32(parts[8]), lightIndicators=int(parts[9])))
            elif parts[0] == 'T':
                # A SUMO state string cannot contain a comma, so the tail is the
                # state even if a junction name somehow did.
                steps[-1]['tls'].append((parts[1], ','.join(parts[2:])))
    return steps


def fnv1a64(text):
    """A 64-bit FNV-1a digest, defined identically in the C++ replay.

    A digest rather than the transcript itself because a 6500-exchange corridor
    trace produces millions of verb lines: comparing one 16-hex digest per step
    finds the FIRST step that differs just as exactly, and a run's output stays
    something a person can read.

    FNV-1a rather than a real hash because it has to be the same eight lines in
    both languages; there is no adversary here, only a diff.
    """
    h = 0xcbf29ce484222325
    for b in text.encode('utf-8'):
        h = ((h ^ b) * 0x100000001b3) & 0xFFFFFFFFFFFFFFFF
    return h


def canonicalise(events, ownerOf):
    """Replace backend handles with the vehicle id that owns them, and sort.

    Two things in a raw transcript carry no meaning and cannot match across the
    two implementations:

    * **Which handle a vehicle got.** The core spawns in the iteration order of
      its received map -- C++ ``unordered_map`` (order unspecified by the
      standard) against a Python insertion-ordered dict -- so when several
      vehicles appear in one exchange they draw pool handles in different orders.
      The handle is opaque either way; what must match is which VEHICLE got which
      pose.
    * **The order of verbs within one step.** Same cause, and they are
      independent writes -- CARLA batches them before they reach the server.

    So every handle -- in the pose, lights and despawn arguments AND in the
    "-> N" a spawn reports -- is rewritten to its owning id, and the step is
    sorted. The spawn line matters as much as the rest: the mock hands handles
    back to a per-class pool on despawn, so the moment two vehicles leave in one
    exchange the two pools are ordered differently, and every LATER spawn reports
    a different number for the same vehicle. What survives is the decision: this
    vehicle, this pose, this step.
    """
    out = []
    for e in events:
        verb, _, rest = e.partition(' ')
        if verb in ('setVehiclePose', 'setVehicleLights', 'despawnVehicle'):
            h, _, tail = rest.partition(' ')
            who = ownerOf.get(int(h), 'h' + h)
            out.append('%s %s%s' % (verb, who, (' ' + tail) if tail else ''))
        elif verb == 'spawnVehicle' and not rest.endswith('kNoHandle') and ' -> ' in rest:
            tag, _, h = rest.rpartition(' -> ')
            out.append('%s %s -> %s' % (verb, tag, ownerOf.get(int(h), 'h' + h)))
        else:
            out.append(e)
    return sorted(out)


def replayTrace(tracePath, substeps, egoId, progressEvery=1000):
    """Drive VirEnvCore through a recorded trace; return the per-step digests."""
    steps = readTrace(tracePath)
    # Pools large enough that the corridor never exhausts them. A pool-full skip is
    # a real decision, but one driven by a pool size CARLA does not have -- it
    # spawns lazily -- so letting it fire here would compare the mock, not the core.
    mock = MockVirEnvBackend(nCars=100000, nTrucks=20000, nBuses=5000)
    core = VirEnvCore()
    core.setBackend(mock)
    core.ENABLE_REALSIM = False
    core.SYNCHRONIZE_TRAFFIC_SIGNAL = True
    core.sendEgoFromCore = False           # the Carla driver owns the send
    core.egoId_ = egoId
    core.interpolateTraffic = substeps > 1
    core.trafficRefreshRate_ = kFeedPeriodS / substeps
    core.Msg_c.set_vehicle_message_field(
        ['id', 'type', 'vehicleClass', 'speed', 'positionX', 'positionY', 'positionZ',
         'heading', 'grade', 'lightIndicators'])
    mock.setMockEgo(EgoState(), False)

    digests = []
    counts = {'spawn': 0, 'despawn': 0, 'pose': 0, 'tls': 0}
    dt = kFeedPeriodS / substeps
    for i, rec in enumerate(steps):
        # A synthetic HOST clock, not the recorded traffic-simulator time: the core
        # tests its exchange boundary on the host clock, which starts at 0 and
        # reaches its first exchange one feed period in. The recorded time rides
        # along as simTimeRecv, exactly as it does on the wire.
        base = i * kFeedPeriodS
        for k in range(substeps):
            onUpdate = (k == 0)
            if onUpdate:
                core.Msg_c.clearRecvStorage()
                for v in rec['veh']:
                    core.Msg_c.VehDataRecv_um[v.id] = v
                for name, state in rec['tls']:
                    core.Msg_c.TlsDataRecv_um[name] = TrafficLightData(0, name, state)
            before = dict((h, vid) for vid, h in core.mappedVehicles().items())
            mock.clear()
            rc, err = core.processStep(base + kFeedPeriodS + k * dt,
                                       onUpdate, rec['state'], rec['t'])
            assert rc == 0, 'processStep failed at t=%s: %s' % (rec['t'], err)
            after = dict((h, vid) for vid, h in core.mappedVehicles().items())
            # A handle spawned this step was in the pool when the step began, so it
            # is not in `before`; a handle despawned this step is. The two sets are
            # disjoint, so before-over-after resolves every event unambiguously.
            owner = dict(after)
            owner.update(before)
            ev = canonicalise(mock.events(), owner)
            for e in ev:
                if e.startswith('spawnVehicle'):
                    counts['spawn'] += 1
                elif e.startswith('despawnVehicle'):
                    counts['despawn'] += 1
                elif e.startswith('setVehiclePose'):
                    counts['pose'] += 1
                elif e.startswith('syncTrafficLight'):
                    counts['tls'] += 1
            digests.append('%016x' % fnv1a64(NL.join(ev)))
        if progressEvery and (i + 1) % progressEvery == 0:
            print('[replay] %d/%d exchanges' % (i + 1, len(steps)))
    return {'exchanges': len(steps), 'substeps': substeps, 'egoId': egoId,
            'counts': counts, 'digests': digests}


def main():
    ap = argparse.ArgumentParser(description=__doc__.split(NL)[0])
    ap.add_argument('--json', action='store_true',
                    help='emit the scripted transcripts as JSON (for the parity test)')
    ap.add_argument('--trace', default=None,
                    help='replay a recorded feed (tests/VirEnv/record_feed.py) instead '
                         'of the scripted scenarios')
    ap.add_argument('--substeps', type=int, default=1,
                    help='host ticks per FIXS exchange; > 1 exercises interpolation')
    ap.add_argument('--ego', default='',
                    help='the id the bridge OWNS and must never spawn (CarlaSetup.EgoId '
                         'when EgoMode >= 1). Empty = the traffic simulator owns it.')
    ap.add_argument('--digest-out', default=None,
                    help='write the per-step digests to this file as JSON')
    args = ap.parse_args()

    if args.trace:
        result = replayTrace(args.trace, max(1, args.substeps), args.ego)
        print('[replay] %(exchanges)d exchanges x %(substeps)d substeps, ego=%(egoId)r'
              % result)
        print('[replay] spawn %(spawn)d  despawn %(despawn)d  pose %(pose)d  tls %(tls)d'
              % result['counts'])
        print('[replay] overall digest %016x' % fnv1a64(NL.join(result['digests'])))
        if args.digest_out:
            with open(args.digest_out, 'w', encoding='utf-8') as f:
                json.dump(result, f, indent=1)
            print('[replay] digests -> %s' % args.digest_out)
        return 0

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
