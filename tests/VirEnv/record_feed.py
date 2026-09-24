"""Record a real FIXS feed to a trace, for replaying through both cores.

The two hand-authored scenarios in ``replay_core`` pin the core's decisions on four
steps and two vehicles. That is enough to catch a broken interpolation or a
mis-ordered spawn, and not nearly enough to catch what #325 says is the expensive
failure: two bridges disagreeing about **who owns the ego**, which shows up on the
tick an id appears, disappears, or is skipped -- events a four-step script does not
contain and a real corridor contains thousands of.

So this attaches to a running TrafficLayer as an ordinary subscriber, records every
exchange verbatim, and writes a trace that ``replay_core.py --trace`` and
``replay_core.exe --trace`` both drive their core through. It needs **no CARLA**: it
stands in for the bridge on the bridge's own port and replies with an empty record
set, which is what a render-only bridge sends anyway (``EnableExternalControl:
false``).

Usage, against the MLK eco-driving stack::

    # start SUMO + TrafficLayer + the controller as usual, then:
    python tests/VirEnv/record_feed.py -f <config.yaml> --port 440 --out mlk.trace

Format
------
Flat and line-oriented rather than JSON, because BOTH replay drivers have to read it
and the C++ one links no JSON parser -- a second format written alongside a first
would be one more thing that can drift. Each exchange is a step line followed by its
records::

    S,<simTime>,<simState>
    V,<id>,<type>,<vehicleClass>,<x>,<y>,<z>,<heading>,<grade>,<lightIndicators>
    T,<name>,<state>

Only the fields VirEnvCore reads are stored: recording whole records would make the
trace several times larger to carry values no core decision depends on. Ids and
names are written as-is; a FIXS id containing a comma would break this, and none
does -- SUMO ids cannot contain one.

Numbers are written at ``%.9g``, which round-trips a float32 exactly. The records
arrive from the decoder as float32, so a replay then feeds the core the SAME bits
the wire carried; rounding them here would make the two cores agree on a number
neither one was actually given.
"""

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

from CommonLib import fixs                                      # noqa: E402

#: Written explicitly rather than left to the platform, so a trace recorded on
#: Windows and one recorded on Linux are the same bytes and the C++ reader (which
#: splits on it) needs no special case. Opening the file newline='' keeps Python
#: from translating it.
NL = chr(10)


def writeStep(f, msgHelper, simState, simTime):
    """Write one exchange, reduced to what the core consumes."""
    f.write('S,%.9g,%d%s' % (simTime, simState, NL))
    for v in msgHelper.VehDataRecv_um.values():
        f.write('V,%s,%s,%s,%.9g,%.9g,%.9g,%.9g,%.9g,%d%s'
                % (v.id, v.type, v.vehicleClass,
                   v.positionX, v.positionY, v.positionZ,
                   v.heading, v.grade, int(v.lightIndicators), NL))
    for t in msgHelper.TlsDataRecv_um.values():
        f.write('T,%s,%s%s' % (t.name, t.state, NL))


def main(argv=None):
    ap = argparse.ArgumentParser(
        description='Record a FIXS feed to a trace for the VirEnvCore parity replay.')
    ap.add_argument('-f', '--file', dest='configPath', default=None,
                    help='the config yaml TrafficLayer is running '
                         '(default: $FIXS_CONFIG_YAML)')
    ap.add_argument('--port', type=int, default=None,
                    help='which VehicleSubscription entry this is (the bridge port)')
    ap.add_argument('--out', required=True, help='trace file to write')
    ap.add_argument('--max-steps', type=int, default=0,
                    help='stop after N exchanges (0 = until TrafficLayer shuts down)')
    ap.add_argument('--probe', default=None,
                    help='report the raw wire fields TrafficLayer sends for this '
                         'id, every --progress exchanges. What the bridge is '
                         'actually GIVEN, before any core or backend touches it.')
    ap.add_argument('--progress', type=int, default=500,
                    help='report every N exchanges (0 = quiet)')
    args = ap.parse_args(argv)

    outDir = os.path.dirname(os.path.abspath(args.out))
    if outDir:
        os.makedirs(outDir, exist_ok=True)

    # role='virenv' for transport(): the trace has to be what the CORE sees, which
    # is the id-keyed Msg_c.VehDataRecv_um, not the view a controller reads. Taking
    # it from the view would re-key it here and hide any keying difference the
    # replay is meant to exercise.
    fixs.connect(args.configPath, port=args.port, role='virenv')
    _sock, msgHelper = fixs.transport()

    steps = 0
    recvSeconds = 0.0
    workSeconds = 0.0
    started = time.monotonic()
    try:
        with open(args.out, 'w', encoding='utf-8', newline='') as f:
            while True:
                _t0 = time.monotonic()
                fixs.recv()
                recvSeconds += time.monotonic() - _t0
                _t0 = time.monotonic()
                writeStep(f, msgHelper, fixs.sim.state, fixs.sim.time)
                # Reply with nothing: a render-only bridge reports no records, and a
                # recorder that reported some would be a participant in the run it
                # is supposed to be observing.
                fixs.send()
                workSeconds += time.monotonic() - _t0
                steps += 1
                if args.probe and args.progress and steps % args.progress == 0:
                    r = msgHelper.VehDataRecv_um.get(args.probe)
                    if r is None:
                        print('[probe] %s not in this exchange' % args.probe)
                    else:
                        print('[probe] %s t=%.1f speed=%.3f speedLimit=%.3f '
                              'speedFreeFlow=%.3f speedDesired=%.3f linkId=%r '
                              'linkIdNext=%r precedingVehicleId=%r'
                              % (r.id, fixs.sim.time, r.speed, r.speedLimit,
                                 r.speedFreeFlow, r.speedDesired, r.linkId,
                                 r.linkIdNext, r.precedingVehicleId))
                if args.progress and steps % args.progress == 0:
                    print('[record] %d exchanges, t=%.1f, %.0f/s'
                          % (steps, fixs.sim.time,
                             steps / max(1e-9, time.monotonic() - started)))
                if args.max_steps and steps >= args.max_steps:
                    print('[record] --max-steps %d reached' % args.max_steps)
                    break
    except fixs.Shutdown:
        print('[record] TrafficLayer ended the run')
    finally:
        fixs.close()

    # The point of this number: it is what a Python FIXS client costs with NO
    # CARLA behind it. If the exchange is slow here too, the Python socket and
    # decode path is the limit; if it is fast, the limit is what the bridge does
    # between exchanges, not how it talks to TrafficLayer.
    if steps:
        _el = time.monotonic() - started
        print('[record] %.1f exchanges/s, %.2f ms/exchange '
              '(recv %.2f ms, decode+reply %.2f ms)'
              % (steps / _el, 1000.0 * _el / steps,
                 1000.0 * recvSeconds / steps, 1000.0 * workSeconds / steps))
    print('[record] %d exchanges -> %s (%.1f KB)'
          % (steps, args.out, os.path.getsize(args.out) / 1024.0))
    return 0 if steps else 1


if __name__ == '__main__':
    sys.exit(main())
