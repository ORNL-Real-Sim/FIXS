"""#356 end-to-end: a FIXS client that also speaks TraCI, through FIXS.

Run by run_relay_test.bat, which starts SUMO and TrafficLayer first. Self-checking:
every assertion below runs against a live co-simulation, and the script exits
non-zero on the first failure.

The interesting assertion is CROSS-CHECK: the same quantity is read twice, once from
the FIXS feed (pushed, free) and once through a relayed TraCI getter (a round trip).
They come from the same SUMO over the same connection, so they must agree exactly --
which is what proves the relay is talking to the run FIXS is driving, and not to
something else.
"""

import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..')))
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', 'CommonLib')))

from CommonLib import fixs                      # noqa: E402
import CommonLib.fixs.traci as traci            # noqa: E402

CONFIG = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'config.yaml')

failures = []


def check(name, condition, detail=''):
    print('  %-34s %s %s' % (name, 'OK  ' if condition else 'FAIL', detail))
    if not condition:
        failures.append(name)


def expectRaises(name, exc, fn):
    try:
        fn()
    except exc as e:
        print('  %-34s OK   raised %s' % (name, type(e).__name__))
        return str(e)
    except Exception as e:                        # noqa: BLE001
        check(name, False, 'raised %s, expected %s' % (type(e).__name__, exc.__name__))
        return ''
    check(name, False, 'did not raise')
    return ''


fixs.connect(CONFIG)
print('[relay-client] connected')

tick = 0
try:
    while tick < 60:
        fixs.recv()
        tick += 1

        ego = fixs.vehicle.get('ego')
        if ego is None:
            fixs.send()
            continue

        if tick == 30:
            print('\n[relay-client] tick %d, ego in the feed at %.3f m/s' % (tick, ego.speed))

            # --- the cross-check -------------------------------------------
            relayedSpeed = traci.vehicle.getSpeed('ego')
            check('getSpeed == feed speed', abs(relayedSpeed - ego.speed) < 1e-6,
                  'relayed %.6f, feed %.6f' % (relayedSpeed, ego.speed))

            # --- things the feed cannot answer at all -----------------------
            lanes = traci.lane.getIDList()
            check('lane.getIDList()', len(lanes) >= 4, '%d lanes' % len(lanes))
            links = traci.lane.getLinks('e0_0')
            check('lane.getLinks()', isinstance(links, list))
            route = traci.vehicle.getRoute('ego')
            check('vehicle.getRoute()', tuple(route) == ('e0', 'e1'), str(route))

            # --- a setter that lands, read back through the relay -----------
            traci.vehicle.setParameter('ego', 'fixs356', 'hello')
            check('setParameter/getParameter',
                  traci.vehicle.getParameter('ego', 'fixs356') == 'hello')

            # --- chunking, both directions ----------------------------------
            # MAX_RECORD_SIZE is 8192 and a wire contract shared with dSPACE, so a
            # payload this size has to be split across records and reassembled --
            # in the request AND in the reply. lane.getIDList() on a real network
            # is over that on its own; this makes it deterministic.
            big = 'x' * 20000
            traci.vehicle.setParameter('ego', 'fixs356big', big)
            echoed = traci.vehicle.getParameter('ego', 'fixs356big')
            check('20 KB payload round trip', echoed == big,
                  '%d bytes back' % len(echoed))

            # --- the error path ---------------------------------------------
            msg = expectRaises('getSpeed on unknown vehicle', traci.TraCIException,
                               lambda: traci.vehicle.getSpeed('nosuchveh'))
            check('  ... carries SUMO\'s own message', 'nosuchveh' in msg, msg)

            # --- refusals ----------------------------------------------------
            expectRaises('simulationStep refused', traci.FatalTraCIError,
                         lambda: traci.simulationStep())
            expectRaises('subscribe refused', traci.FatalTraCIError,
                         lambda: traci.vehicle.subscribe('ego', [0x40]))
            expectRaises('close refused via _sendCmd', traci.FatalTraCIError,
                         lambda: traci.vehicle._connection._sendCmd(0x7F, None, None))

            # --- traci.start(): verify, do not launch -------------------------
            version = traci.start(['sumo', '-c', os.path.join(
                os.path.dirname(CONFIG), '..', '..', 'Sumo', 'Probes', 'TraciRelay',
                'net', 'probe.sumocfg')])
            check('traci.start() on the right net', version[0] > 0, str(version))
            expectRaises('traci.start() on another net', traci.FatalTraCIError,
                         lambda: traci.start(['sumo', '-c', 'somewhere_else.sumocfg']))

        if tick == 31:
            # A relayed setter must survive into the next tick, i.e. it really went
            # into SUMO rather than into a buffer somewhere.
            check('parameter survives a tick',
                  traci.vehicle.getParameter('ego', 'fixs356') == 'hello')

            # And the co-simulation must still be healthy afterwards: the feed keeps
            # arriving, which is the thing a relay done wrong would break.
            check('feed still alive after relaying', ego.speed >= 0.0,
                  'ego at %.3f m/s' % ego.speed)

        fixs.send()

except fixs.Shutdown:
    print('[relay-client] TrafficLayer signalled shutdown')
finally:
    fixs.close()

print('\n[relay-client] %s' % ('PASS' if not failures else 'FAIL: ' + ', '.join(failures)))
sys.exit(0 if not failures else 1)
