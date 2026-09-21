"""fixs.traci -- your traci calls, executed on the connection FIXS owns (#356).

    -import traci
    +import fixs.traci as traci

Everything here is SUMO's own traci: the domains, the serialization, the response
parsers, the constants, the exceptions. Only the socket underneath is replaced. A
call goes out as the four values traci already produces -- command id, variable id,
object id and the bytes traci packed -- and TrafficLayer executes it on the single
libtraci connection it owns. No second TraCI client, no ``--num-clients``, no
stepping contract to get wrong.

Why this works at all: every one of traci's ~800 domain functions funnels through
``Connection._sendCmd(cmdID, varID, objID, format, *values)``, and the arguments are
already serialized by ``_pack`` before they reach it. Replacing ``_sendCmd`` taps a
pipeline that has already run, so neither side has to enumerate the TraCI API. The
payload is opaque from here to SUMO and back.

Not everything is relayed. FIXS owns the clock and the session, and the feed already
IS a subscription; those calls are refused with a message naming what to use instead.

Requires ``fixs.connect()`` first, and calls are legal only between ``fixs.recv()``
and ``fixs.send()`` -- the window in which a controller runs, and the window in which
TrafficLayer is listening.
"""

import traci as _traci
from traci.connection import Connection as _Connection
from traci.domain import DOMAINS as _DOMAINS
from traci.exceptions import FatalTraCIError, TraCIException
from traci.storage import Storage as _Storage

from . import _relay

__all__ = ['start', 'init', 'close', 'getVersion']

# ---------------------------------------------------------------------------
# What FIXS will not relay
# ---------------------------------------------------------------------------
# Refused in two places on purpose. Here, so an adopter gets a sentence they can act
# on; and again in TraciRelay.cpp, so nothing slips through from a client FIXS did
# not write. These are not defensive: a relayed CMD_SIMSTEP was measured moving the
# clock behind TrafficLayer's back (tests/Sumo/Probes/TraciRelay/FINDINGS.md).
_REFUSED = {
    0x02: 'traci.simulationStep() is not relayed: TrafficLayer owns the clock. '
          'Delete the call -- fixs.recv() and fixs.send() advance the co-simulation.',
    0x7F: 'traci.close() is not relayed: TrafficLayer owns the session. '
          'Use fixs.close().',
    0x03: 'traci.setOrder() is not relayed: FIXS is the only TraCI client, so client '
          'ordering is meaningless here.',
    0x01: 'traci.load() is not relayed: it would reload the network under a running '
          'co-simulation.',
}

_SUBSCRIBE_MSG = (
    'subscriptions are not relayed: the FIXS feed IS the subscription. The vehicles '
    'and fields your config named arrive on every fixs.recv() -- read them with '
    'fixs.vehicle.get(id) / fixs.vehicle.getAll(), which cost nothing per call '
    'because the data is already here.'
)


def _isSubscribe(cmdID, varID):
    # A subscription's varID is a PAIR of doubles (begin, end), which the relay's
    # four-value contract cannot carry at all -- so shape alone identifies it. The
    # command ranges are checked too, for the subscription commands that reach here
    # some other way. CONTEXT: 0x04-0x0b, 0x80-0x8f. VARIABLE: 0x54-0x5b, 0xd0-0xdf.
    if isinstance(varID, tuple):
        return True
    return (0x04 <= cmdID <= 0x0b or 0x80 <= cmdID <= 0x8f or
            0x54 <= cmdID <= 0x5b or 0xd0 <= cmdID <= 0xdf)


class _RelayConnection(object):
    """What traci's domains think is their socket connection.

    ``_pack`` is reused verbatim, unbound, from the real Connection: this shim
    serializes nothing itself. That is the whole point -- traci's own packer already
    covers every TraCI type there is, and the bytes it produces drop into SUMO
    unchanged (verified byte for byte in the #356 probe).
    """

    _pack = _Connection._pack

    def _sendCmd(self, cmdID, varID, objID, format="", *values):
        if cmdID in _REFUSED:
            raise FatalTraCIError(_REFUSED[cmdID])
        if _isSubscribe(cmdID, varID):
            raise FatalTraCIError(_SUBSCRIBE_MSG)

        payload = self._pack(format, *values)
        status, body = _relay.request(cmdID, -1 if varID is None else varID,
                                      '' if objID is None else objID, payload)
        if status == _relay.TRACI_ERROR:
            # SUMO's own message, relayed verbatim, re-raised as the exception an
            # adopter's `except traci.TraCIException` already catches.
            raise TraCIException(body.decode('utf8', 'replace'), cmdID, None)
        if status != _relay.TRACI_OK:
            raise FatalTraCIError(body.decode('utf8', 'replace'))

        # doCommand was called with expectedType = -1, so these bytes start exactly
        # where Domain._getCmd starts reading. No re-framing, stock parsers.
        return _Storage(body)

    def _subscribe(self, *args, **kwargs):
        raise FatalTraCIError(_SUBSCRIBE_MSG)

    def _subscribeContext(self, *args, **kwargs):
        raise FatalTraCIError(_SUBSCRIBE_MSG)

    def _getSubscriptionResults(self, *args, **kwargs):
        raise FatalTraCIError(_SUBSCRIBE_MSG)


_conn = _RelayConnection()

# Bind the REAL traci module's domain singletons -- the same thing traci.switch()
# does. One consequence worth knowing: a third-party library that does its own
# `import traci` is relayed too, because there is only one traci.vehicle in the
# process. That is usually what you want; it is documented because it can surprise.
for _d in _DOMAINS:
    _d._setConnection(_conn)


# ---------------------------------------------------------------------------
# The handful of module-level functions that are not domain calls
# ---------------------------------------------------------------------------

def getVersion():
    """(apiVersion, sumoVersion) from the SUMO that TrafficLayer is driving."""
    status, body = _relay.request(0x00, -1, '', b'')
    if status != _relay.TRACI_OK:
        raise FatalTraCIError(body.decode('utf8', 'replace'))
    result = _Storage(body)
    result.readLength()
    response = result.read("!B")[0]
    if response != 0x00:
        raise FatalTraCIError('unexpected answer %s to CMD_GETVERSION' % response)
    return result.readInt(), result.readString()


def start(cmd=None, port=None, numRetries=None, label="default", verbose=False,
          traceFile=None, traceGetters=True, stdout=None, doSwitch=True):
    """Not a launch: a check that you and FIXS mean the same scenario.

    An adopter's script opens with ``traci.start(['sumo-gui', '-c', 'x.sumocfg'])``.
    FIXS cannot honour that -- TrafficLayer already launched SUMO -- but silently
    treating it as a no-op would let the script believe it is driving the network it
    named while TrafficLayer runs another one. So the arguments are verified against
    the config FIXS actually loaded, and a disagreement is raised HERE rather than
    surfacing fifty ticks later as vehicles in the wrong place.
    """
    if not _relay.connected():
        raise FatalTraCIError(
            'traci.start() cannot launch SUMO: TrafficLayer owns it. Call '
            "fixs.connect('config.yaml') first -- then this line is a no-op.")
    _assertSameScenario(cmd)
    return getVersion()


def init(port=8813, numRetries=None, host="localhost", label="default", proc=None,
         doSwitch=True, traceFile=None, traceGetters=True):
    """Same as start(): FIXS is already connected, so this only verifies."""
    if not _relay.connected():
        raise FatalTraCIError(
            'traci.init() cannot connect: FIXS owns the TraCI connection. Call '
            "fixs.connect('config.yaml') first -- then this line is a no-op.")
    return getVersion()


def close(wait=True):
    """No-op: the run ends with fixs.close(), or when TrafficLayer signals shutdown."""
    return None


def load(args):
    raise FatalTraCIError(_REFUSED[0x01])


def simulationStep(step=0.):
    raise FatalTraCIError(_REFUSED[0x02])


def _assertSameScenario(cmd):
    """Raise if a traci.start() command line contradicts the FIXS config."""
    if not cmd:
        return
    import os
    import sys as _sys

    named = None
    for i, arg in enumerate(cmd):
        if arg in ('-c', '--configuration-file') and i + 1 < len(cmd):
            named = cmd[i + 1]
        elif arg.endswith('.sumocfg'):
            named = arg
    if not named:
        return

    fixs = _sys.modules[_relay.__package__]
    configured = getattr(fixs, '_sumoConfigFile', None)
    if not configured:
        return
    if os.path.normcase(os.path.abspath(named)) != os.path.normcase(os.path.abspath(configured)):
        raise FatalTraCIError(
            'traci.start() names %s, but TrafficLayer is running %s. One of the two '
            'is wrong -- the co-simulation would otherwise run the second while your '
            'script reasons about the first.' % (named, configured))


def __getattr__(name):
    """Everything not overridden above is the real traci, unchanged."""
    return getattr(_traci, name)
