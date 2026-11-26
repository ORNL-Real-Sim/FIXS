"""import fixs.sumo.traci as traci -- a TraCI script, run on the connection FIXS owns.

The domains, packing, parsers, constants and exceptions are SUMO's own traci; only
the socket under them is replaced. Each call leaves as the (cmdID, varID, objID,
payload) traci already produced, and TrafficLayer runs it on its libtraci connection.

    traci.start(cmd)        reports cmd's -c file as this run's scenario, then connects
    traci.simulationStep()  hands the held tick back to FIXS and waits for the next one
    traci.close()           fixs.close()

subscribe() places a SUMO subscription through TrafficLayer, and each
simulationStep() brings its results back in one call, as native traci's step does.
Not relayed: load and setOrder, which FIXS owns, and context subscriptions.
Needs SumoSetup.EnableTraciRelay: true.
"""
import struct

import os

import traci as _traci
import traci.constants as tc
from traci.connection import Connection as _Connection
from traci.domain import DOMAINS as _DOMAINS
from traci.domain import SubscriptionResults as _SubscriptionResults
from traci.exceptions import FatalTraCIError, TraCIException
from traci.storage import Storage as _Storage

from .. import _relay
from . import build as _build

_REFUSED = {
    0x7F: 'close() is not relayed: use traci.close() or fixs.close().',
    0x03: 'setOrder() is not relayed: FIXS is the only TraCI client.',
    0x01: 'load() is not relayed: it would reload the network under a running co-simulation.',
    0x02: 'a raw CMD_SIMSTEP is not relayed: TrafficLayer owns the clock. '
          'traci.simulationStep() advances it through FIXS.',
    0x7E: 'subscription filters are not relayed: they apply to context subscriptions.',
}
_CONTEXT = ('context subscriptions are not relayed: subscribe each object with subscribe(), '
            'or read the vehicles the config names from fixs.vehicle.')
_RAW_SUBSCRIBE = "a raw subscription command is not relayed: use the domain's subscribe()."


def _fixs():
    return _relay._fixs()


def _isSubscribe(cmdID, varID):
    # A subscription's varID is a (begin, end) pair, which the relay cannot carry.
    if isinstance(varID, tuple):
        return True
    return (0x04 <= cmdID <= 0x0b or 0x80 <= cmdID <= 0x8f or
            0x54 <= cmdID <= 0x5b or 0xd0 <= cmdID <= 0xdf)


class _RelayConnection(object):
    """Stands in for traci's socket connection; _pack is traci's own."""

    _pack = _Connection._pack

    def _sendCmd(self, cmdID, varID, objID, format="", *values):
        if cmdID in _REFUSED:
            raise FatalTraCIError(_REFUSED[cmdID])
        if _isSubscribe(cmdID, varID):
            raise FatalTraCIError(_RAW_SUBSCRIBE)
        _firstTickForEarlyCall()
        status, body = _relay.request(cmdID, -1 if varID is None else varID,
                                      '' if objID is None else objID,
                                      self._pack(format, *values))
        if status == _relay.TRACI_ERROR:
            raise TraCIException(body.decode('utf8', 'replace'), cmdID, None)
        if status != _relay.TRACI_OK:
            raise FatalTraCIError(body.decode('utf8', 'replace'))
        return _Storage(body)

    _readSubscription = _Connection._readSubscription
    _getSubscriptionResults = _Connection._getSubscriptionResults

    def __init__(self):
        self._subscriptionMapping = {}
        self._subscribed = False

    def _subscribe(self, cmdID, begin, end, objID, varIDs, parameters):
        payload = struct.pack("!ddB", begin, end, len(varIDs))
        for v in varIDs:
            param = parameters.get(v) if parameters is not None else None
            payload += struct.pack("!BB", v, param is not None)
            if param is not None:
                payload += self._pack(*_paramFormat(param))
        status, body = self._request(cmdID, _relay.TRACI_SUBSCRIPTION_VAR, objID, payload)
        self._subscribed = True
        if varIDs:
            objectID, response = self._readSubscription(_Storage(body))
            if response - cmdID != 16 or objectID != objID:
                raise FatalTraCIError("Received answer %02x,%s for subscription command %02x,%s." % (
                    response, objectID, cmdID, objID))

    def _subscribeContext(self, *args, **kwargs):
        raise FatalTraCIError(_CONTEXT)

    def _refreshSubscriptions(self):
        """This tick's subscription results, read the way native traci reads its step answer."""
        for results in self._subscriptionMapping.values():
            results.reset()
        if not self._subscribed:
            return
        status, body = self._request(tc.CMD_SIMSTEP, _relay.TRACI_SUBSCRIPTION_VAR, '', b'')
        result = _Storage(body)
        for _ in range(result.readInt()):
            self._readSubscription(result)

    def _request(self, cmdID, varID, objID, payload):
        _firstTickForEarlyCall()
        status, body = _relay.request(cmdID, varID, objID, payload)
        if status == _relay.TRACI_ERROR:
            raise TraCIException(body.decode('utf8', 'replace'), cmdID, None)
        if status != _relay.TRACI_OK:
            raise FatalTraCIError(body.decode('utf8', 'replace'))
        return status, body


def _paramFormat(param):
    """A subscription parameter as (format, value): one typed int, double or string."""
    if isinstance(param, tuple):
        if len(param) != 2 or param[0] not in ("i", "d", "s"):
            raise FatalTraCIError("subscription parameter %r is not relayed: one int, double "
                                  "or string per variable is." % (param,))
        return param
    if isinstance(param, int):
        return "i", param
    if isinstance(param, float):
        return "d", param
    return "s", param


_conn = _RelayConnection()
# The real traci's domain singletons, so any `import traci` in this process is relayed too.
for _d in _DOMAINS:
    _d._setConnection(_conn)
    # As traci's Domain._register does for a connection of its own.
    _results = _SubscriptionResults(_d._retValFunc)
    for _id in (_d._subscribeResponseID, _d._contextResponseID, _d._cmdGetID):
        _conn._subscriptionMapping[_id] = _results


# True from start() until the first simulationStep(): a call in that window pulls FIXS's
# first tick early, and that step then consumes it instead of advancing again.
_beforeFirstStep = False
_firstTickPulled = False


def _firstTickForEarlyCall():
    global _firstTickPulled
    fixs = _fixs()
    if _beforeFirstStep and not _firstTickPulled and not getattr(fixs, '_armed', False):
        try:
            fixs.recv()
        except fixs.Shutdown:
            raise FatalTraCIError("Connection closed by SUMO.")
        _firstTickPulled = True


def _connect():
    global _beforeFirstStep, _firstTickPulled
    fixs = _fixs()
    if getattr(fixs, '_sock', None) is None:
        fixs.connect()
    _beforeFirstStep, _firstTickPulled = True, False


def start(cmd, port=None, numRetries=None, label="default", verbose=False,
          traceFile=None, traceGetters=True, stdout=None, doSwitch=True):
    """Report cmd's scenario to FIXS and connect. SUMO itself is started by FIXS."""
    cfg = _configFile(cmd)
    if cfg:
        _build._report(cfg)
    _connect()
    return _startedVersion()


def init(port=8813, numRetries=None, host="localhost", label="default", proc=None,
         doSwitch=True, traceFile=None, traceGetters=True):
    """Connect to FIXS, which is already driving SUMO."""
    _connect()
    return _startedVersion()


def _startedVersion():
    # Answered without a round trip, so start() does not wait for FIXS's first tick.
    return _traci.constants.TRACI_VERSION, "SUMO via FIXS"


def simulationStep(step=0.):
    """Hand the held tick back to FIXS and wait for the next one, or until `step` seconds."""
    global _beforeFirstStep, _firstTickPulled
    fixs = _fixs()
    if _beforeFirstStep:
        _beforeFirstStep = False
        if _firstTickPulled:
            _firstTickPulled = False
            if not step or fixs.sim.time >= step - 1e-9:
                _conn._refreshSubscriptions()
                return
    try:
        while True:
            if getattr(fixs, '_armed', False):
                fixs.send()
            fixs.recv()
            if not step or fixs.sim.time >= step - 1e-9:
                _conn._refreshSubscriptions()
                return
    except fixs.Shutdown:
        # What native traci raises when SUMO ends the run.
        raise FatalTraCIError("Connection closed by SUMO.")


def close(wait=True):
    _fixs().close()


def getVersion():
    _firstTickForEarlyCall()
    status, body = _relay.request(0x00, -1, '', b'')
    if status != _relay.TRACI_OK:
        raise FatalTraCIError(body.decode('utf8', 'replace'))
    result = _Storage(body)
    result.readLength()
    if result.read("!B")[0] != 0x00:
        raise FatalTraCIError('unexpected answer to CMD_GETVERSION')
    return result.readInt(), result.readString()


def load(args):
    raise FatalTraCIError(_REFUSED[0x01])


def isLibsumo():
    return False


def _configFile(cmd):
    if not cmd:
        return None
    for i, arg in enumerate(cmd):
        if arg in ('-c', '--configuration-file') and i + 1 < len(cmd):
            return os.path.abspath(cmd[i + 1])
    for arg in cmd:
        if str(arg).endswith('.sumocfg'):
            return os.path.abspath(arg)
    return None


def __getattr__(name):
    return getattr(_traci, name)
