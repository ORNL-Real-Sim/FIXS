"""import fixs.sumo.traci as traci -- a TraCI script, run on the connection FIXS owns.

The domains, packing, parsers, constants and exceptions are SUMO's own traci; only
the socket under them is replaced. Each call leaves as the (cmdID, varID, objID,
payload) traci already produced, and TrafficLayer runs it on its libtraci connection.

    traci.start(cmd)        reports cmd's -c file as this run's scenario, then connects
    traci.simulationStep()  hands the held tick back to FIXS and waits for the next one
    traci.close()           fixs.close()

Not relayed, because FIXS owns them: load, setOrder, and subscriptions (the FIXS
feed is the subscription). Needs SumoSetup.EnableTraciRelay: true.
"""
import os

import traci as _traci
from traci.connection import Connection as _Connection
from traci.domain import DOMAINS as _DOMAINS
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
}
_SUBSCRIBE = ('subscriptions are not relayed: the FIXS feed is the subscription. Read the '
              'vehicles and fields the config names from fixs.vehicle, which is already here.')


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
            raise FatalTraCIError(_SUBSCRIBE)
        _firstTickForEarlyCall()
        status, body = _relay.request(cmdID, -1 if varID is None else varID,
                                      '' if objID is None else objID,
                                      self._pack(format, *values))
        if status == _relay.TRACI_ERROR:
            raise TraCIException(body.decode('utf8', 'replace'), cmdID, None)
        if status != _relay.TRACI_OK:
            raise FatalTraCIError(body.decode('utf8', 'replace'))
        return _Storage(body)

    def _subscribe(self, *args, **kwargs):
        raise FatalTraCIError(_SUBSCRIBE)

    def _subscribeContext(self, *args, **kwargs):
        raise FatalTraCIError(_SUBSCRIBE)

    def _getSubscriptionResults(self, *args, **kwargs):
        raise FatalTraCIError(_SUBSCRIBE)


_conn = _RelayConnection()
# The real traci's domain singletons, so any `import traci` in this process is relayed too.
for _d in _DOMAINS:
    _d._setConnection(_conn)


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
                return
    try:
        while True:
            if getattr(fixs, '_armed', False):
                fixs.send()
            fixs.recv()
            if not step or fixs.sim.time >= step - 1e-9:
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
