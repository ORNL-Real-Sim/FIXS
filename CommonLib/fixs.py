"""FIXS client API -- one import, no hand-written socket code (#316).

A Python application talks to TrafficLayer like this::

    from CommonLib import fixs          # or `import fixs` once it is on the path

    fixs.connect('config.yaml')
    while True:
        simTime = fixs.sync()
        if simTime is None:                       # TrafficLayer has shut down
            break
        ego = fixs.ego.getAll().get('ego')
        if ego is not None:
            fixs.vehicle.setSpeedDesired('ego', plan(ego.speed))

Everything the co-simulation protocol requires -- connecting and retrying,
framing, replying exactly once per tick, replying even when there is nothing to
say, honouring the shutdown signal, clearing state between ticks -- is handled
here. None of it is the application's to remember.

This module is a facade over the existing helpers, which are unchanged:
ConfigHelper reads the yaml, MsgHelper packs and unpacks, SocketHelper frames.
Scripts that use those directly keep working exactly as before.


Why this looks like TraCI, and where it deliberately does not
-------------------------------------------------------------
The familiar half is borrowed on purpose: ``fixs.vehicle`` / ``fixs.trafficlight``
namespaces, ``getIDList()``, ``getAll()``, ``set<Field>(id, value)``.

The part that is NOT copied is TraCI's one-getter-per-field style. Each TraCI
getter is a separate request to SUMO -- that is why ``getSpeed`` and
``getPosition`` are distinct methods. FIXS is push-based: TrafficLayer sends one
message per tick holding exactly the vehicles the subscription selected and
exactly the fields ``VehicleMessageField`` declared, so by the time your code
runs the data is already in memory. The closest TraCI analogue is therefore not
its getters but its SUBSCRIPTIONS -- ``subscribe()`` plus
``getAllSubscriptionResults()`` -- which is the shape used here: declare fields
in the yaml, receive them in bulk, read them off a record.

``fixs.sync()`` is named for the same reason. TraCI's ``simulationStep()``
advances the simulator's clock; this does not. TrafficLayer owns the clock. One
call sends the commands you set and waits for the next tick to arrive.

Views are rebound on every ``sync()``:

    fixs.vehicle        every vehicle in this tick's feed
    fixs.ego            only the ids this client declared (see connect(ego=...))
    fixs.trafficlight   this tick's signal states

Holding a record across ticks is safe: every tick decodes fresh objects, so
``previous = fixs.vehicle.getAll()`` keeps that tick's values rather than
aliasing something about to be overwritten.
"""

import atexit
import os
import socket
import time as _time
import typing

from CommonLib.ConfigHelper import ConfigHelper
from CommonLib.MsgHelper import MsgHelper
from CommonLib.SocketHelper import SocketHelper
from CommonLib.VehDataMsgDefs import VehData

__all__ = [
    'connect', 'sync', 'close', 'echoAll', 'getFields', 'getTime',
    'vehicle', 'ego', 'trafficlight', 'verbose',
    'Vehicle', 'FixsError', 'NotConnected', 'FieldsMissing',
]


class FixsError(Exception):
    """Base class for every error this module raises."""


class NotConnected(FixsError):
    """Raised when the API is used before connect() or after close()."""


class FieldsMissing(FixsError):
    """Raised by connect(require=...) when the config cannot deliver a field.

    Raised at startup rather than letting the application discover it per tick.
    An omitted field and a genuine zero are the same bytes on the wire, so a
    controller planning against a field the config never declared plans against
    0.0 and looks like it is working -- which is how `speedFreeFlow` (free-flow
    desire) silently degrades into `speedLimit` (posted limit).
    """


# ---------------------------------------------------------------------------
# Records
# ---------------------------------------------------------------------------

class Vehicle(VehData):
    """One vehicle, for one tick. Read-only.

    Every field can be read; none can be assigned. Commands go through the
    setters on :data:`fixs.vehicle`, which is the only way to put anything on
    the wire. Two reasons for the asymmetry:

    * **A local calculation must not become a command.** Converting units in
      place (``veh.speed *= 2.23694`` -- the eco controller does exactly this
      conversion on its dataframe) would otherwise be indistinguishable from an
      instruction to SUMO.

    * **You cannot fabricate a reply record.** A fresh ``VehData`` leaves every
      unset field at its dataclass default -- position (0,0,0), heading 0 -- and
      TrafficLayer publishes a client's return to later clients by REPLACING the
      whole record for that id, not by merging fields. That is how a controller
      on a low port put the ego at the world origin in the CARLA bridge's copy
      (ORNL-Real-Sim/FIXS_Applications#25). Here the only record that can be
      returned is the one that arrived.

    This is a subclass of ``VehData`` with the same fields, so
    ``isinstance(rec, VehData)`` still holds and adding a field to
    ``VehDataMsgDefs.py`` surfaces it here automatically.
    """

    #: The wire's command channel: the only fields a client may write.
    COMMAND_FIELDS = frozenset({
        'speedDesired',
        'accelerationDesired',
        'steerAngleDesired',
        'acceleratorPedalDesired',
        'brakePedalDesired',
    })

    def __setattr__(self, name, value):
        if name.startswith('_'):
            object.__setattr__(self, name, value)
            return
        if name in self.COMMAND_FIELDS:
            raise AttributeError(
                f"records are read-only; use fixs.vehicle.set{name[0].upper()}"
                f"{name[1:]}(vehID, value) to command '{name}'"
            )
        raise AttributeError(
            f"'{name}' is measured data from the traffic simulator and cannot "
            f"be assigned. Records are read-only."
        )


# ---------------------------------------------------------------------------
# Views -- this tick's records of one kind, addressed by id
# ---------------------------------------------------------------------------

class _View:
    """Records of one kind for the current tick.

    ``getAll()`` is the FIXS equivalent of TraCI's
    ``getAllSubscriptionResults()``: the whole tick in one call, keyed by id.
    """

    _KIND = 'record'

    def __init__(self, records=(), key=lambda r: r.id.strip()):
        self._byId = {key(r): r for r in records}

    def getIDList(self):
        """() -> list[string]"""
        return list(self._byId)

    def getAll(self):
        """() -> dict[string, record] -- every record in this tick's feed."""
        return dict(self._byId)

    def get(self, objectIDs):
        """(list[string]) -> dict[string, record]

        Ids that are not in this tick's feed are simply absent from the result,
        which is also how TraCI subscription results behave.
        """
        if isinstance(objectIDs, str):
            raise TypeError(
                f'get() takes a list of ids, not a single string. '
                f'Use get([{objectIDs!r}]).'
            )
        return {i: self._byId[i] for i in objectIDs if i in self._byId}

    def __len__(self):
        return len(self._byId)

    def __repr__(self):
        return (f'<{len(self._byId)} {self._KIND}(s): '
                f'{", ".join(sorted(self._byId)) or "none"}>')


class _VehicleView(_View):
    """Vehicles, plus the command channel back to TrafficLayer."""

    _KIND = 'vehicle'

    def _command(self, vehID, field, value):
        record = self._byId.get(vehID)
        if record is None:
            # Commanding a vehicle that is not in this tick is a bug, not a
            # state to tolerate: a typo would otherwise do nothing, silently,
            # for the whole run.
            raise KeyError(
                f"cannot command '{vehID}': not in this tick's feed. Present: "
                f"{', '.join(sorted(self._byId)) or '(none)'}"
            )
        object.__setattr__(record, field, value)
        _commanded.add(vehID)

    def setSpeedDesired(self, vehID, value):
        """(string, double) -> None"""
        self._command(vehID, 'speedDesired', value)

    def setAccelerationDesired(self, vehID, value):
        """(string, double) -> None"""
        self._command(vehID, 'accelerationDesired', value)

    def setSteerAngleDesired(self, vehID, value):
        """(string, double) -> None -- desired front road-wheel angle, rad."""
        self._command(vehID, 'steerAngleDesired', value)

    def setAcceleratorPedalDesired(self, vehID, value):
        """(string, double) -> None -- pedal position in [0, 1]."""
        self._command(vehID, 'acceleratorPedalDesired', value)

    def setBrakePedalDesired(self, vehID, value):
        """(string, double) -> None -- pedal position in [0, 1]."""
        self._command(vehID, 'brakePedalDesired', value)


class _TrafficLightView(_View):
    _KIND = 'traffic light'


# ---------------------------------------------------------------------------
# Module state
# ---------------------------------------------------------------------------

vehicle: _VehicleView = _VehicleView()
ego: _VehicleView = _VehicleView()
trafficlight: _TrafficLightView = _TrafficLightView()
verbose: bool = False           # SimulationSetup.EnableVerboseLog, from the config

_helper: typing.Optional[SocketHelper] = None
_sock: typing.Optional[socket.socket] = None
_egoIds: typing.List[str] = []
_commanded: typing.Set[str] = set()
_echoLimit: typing.Optional[int] = None

# The tick currently held awaiting its reply.
_armed: bool = False
_received: typing.List[Vehicle] = []
_simState: int = 0
_simTime: float = 0.0


def _requireConnection():
    if _helper is None or _sock is None:
        raise NotConnected('call fixs.connect(...) first')


# ---------------------------------------------------------------------------
# Connecting
# ---------------------------------------------------------------------------

def connect(configPath=None, *, port=None, host=None, ego=None, require=(),
            connectTimeout=None, recvTimeout=None):
    """(string, ...) -> (string, integer) -- connect and return the endpoint.

    :param configPath: the config yaml TrafficLayer is running. Defaults to
        ``$FIXS_CONFIG_YAML``, which run_cosim sets for exactly this purpose:
        the wire format IS ``SimulationSetup.VehicleMessageField``, so two yamls
        that disagree decode the same bytes differently.
    :param port: which ``ApplicationSetup.VehicleSubscription`` entry is this
        client's. Required only when the config declares more than one, and
        then it is required rather than guessed -- silently taking entry 0 is
        how two clients end up sharing one endpoint.
    :param host: override the address from the config.
    :param ego: id, or list of ids, this client controls, exposed as
        ``fixs.ego``. Defaults to the ``attribute.id`` list of the selected
        subscription, so the yaml does not have to be restated in code.
    :param require: field names that must be on the wire. Raises
        :class:`FieldsMissing` here rather than degrading silently per tick.
    :param connectTimeout: seconds to keep retrying the connect; ``None``
        retries forever, which is right under a supervisor (run_cosim) that
        stops the stack itself when something upstream dies.
    :param recvTimeout: seconds to wait for the next message; ``None`` waits
        indefinitely. TrafficLayer advances only once EVERY subscriber has
        replied, so a deadline here fires when some OTHER client stalls.
    """
    global _helper, _sock, _egoIds, verbose

    if _sock is not None:
        close()

    configPath = configPath or os.environ.get('FIXS_CONFIG_YAML') or 'config.yaml'
    if not os.path.isfile(configPath):
        raise FixsError(f'config not found: {configPath}')

    config = ConfigHelper()
    config.getConfig(configPath)

    declared = config.simulation_setup.get('VehicleMessageField') or ['id', 'speed']
    missing = [f for f in require if f not in declared]
    if missing:
        raise FieldsMissing(
            f"{configPath} does not put {', '.join(missing)} on the wire. Add "
            f"them to SimulationSetup.VehicleMessageField, which declares: "
            f"{', '.join(declared)}."
        )

    subscription = _selectSubscription(config, configPath, port)
    if host is None:
        host = subscription['ip'][0]
    if port is None:
        port = subscription['port'][0]

    if ego is None:
        ego = (subscription.get('attribute') or {}).get('id') or []
    _egoIds = [ego] if isinstance(ego, str) else list(ego)

    msgHelper = MsgHelper()
    msgHelper.set_vehicle_message_field(declared)

    verbose = bool(config.simulation_setup.get('EnableVerboseLog', False))
    _helper = SocketHelper(config_helper=config, msg_helper=msgHelper)
    _sock = _openSocket(host, int(port), connectTimeout, recvTimeout)
    atexit.register(close)          # a held reply must still go out on exit
    return host, int(port)


def _selectSubscription(config, configPath, port):
    subscriptions = config.application_setup.get('VehicleSubscription') or []
    if not subscriptions:
        raise FixsError(
            f'{configPath} has no ApplicationSetup.VehicleSubscription, so '
            f'there is no endpoint to connect to.'
        )
    if port is not None:
        for entry in subscriptions:
            if port in entry.get('port', []):
                return entry
        declaredPorts = [p for e in subscriptions for p in e.get('port', [])]
        raise FixsError(
            f'{configPath} declares no subscription on port {port}. '
            f'It declares: {declaredPorts}.'
        )
    if len(subscriptions) > 1:
        declaredPorts = [p for e in subscriptions for p in e.get('port', [])]
        raise FixsError(
            f'{configPath} declares {len(subscriptions)} vehicle '
            f'subscriptions (ports {declaredPorts}); pass port= to say which '
            f'one is this client.'
        )
    return subscriptions[0]


def _openSocket(host, port, connectTimeout, recvTimeout):
    """Connect, retrying until TrafficLayer is actually listening.

    A client is often started BEFORE TrafficLayer -- which may itself be waiting
    on a map cook or a simulator launch -- so a single attempt fails on a
    perfectly healthy stack.
    """
    deadline = None if connectTimeout is None else _time.time() + connectTimeout
    announced = False
    while True:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
        try:
            sock.connect((host, port))
        except OSError as exc:
            sock.close()
            if deadline is not None and _time.time() >= deadline:
                raise FixsError(
                    f'could not reach TrafficLayer at {host}:{port} within '
                    f'{connectTimeout}s: {exc}'
                ) from exc
            if not announced:
                print(f'[fixs] waiting for TrafficLayer on {host}:{port} ...')
                announced = True
            _time.sleep(0.5)
            continue

        sock.settimeout(recvTimeout)
        print(f'[fixs] connected to TrafficLayer on {host}:{port}')
        return sock


def close():
    """Send any held reply, then close. Safe to call more than once."""
    global _sock, _helper, _egoIds, _armed, _received, _echoLimit
    if _armed and _sock is not None:
        try:
            _sendReply()
        except OSError:
            pass                # peer already gone; nothing to deliver
    if _sock is not None:
        try:
            _sock.close()
        except OSError:
            pass
    _sock = None
    _helper = None
    _egoIds = []
    _armed = False
    _received = []
    _echoLimit = None


def getFields():
    """() -> list[string] -- the VehicleMessageField list this connection decodes."""
    _requireConnection()
    return list(_helper.msg_helper.vehicle_msg_field)


def getTime():
    """() -> double -- simulation time of the tick currently held."""
    return _simTime


# ---------------------------------------------------------------------------
# The tick
# ---------------------------------------------------------------------------

def echoAll(limit=0):
    """(integer) -> None -- return every vehicle received, not only commanded ones.

    The default reply carries only the records this client actually commanded:
    returning ~200 untouched records every tick doubles upstream volume to say
    nothing.

    Echo clients are the exception -- returning the whole feed is the behaviour
    under test. ``limit`` caps how many go back (0 = all); a cap of 1 mirrors
    real XIL, where the client returns only the ego pose.

    Applies to the current tick only.
    """
    global _echoLimit
    _requireConnection()
    _echoLimit = limit


def sync():
    """() -> double | None -- send this tick's commands, receive the next tick.

    Returns the new simulation time, or ``None`` once TrafficLayer signals
    shutdown, so the usual shape is::

        while True:
            simTime = fixs.sync()
            if simTime is None:
                break

    One call is one exchange, which is what makes the protocol's rules hold
    without the application maintaining them:

    * Every received tick is answered exactly once -- there is no way to receive
      without having sent, because it is the same call.
    * A tick with nothing to say still answers, with a record-less message.
      TrafficLayer does not advance until every subscriber has replied.
    * Shutdown ends the loop. TrafficLayer signals it with state 0, which a
      hand-written loop has to notice and typically does not -- the reference
      controller echoes it straight back and runs on until a time bound.

    NOTE this does not advance the simulator. TrafficLayer owns the clock; see
    the module docstring on why it is not called ``simulationStep``.
    """
    _requireConnection()
    global vehicle, ego, trafficlight, _armed, _received
    global _simState, _simTime, _echoLimit

    if _armed:
        _sendReply()

    _helper.clear_data()
    simState, simTime = _helper.recv_data(_sock)
    if simState == 0:
        _armed = False
        vehicle, ego = _VehicleView(), _VehicleView()
        trafficlight = _TrafficLightView()
        return None

    _received = _helper.vehicle_data_receive_list
    for record in _received:
        record.__class__ = Vehicle      # adopt in place: no copy, no re-decode
    vehicle = _VehicleView(_received)
    ego = _VehicleView(r for r in _received if r.id.strip() in _egoIds)
    trafficlight = _TrafficLightView(
        _helper.traffic_light_data_receive_list,
        key=lambda r: (r.name or '').strip())

    _simState, _simTime = simState, simTime
    _commanded.clear()
    _echoLimit = None
    _armed = True
    return simTime


def _sendReply():
    """Put the held tick's answer on the wire."""
    global _armed
    if _echoLimit is not None:
        sendList = _received if _echoLimit == 0 else _received[:_echoLimit]
    else:
        sendList = [r for r in _received if r.id.strip() in _commanded]

    # A tick with nothing to say sends a header and no records. That is a real
    # message -- it is what tells TrafficLayer this client is done and the tick
    # may advance -- and it is what the reference echo clients have always sent
    # on an empty feed. Do NOT substitute a placeholder record: it would put a
    # vehicle with an empty id and zeroed fields on the wire every idle tick,
    # which over a long run is most of the traffic.
    _helper.vehicle_data_send_list.extend(sendList)
    _helper.sendData(_simState, _simTime, _sock)
    _armed = False


def __getattr__(name):
    # Detector records are received but not decoded -- SocketHelper.recv_data
    # drops them. Say so rather than handing back an empty view, which would
    # read as "no detectors this tick".
    if name == 'detector':
        raise NotImplementedError(
            'detector data is not decoded yet -- SocketHelper.recv_data drops '
            'DetectorData records (see the TODO in CommonLib/SocketHelper.py)'
        )
    raise AttributeError(f'module {__name__!r} has no attribute {name!r}')
