"""FIXS client API -- one import, no hand-written socket code (#316).

A Python application talks to TrafficLayer like this::

    from CommonLib import fixs          # or `import fixs` once it is on the path

    fixs.connect('config.yaml', ego='ego')
    for t in fixs.steps():
        if fixs.ego is None:
            continue
        fixs.ego.speedDesired = my_controller(fixs.ego.speed)

Everything the co-simulation protocol requires -- connecting and retrying,
framing, replying exactly once per tick, replying even when there is nothing to
say, honouring the shutdown signal, clearing state between ticks -- is handled
here. None of it is the application's to remember.

This module is a facade over the existing helpers, which are unchanged:
ConfigHelper reads the yaml, MsgHelper packs and unpacks, SocketHelper frames.
Scripts that use those directly keep working exactly as before.

Module attributes are rebound on every tick of :func:`steps`:

    fixs.ego            the vehicle named by connect(ego=...), or None
    fixs.vehicle        this tick's vehicles -- fixs.vehicle['id'], iterable
    fixs.trafficlight   this tick's signal states, same shape
    fixs.time           simulation time of this tick
    fixs.state          simulation state word of this tick

Holding a reference across ticks is safe: every tick decodes fresh record
objects, so `previous = fixs.ego` keeps that tick's values rather than aliasing
something that is about to be overwritten.
"""

import os
import socket
import time as _time
import typing

from CommonLib.ConfigHelper import ConfigHelper
from CommonLib.MsgHelper import MsgHelper
from CommonLib.SocketHelper import SocketHelper
from CommonLib.VehDataMsgDefs import VehData

__all__ = [
    'connect', 'steps', 'close', 'echo_all', 'fields',
    'ego', 'vehicle', 'trafficlight', 'time', 'state', 'verbose',
    'FixsError', 'NotConnected', 'FieldsMissing',
]


class FixsError(Exception):
    """Base class for every error this module raises."""


class NotConnected(FixsError):
    """Raised when the API is used before connect() or after close()."""


class FieldsMissing(FixsError):
    """Raised by connect(require=...) when the config cannot deliver a field.

    Raised at startup rather than letting the application discover it per tick.
    A missing field is otherwise indistinguishable from a legitimate default,
    which is how `speedFreeFlow` (free-flow desire) silently degrades into
    `speedLimit` (posted limit) -- a different physical quantity, and plausible
    enough that the results look fine.
    """


# ---------------------------------------------------------------------------
# Records
# ---------------------------------------------------------------------------

class Vehicle(VehData):
    """One vehicle, for one tick.

    Measured fields are read-only. The five ``*Desired`` fields are writable,
    and **writing one is what returns this record to TrafficLayer** -- there is
    no separate send call.

    Two failure modes are closed by that rule:

    * You cannot construct a fresh record to reply with. A fresh ``VehData``
      leaves every unset field at its dataclass default -- position (0, 0, 0),
      heading 0, length 0 -- and TrafficLayer publishes a client's return to
      later clients by REPLACING the whole record for that id, not by merging
      fields. That is how an eco-driving controller on a low port put the ego at
      the world origin in the CARLA bridge's copy
      (ORNL-Real-Sim/FIXS_Applications#25). Here the only record you can return
      is the one that arrived, so the other fields are necessarily intact.

    * You cannot turn a local calculation into a command by accident. Unit
      conversion in place (``veh.speed *= 2.23694``) is a natural thing to write
      and would otherwise be indistinguishable from an instruction to SUMO. It
      raises instead.
    """

    #: The wire's command channel: everything a client is allowed to write.
    COMMAND_FIELDS = frozenset({
        'speedDesired',
        'accelerationDesired',
        'steerAngleDesired',
        'acceleratorPedalDesired',
        'brakePedalDesired',
    })

    _commanded = False

    def __setattr__(self, name, value):
        if name in self.COMMAND_FIELDS:
            object.__setattr__(self, name, value)
            object.__setattr__(self, '_commanded', True)
        elif name.startswith('_'):
            object.__setattr__(self, name, value)
        else:
            raise AttributeError(
                f"'{name}' is measured data from the traffic simulator and is "
                f"read-only. Writable fields are: "
                f"{', '.join(sorted(self.COMMAND_FIELDS))}."
            )


class _View:
    """This tick's records of one kind, addressed by id.

    Iterating yields the RECORDS, not the ids -- a deliberate break from the
    Mapping convention, because the records are what callers want and each one
    already carries its own id.
    """

    __slots__ = ('_by_id', '_kind')

    def __init__(self, records, kind, key=lambda r: r.id.strip()):
        self._kind = kind
        self._by_id = {key(r): r for r in records}

    def __getitem__(self, record_id):
        try:
            return self._by_id[record_id]
        except KeyError:
            raise KeyError(
                f"no {self._kind} '{record_id}' in this tick. Present: "
                f"{', '.join(sorted(self._by_id)) or '(none)'}"
            ) from None

    def get(self, record_id, default=None):
        return self._by_id.get(record_id, default)

    def __contains__(self, record_id):
        return record_id in self._by_id

    def __iter__(self):
        return iter(self._by_id.values())

    def __len__(self):
        return len(self._by_id)

    def ids(self):
        return list(self._by_id)

    def __repr__(self):
        return f'<{len(self._by_id)} {self._kind}(s): {", ".join(sorted(self._by_id))}>'


# ---------------------------------------------------------------------------
# Module state -- rebound every tick by steps()
# ---------------------------------------------------------------------------

ego: typing.Optional[Vehicle] = None
vehicle: _View = _View([], 'vehicle')
trafficlight: _View = _View([], 'traffic light')
time: float = 0.0
state: int = 0
verbose: bool = False           # SimulationSetup.EnableVerboseLog, from the config

_helper: typing.Optional[SocketHelper] = None
_sock: typing.Optional[socket.socket] = None
_ego_id: typing.Optional[str] = None
_echo_limit: typing.Optional[int] = None


def _require_connection():
    if _helper is None or _sock is None:
        raise NotConnected('call fixs.connect(...) first')


# ---------------------------------------------------------------------------
# Connecting
# ---------------------------------------------------------------------------

def connect(config=None, *, ego=None, host=None, port=None,
            require=(), connect_timeout=None, recv_timeout=None):
    """Connect to TrafficLayer and return the resolved (host, port).

    :param config: path to the config yaml TrafficLayer is running. Defaults to
        ``$FIXS_CONFIG_YAML`` -- which run_cosim sets for exactly this purpose,
        because the wire format IS ``SimulationSetup.VehicleMessageField`` and
        two yamls that disagree decode the same bytes differently.
    :param ego: id of the vehicle this application controls. Exposed as
        ``fixs.ego`` each tick, or None on ticks where it is not in the feed.
    :param host, port: override the endpoint from ``ApplicationSetup``.
    :param require: field names that must be in ``VehicleMessageField``.
        Raises :class:`FieldsMissing` here rather than degrading silently later.
    :param connect_timeout: seconds to keep retrying the connect. ``None``
        retries forever, which is the right choice under a supervisor
        (run_cosim) that stops the stack itself when something upstream dies --
        a deadline here can only turn a slow start into a spurious failure.
    :param recv_timeout: seconds to wait for TrafficLayer's next message.
        ``None`` waits indefinitely. TrafficLayer advances a tick only once
        EVERY subscriber has replied, so a deadline here fires when some OTHER
        client stalls, and the run dies naming the component that was merely
        waiting.
    """
    global _helper, _sock, _ego_id, _echo_limit, verbose

    if _sock is not None:
        close()

    config = config or os.environ.get('FIXS_CONFIG_YAML') or 'config.yaml'
    if not os.path.isfile(config):
        raise FixsError(f'config not found: {config}')

    config_helper = ConfigHelper()
    config_helper.getConfig(config)

    declared = config_helper.simulation_setup.get('VehicleMessageField') or ['id', 'speed']
    missing = [f for f in require if f not in declared]
    if missing:
        raise FieldsMissing(
            f"{config} does not put {', '.join(missing)} on the wire. "
            f"Add them to SimulationSetup.VehicleMessageField, which currently "
            f"declares: {', '.join(declared)}."
        )

    msg_helper = MsgHelper()
    msg_helper.set_vehicle_message_field(declared)

    if host is None or port is None:
        subscriptions = config_helper.application_setup.get('VehicleSubscription') or []
        if not subscriptions:
            raise FixsError(
                f'{config} has no ApplicationSetup.VehicleSubscription, so there '
                f'is no endpoint to connect to. Pass host=/port= to override.'
            )
        host = host or subscriptions[0]['ip'][0]
        port = port if port is not None else subscriptions[0]['port'][0]

    verbose = bool(config_helper.simulation_setup.get('EnableVerboseLog', False))
    _helper = SocketHelper(config_helper=config_helper, msg_helper=msg_helper)
    _ego_id = ego
    _echo_limit = None
    _sock = _open_socket(host, int(port), connect_timeout, recv_timeout)
    return host, int(port)


def _open_socket(host, port, connect_timeout, recv_timeout):
    """Connect, retrying until TrafficLayer is actually listening.

    Retry rather than connect once: a client is often started BEFORE
    TrafficLayer (which may itself be waiting on a map cook or a simulator
    launch), so a single attempt fails on a healthy stack.
    """
    deadline = None if connect_timeout is None else _time.time() + connect_timeout
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
                    f'{connect_timeout}s: {exc}'
                ) from exc
            if not announced:
                print(f'[fixs] waiting for TrafficLayer on {host}:{port} ...')
                announced = True
            _time.sleep(0.5)
            continue

        sock.settimeout(recv_timeout)
        print(f'[fixs] connected to TrafficLayer on {host}:{port}')
        return sock


def close():
    """Close the connection. Safe to call more than once."""
    global _sock, _helper, _ego_id, _echo_limit
    if _sock is not None:
        try:
            _sock.close()
        except OSError:
            pass
    _sock = None
    _helper = None
    _ego_id = None
    _echo_limit = None


def fields():
    """The VehicleMessageField list this connection decodes."""
    _require_connection()
    return list(_helper.msg_helper.vehicle_msg_field)


# ---------------------------------------------------------------------------
# The tick loop
# ---------------------------------------------------------------------------

def echo_all(limit=0):
    """Return every vehicle received this tick, not only the commanded ones.

    The default reply carries only records the application wrote a ``*Desired``
    field on, which is what a controller wants: returning ~200 untouched records
    every tick doubles upstream volume to say nothing.

    Echo clients are the exception -- returning the whole feed is the behaviour
    under test. ``limit`` caps how many are returned (0 = all); a cap of 1
    mirrors real XIL, where the client returns only the ego pose, so
    TrafficLayer's receive path sees one record rather than a crowd.

    Applies to the current tick only; call it each tick you want it.
    """
    global _echo_limit
    _require_connection()
    _echo_limit = limit


def steps():
    """Yield the simulation time of each tick, replying to TrafficLayer for you.

    One iteration is one tick: this receives, hands control to the loop body,
    then sends whatever the body commanded. That structure is what makes the
    protocol's rules unbreakable from application code:

    * Every received tick is answered exactly once -- including the ticks a
      body skips with ``continue``, which is what warm-up looks like here.
    * A tick with nothing to say still answers -- with a record-less message,
      not a placeholder record -- because TrafficLayer does not advance until
      every subscriber has replied.
    * Shutdown ends the loop. TrafficLayer signals it with state 0, which a
      hand-written loop has to notice and typically does not -- the reference
      controller echoes it straight back and runs on until a time bound.
    """
    _require_connection()
    global ego, vehicle, trafficlight, time, state, _echo_limit
    try:
        while True:
            _helper.clear_data()
            sim_state, sim_time = _helper.recv_data(_sock)
            if sim_state == 0:
                return

            received = _helper.vehicle_data_receive_list
            for record in received:
                record.__class__ = Vehicle      # adopt in place; no copy, no re-decode
            vehicle = _View(received, 'vehicle')
            trafficlight = _View(
                _helper.traffic_light_data_receive_list, 'traffic light',
                key=lambda r: (r.name or '').strip())
            ego = vehicle.get(_ego_id) if _ego_id else None
            time, state = sim_time, sim_state
            _echo_limit = None

            try:
                yield sim_time
            finally:
                # In a finally so that leaving the loop -- `break`, `return`, an
                # exception -- still answers the tick that was already received.
                # TrafficLayer is blocked waiting for this reply; walking away
                # without it strands the whole co-simulation rather than just
                # this client.
                try:
                    _reply(received, sim_state, sim_time)
                except OSError:
                    # Peer already gone. Nothing to deliver, and raising here
                    # would replace whatever exception we are unwinding from.
                    pass
    finally:
        close()


def _reply(received, sim_state, sim_time):
    """Put this tick's answer on the wire."""
    if _echo_limit is not None:
        send_list = received if _echo_limit == 0 else received[:_echo_limit]
    else:
        send_list = [r for r in received if r._commanded]

    # A tick with nothing to say sends a header and no records. That is a real
    # message -- it is what tells TrafficLayer this client is done and the tick
    # may advance -- and it is what the reference echo clients have always sent
    # on an empty feed. Do NOT substitute a placeholder record here: it would
    # put a vehicle with an empty id and zeroed fields on the wire every idle
    # tick, which is both a lie and, over a long run, most of the traffic.
    _helper.vehicle_data_send_list.extend(send_list)
    _helper.sendData(sim_state, sim_time, _sock)


def __getattr__(name):
    # Detector records are received but not decoded -- SocketHelper.recv_data
    # drops them on the floor. Say so rather than handing back an empty view
    # that reads as "no detectors this tick".
    if name == 'detector':
        raise NotImplementedError(
            'detector data is not decoded yet -- SocketHelper.recv_data drops '
            'DetectorData records (see the TODO in CommonLib/SocketHelper.py)'
        )
    raise AttributeError(f'module {__name__!r} has no attribute {name!r}')
