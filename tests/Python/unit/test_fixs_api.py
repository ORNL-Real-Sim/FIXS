"""Simulator-free tests for the fixs client API (#316).

These cover the rules the API exists to enforce -- what may be written, what
gets returned, and what happens on a tick the application skips -- using a fake
socket in place of TrafficLayer, so they run anywhere.
"""
import os

import pytest

from CommonLib import fixs
from CommonLib.MsgHelper import MsgHelper
from CommonLib.VehDataMsgDefs import VehData

SIMPLE_ECHO_CONFIG = os.path.join(
    os.path.dirname(__file__), '..', 'SimpleEchoClient', 'config.yaml'
)


def _adopted(**kwargs):
    record = VehData(**kwargs)
    record.__class__ = fixs.Vehicle
    return record


# ---------------------------------------------------------------------------
# Records are read-only
# ---------------------------------------------------------------------------

def test_adopted_record_is_still_a_vehdata():
    """Adoption must not break code that type-checks against the wire struct."""
    assert isinstance(_adopted(id='ego'), VehData)


def test_measured_fields_cannot_be_assigned():
    """Unit conversion in place must not silently become a command."""
    veh = _adopted(id='ego', speed=10.0)
    with pytest.raises(AttributeError):
        veh.speed = veh.speed * 2.23694
    assert veh.speed == 10.0


@pytest.mark.parametrize('field', sorted(fixs.Vehicle.COMMAND_FIELDS))
def test_command_fields_cannot_be_assigned_either(field):
    """One way to command: the setter. Assignment points at it."""
    veh = _adopted(id='ego')
    with pytest.raises(AttributeError) as excinfo:
        setattr(veh, field, 1.0)
    assert 'fixs.vehicle.set' in str(excinfo.value)


# ---------------------------------------------------------------------------
# Views
# ---------------------------------------------------------------------------

def test_getall_and_getidlist():
    a, b = _adopted(id='ego  '), _adopted(id='veh1 ')
    view = fixs._VehicleView([a, b])
    assert view.getAll() == {'ego': a, 'veh1': b}
    assert sorted(view.getIDList()) == ['ego', 'veh1']
    assert len(view) == 2


def test_get_takes_a_list_and_skips_absent_ids():
    view = fixs._VehicleView([_adopted(id='ego')])
    assert view.get(['ego', 'nope']) == {'ego': view.getAll()['ego']}


def test_get_rejects_a_bare_string():
    """A string is iterable, so silently accepting one would return per-character
    lookups and an empty dict rather than an error."""
    view = fixs._VehicleView([_adopted(id='ego')])
    with pytest.raises(TypeError) as excinfo:
        view.get('ego')
    assert "get(['ego'])" in str(excinfo.value)


def test_getall_returns_a_copy():
    view = fixs._VehicleView([_adopted(id='ego')])
    view.getAll().clear()
    assert view.getIDList() == ['ego']


# ---------------------------------------------------------------------------
# Connect-time contracts
# ---------------------------------------------------------------------------

def test_require_missing_field_fails_at_connect():
    """A field the config cannot deliver is a startup error, not a per-tick surprise."""
    if not os.path.exists(SIMPLE_ECHO_CONFIG):
        pytest.skip('config file not found')
    with pytest.raises(fixs.FieldsMissing) as excinfo:
        fixs.connect(SIMPLE_ECHO_CONFIG, require=['speedFreeFlow'])
    assert 'speedFreeFlow' in str(excinfo.value)


def test_require_present_field_gets_past_validation():
    """Validation happens before any network work.

    Port 1 rather than the configured one, so the test cannot accidentally reach
    a TrafficLayer someone left running.
    """
    if not os.path.exists(SIMPLE_ECHO_CONFIG):
        pytest.skip('config file not found')
    with pytest.raises(fixs.FixsError) as excinfo:
        fixs.connect(SIMPLE_ECHO_CONFIG, require=['speed'],
                     host='127.0.0.1', port=1, connectTimeout=0.1)
    assert not isinstance(excinfo.value, fixs.FieldsMissing)


def test_missing_config_is_reported_by_path():
    with pytest.raises(fixs.FixsError) as excinfo:
        fixs.connect('no_such_config.yaml')
    assert 'no_such_config.yaml' in str(excinfo.value)


def test_sync_before_connect_raises():
    fixs.close()
    with pytest.raises(fixs.NotConnected):
        fixs.sync()


def test_detector_says_it_is_not_decoded():
    """An empty view would read as 'no detectors this tick', which is a lie."""
    with pytest.raises(NotImplementedError):
        fixs.detector


class _Cfg:
    """Minimal ConfigHelper stand-in for subscription-selection tests."""

    def __init__(self, subscriptions):
        self.simulation_setup = {'VehicleMessageField': ['id', 'speed']}
        self.application_setup = {'VehicleSubscription': subscriptions}


def _sub(port, ids=('ego',)):
    return {'type': 'ego', 'attribute': {'id': list(ids), 'radius': [0]},
            'ip': ['127.0.0.1'], 'port': [port]}


def test_single_subscription_is_selected_without_a_port():
    entry = fixs._selectSubscription(_Cfg([_sub(2444)]), 'c.yaml', None)
    assert entry['port'] == [2444]


def test_several_subscriptions_require_a_port():
    """Silently taking entry 0 is how two clients end up sharing one endpoint."""
    with pytest.raises(fixs.FixsError) as excinfo:
        fixs._selectSubscription(_Cfg([_sub(430), _sub(440)]), 'c.yaml', None)
    assert '430' in str(excinfo.value) and '440' in str(excinfo.value)


def test_port_selects_its_own_subscription():
    entry = fixs._selectSubscription(_Cfg([_sub(430), _sub(440)]), 'c.yaml', 440)
    assert entry['port'] == [440]


def test_unknown_port_lists_what_is_declared():
    with pytest.raises(fixs.FixsError) as excinfo:
        fixs._selectSubscription(_Cfg([_sub(430)]), 'c.yaml', 999)
    assert '430' in str(excinfo.value)


# ---------------------------------------------------------------------------
# The reply rules, against a fake TrafficLayer
# ---------------------------------------------------------------------------

FIELDS = ['id', 'speed', 'speedDesired']


class FakeSocket:
    """Serves a canned sequence of ticks and records everything sent back."""

    def __init__(self, ticks, fields):
        helper = MsgHelper()
        helper.set_vehicle_message_field(fields)
        self._stream = b''.join(self._encode(helper, *tick) for tick in ticks)
        self._read = 0
        self.sent = []

    @staticmethod
    def _encode(helper, simState, simTime, vehicles):
        body = bytearray(65536)
        index = 0
        total = helper.msg_header_size
        for veh in vehicles:
            _, size, index = helper.pack_veh_data(body, index, veh)
            total += size
        header = bytearray(81728)
        header, headerLen = helper.pack_msg_header(header, simState, simTime, total)
        return bytes(header[:headerLen]) + bytes(body[:index])

    def recv(self, n):
        chunk = self._stream[self._read:self._read + n]
        self._read += len(chunk)
        if not chunk:
            raise ConnectionError('fake stream exhausted')
        return chunk

    def sendall(self, data):
        self.sent.append(bytes(data))

    def settimeout(self, _):
        pass

    def close(self):
        pass


class _ConfigStub:
    simulation_setup = {'EnableVerboseLog': False}
    application_setup = {}


def _install(ticks, egoIds=('ego',), shutdown=True):
    """Point the module at a fake TrafficLayer without touching a real socket."""
    from CommonLib.SocketHelper import SocketHelper
    if shutdown:
        ticks = list(ticks) + [(0, 99.0, [])]
    fixs.close()
    msgHelper = MsgHelper()
    msgHelper.set_vehicle_message_field(FIELDS)
    fixs._helper = SocketHelper(config_helper=_ConfigStub(), msg_helper=msgHelper)
    fixs._sock = FakeSocket(ticks, FIELDS)
    fixs._egoIds = list(egoIds)
    fixs._armed = False
    fixs._echoLimit = None
    return fixs._sock


def _veh(vehID, speed=1.0):
    return VehData(id=vehID, speed=speed)


def _drain():
    """Run the loop the way a client does, returning the tick times."""
    times = []
    while True:
        t = fixs.sync()
        if t is None:
            return times
        times.append(t)


def _recordCount(message):
    """Count vehicle records in one encoded FIXS message."""
    helper = MsgHelper()
    helper.set_vehicle_message_field(FIELDS)
    _, _, total = helper.depack_msg_header(message[:helper.msg_header_size])
    index = helper.msg_header_size
    count = 0
    while index < total:
        size, _kind = helper.depack_msg_type(
            message[index:index + helper.msg_each_header_size])
        index += size
        count += 1
    return count


def test_skipped_tick_still_replies():
    """A body that does nothing is warm-up; TrafficLayer must still be answered."""
    sock = _install([(1, 0.1, [_veh('ego')]), (1, 0.2, [_veh('ego')])])
    assert _drain() == [pytest.approx(0.1), pytest.approx(0.2)]
    assert len(sock.sent) == 2


def test_reply_carries_only_commanded_records_by_default():
    sock = _install([(1, 0.1, [_veh('ego', 5.0), _veh('other', 6.0)])])
    while True:
        if fixs.sync() is None:
            break
        fixs.vehicle.setSpeedDesired('ego', 9.0)
    assert len(sock.sent) == 1
    # One record on the wire, not two: the untouched vehicle is not returned.
    assert _recordCount(sock.sent[0]) == 1


def test_setter_writes_the_value_that_goes_out():
    sock = _install([(1, 0.1, [_veh('ego', 5.0)])])
    while True:
        if fixs.sync() is None:
            break
        fixs.vehicle.setSpeedDesired('ego', 9.0)
        assert fixs.vehicle.getAll()['ego'].speedDesired == 9.0


def test_setter_on_an_absent_id_raises():
    """A typo would otherwise do nothing, silently, for the whole run."""
    _install([(1, 0.1, [_veh('ego')])])
    fixs.sync()
    with pytest.raises(KeyError) as excinfo:
        fixs.vehicle.setSpeedDesired('egoo', 1.0)
    assert 'ego' in str(excinfo.value)
    _drain()


def test_ego_view_holds_only_declared_ids():
    _install([(1, 0.1, [_veh('ego'), _veh('other')])], egoIds=('ego',))
    fixs.sync()
    assert fixs.ego.getIDList() == ['ego']
    assert sorted(fixs.vehicle.getIDList()) == ['ego', 'other']
    _drain()


def test_multiple_egos_are_supported():
    _install([(1, 0.1, [_veh('e1'), _veh('e2'), _veh('bg')])], egoIds=('e1', 'e2'))
    fixs.sync()
    assert sorted(fixs.ego.getIDList()) == ['e1', 'e2']
    _drain()


def test_echo_all_returns_the_whole_feed():
    sock = _install([(1, 0.1, [_veh('ego', 5.0), _veh('other', 6.0)])])
    while True:
        if fixs.sync() is None:
            break
        fixs.echoAll()
    assert _recordCount(sock.sent[0]) == 2


def test_echo_all_limit_caps_the_reply():
    sock = _install([(1, 0.1, [_veh('ego'), _veh('a'), _veh('b')])])
    while True:
        if fixs.sync() is None:
            break
        fixs.echoAll(limit=1)
    assert _recordCount(sock.sent[0]) == 1


def test_empty_tick_replies_with_no_records():
    """A quiet tick sends a header and nothing else.

    Not a placeholder record: that would put a vehicle with an empty id and
    zeroed fields on the wire every idle tick. The reference echo clients have
    always replied this way, so it is also what byte-identity requires.
    """
    sock = _install([(1, 0.1, [])])
    _drain()
    assert len(sock.sent) == 1
    assert _recordCount(sock.sent[0]) == 0


def test_shutdown_ends_the_loop_without_replying():
    sock = _install([(1, 0.1, [_veh('ego')]), (0, 0.2, [])], shutdown=False)
    assert _drain() == [pytest.approx(0.1)]
    assert len(sock.sent) == 1          # the shutdown tick is not answered


def test_close_flushes_the_held_reply():
    """Breaking out of the loop must not strand TrafficLayer mid-tick.

    This is what `--steps N` does in simple_echo_client: the old client replied
    to step N and then broke, so the reply has to survive the break.
    """
    sock = _install([(1, 0.1, [_veh('ego')]), (1, 0.2, [_veh('ego')])])
    fixs.sync()
    assert len(sock.sent) == 0          # nothing sent yet for this tick
    fixs.close()
    assert len(sock.sent) == 1          # ... until close flushes it


def test_close_is_idempotent():
    _install([(1, 0.1, [])])
    fixs.sync()
    fixs.close()
    fixs.close()
