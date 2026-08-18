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


# ---------------------------------------------------------------------------
# Records: what may be written, and what that means
# ---------------------------------------------------------------------------

def _adopted(**kwargs):
    record = VehData(**kwargs)
    record.__class__ = fixs.Vehicle
    return record


def test_adopted_record_is_still_a_vehdata():
    """Adoption must not break code that type-checks against the wire struct."""
    assert isinstance(_adopted(id='ego'), VehData)


def test_command_fields_are_writable_and_mark_the_record():
    veh = _adopted(id='ego', speed=10.0)
    assert veh._commanded is False
    veh.speedDesired = 7.5
    assert veh.speedDesired == 7.5
    assert veh._commanded is True


def test_measured_fields_are_read_only():
    """Unit conversion in place must not silently become a command."""
    veh = _adopted(id='ego', speed=10.0)
    with pytest.raises(AttributeError):
        veh.speed = veh.speed * 2.23694
    assert veh.speed == 10.0
    assert veh._commanded is False


@pytest.mark.parametrize('field', sorted(fixs.Vehicle.COMMAND_FIELDS))
def test_every_command_field_is_writable(field):
    veh = _adopted(id='ego')
    setattr(veh, field, 1.0)
    assert getattr(veh, field) == 1.0


# ---------------------------------------------------------------------------
# Views
# ---------------------------------------------------------------------------

def test_view_lookup_iteration_and_membership():
    a, b = _adopted(id='ego  '), _adopted(id='veh1 ')
    view = fixs._View([a, b], 'vehicle')
    assert view['ego'] is a
    assert 'veh1' in view
    assert len(view) == 2
    assert sorted(view.ids()) == ['ego', 'veh1']
    # Iteration yields records, not ids -- deliberate, see _View.
    assert {type(r).__name__ for r in view} == {'Vehicle'}


def test_view_missing_id_names_what_is_present():
    view = fixs._View([_adopted(id='ego')], 'vehicle')
    with pytest.raises(KeyError) as excinfo:
        view['typo']
    assert 'ego' in str(excinfo.value)


def test_view_get_returns_default():
    view = fixs._View([], 'vehicle')
    assert view.get('ego') is None


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
    """`speed` is in the echo config, so require() must not reject it.

    The connect then fails on the socket, which is the point: field validation
    happens before any network work. Port 1 rather than the configured one, so
    the test cannot accidentally reach a TrafficLayer someone left running.
    """
    if not os.path.exists(SIMPLE_ECHO_CONFIG):
        pytest.skip('config file not found')
    with pytest.raises(fixs.FixsError) as excinfo:
        fixs.connect(SIMPLE_ECHO_CONFIG, require=['speed'],
                     host='127.0.0.1', port=1, connect_timeout=0.1)
    assert not isinstance(excinfo.value, fixs.FieldsMissing)


def test_missing_config_is_reported_by_path():
    with pytest.raises(fixs.FixsError) as excinfo:
        fixs.connect('no_such_config.yaml')
    assert 'no_such_config.yaml' in str(excinfo.value)


def test_steps_before_connect_raises():
    fixs.close()
    with pytest.raises(fixs.NotConnected):
        next(fixs.steps())


def test_detector_says_it_is_not_decoded():
    """An empty view would read as 'no detectors this tick', which is a lie."""
    with pytest.raises(NotImplementedError):
        fixs.detector


# ---------------------------------------------------------------------------
# The reply rules, against a fake TrafficLayer
# ---------------------------------------------------------------------------

class FakeSocket:
    """Serves a canned sequence of ticks and records everything sent back."""

    def __init__(self, ticks, fields):
        helper = MsgHelper()
        helper.set_vehicle_message_field(fields)
        self._stream = b''.join(self._encode(helper, *tick) for tick in ticks)
        self._read = 0
        self.sent = []

    @staticmethod
    def _encode(helper, sim_state, sim_time, vehicles):
        body = bytearray(65536)
        index = 0
        total = helper.msg_header_size
        for veh in vehicles:
            _, size, index = helper.pack_veh_data(body, index, veh)
            total += size
        header = bytearray(81728)
        header, header_len = helper.pack_msg_header(header, sim_state, sim_time, total)
        return bytes(header[:header_len]) + bytes(body[:index])

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


FIELDS = ['id', 'speed', 'speedDesired']


def _install(ticks, shutdown=True):
    """Point the module at a fake TrafficLayer without touching a real socket.

    A shutdown tick is appended by default so `for _ in fixs.steps()` ends the
    way it does against a real TrafficLayer, rather than running the canned
    stream dry.
    """
    if shutdown:
        ticks = list(ticks) + [(0, 99.0, [])]
    fixs.close()
    helper_config = _ConfigStub()
    msg_helper = MsgHelper()
    msg_helper.set_vehicle_message_field(FIELDS)
    from CommonLib.SocketHelper import SocketHelper
    fixs._helper = SocketHelper(config_helper=helper_config, msg_helper=msg_helper)
    fixs._sock = FakeSocket(ticks, FIELDS)
    fixs._ego_id = 'ego'
    fixs._echo_limit = None
    return fixs._sock


class _ConfigStub:
    simulation_setup = {'EnableVerboseLog': False}
    application_setup = {}


def _veh(veh_id, speed=1.0):
    return VehData(id=veh_id, speed=speed)


def test_skipped_tick_still_replies():
    """`continue` is what warm-up looks like; TrafficLayer must still be answered."""
    sock = _install([(1, 0.1, [_veh('ego')]), (1, 0.2, [_veh('ego')])])
    for _ in fixs.steps():
        continue
    assert len(sock.sent) == 2


def test_reply_carries_only_commanded_records_by_default():
    sock = _install([(1, 0.1, [_veh('ego', 5.0), _veh('other', 6.0)])])
    for _ in fixs.steps():
        fixs.ego.speedDesired = 9.0
    assert len(sock.sent) == 1
    # One record on the wire, not two: the untouched vehicle is not returned.
    assert _record_count(sock.sent[0]) == 1


def test_echo_all_returns_the_whole_feed():
    sock = _install([(1, 0.1, [_veh('ego', 5.0), _veh('other', 6.0)])])
    for _ in fixs.steps():
        fixs.echo_all()
    assert _record_count(sock.sent[0]) == 2


def test_echo_all_limit_caps_the_reply():
    sock = _install([(1, 0.1, [_veh('ego'), _veh('a'), _veh('b')])])
    for _ in fixs.steps():
        fixs.echo_all(limit=1)
    assert _record_count(sock.sent[0]) == 1


def test_empty_tick_replies_with_no_records():
    """A quiet tick sends a header and nothing else.

    Not a placeholder record: that would put a vehicle with an empty id and
    zeroed fields on the wire every idle tick. The reference echo clients have
    always replied this way, so it is also what byte-identity requires.
    """
    sock = _install([(1, 0.1, [])])
    for _ in fixs.steps():
        pass
    assert len(sock.sent) == 1
    assert _record_count(sock.sent[0]) == 0


def test_shutdown_state_ends_the_loop_without_replying():
    sock = _install([(1, 0.1, [_veh('ego')]), (0, 0.2, [])], shutdown=False)
    ticks = [t for t in fixs.steps()]
    assert ticks == [pytest.approx(0.1)]
    assert len(sock.sent) == 1          # the shutdown tick is not answered


def test_break_still_answers_the_tick_it_leaves_on():
    """Walking away mid-tick would strand TrafficLayer waiting for a reply."""
    sock = _install([(1, 0.1, [_veh('ego')]), (1, 0.2, [_veh('ego')])])
    for _ in fixs.steps():
        break
    assert len(sock.sent) == 1


def test_ego_is_none_when_absent_from_the_feed():
    _install([(1, 0.1, [_veh('other')])])
    seen = []
    for _ in fixs.steps():
        seen.append(fixs.ego)
    assert seen == [None]


def _record_count(message):
    """Count vehicle records in one encoded FIXS message."""
    helper = MsgHelper()
    helper.set_vehicle_message_field(FIELDS)
    _, _, total = helper.depack_msg_header(message[:helper.msg_header_size])
    index = helper.msg_header_size
    count = 0
    while index < total:
        size, _type = helper.depack_msg_type(
            message[index:index + helper.msg_each_header_size])
        index += size
        count += 1
    return count
