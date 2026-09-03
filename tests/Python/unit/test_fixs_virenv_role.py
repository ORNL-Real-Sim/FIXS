"""fixs.py's virenv role: a bridge reports measured state; a controller cannot.

The two roles have opposite contracts and the split is the point of the role
parameter, so these tests assert BOTH halves:

* a bridge may :func:`fixs.emit` a record it built -- including for an id that
  never arrived -- because reporting the pose and speed its backend measured is
  the bridge's whole job;
* a controller may not, and the protection that stopped a controller putting the
  ego at the world origin (ORNL-Real-Sim/FIXS_Applications#25) is unchanged.

The fake-TrafficLayer fixtures are reused from test_fixs_api.py rather than
duplicated, so both role's tests are measured against the same encoder.
"""

import pytest

from CommonLib import fixs
from CommonLib.VehDataMsgDefs import VehData

from .test_fixs_api import FIELDS, _install, _recordCount, _veh


def _installVirenv(ticks, **kwargs):
    """_install(), then promote the connection to the bridge role."""
    sock = _install(ticks, **kwargs)
    fixs._role = 'virenv'
    return sock


def _decode(message, fields=None):
    """Pull the vehicle records back out of one encoded FIXS message."""
    from CommonLib.MsgHelper import MsgHelper
    helper = MsgHelper()
    helper.set_vehicle_message_field(list(fields or FIELDS))
    _, _, total = helper.depack_msg_header(message[:helper.msg_header_size])
    index = helper.msg_header_size
    out = []
    while index < total:
        size, _kind = helper.depack_msg_type(
            message[index:index + helper.msg_each_header_size])
        body = message[index + helper.msg_each_header_size:index + size]
        out.append(helper.depack_veh_data(body))
        index += size
    return out


# ---------------------------------------------------------------------------
# The role itself
# ---------------------------------------------------------------------------

def test_unknown_role_is_rejected_by_name():
    with pytest.raises(fixs.FixsError) as exc:
        fixs.connect('irrelevant.yaml', role='brige')       # typo on purpose
    assert 'brige' in str(exc.value)
    assert 'controller' in str(exc.value) and 'virenv' in str(exc.value)


def test_a_controller_cannot_emit():
    _install([(1, 0.1, [_veh('ego')])])
    fixs.recv()
    with pytest.raises(fixs.ProtocolError) as exc:
        fixs.emit(VehData(id='ego', speed=3.0))
    assert 'controller' in str(exc.value)
    assert "role='virenv'" in str(exc.value)
    fixs.close()


def test_a_controller_cannot_reach_the_raw_transport():
    _install([(1, 0.1, [_veh('ego')])])
    with pytest.raises(fixs.ProtocolError):
        fixs.transport()
    fixs.close()


def test_a_bridge_gets_the_socket_and_msg_helpers():
    _installVirenv([(1, 0.1, [_veh('ego')])])
    sockHelper, msgHelper = fixs.transport()
    assert sockHelper is fixs._helper
    assert msgHelper is fixs._helper.msg_helper
    fixs.close()


def test_close_returns_the_connection_to_the_controller_role():
    """A bridge's powers must not survive into the next connect()."""
    _installVirenv([(1, 0.1, [_veh('ego')])])
    fixs.close()
    assert fixs._role == 'controller'


# ---------------------------------------------------------------------------
# What emit() actually puts on the wire
# ---------------------------------------------------------------------------

def test_emit_reports_a_measured_record():
    sock = _installVirenv([(1, 0.1, [_veh('ego', 5.0)])])
    fixs.recv()
    fixs.emit(VehData(id='ego', speed=11.5, speedDesired=11.5))
    fixs.send()
    fixs.close()

    records = _decode(sock.sent[0])
    assert len(records) == 1
    assert records[0].id == 'ego'
    # 11.5, not the 5.0 that arrived: the bridge reports what it measured.
    assert records[0].speed == pytest.approx(11.5)


def test_emit_can_report_an_id_that_never_arrived():
    """A deferred ego spawn: CARLA owns the ego before the traffic sim has it."""
    sock = _installVirenv([(1, 0.1, [_veh('other', 2.0)])])
    fixs.recv()
    assert fixs.vehicle.get('ego') is None          # not in this tick's feed
    fixs.emit(VehData(id='ego', speed=7.25))
    fixs.send()
    fixs.close()

    records = _decode(sock.sent[0])
    assert [r.id for r in records] == ['ego']
    assert records[0].speed == pytest.approx(7.25)


def test_a_measurement_wins_over_an_echo_of_the_same_id():
    """emit + send([id]) reports the measurement, not the stale input.

    TrafficLayer keeps the last record for an id, so an echo appended after a
    measurement would silently overwrite it. The bridge is reporting; the echo is
    not information.
    """
    sock = _installVirenv([(1, 0.1, [_veh('ego', 5.0)])])
    fixs.recv()
    fixs.emit(VehData(id='ego', speed=11.5))
    fixs.send(['ego'])
    fixs.close()

    records = _decode(sock.sent[0])
    assert len(records) == 1, 'the id was reported twice'
    assert records[0].speed == pytest.approx(11.5)


def test_emitted_records_do_not_leak_into_the_next_tick():
    sock = _installVirenv([(1, 0.1, [_veh('ego')]), (1, 0.2, [_veh('ego')])])
    fixs.recv()
    fixs.emit(VehData(id='ego', speed=1.0))
    fixs.send()
    fixs.recv()
    fixs.send()
    fixs.close()

    assert _recordCount(sock.sent[0]) == 1
    assert _recordCount(sock.sent[1]) == 0


def test_emit_needs_a_tick_to_answer():
    _installVirenv([(1, 0.1, [_veh('ego')])])
    with pytest.raises(fixs.ProtocolError) as exc:
        fixs.emit(VehData(id='ego'))
    assert 'recv' in str(exc.value)
    fixs.close()


def test_emit_rejects_something_that_is_not_a_record():
    _installVirenv([(1, 0.1, [_veh('ego')])])
    fixs.recv()
    with pytest.raises(TypeError):
        fixs.emit({'id': 'ego', 'speed': 3.0})
    fixs.close()
