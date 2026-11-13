"""fixs.sumo.traci against a fake socket: the half of the relay that needs no SUMO."""
import json
import os
import struct
import sys
import types

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", ".."))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "..", "CommonLib"))

traci_mod = pytest.importorskip("traci", reason="SUMO's traci package is not installed")

from CommonLib.fixs import _relay  # noqa: E402
import CommonLib.fixs.sumo.traci as shim  # noqa: E402

FIXS = sys.modules[_relay.__package__]


class FakeSocket:
    def __init__(self, replies=b""):
        self.sent = b""
        self.toRead = replies

    def sendall(self, data):
        self.sent += data

    def recv(self, n):
        chunk, self.toRead = self.toRead[:n], self.toRead[n:]
        return chunk


def response(status, body, chunk=8000):
    pieces = [body[i:i + chunk] for i in range(0, len(body), chunk)] or [b""]
    records = []
    for i, piece in enumerate(pieces):
        rec = _relay._RSP_HEAD.pack(status, 1 if i < len(pieces) - 1 else 0, len(piece)) + piece
        records.append(_relay._REC.pack(_relay.REC_HEADER_SIZE + len(rec), _relay.MSG_TRACI_RESPONSE) + rec)
    return _relay._HEADER.pack(1, 0.0, _relay.HEADER_SIZE + sum(map(len, records))) + b"".join(records)


def decode(raw):
    """(cmdID, varID, objID, payload, records) from one request message."""
    _, _, total = _relay._HEADER.unpack_from(raw)
    assert total == len(raw)
    pos, payload, n = _relay.HEADER_SIZE, b"", 0
    while pos < total:
        size, kind = _relay._REC.unpack_from(raw, pos)
        assert kind == _relay.MSG_TRACI_REQUEST and size <= _relay.MAX_RECORD_SIZE
        body = raw[pos + _relay.REC_HEADER_SIZE:pos + size]
        cmd, var, idLen = _relay._REQ_HEAD.unpack_from(body)
        p = _relay._REQ_HEAD.size
        obj = body[p:p + idLen].decode()
        _, chunkLen = _relay._REQ_TAIL.unpack_from(body, p + idLen)
        payload += body[p + idLen + _relay._REQ_TAIL.size:][:chunkLen]
        pos, n = pos + size, n + 1
    return cmd, var, obj, payload, n


@pytest.fixture
def held(monkeypatch):
    """fixs connected and holding a tick."""
    sock = FakeSocket()
    monkeypatch.setattr(FIXS, "_sock", sock, raising=False)
    monkeypatch.setattr(FIXS, "_armed", True, raising=False)
    monkeypatch.setattr(FIXS, "_simTime", 1.0, raising=False)
    return sock


def speed_reply(value):
    # RESPONSE_GET_VEHICLE_VARIABLE: length, 0xb4, VAR_SPEED, objID, TYPE_DOUBLE, value.
    return bytes([19, 0xb4, 0x40]) + struct.pack("!i", 3) + b"ego" + bytes([0x0b]) + struct.pack("!d", value)


# --- the seam --------------------------------------------------------------------

def test_getter_sends_tracis_values_and_parses_the_reply_unmodified(held):
    held.toRead = response(_relay.TRACI_OK, speed_reply(12.5))
    assert shim.vehicle.getSpeed("ego") == 12.5
    assert decode(held.sent)[:4] == (0xa4, 0x40, "ego", b"")


def test_payload_is_tracis_own_pack(held):
    held.toRead = response(_relay.TRACI_OK, b"")
    shim.vehicle.changeLane("ego", 1, 3.0)
    cmd, var, obj, payload, _ = decode(held.sent)
    assert (cmd, var, obj) == (0xc4, 0x13, "ego")
    assert payload == traci_mod.connection.Connection._pack(None, "tbd", 2, 1, 3.0)
    assert payload.hex() == "0f0000000208010b4008000000000000"


def test_large_request_is_split_into_records_and_reassembles(held):
    held.toRead = response(_relay.TRACI_OK, b"")
    shim.vehicle.setParameter("ego", "k", "y" * 20000)
    *_, payload, records = decode(held.sent)
    assert records == 3
    assert payload == traci_mod.connection.Connection._pack(None, "tss", 2, "k", "y" * 20000)


def test_large_reply_is_reassembled(held):
    body = os.urandom(30000)
    held.toRead = response(_relay.TRACI_OK, body)
    assert _relay.request(0xa4, 0x40, "ego", b"") == (_relay.TRACI_OK, body)


# --- errors and refusals -----------------------------------------------------------

def test_sumo_error_is_TraCIException_with_sumos_text(held):
    held.toRead = response(_relay.TRACI_ERROR, b"Vehicle 'nope' is not known.")
    with pytest.raises(traci_mod.TraCIException, match="not known"):
        shim.vehicle.getSpeed("nope")


def test_refusal_is_FatalTraCIError(held):
    held.toRead = response(_relay.TRACI_REFUSED, b"not relaying")
    with pytest.raises(traci_mod.FatalTraCIError, match="not relaying"):
        shim.vehicle.getSpeed("ego")


@pytest.mark.parametrize("cmdID", [0x7F, 0x03, 0x01, 0x02, 0x7E])
def test_owned_commands_never_reach_the_wire(held, cmdID):
    with pytest.raises(traci_mod.FatalTraCIError):
        shim._conn._sendCmd(cmdID, None, None)
    assert held.sent == b""


def speed_block(value, objID="ego"):
    # A RESPONSE_SUBSCRIBE_VEHICLE_VARIABLE block: VAR_SPEED, status OK, TYPE_DOUBLE, value.
    body = (bytes([0xe4]) + struct.pack("!i", len(objID)) + objID.encode() + bytes([1, 0x40, 0x00, 0x0b]) +
            struct.pack("!d", value))
    return b"\x00" + struct.pack("!i", 5 + len(body)) + body


@pytest.fixture
def no_subscriptions():
    yield
    shim._conn._subscribed = False
    for results in shim._conn._subscriptionMapping.values():
        results.reset()


def test_subscribe_places_it_through_trafficlayer_and_reads_the_first_answer(held, no_subscriptions):
    held.toRead = response(_relay.TRACI_OK, speed_block(12.5))
    shim.vehicle.subscribeLeader("ego", 50.0)
    cmd, var, obj, payload, _ = decode(held.sent)
    assert (cmd, var, obj) == (0xd4, _relay.TRACI_SUBSCRIPTION_VAR, "ego")
    begin, end, n, varID, hasParam = struct.unpack_from("!ddBBB", payload)
    assert (n, varID, hasParam) == (1, 0x68, 1)
    assert payload[struct.calcsize("!ddBBB"):] == struct.pack("!Bd", 0x0b, 50.0)
    assert shim.vehicle.getSubscriptionResults("ego") == {0x40: 12.5}


def test_each_step_replaces_the_results_with_trafficlayers_answer(held, no_subscriptions):
    held.toRead = response(_relay.TRACI_OK, speed_block(1.0))
    shim.vehicle.subscribe("ego", [0x40])
    held.sent = b""
    held.toRead = response(_relay.TRACI_OK, struct.pack("!i", 1) + speed_block(2.0, "car"))
    shim._conn._refreshSubscriptions()
    assert decode(held.sent)[:4] == (0x02, _relay.TRACI_SUBSCRIPTION_VAR, "", b"")
    assert shim.vehicle.getAllSubscriptionResults() == {"car": {0x40: 2.0}}


def test_no_subscription_costs_no_round_trip(held, no_subscriptions):
    shim._conn._refreshSubscriptions()
    assert held.sent == b""


def test_context_subscribe_is_refused_before_the_wire(held):
    with pytest.raises(traci_mod.FatalTraCIError, match="context subscriptions"):
        shim.vehicle.subscribeContext("ego", 0xa4, 50.0, [0x40])
    assert held.sent == b""


def test_multi_value_parameter_is_refused_before_the_wire(held, no_subscriptions):
    with pytest.raises(traci_mod.FatalTraCIError, match="parameter"):
        shim.vehicle.subscribe("ego", [0x68], parameters={0x68: ("dd", 1.0, 2.0)})
    assert held.sent == b""


def test_every_subscribe_command_range_is_caught():
    for cmdID in (0x04, 0x0b, 0x54, 0x5b, 0x80, 0x8f, 0xd0, 0xdf):
        assert shim._isSubscribe(cmdID, 0x40), hex(cmdID)
    assert shim._isSubscribe(0xa4, (0.0, 100.0))
    assert not shim._isSubscribe(0xa4, 0x40)


def test_no_connection_names_connect(monkeypatch):
    monkeypatch.setattr(FIXS, "_sock", None, raising=False)
    with pytest.raises(_relay.RelayUnavailable, match="connect"):
        _relay.request(0xa4, 0x40, "ego", b"")


def test_no_held_tick_names_simulationStep(monkeypatch):
    monkeypatch.setattr(FIXS, "_sock", FakeSocket(), raising=False)
    monkeypatch.setattr(FIXS, "_armed", False, raising=False)
    with pytest.raises(_relay.RelayUnavailable, match="simulationStep"):
        _relay.request(0xa4, 0x40, "ego", b"")


# --- the script's own lifecycle: start / simulationStep / close ----------------------

class FakeTicks:
    """Stands in for fixs.connect / recv / send / close and records the calls."""

    def __init__(self, monkeypatch, times, end_with_shutdown=False):
        self.calls, self.times, self.end = [], list(times), end_with_shutdown
        monkeypatch.setattr(FIXS, "_sock", None, raising=False)
        monkeypatch.setattr(FIXS, "_armed", False, raising=False)
        for name in ("connect", "recv", "send", "close"):
            monkeypatch.setattr(FIXS, name, getattr(self, name))

    def connect(self, *a, **k):
        self.calls.append("connect")
        FIXS._sock = FakeSocket()

    def recv(self):
        self.calls.append("recv")
        if not self.times:
            if self.end:
                raise FIXS.Shutdown()
            raise AssertionError("recv past the last tick")
        FIXS.sim = types.SimpleNamespace(time=self.times.pop(0))
        FIXS._armed = True

    def send(self):
        self.calls.append("send")
        FIXS._armed = False

    def close(self):
        self.calls.append("close")


def test_start_reports_the_scenario_and_connects_without_a_round_trip(monkeypatch, tmp_path):
    handoff = tmp_path / "handoff.json"
    monkeypatch.setenv("FIXS_HANDOFF", str(handoff))
    ticks = FakeTicks(monkeypatch, [])
    cfg = tmp_path / "mine.sumocfg"
    cfg.write_text("<configuration/>")

    version = shim.start(["sumo", "-c", str(cfg), "--step-length", "0.1"])

    assert json.loads(handoff.read_text())["sumocfg"] == str(cfg.resolve())
    assert ticks.calls == ["connect"]
    assert version[0] == traci_mod.constants.TRACI_VERSION


def test_first_step_only_receives_later_steps_hand_the_tick_back(monkeypatch):
    ticks = FakeTicks(monkeypatch, [29100.0, 29100.1, 29100.2])
    shim.start(["sumo"])
    shim.simulationStep()
    shim.simulationStep()
    shim.simulationStep()
    assert ticks.calls == ["connect", "recv", "send", "recv", "send", "recv"]


def test_a_call_before_the_first_step_pulls_the_first_tick_which_that_step_then_consumes(monkeypatch):
    # while traci.simulation.getMinExpectedNumber() > 0: traci.simulationStep()
    ticks = FakeTicks(monkeypatch, [28800.1, 28800.2])
    reply = bytes([10, 0xbb, 0x7d]) + struct.pack("!i", 0) + bytes([0x09]) + struct.pack("!i", 42)
    shim.start(["sumo"])
    FIXS._sock.toRead = response(_relay.TRACI_OK, reply)
    assert shim.simulation.getMinExpectedNumber() == 42
    shim.simulationStep()
    assert FIXS.sim.time == 28800.1
    shim.simulationStep()
    assert FIXS.sim.time == 28800.2
    assert ticks.calls == ["connect", "recv", "send", "recv"]


def test_a_call_after_the_first_step_without_a_held_tick_still_names_simulationStep(monkeypatch):
    FakeTicks(monkeypatch, [1.0])
    shim.start(["sumo"])
    shim.simulationStep()
    FIXS._armed = False
    with pytest.raises(_relay.RelayUnavailable, match="simulationStep"):
        shim.vehicle.getSpeed("ego")


def test_step_to_a_target_time_advances_until_it_is_reached(monkeypatch):
    ticks = FakeTicks(monkeypatch, [10.0, 10.1, 10.2, 10.3])
    shim.start(["sumo"])
    shim.simulationStep(10.2)
    assert FIXS.sim.time == 10.2
    assert ticks.calls.count("recv") == 3


def test_end_of_run_is_native_tracis_error(monkeypatch):
    FakeTicks(monkeypatch, [5.0], end_with_shutdown=True)
    shim.start(["sumo"])
    shim.simulationStep()
    with pytest.raises(traci_mod.FatalTraCIError, match="closed by SUMO"):
        shim.simulationStep()


def test_close_is_fixs_close(monkeypatch):
    ticks = FakeTicks(monkeypatch, [])
    shim.close()
    assert ticks.calls == ["close"]
