"""fixs.traci without a simulator (#356).

Everything here runs against a fake socket, so it is part of the one CI-able path
(`python -m pytest tests/Python/unit/ -v`). What it pins down is the half of the
relay that does not need SUMO to be wrong:

  * the four values leaving the seam are the ones traci produced, byte for byte,
    including the payload -- which is the property the whole design rests on;
  * chunking round-trips a payload larger than MAX_RECORD_SIZE in both directions;
  * refusals happen BEFORE anything reaches the wire, and name what to use instead;
  * a reply is handed to the stock traci parsers unmodified.

The end-to-end counterpart, against a live co-simulation, is
tests/Python/TraciRelay/run_relay_test.ps1.
"""
import os
import struct
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", ".."))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "..", "CommonLib"))

traci_mod = pytest.importorskip("traci", reason="SUMO's traci package is not installed")

from CommonLib.fixs import _relay  # noqa: E402
import CommonLib.fixs.traci as fixstraci  # noqa: E402


# ---------------------------------------------------------------------------
# A socket that answers from a script, and remembers what it was asked
# ---------------------------------------------------------------------------
class FakeSocket:
    def __init__(self):
        self.sent = b""
        self.toRead = b""

    def sendall(self, data):
        self.sent += data

    def recv(self, n):
        chunk, self.toRead = self.toRead[:n], self.toRead[n:]
        return chunk


def encodeResponse(status, body, chunk=_relay.REQ_CHUNK):
    """Build the message TrafficLayer would send back."""
    pieces = [body[i:i + chunk] for i in range(0, len(body), chunk)] or [b""]
    records = []
    for i, piece in enumerate(pieces):
        rec = _relay._RSP_HEAD.pack(status, 1 if i < len(pieces) - 1 else 0, len(piece)) + piece
        records.append(_relay._REC.pack(_relay.REC_HEADER_SIZE + len(rec),
                                        _relay.MSG_TRACI_RESPONSE) + rec)
    total = _relay.HEADER_SIZE + sum(len(r) for r in records)
    return _relay._HEADER.pack(1, 0.0, total) + b"".join(records)


def decodeRequest(raw):
    """Parse what the client sent: (cmdID, varID, objID, payload)."""
    _, _, total = _relay._HEADER.unpack(raw[:_relay.HEADER_SIZE])
    assert total == len(raw), "header totalMsgSize disagrees with the bytes sent"
    pos = _relay.HEADER_SIZE
    payload = b""
    cmdID = varID = None
    objID = None
    while pos < total:
        recSize, msgType = _relay._REC.unpack(raw[pos:pos + _relay.REC_HEADER_SIZE])
        assert msgType == _relay.MSG_TRACI_REQUEST
        body = raw[pos + _relay.REC_HEADER_SIZE:pos + recSize]
        assert recSize <= _relay.MAX_RECORD_SIZE, "record exceeds the wire contract"
        cmdID, varID, idLen = _relay._REQ_HEAD.unpack(body[:_relay._REQ_HEAD.size])
        p = _relay._REQ_HEAD.size
        objID = body[p:p + idLen].decode("utf8")
        p += idLen
        _, chunkLen = _relay._REQ_TAIL.unpack(body[p:p + _relay._REQ_TAIL.size])
        p += _relay._REQ_TAIL.size
        payload += body[p:p + chunkLen]
        pos += recSize
    return cmdID, varID, objID, payload


@pytest.fixture
def wired(monkeypatch):
    """Pretend fixs is connected and holding a tick."""
    fixs = sys.modules[_relay.__package__]
    sock = FakeSocket()
    monkeypatch.setattr(fixs, "_sock", sock, raising=False)
    monkeypatch.setattr(fixs, "_armed", True, raising=False)
    monkeypatch.setattr(fixs, "_simTime", 1.0, raising=False)
    return sock


# ---------------------------------------------------------------------------
# The seam
# ---------------------------------------------------------------------------
def test_getter_sends_the_four_values_traci_produced(wired):
    # traci.vehicle.getSpeed("ego") is (0xa4, 0x40, "ego", no payload).
    # The reply is a real RESPONSE_GET_VEHICLE_VARIABLE message: length, response id
    # (0xa4 + 0x10), varID, objID, then TYPE_DOUBLE and the value -- 19 bytes, the
    # same length the #356 probe measured off the wire.
    reply = (bytes([19, 0xb4, 0x40]) + struct.pack("!i", 3) + b"ego" +
             bytes([0x0b]) + struct.pack("!d", 12.5))
    assert len(reply) == 19
    wired.toRead = encodeResponse(_relay.TRACI_OK, reply)

    speed = fixstraci.__getattr__("vehicle").getSpeed("ego")

    cmdID, varID, objID, payload = decodeRequest(wired.sent)
    assert (cmdID, varID, objID) == (0xa4, 0x40, "ego")
    assert payload == b""
    # parsed by stock traci, from bytes the relay did not touch
    assert speed == 12.5


def test_payload_is_tracis_own_pack_byte_for_byte(wired):
    # changeLane("ego", 1, 3.0) -- the compound case, and the one the #356 probe used
    # because it is the hardest shape: tc.TYPE_COMPOUND, a byte, and a double.
    expected = traci_mod.connection.Connection._pack(None, "tbd", 2, 1, 3.0)
    wired.toRead = encodeResponse(_relay.TRACI_OK, b"")
    fixstraci.__getattr__("vehicle").changeLane("ego", 1, 3.0)
    cmdID, varID, objID, payload = decodeRequest(wired.sent)
    assert (cmdID, varID, objID) == (0xc4, 0x13, "ego")
    assert payload == expected
    assert payload.hex() == "0f0000000208010b4008000000000000"  # measured in the probe


def test_large_payload_is_chunked_and_reassembled(wired):
    big = "y" * 20000
    wired.toRead = encodeResponse(_relay.TRACI_OK, b"")
    fixstraci.__getattr__("vehicle").setParameter("ego", "k", big)
    cmdID, varID, objID, payload = decodeRequest(wired.sent)
    # decodeRequest asserts every record stays under MAX_RECORD_SIZE, so reaching
    # here means it really was split -- and the payload came back whole.
    assert payload == traci_mod.connection.Connection._pack(None, "tss", 2, "k", big)
    assert len(wired.sent) > _relay.MAX_RECORD_SIZE


def test_large_reply_is_reassembled(wired):
    body = b"\x00" + os.urandom(30000)
    wired.toRead = encodeResponse(_relay.TRACI_OK, body, chunk=8000)
    status, got = _relay.request(0xa4, 0x40, "ego", b"")
    assert status == _relay.TRACI_OK
    assert got == body


# ---------------------------------------------------------------------------
# Errors and refusals
# ---------------------------------------------------------------------------
def test_sumo_error_becomes_TraCIException(wired):
    wired.toRead = encodeResponse(_relay.TRACI_ERROR, b"Vehicle 'nope' is not known.")
    with pytest.raises(traci_mod.TraCIException) as exc:
        fixstraci.__getattr__("vehicle").getSpeed("nope")
    assert "not known" in str(exc.value)


def test_refusal_becomes_FatalTraCIError(wired):
    wired.toRead = encodeResponse(_relay.TRACI_REFUSED, b"nope, TrafficLayer owns that")
    with pytest.raises(traci_mod.FatalTraCIError):
        fixstraci.__getattr__("vehicle").getSpeed("ego")


@pytest.mark.parametrize("cmdID,needle", [
    (0x02, "owns the clock"),
    (0x7F, "fixs.close"),
    (0x03, "only TraCI client"),
    (0x01, "reload the network"),
])
def test_owned_commands_are_refused_before_the_wire(wired, cmdID, needle):
    with pytest.raises(traci_mod.FatalTraCIError) as exc:
        fixstraci._conn._sendCmd(cmdID, None, None)
    assert needle in str(exc.value)
    assert wired.sent == b"", "a refused command must not reach the socket"


def test_subscriptions_are_refused_and_point_at_the_feed(wired):
    with pytest.raises(traci_mod.FatalTraCIError) as exc:
        fixstraci.__getattr__("vehicle").subscribe("ego", [0x40])
    assert "fixs.vehicle" in str(exc.value)
    assert wired.sent == b""


def test_refusal_covers_the_whole_subscribe_range(wired):
    for cmdID in (0x04, 0x0b, 0x54, 0x5b, 0x80, 0x8f, 0xd0, 0xdf):
        assert fixstraci._isSubscribe(cmdID, 0x40), hex(cmdID)
    # a subscription's varID is a pair of doubles, so shape alone is enough too
    assert fixstraci._isSubscribe(0xa4, (0.0, 100.0))
    # and an ordinary getter is not caught by either test
    assert not fixstraci._isSubscribe(0xa4, 0x40)


# ---------------------------------------------------------------------------
# The window
# ---------------------------------------------------------------------------
def test_call_outside_a_tick_is_refused(monkeypatch):
    fixs = sys.modules[_relay.__package__]
    monkeypatch.setattr(fixs, "_sock", FakeSocket(), raising=False)
    monkeypatch.setattr(fixs, "_armed", False, raising=False)
    with pytest.raises(_relay.RelayUnavailable) as exc:
        _relay.request(0xa4, 0x40, "ego", b"")
    assert "fixs.recv()" in str(exc.value)


def test_call_without_connect_is_refused(monkeypatch):
    fixs = sys.modules[_relay.__package__]
    monkeypatch.setattr(fixs, "_sock", None, raising=False)
    with pytest.raises(_relay.RelayUnavailable) as exc:
        _relay.request(0xa4, 0x40, "ego", b"")
    assert "fixs.connect" in str(exc.value)


def test_start_without_connect_names_fixs_connect(monkeypatch):
    fixs = sys.modules[_relay.__package__]
    monkeypatch.setattr(fixs, "_sock", None, raising=False)
    with pytest.raises(traci_mod.FatalTraCIError) as exc:
        fixstraci.start(["sumo", "-c", "whatever.sumocfg"])
    assert "fixs.connect" in str(exc.value)
