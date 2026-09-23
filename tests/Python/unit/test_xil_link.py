"""The wire between simulator and dyno cell (CommonLib/xil/link.py).

Pins the packet format and the shape both transports share. Does not test the
network.

Run:  pytest tests/Python/unit/test_xil_link.py -v
"""

import os
import socket
import struct
import sys
import time

import pytest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__),
                                                '..', '..', '..')))

from CommonLib.xil.link import (LocalLink, TcpLink, UdpLink,  # noqa: E402
                                MEASUREMENT_PORT, PACKET_SIZE,
                                REFERENCE_PORT, pack, unpack)


class FakeClock(object):
    """So freshness can be tested without sleeping."""

    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


def free_port(kind=socket.SOCK_STREAM):
    """A port the OS just handed out FOR THIS PROTOCOL.

    The TCP and UDP port spaces are independent, so a number free for UDP says
    nothing about TCP -- and this used to pick every port with a UDP socket,
    including the ones a TcpLink then tried to bind. Windows refuses some of
    those with WinError 10013, an access error rather than "in use", which
    surfaces as a test dying in TcpLink.__init__ before any assertion runs.
    Measured: 1 full-suite run in 25.
    """
    s = socket.socket(socket.AF_INET, kind)
    s.bind(('127.0.0.1', 0))
    port = s.getsockname()[1]
    s.close()
    return port


def free_udp_port():
    return free_port(socket.SOCK_DGRAM)


def dyno_link(attempts=8):
    """A listening TcpLink on a port that actually accepted the bind.

    Naming a free port and binding it are two steps with a gap between them, and
    nothing can close that gap from here. Retrying is the honest fix: the
    alternative is a test that fails for a reason unrelated to what it asserts.
    """
    for i in range(attempts):
        port = free_port()
        try:
            return TcpLink('dyno', port=port), port
        except OSError:
            if i == attempts - 1:
                raise


# -------------------------------------------------------------- wire format

def test_the_packet_matches_the_ornl_cell():
    """Two little-endian float32, 8 bytes. If this changes, our software stops
    being able to talk to their hardware without a translation layer."""
    assert PACKET_SIZE == 8
    assert REFERENCE_PORT == 5010
    assert MEASUREMENT_PORT == 5011
    assert struct.unpack('<2f', pack(13.5, -4.25)) == pytest.approx((13.5, -4.25))


def test_pack_and_unpack_round_trip():
    speed, steer = unpack(pack(22.25, 30.0))
    assert speed == pytest.approx(22.25)
    assert steer == pytest.approx(30.0)


def test_a_short_packet_is_rejected_rather_than_misread():
    with pytest.raises(ValueError):
        unpack(b'\x00\x00\x00')


# --------------------------------------------------------------------- local

def test_local_carries_each_direction_separately():
    link = LocalLink()
    assert link.recv_reference() is None
    assert link.recv_measurement() is None

    link.send_reference(17.0, 3.0)
    assert link.recv_reference() == pytest.approx((17.0, 3.0))
    assert link.recv_measurement() is None, 'the wire is not a loopback'

    link.send_measurement(16.2, 2.5)
    assert link.recv_measurement() == pytest.approx((16.2, 2.5))


def test_the_newest_value_wins():
    link = LocalLink()
    for v in (1.0, 2.0, 3.0):
        link.send_reference(v)
    assert link.recv_reference()[0] == pytest.approx(3.0)


def test_age_reports_how_stale_a_value_is():
    clock = FakeClock()
    link = LocalLink(clock=clock)
    assert link.reference_age() is None
    link.send_reference(10.0)
    assert link.reference_age() == pytest.approx(0.0)
    clock.t = 0.4
    assert link.reference_age() == pytest.approx(0.4)
    assert link.recv_reference() is not None, \
        'stale is not gone -- the caller decides what to do about age'


# ----------------------------------------------------------------------- udp

def test_udp_carries_both_directions_on_loopback():
    ref, meas = free_udp_port(), free_udp_port()
    sim = UdpLink('simulator', reference_port=ref, measurement_port=meas)
    dyno = UdpLink('dyno', reference_port=ref, measurement_port=meas)
    try:
        sim.send_reference(21.0, 1.5)
        got = None
        for _ in range(200):                    # free-running, so poll for it
            got = dyno.recv_reference()
            if got is not None:
                break
        assert got == pytest.approx((21.0, 1.5))

        dyno.send_measurement(20.4, 1.2)
        got = None
        for _ in range(200):
            got = sim.recv_measurement()
            if got is not None:
                break
        assert got == pytest.approx((20.4, 1.2))
    finally:
        sim.close()
        dyno.close()


def test_udp_never_blocks_when_nothing_has_arrived():
    link = UdpLink('simulator', reference_port=free_udp_port(),
                   measurement_port=free_udp_port())
    try:
        assert link.recv() is None
        assert link.age() is None
    finally:
        link.close()


def test_udp_keeps_only_the_newest_of_a_burst():
    ref, meas = free_udp_port(), free_udp_port()
    sim = UdpLink('simulator', reference_port=ref, measurement_port=meas)
    dyno = UdpLink('dyno', reference_port=ref, measurement_port=meas)
    try:
        for v in (1.0, 2.0, 3.0, 4.0, 5.0):
            sim.send_reference(v)
        got = None
        for _ in range(200):
            got = dyno.recv_reference()
            if got is not None and got[0] == pytest.approx(5.0):
                break
        assert got[0] == pytest.approx(5.0)
    finally:
        sim.close()
        dyno.close()


def test_an_unknown_end_is_rejected():
    with pytest.raises(ValueError):
        UdpLink('middle')


def test_both_transports_present_the_same_calls():
    """The point of having two: swapping them must not change a caller."""
    for name in ('send_reference', 'recv_reference', 'reference_age',
                 'send_measurement', 'recv_measurement', 'measurement_age',
                 'close'):
        assert hasattr(LocalLink(), name), name
        assert hasattr(UdpLink, name), name
        assert hasattr(TcpLink, name), name


# ---------------------------------------------------------------------- tcp

def tcp_pair(timeout=2.0):
    """dyno first: it listens, the simulator end connects, and BOTH are up
    before the caller sends anything.

    The link is non-blocking at both ends on purpose: before the peer is there
    `send` does nothing and `recv` answers None, which is what a real bench does
    while the other end is still starting. That makes a send issued before the
    handshake a SILENT no-op -- the packet is not queued, it is dropped -- so a
    test that sends first is racing the connect, not testing the wire. The
    simulator end gives connect 0.05 s; under a loaded machine that expires.
    Measured: 2 of 20 full-suite runs lost their first sends exactly here.
    """
    dyno, port = dyno_link()
    sim = TcpLink('simulator', peer_ip='127.0.0.1', port=port)
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if sim._connected() is not None and dyno._connected() is not None:
            return sim, dyno, port
        time.sleep(0.005)
    sim.close()
    dyno.close()
    raise AssertionError('TcpLink pair did not connect within %.1f s' % timeout)


def poll(fn, timeout=2.0):
    """Wait for a non-None value, bounded by TIME rather than by a spin count.

    A fixed iteration budget is a race against the OS: 200 non-blocking reads
    can all complete before loopback has delivered anything, and the test then
    fails for running fast on a busy machine.
    """
    deadline = time.monotonic() + timeout
    while True:
        got = fn()
        if got is not None:
            return got
        if time.monotonic() >= deadline:
            return None
        time.sleep(0.001)


def poll_for(fn, want, timeout=2.0):
    """Wait until `fn` reports `want`; returns what it last reported."""
    deadline = time.monotonic() + timeout
    got = None
    while True:
        got = fn()
        if got is not None and got == want:
            return got
        if time.monotonic() >= deadline:
            return got
        time.sleep(0.001)


def test_tcp_carries_a_value_each_way():
    sim, dyno, _ = tcp_pair()
    try:
        sim.send_reference(21.0, 1.5)
        assert poll(dyno.recv_reference) == pytest.approx((21.0, 1.5))

        dyno.send_measurement(20.4, 1.2)
        assert poll(sim.recv_measurement) == pytest.approx((20.4, 1.2))
    finally:
        sim.close()
        dyno.close()


def test_tcp_answers_none_before_the_peer_is_there():
    """A run starts before the cell is listening, and must not block on it."""
    link = TcpLink('simulator', port=free_port())
    try:
        assert link.recv() is None
        assert link.age() is None
        link.send(12.0)                         # and this is not an error
    finally:
        link.close()


def test_tcp_waits_for_the_rest_of_a_split_packet():
    """The reason this is not UdpLink: a stream can hand over half a packet.

    Five bytes of an eight-byte packet is not three bytes lost, so they are held
    until the rest arrives rather than being decoded or discarded.
    """
    dyno, port = dyno_link()
    raw = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        raw.connect(('127.0.0.1', port))
        wire = pack(17.5, 0.25)
        raw.sendall(wire[:5])
        for _ in range(50):
            assert dyno.recv() is None          # nothing whole yet
        raw.sendall(wire[5:])
        assert poll(dyno.recv) == pytest.approx((17.5, 0.25))
    finally:
        raw.close()
        dyno.close()


def test_tcp_keeps_only_the_newest_of_a_burst():
    """Three references, one settled answer: the newest, and it stays.

    Asserted as "wait for 12.0, then it does not change" rather than "read 51
    times and the 51st is 12.0" -- how many reads it takes for all three to
    land is the OS's business, not the contract's.
    """
    sim, dyno, _ = tcp_pair()
    try:
        for v in (10.0, 11.0, 12.0):
            sim.send_reference(v)
        newest = pytest.approx((12.0, 0.0))
        assert poll_for(dyno.recv_reference, newest) == newest
        for _ in range(50):
            assert dyno.recv_reference() == newest   # sticky, never rewinds
    finally:
        sim.close()
        dyno.close()
