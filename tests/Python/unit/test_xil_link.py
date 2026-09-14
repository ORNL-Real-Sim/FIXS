"""The wire between simulator and dyno cell (CommonLib/xil/link.py).

Pins the packet format and the shape both transports share. Does not test the
network.

Run:  pytest tests/Python/unit/test_xil_link.py -v
"""

import os
import socket
import struct
import sys

import pytest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__),
                                                '..', '..', '..')))

from CommonLib.xil.link import (LocalLink, UdpLink,  # noqa: E402
                                MEASUREMENT_PORT, PACKET_SIZE,
                                REFERENCE_PORT, pack, unpack)


class FakeClock(object):
    """So freshness can be tested without sleeping."""

    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


def free_port():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(('127.0.0.1', 0))
    port = s.getsockname()[1]
    s.close()
    return port


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
    ref, meas = free_port(), free_port()
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
    link = UdpLink('simulator', reference_port=free_port(),
                   measurement_port=free_port())
    try:
        assert link.recv() is None
        assert link.age() is None
    finally:
        link.close()


def test_udp_keeps_only_the_newest_of_a_burst():
    ref, meas = free_port(), free_port()
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
