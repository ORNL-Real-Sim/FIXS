"""The wire between simulator and dyno cell (CommonLib/xil/link.py, #323).

Two transports behind one shape, so that a difference between an in-process run
and a bench run is the transport and not the model. These tests pin the shape
and the wire format; they do not test the network.

Run:  pytest tests/Python/unit/test_xil_link.py -v
"""

import os
import socket
import struct
import sys

import pytest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__),
                                                '..', '..', '..')))

from CommonLib.xil import (  # noqa: E402
    DEFAULT_MEASUREMENT_PORT, DEFAULT_REFERENCE_PORT, PACKET_SIZE,
    InProcessPair, UdpDynoSide, UdpSimulatorSide, pack, unpack,
)


class FakeClock:
    """So freshness can be tested without sleeping."""

    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


# ------------------------------------------------------------- wire format

def test_the_packet_matches_the_ornl_cell():
    """Two little-endian float32, 8 bytes. If this changes, our software stops
    being able to talk to their hardware without a translation layer."""
    assert PACKET_SIZE == 8
    raw = pack(13.5, -4.25)
    assert len(raw) == 8
    assert struct.unpack('<2f', raw) == pytest.approx((13.5, -4.25))


def test_ports_match_the_ornl_cell():
    assert DEFAULT_REFERENCE_PORT == 5010       # simulator -> dyno
    assert DEFAULT_MEASUREMENT_PORT == 5011     # dyno -> simulator


def test_pack_and_unpack_round_trip():
    v, s = unpack(pack(22.25, 30.0))
    assert v == pytest.approx(22.25)
    assert s == pytest.approx(30.0)


def test_a_short_packet_is_rejected_rather_than_misread():
    with pytest.raises(ValueError):
        unpack(b'\x00\x00\x00')


def test_steer_is_carried_even_though_nothing_acts_on_it():
    """The bench is longitudinal. The field is relayed so it exists when
    somebody wants it, not because anything here uses it."""
    _, steer = unpack(pack(10.0, 12.5))
    assert steer == pytest.approx(12.5)


# --------------------------------------------------------------- in process

def test_in_process_carries_a_reference_one_way_and_a_measurement_the_other():
    pair = InProcessPair()
    assert pair.dyno.latest_reference() is None
    assert pair.simulator.latest_measurement() is None

    pair.simulator.send_reference(17.0, 3.0)
    assert pair.dyno.latest_reference() == pytest.approx((17.0, 3.0))
    assert pair.simulator.latest_measurement() is None, 'the wire is not a loopback'

    pair.dyno.send_measurement(16.2, 2.5)
    assert pair.simulator.latest_measurement() == pytest.approx((16.2, 2.5))


def test_the_newest_value_wins():
    pair = InProcessPair()
    for v in (1.0, 2.0, 3.0):
        pair.simulator.send_reference(v)
    assert pair.dyno.latest_reference()[0] == pytest.approx(3.0)


def test_nothing_received_is_not_fresh():
    pair = InProcessPair()
    assert not pair.dyno.is_fresh()
    assert pair.dyno.age_s() is None


def test_freshness_expires_on_the_clock():
    clk = FakeClock()
    pair = InProcessPair(stale_after_s=0.15, clock=clk)
    pair.simulator.send_reference(10.0)
    assert pair.dyno.is_fresh()
    clk.t = 0.14
    assert pair.dyno.is_fresh()
    clk.t = 0.16
    assert not pair.dyno.is_fresh()
    assert pair.dyno.latest_reference() is not None, \
        'stale is not gone -- the caller decides what to do about age'


def test_a_stale_link_still_reports_its_last_value():
    """Whether to keep using it or fall back is the caller's call, and both are
    right somewhere, so the link does not choose."""
    clk = FakeClock()
    pair = InProcessPair(stale_after_s=0.05, clock=clk)
    pair.dyno.send_measurement(9.0)
    clk.t = 5.0
    assert not pair.simulator.is_fresh()
    assert pair.simulator.latest_measurement()[0] == pytest.approx(9.0)


# ---------------------------------------------------------------------- udp

def _free_port():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(('127.0.0.1', 0))
    p = s.getsockname()[1]
    s.close()
    return p


def test_udp_carries_both_directions_on_loopback():
    ref_port, meas_port = _free_port(), _free_port()
    sim = UdpSimulatorSide('127.0.0.1', tx_port=ref_port, rx_port=meas_port)
    dyn = UdpDynoSide('127.0.0.1', tx_port=meas_port, rx_port=ref_port)
    try:
        sim.send_reference(21.0, 1.5)
        got = None
        for _ in range(200):                    # free-running, so poll for it
            got = dyn.latest_reference()
            if got is not None:
                break
        assert got == pytest.approx((21.0, 1.5))

        dyn.send_measurement(20.4, 1.2)
        got = None
        for _ in range(200):
            got = sim.latest_measurement()
            if got is not None:
                break
        assert got == pytest.approx((20.4, 1.2))
    finally:
        sim.close()
        dyn.close()


def test_udp_polling_never_blocks_when_nothing_has_arrived():
    port = _free_port()
    sim = UdpSimulatorSide('127.0.0.1', tx_port=_free_port(), rx_port=port)
    try:
        assert sim.latest_measurement() is None
        assert not sim.is_fresh()
    finally:
        sim.close()


def test_udp_keeps_only_the_newest_of_a_burst():
    ref_port, meas_port = _free_port(), _free_port()
    sim = UdpSimulatorSide('127.0.0.1', tx_port=ref_port, rx_port=meas_port)
    dyn = UdpDynoSide('127.0.0.1', tx_port=meas_port, rx_port=ref_port)
    try:
        for v in (1.0, 2.0, 3.0, 4.0, 5.0):
            sim.send_reference(v)
        got = None
        for _ in range(200):
            got = dyn.latest_reference()
            if got is not None and got[0] == pytest.approx(5.0):
                break
        assert got[0] == pytest.approx(5.0)
    finally:
        sim.close()
        dyn.close()


def test_both_transports_present_the_same_interface():
    """The point of having two: swapping them must not change a caller."""
    for name in ('send_reference', 'latest_measurement', 'is_fresh', 'age_s'):
        assert hasattr(InProcessPair().simulator, name)
        assert hasattr(UdpSimulatorSide, name)
    for name in ('send_measurement', 'latest_reference', 'is_fresh', 'age_s'):
        assert hasattr(InProcessPair().dyno, name)
        assert hasattr(UdpDynoSide, name)
