"""XilSetup.Transport udp against inprocess, over loopback.

A DynoSim behind UdpLink('dyno') in a thread stands in for the cell, stepping
once per reference it receives. The udp exchange waits for the answer to the
reference it just sent, and LocalLink carries the same float32 packet the wire
does, so it must reproduce the inprocess bench exactly, tick for tick.

Run:  pytest tests/Python/unit/test_xil_udp_lockstep.py -v
"""

import functools
import math
import os
import socket
import sys
import threading

import pytest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__),
                                                '..', '..', '..')))

import CommonLib.xil.link as linkmod                              # noqa: E402
from CommonLib.fixs import FixsError                              # noqa: E402
from CommonLib.fixs import xil as fixsxil                         # noqa: E402
from CommonLib.xil.driver import RobotDriver                      # noqa: E402
from CommonLib.xil.dynosim import Dyno, DynoSim                   # noqa: E402
from CommonLib.xil.vehicle import Vehicle                         # noqa: E402

VEHICLE = {'mass_kg': 2100.0}
DYNO = {'road_A_N': 111.0, 'roller_inertia_kgm2': 40.0}
DRIVER = {'max_accel_mps2': 1.8, 'max_decel_mps2': 1.8}
DT = 0.05


def free_udp_port():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(('127.0.0.1', 0))
    port = s.getsockname()[1]
    s.close()
    return port


@pytest.fixture
def ports(monkeypatch):
    """_Dyno builds UdpLink with the default ports; point it at free ones."""
    ref, meas = free_udp_port(), free_udp_port()
    monkeypatch.setattr(linkmod, 'UdpLink', functools.partial(
        linkmod.UdpLink, reference_port=ref, measurement_port=meas))
    return ref, meas


def cell(ref, meas, stop):
    """The far end: one DynoSim step per reference received."""
    sim = DynoSim(Vehicle(**VEHICLE), Dyno(**DYNO), RobotDriver(**DRIVER))
    link = linkmod.UdpLink.func('dyno', reference_port=ref, measurement_port=meas)
    try:
        while not stop.is_set():
            got = link.recv_wait(0.05)
            if got is not None:
                sim.step(got[0], DT)
                link.send_measurement(sim.speed)
    finally:
        link.close()


def references(n=400):
    """Ramps, a hold, a step down to zero and back: what a driver asks for."""
    out = []
    for k in range(n):
        t = k * DT
        out.append(10.0 if t < 5 else 10.0 + 4.0 * math.sin(t) if t < 12 else
                   0.0 if t < 15 else 6.0)
    return out


def test_udp_reproduces_inprocess_tick_for_tick(ports):
    stop = threading.Event()
    th = threading.Thread(target=cell, args=(*ports, stop), daemon=True)
    th.start()
    inp = fixsxil._Dyno('inprocess', vehicle=VEHICLE, dyno=DYNO, driver=DRIVER)
    udp = fixsxil._Dyno('udp', '127.0.0.1')
    try:
        diffs = [udp.exchange(r, DT) - inp.exchange(r, DT) for r in references()]
    finally:
        stop.set()
        th.join(2.0)
        udp.close()
    assert udp.misses == 0
    assert max(abs(d) for d in diffs) == 0.0          # both carry the float32 packet


def test_a_silent_cell_stops_the_run_after_enough_misses(ports, monkeypatch):
    monkeypatch.setattr(fixsxil, 'UDP_REPLY_TIMEOUT_S', 0.01)
    monkeypatch.setattr(fixsxil, 'UDP_MAX_MISSES', 5)
    udp = fixsxil._Dyno('udp', '127.0.0.1')
    try:
        for _ in range(4):
            assert udp.exchange(7.0, DT) == 7.0       # a miss passes the reference
        with pytest.raises(FixsError, match='has not answered'):
            udp.exchange(7.0, DT)
    finally:
        udp.close()
