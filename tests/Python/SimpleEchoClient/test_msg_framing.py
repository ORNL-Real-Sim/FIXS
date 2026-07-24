"""Simulator-free regression test for the message framing fix (#176).

Background: MsgHelper.pack_veh_data used to return the body-only size, so
SocketHelper.sendData under-declared total_msg_size by msg_each_header_size (3 bytes)
PER vehicle. A 1-vehicle message self-corrected by luck; with >=2 vehicles the declared
header total was shorter than the bytes actually sent, so a receiver stopped early and
desynced the stream -- which is what deadlocked / crashed TrafficLayer when a Python
client (e.g. a centralized CAV controller) echoed multiple vehicles back.

This test drives the REAL CommonLib code (no sockets, no simulator): it packs N vehicles
with SocketHelper.sendData, then parses the bytes back with SocketHelper.recv_data via an
in-memory fake socket. To catch the desync it concatenates TWO messages back to back and
checks the receiver recovers every vehicle from BOTH and consumes the stream exactly --
a framing error in the first message corrupts the second.

Run:  <realsim_python> test_msg_framing.py
"""
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).parents[3]))

from CommonLib.MsgHelper import MsgHelper
from CommonLib.SocketHelper import SocketHelper
from CommonLib.VehDataMsgDefs import VehData

FIELDS = ['id', 'type', 'vehicleClass', 'speed', 'acceleration', 'speedDesired']


def make_veh(i):
    v = VehData()
    v.id = f'veh{i}'          # variable-length id so record sizes differ (33/35-style)
    v.type = 'car'
    v.vehicleClass = 'passenger'
    v.speed = float(i)
    v.acceleration = 0.25
    v.speedDesired = 10.0
    return v


def _make_helper():
    """A SocketHelper wired for pack/parse without a ConfigHelper or a socket."""
    mh = MsgHelper()
    mh.set_vehicle_message_field(FIELDS)
    sh = SocketHelper.__new__(SocketHelper)
    sh.msg_helper = mh
    sh.msg_header_size = mh.msg_header_size
    sh.msg_each_header_size = mh.msg_each_header_size
    sh.enable_verbose_log = False
    sh.vehicle_data_send_list = []
    sh.traffic_light_data_send_list = []
    sh.detector_data_send_list = []
    sh.vehicle_data_receive_list = []
    sh.traffic_light_data_receive_list = []
    sh.detector_data_receive_list = []
    return sh


class _CaptureSock:
    """sendall() target: just accumulate the bytes."""
    def __init__(self):
        self.data = bytearray()

    def sendall(self, b):
        self.data += b


class _ReplaySock:
    """recv() source over a fixed byte buffer (returns full requested length, loopback-style)."""
    def __init__(self, data):
        self.buf = bytes(data)
        self.pos = 0

    def recv(self, n):
        chunk = self.buf[self.pos:self.pos + n]
        self.pos += len(chunk)
        return chunk

    def remaining(self):
        return len(self.buf) - self.pos


def pack_message(n_veh, sim_time):
    """Pack n_veh vehicles into one wire message using the real sendData."""
    sh = _make_helper()
    sh.vehicle_data_send_list = [make_veh(i) for i in range(n_veh)]
    cap = _CaptureSock()
    sh.sendData(1, sim_time, cap)
    return bytes(cap.data)


def check(n_first, n_second):
    """Two complementary checks:
    1. Invariant: the total_msg_size written into the header equals the actual bytes sent
       (the precise thing the bug broke -- off by msg_each_header_size per vehicle).
    2. Behaviour: two back-to-back messages are recovered exactly via the real recv_data
       (a short total in message1 leaves bytes that desync message2)."""
    errs = []
    mh = MsgHelper(); mh.set_vehicle_message_field(FIELDS)

    # --- check 1: header total == actual bytes, for each message size ---
    for n in (n_first, n_second):
        msg = pack_message(n, 1.0)
        _, _, declared_total = mh.depack_msg_header(msg[:mh.msg_header_size])
        if declared_total != len(msg):
            errs.append(f'n={n}: header total={declared_total} != actual bytes={len(msg)} '
                        f'(off by {len(msg) - declared_total} = {n} x msg_each_header_size)')

    # --- check 2: back-to-back round trip through the real recv_data ---
    stream = pack_message(n_first, 1.0) + pack_message(n_second, 2.0)
    sock = _ReplaySock(stream)
    sh = _make_helper()
    try:
        sh.recv_data(sock)
        first = list(sh.vehicle_data_receive_list)
        sh.vehicle_data_receive_list = []
        sh.recv_data(sock)
        second = list(sh.vehicle_data_receive_list)
    except Exception as e:
        errs.append(f'recv_data raised {type(e).__name__}: {e} (stream desync)')
        return errs

    if len(first) != n_first:
        errs.append(f'message1: recovered {len(first)} vehicles, expected {n_first}')
    if len(second) != n_second:
        errs.append(f'message2: recovered {len(second)} vehicles, expected {n_second} '
                    f'(a short total in message1 desyncs message2)')
    if sock.remaining() != 0:
        errs.append(f'{sock.remaining()} leftover bytes -- stream not consumed exactly')
    if [v.id.strip() for v in first] != [f'veh{i}' for i in range(n_first)]:
        errs.append('message1 ids mismatch')
    return errs


def main():
    cases = [(1, 1), (2, 2), (5, 5), (13, 13), (40, 40), (1, 13), (13, 1)]
    failed = 0
    for a, b in cases:
        errs = check(a, b)
        if errs:
            failed += 1
            print(f'FAIL  ({a},{b}): ' + '; '.join(errs))
        else:
            print(f'PASS  ({a},{b}): both messages recovered, stream consumed exactly')
    if failed:
        print(f'\n{failed} case(s) FAILED')
        return 1
    print('\nAll framing cases passed (#176 regression).')
    return 0


if __name__ == '__main__':
    sys.exit(main())
