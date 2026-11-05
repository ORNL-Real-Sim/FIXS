"""UdpRig and the run start/stop codes (CommonLib/xil/link.py, #473).

A fake rig on loopback answers the way the wire says a rig must: once per command,
START/STOP acknowledged with the code in the speed slot.

Run:  pytest tests/Python/unit/test_xil_rig.py -v
"""
import os
import socket
import sys
import threading

import pytest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..')))

from CommonLib.xil.link import (FAULT, HELLO, START, STOP, RigError,  # noqa: E402
                                UdpRig, code, pack, unpack)


def _free_udp_port():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(('127.0.0.1', 0))
    port = s.getsockname()[1]
    s.close()
    return port


class FakeRig(threading.Thread):
    """Answers on loopback. `answer(speed, second)` returns the packets to send back."""

    def __init__(self, answer):
        super().__init__(daemon=True)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('127.0.0.1', 0))
        self.sock.settimeout(0.2)
        self.port = self.sock.getsockname()[1]
        self.reply_port = _free_udp_port()
        self.answer = answer
        self.got = []
        self.done = False

    def run(self):
        while not self.done:
            try:
                data, _ = self.sock.recvfrom(64)
            except (socket.timeout, ConnectionResetError):
                continue
            except OSError:
                return
            pkt = unpack(data)
            self.got.append(pkt)
            for out in self.answer(*pkt) or ():
                self.sock.sendto(pack(*out), ('127.0.0.1', self.reply_port))

    def stop(self):
        self.done = True
        self.join(2)
        self.sock.close()


def _rig(answer, **kw):
    fake = FakeRig(answer)
    fake.start()
    kw.setdefault('step_timeout_s', 0.3)
    kw.setdefault('resend_s', 0.2)
    rig = UdpRig('127.0.0.1', fake.port, fake.reply_port, log=lambda m: None, **kw)
    return fake, rig


def _bench(speed, second):
    """A rig that settles instantly, reaches 90% of each command, and stops."""
    c = code(speed)
    if c in (START, STOP):
        return [(c, second if c == START else 0.0)]
    return [(0.9 * speed, 0.0)]


def test_codes_are_negative_whole_numbers_and_speeds_are_not_codes():
    assert code(0.0) is None and code(12.5) is None and code(float('nan')) is None
    assert [code(float(c)) for c in (HELLO, START, STOP, FAULT)] == [-1, -2, -3, -4]
    assert unpack(pack(START, 3.0)) == (-2.0, 3.0)      # exact in float32


def test_a_run_is_start_steps_stop():
    fake, rig = _rig(_bench)
    try:
        assert rig.start(0.0) == 0.0
        assert rig.exchange(10.0, 0.5, 0.05) == pytest.approx(9.0)
        assert rig.stop() == 0.0
        assert [code(s) for s, _ in fake.got] == [START, None, STOP]
    finally:
        rig.close()
        fake.stop()


def test_start_carries_the_initial_speed_and_returns_where_the_rig_settled():
    fake, rig = _rig(lambda s, a: [(START, a - 0.1)] if code(s) == START else [(s, 0.0)])
    try:
        assert rig.start(6.0) == pytest.approx(5.9)
        assert fake.got[0] == (START, 6.0)
    finally:
        rig.close()
        fake.stop()


def test_start_is_resent_until_a_late_rig_answers():
    seen = []

    def late(s, a):
        seen.append(s)
        return [(START, a)] if len(seen) >= 3 else []
    fake, rig = _rig(late)
    try:
        assert rig.start(0.0) == 0.0
        assert len(seen) == 3 and all(code(s) == START for s in seen)
    finally:
        rig.close()
        fake.stop()


def test_no_start_answer_is_an_error_not_a_hang():
    fake, rig = _rig(lambda s, a: [], start_timeout_s=0.6)
    try:
        with pytest.raises(RigError, match='did not answer START'):
            rig.start(0.0)
    finally:
        rig.close()
        fake.stop()


def test_a_negative_command_cannot_be_read_as_a_code():
    fake, rig = _rig(_bench)
    try:
        rig.start(0.0)
        rig.exchange(-0.4, 0.0, 0.05)
        assert fake.got[-1][0] == 0.0
    finally:
        rig.close()
        fake.stop()


def test_a_fault_ends_the_run():
    fake, rig = _rig(lambda s, a: _bench(s, a) if code(s) == START else [(FAULT, 3.0)])
    try:
        rig.start(0.0)
        with pytest.raises(RigError, match='fault'):
            rig.exchange(5.0, 0.0, 0.05)
    finally:
        rig.close()
        fake.stop()


def test_a_hello_mid_run_means_the_rig_restarted():
    fake, rig = _rig(lambda s, a: _bench(s, a) if code(s) == START else [(HELLO, 0.0)])
    try:
        rig.start(0.0)
        with pytest.raises(RigError, match='restarted'):
            rig.exchange(5.0, 0.0, 0.05)
    finally:
        rig.close()
        fake.stop()


def test_a_late_acknowledgement_is_not_taken_as_a_speed():
    fake, rig = _rig(lambda s, a: [(START, 0.0)] if code(s) == START
                     else [(START, 0.0), (0.9 * s, 0.0)])
    try:
        rig.start(0.0)
        assert rig.exchange(10.0, 0.0, 0.05) == pytest.approx(9.0)
    finally:
        rig.close()
        fake.stop()


def test_unanswered_steps_are_none_until_too_many_in_a_row():
    fake, rig = _rig(lambda s, a: _bench(s, a) if code(s) == START else [], max_misses=3,
                     step_timeout_s=0.1)
    try:
        rig.start(0.0)
        assert rig.exchange(5.0, 0.0, 0.05) is None
        assert rig.exchange(5.0, 0.0, 0.05) is None
        with pytest.raises(RigError, match='3 steps in a row'):
            rig.exchange(5.0, 0.0, 0.05)
        assert rig.misses == 3
    finally:
        rig.close()
        fake.stop()


def test_stop_without_an_answer_returns_none_and_does_not_raise():
    fake, rig = _rig(lambda s, a: _bench(s, a) if code(s) == START else [], stop_timeout_s=0.4)
    try:
        rig.start(0.0)
        assert rig.stop() is None
        assert rig.stop() is None           # already stopped: nothing sent
        assert sum(1 for s, _ in fake.got if code(s) == STOP) >= 1
    finally:
        rig.close()
        fake.stop()
