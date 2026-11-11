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

from CommonLib.xil.link import (EMERGENCY, FAULT, HELLO, START, STOP, RigError,  # noqa: E402
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


def test_a_slightly_negative_speed_is_a_speed_not_a_code():
    assert code(-0.01) is None and code(-0.6) is None and code(-1.4) is None
    assert code(float('-inf')) is None
    fake, rig = _rig(lambda s, a: _bench(s, a) if code(s) == START else [(-0.02, 0.0)])
    try:
        rig.start(0.0)
        assert rig.exchange(0.0, 0.0, 0.05) == pytest.approx(-0.02)
        assert rig.misses == 0
    finally:
        rig.close()
        fake.stop()


def test_with_no_start_limit_it_waits_and_says_so(monkeypatch):
    import CommonLib.xil.link as link
    monkeypatch.setattr(link, 'REMIND_S', 0.3)
    said, seen = [], []

    def late(s, a):
        seen.append(s)
        return [(START, a)] if len(seen) >= 6 else []
    fake = FakeRig(late)
    fake.start()
    rig = UdpRig('127.0.0.1', fake.port, fake.reply_port, resend_s=0.2, log=said.append)
    try:
        assert rig.start_timeout_s == 0
        assert rig.start(0.0) == 0.0
        assert any('still waiting for the rig to answer START' in m for m in said)
    finally:
        rig.close()
        fake.stop()


def test_a_hello_from_a_rig_that_just_came_up_gets_start_again_at_once():
    import time
    seen = []

    def fresh(s, a):
        seen.append(s)
        return [(HELLO, 0.0)] if len(seen) == 1 else [(START, a)]
    fake, rig = _rig(fresh, resend_s=5.0)
    try:
        t0 = time.monotonic()
        assert rig.start(0.0) == 0.0
        assert time.monotonic() - t0 < 2.0 and len(seen) == 2
    finally:
        rig.close()
        fake.stop()


def test_stop_answered_while_moving_is_a_release_to_the_driver():
    said = []
    fake, rig = _rig(lambda s, a: [(START, 0.0)] if code(s) == START else [(STOP, 4.0)])
    rig._log = said.append
    try:
        rig.start(0.0)
        assert rig.stop() == pytest.approx(4.0)
        assert any('released at 4.00 m/s' in m for m in said)
    finally:
        rig.close()
        fake.stop()


def test_max_misses_zero_never_ends_the_run():
    fake, rig = _rig(lambda s, a: _bench(s, a) if code(s) == START else [], max_misses=0,
                     step_timeout_s=0.05)
    try:
        rig.start(0.0)
        assert [rig.exchange(5.0, 0.0, 0.05) for _ in range(8)] == [None] * 8
    finally:
        rig.close()
        fake.stop()


def test_from_config_takes_the_rig_limits_and_keeps_the_defaults():
    port = _free_udp_port()
    rig = UdpRig.fromConfig({'ip': '127.0.0.1', 'port': 5010,
                             'rig': {'StopTimeout': 120.0, 'MaxMisses': 0}}, port, log=lambda m: None)
    try:
        assert rig.peer == ('127.0.0.1', 5010)
        assert (rig.stop_timeout_s, rig.max_misses) == (120.0, 0)
        assert (rig.start_timeout_s, rig.step_timeout_s) == (0.0, 1.0)
    finally:
        rig.close()


def test_the_yaml_rig_block_is_checked():
    from CommonLib.ConfigHelper import ConfigHelper
    parse = ConfigHelper().parseRigLimits
    assert parse(None) == {}
    assert parse({'StopTimeout': 90, 'MaxMisses': '3'}) == {'StopTimeout': 90.0, 'MaxMisses': 3}
    assert parse({'EmergencyCode': True}) == {'EmergencyCode': True}
    with pytest.raises(SystemExit, match='true or false'):
        parse({'EmergencyCode': 'yes'})
    with pytest.raises(SystemExit, match='has no StopTimout'):
        parse({'StopTimout': 90})
    with pytest.raises(SystemExit, match='>= 0'):
        parse({'StartTimeout': -1})
    with pytest.raises(SystemExit, match='mapping'):
        parse(60)


@pytest.mark.parametrize('opt_in, sent', [(False, (0.0, -3.5)), (True, (EMERGENCY, -3.5))])
def test_an_emergency_step_is_a_code_only_for_a_rig_that_knows_it(opt_in, sent):
    fake, rig = _rig(lambda s, a: [(START, 0.0)] if code(s) == START else [(4.0, 0.0)],
                     emergency_code=opt_in)
    try:
        rig.start(0.0)
        assert rig.exchange(6.0, -3.5, 0.05, emergency=True) == pytest.approx(4.0)
        assert rig.exchange(6.0, 0.5, 0.05) == pytest.approx(4.0)
        assert fake.got[1] == pytest.approx(sent)
        assert fake.got[2] == pytest.approx((6.0, 0.5))
    finally:
        rig.close()
        fake.stop()


def test_from_config_takes_the_emergency_opt_in():
    rig = UdpRig.fromConfig({'ip': '127.0.0.1', 'port': 5010, 'rig': {'EmergencyCode': True}},
                            _free_udp_port(), log=lambda m: None)
    try:
        assert rig.emergency_code
    finally:
        rig.close()


def test_the_robot_brakes_harder_only_when_it_has_an_emergency_brake():
    from CommonLib.xil.driver import RobotDriver
    from CommonLib.xil.dynosim import Dyno, DynoSim
    from CommonLib.xil.vehicle import Vehicle

    def decel(emergency_decel, emergency):
        sim = DynoSim(Vehicle(mass_kg=2100.0), Dyno(),
                      RobotDriver(max_accel_mps2=2.0, max_decel_mps2=2.0,
                                  emergency_decel_mps2=emergency_decel))
        for _ in range(300):
            sim.step(12.0, 0.05)
        v0 = sim.speed
        for _ in range(20):
            sim.step(0.0, 0.05, a_ref=-3.5, emergency=emergency)
        v1 = sim.speed
        for _ in range(20):
            sim.step(0.0, 0.05, a_ref=-3.5, emergency=emergency)
        return v1 - sim.speed                              # over the second second of braking

    normal = decel(None, True)
    assert normal < 2.3                                   # no emergency brake: the envelope holds
    assert decel(3.5, False) == pytest.approx(normal)     # not told: no different
    assert decel(3.5, True) == pytest.approx(3.5, abs=0.4)


def test_rig_events_go_to_the_run_folder(tmp_path, monkeypatch):
    monkeypatch.setenv('FIXS_RUN_DIR', str(tmp_path))
    rig = UdpRig('127.0.0.1', 1, 0, log=lambda msg: None)
    rig._note('[rig] ready at 0.00 m/s')
    lines = (tmp_path / 'dyno_events.txt').read_text(encoding='utf-8').splitlines()
    assert len(lines) == 1 and lines[0].endswith('[rig] ready at 0.00 m/s')
    monkeypatch.delenv('FIXS_RUN_DIR')
    assert UdpRig('127.0.0.1', 1, 0, log=lambda msg: None).events is None
