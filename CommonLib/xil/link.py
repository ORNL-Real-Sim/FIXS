"""The wire between the simulator and the dyno cell (#323).

One packet format, three transports. ``LocalLink`` runs both ends in one
process; ``UdpLink`` and ``TcpLink`` put them on a network. Same three calls
either way, so a difference between a laptop run and a bench run is the
transport and not the model.

    send(speed, steer)          push a value to the other end
    recv()                      newest value received, or None
    age()                       seconds since it arrived, or None

Wire format matches the ORNL cell so our software can talk to it directly: two
little-endian float32, 8 bytes, references on :5010 and measurements on :5011.
Steer is carried because the packet carries it; nothing here acts on it, because
this bench is longitudinal.

``UdpRig`` is the lockstep alternative for a rig that answers every command
once, with START/STOP codes in the speed slot (see ``HELLO``..``FAULT``).

UdpLink is free-running with no handshake. The newest datagram wins and nothing
blocks. TCP costs a connection and gives back delivery and ordering; it never
blocks either, so a run starts before the far end is listening. A caller that
has gone stale decides for itself what to do about it -- "keep the last value"
and "fall back to your own reference" are both right somewhere, so the link does
not choose.
"""

import math
import socket
import struct
import time

_PACKET = struct.Struct('<2f')
PACKET_SIZE = _PACKET.size                  # 8
REFERENCE_PORT = 5010                       # simulator -> dyno
MEASUREMENT_PORT = 5011                     # dyno -> simulator
# Port slot k >= 1 (run_cosim --slot) moves both into its block: 20000 + 100k + 70 / + 71.
_SLOT_BASE, _SLOT_STRIDE, _SLOT_DYNO = 20000, 100, 70


def slotPorts(slot=0):
    """(reference_port, measurement_port) for a run on port slot `slot`."""
    slot = int(slot or 0)
    if slot <= 0:
        return REFERENCE_PORT, MEASUREMENT_PORT
    base = _SLOT_BASE + _SLOT_STRIDE * slot + _SLOT_DYNO
    return base, base + 1


def pack(speed, steer=0.0):
    return _PACKET.pack(float(speed), float(steer))


def unpack(data):
    if len(data) < PACKET_SIZE:
        raise ValueError('packet is %d bytes, expected %d'
                         % (len(data), PACKET_SIZE))
    speed, steer = _PACKET.unpack(data[:PACKET_SIZE])
    return float(speed), float(steer)


class LocalLink(object):
    """Both ends in one process. No latency, no loss, no jitter.

    The baseline: whatever a UDP run does differently from this is transport.
    Values pass through the same float32 packet the wire carries.
    """

    def __init__(self, clock=time.monotonic):
        self._clock = clock
        self._to_dyno = None
        self._to_sim = None

    def _put(self, which, value, stamp):
        setattr(self, which, (value, stamp))

    # simulator end -------------------------------------------------------
    def send_reference(self, speed, steer=0.0):
        self._to_dyno = (unpack(pack(speed, steer)), self._clock())

    def recv_measurement(self):
        return self._to_sim[0] if self._to_sim else None

    def measurement_age(self):
        return None if self._to_sim is None else self._clock() - self._to_sim[1]

    # dyno end ------------------------------------------------------------
    def send_measurement(self, speed, steer=0.0):
        self._to_sim = (unpack(pack(speed, steer)), self._clock())

    def recv_reference(self):
        return self._to_dyno[0] if self._to_dyno else None

    def reference_age(self):
        return None if self._to_dyno is None else self._clock() - self._to_dyno[1]

    def close(self):
        pass


class UdpLink(object):
    """One end of the wire. ``end`` is 'simulator' or 'dyno'.

    The simulator end sends references and receives measurements; the dyno end
    does the reverse.
    """

    def __init__(self, end, peer_ip='127.0.0.1', reference_port=REFERENCE_PORT,
                 measurement_port=MEASUREMENT_PORT, clock=time.monotonic):
        if end not in ('simulator', 'dyno'):
            raise ValueError("end must be 'simulator' or 'dyno', got %r" % (end,))
        self.end = end
        self._clock = clock
        tx, rx = ((reference_port, measurement_port) if end == 'simulator'
                  else (measurement_port, reference_port))
        self._peer = (peer_ip, tx)
        self._tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._rx.bind(('0.0.0.0', rx))
        self._rx.setblocking(False)
        self._last = None
        self._stamp = None

    def send(self, speed, steer=0.0):
        try:
            self._tx.sendto(pack(speed, steer), self._peer)
        except OSError:
            pass                                # free-running: a lost send is lost

    def recv(self):
        """Drain the socket, keep the newest. Never blocks."""
        while True:
            try:
                data, _ = self._rx.recvfrom(64)
            except (BlockingIOError, OSError):
                break
            try:
                self._last = unpack(data)
                self._stamp = self._clock()
            except ValueError:
                continue                        # short packet, not ours
        return self._last

    def age(self):
        return None if self._stamp is None else self._clock() - self._stamp

    def close(self):
        self._tx.close()
        self._rx.close()

    # aliases, so a caller reads the same either way
    send_reference = send
    send_measurement = send
    recv_reference = recv
    recv_measurement = recv
    reference_age = age
    measurement_age = age


class TcpLink(object):
    """One end of the wire, over TCP. ``end`` is 'simulator' or 'dyno'.

    Same three calls as the others, and the same 8-byte packet -- but TCP is a
    stream, so a read that returns 5 bytes has not lost 3, it has not been given
    them yet. Bytes are buffered and only whole packets are handed up.

    The dyno end listens and the simulator end connects, because the cell is the
    fixed address in a real installation. Neither blocks: before the peer is
    there, send does nothing and recv answers None, which is the same thing a
    UDP run does while the other end is still starting.
    """

    def __init__(self, end, peer_ip='127.0.0.1', port=REFERENCE_PORT,
                 clock=time.monotonic):
        if end not in ('simulator', 'dyno'):
            raise ValueError("end must be 'simulator' or 'dyno', got %r" % (end,))
        self.end = end
        self._clock = clock
        self._peer = (peer_ip, port)
        self._sock = None
        self._listener = None
        self._buf = b''
        self._last = None
        self._stamp = None
        if end == 'dyno':
            self._listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self._listener.bind(('0.0.0.0', port))
            self._listener.listen(1)
            self._listener.setblocking(False)

    def _connected(self):
        """The socket if the peer is there, else None. Never blocks."""
        if self._sock is not None:
            return self._sock
        if self._listener is not None:
            try:
                self._sock, _ = self._listener.accept()
            except (BlockingIOError, OSError):
                return None
        else:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(0.05)
            try:
                sock.connect(self._peer)
            except OSError:
                sock.close()
                return None
            self._sock = sock
        self._sock.setblocking(False)
        # Speed matters more than packing here: a 0.05 s command held back by
        # Nagle's algorithm waiting for a second packet is a 40 ms lie about
        # what the bench was asked for.
        self._sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        return self._sock

    def _drop(self):
        if self._sock is not None:
            self._sock.close()
            self._sock = None
        self._buf = b''

    def send(self, speed, steer=0.0):
        sock = self._connected()
        if sock is None:
            return
        try:
            sock.sendall(pack(speed, steer))
        except OSError:
            self._drop()                        # peer went away; reconnect later

    def recv(self):
        """Drain what has arrived, keep the newest whole packet. Never blocks."""
        sock = self._connected()
        if sock is None:
            return self._last
        while True:
            try:
                chunk = sock.recv(4096)
            except (BlockingIOError, OSError):
                break
            if not chunk:
                self._drop()                    # orderly close
                break
            self._buf += chunk
        while len(self._buf) >= PACKET_SIZE:
            self._last = unpack(self._buf[:PACKET_SIZE])
            self._stamp = self._clock()
            self._buf = self._buf[PACKET_SIZE:]
        return self._last

    def age(self):
        return None if self._stamp is None else self._clock() - self._stamp

    def close(self):
        self._drop()
        if self._listener is not None:
            self._listener.close()
            self._listener = None

    # aliases, so a caller reads the same either way
    send_reference = send
    send_measurement = send
    recv_reference = recv
    recv_measurement = recv
    reference_age = age
    measurement_age = age


# --------------------------------------------------------------------------- #
# run start and stop, for a rig answering in lockstep (#473)
# --------------------------------------------------------------------------- #
#: Control codes, carried in the packet's speed slot: a commanded speed is never negative.
#: Host -> rig: (START, v0) ramp to v0 and start logging; (STOP, 0) the car goes back to the rig.
#: Rig -> host: (HELLO, 0) on start-up; (START, v) settled at v; (STOP, v) released at v (0 once at rest);
#: (FAULT, v) fault or e-stop.
HELLO, START, STOP, FAULT = -1, -2, -3, -4
_NAMES = {HELLO: 'HELLO', START: 'START', STOP: 'STOP', FAULT: 'FAULT'}


def code(speed):
    """The control code in a packet's speed slot, or None if it is a speed.

    Codes are exact negative whole numbers. Anything else is a speed, so a rig
    may report -0.01 m/s of sensor noise without it reading as a code.
    """
    speed = float(speed)
    if not (speed < 0 and math.isfinite(speed) and speed.is_integer()):
        return None
    return int(speed)


def codeName(c):
    return _NAMES.get(c, 'code %s' % c)


class RigError(RuntimeError):
    """The rig reported a fault, restarted mid-run, or stopped answering."""


def _say(msg):
    print(msg, flush=True)


#: While waiting on START or STOP, say so this often [s].
REMIND_S = 30.0
#: A STOP answered below this speed [m/s] is "at rest"; above it, the rig has taken the car moving.
AT_REST_MPS = 0.05


def _minutes(s):
    return '%d s' % s if s < 120 else '%.0f min' % (s / 60.0)


class UdpRig(object):
    """The simulator end of a rig that answers each command once (lockstep).

        rig = UdpRig(ip, 5010, 5011)
        rig.start(v0)                    # until the rig says it has settled at v0
        speed = rig.exchange(vref, aref, dt)
        rig.stop()                       # until the rig says it is at rest

    START and STOP are resent every ``resend_s`` until answered; the rig answers a
    repeat by acknowledging again. A rig must also ramp down on its own when no
    packet arrives for a few seconds, since a killed process never sends STOP.

    The rig answers STOP once FIXS no longer controls the car: at rest after its own
    ramp-down, or at once when a driver takes over. Its program may exit after that
    and start again for the next run; START waits for it, and its hello cuts the wait.

    A limit of 0 is no limit: START waits for the rig as long as it takes, which is
    the default because the run is paused meanwhile.
    """

    def __init__(self, ip, port=REFERENCE_PORT, reply_port=MEASUREMENT_PORT,
                 step_timeout_s=1.0, max_misses=5, start_timeout_s=0.0,
                 stop_timeout_s=60.0, resend_s=1.0, log=_say, clock=time.monotonic):
        self.peer = (ip, int(port))
        self.reply_port = int(reply_port)
        self.step_timeout_s = float(step_timeout_s)
        self.max_misses = int(max_misses)
        self.start_timeout_s = float(start_timeout_s)
        self.stop_timeout_s = float(stop_timeout_s)
        self.resend_s = float(resend_s)
        self._log = log
        self._clock = clock
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind(('', self.reply_port))
        self.started = False
        #: The speed the rig settled at on START, and steps it did not answer.
        self.startSpeed = None
        self.misses = 0
        self._missRun = 0

    @classmethod
    def fromConfig(cls, xil, reply_port=MEASUREMENT_PORT, **options):
        """A rig at XilSetup's ip and port, with XilSetup.Rig's limits: ``fixs.config.get('xil')``."""
        limits = dict(xil.get('rig') or {})
        for key, name in (('StartTimeout', 'start_timeout_s'), ('StopTimeout', 'stop_timeout_s'),
                          ('StepTimeout', 'step_timeout_s'), ('MaxMisses', 'max_misses')):
            if key in limits:
                options.setdefault(name, limits[key])
        return cls(xil['ip'], xil['port'], reply_port, **options)

    def _send(self, speed, second):
        try:
            self._sock.sendto(pack(speed, second), self.peer)
        except OSError:
            pass

    def _drain(self):
        """Drop whatever is queued: a late answer belongs to an earlier command."""
        self._sock.setblocking(False)
        try:
            while True:
                self._sock.recvfrom(64)
        except OSError:
            pass
        finally:
            self._sock.setblocking(True)

    def _recv(self, timeout):
        """The next whole packet within `timeout` s as (speed, second), or None."""
        deadline = self._clock() + timeout
        while True:
            left = deadline - self._clock()
            if left <= 0:
                return None
            self._sock.settimeout(None if math.isinf(left) else left)
            try:
                data, _ = self._sock.recvfrom(64)
            except socket.timeout:
                return None
            except ConnectionResetError:    # Windows: an earlier send found no rig listening
                continue
            if len(data) >= PACKET_SIZE:
                return unpack(data)

    def _request(self, which, value, timeout):
        """Send (which, value) until the rig acknowledges it; the ack's value, or None on timeout (0: none)."""
        begun = self._clock()
        deadline = begun + timeout if timeout > 0 else float('inf')
        remind = begun + REMIND_S
        self._drain()
        while self._clock() < deadline:
            if self._clock() >= remind:
                self._log('[rig] still waiting for the rig to answer %s (%s)'
                          % (codeName(which), _minutes(self._clock() - begun)))
                remind += REMIND_S
            self._send(which, value)
            resend_at = min(deadline, self._clock() + self.resend_s)
            while True:
                got = self._recv(resend_at - self._clock())
                if got is None:
                    break
                c = code(got[0])
                if c == which:
                    return got[1]
                if c == FAULT:
                    raise RigError('rig %s:%d reported a fault (%g) during %s'
                                   % (self.peer + (got[1], codeName(which))))
                if c == HELLO:
                    break                   # it has just come up: ask again now
        return None

    def start(self, v0=0.0):
        """Ask the rig to ramp to `v0` m/s; return the speed it settled at."""
        v0 = max(0.0, float(v0))
        self._log('[rig] START: ramp to %.2f m/s (%s:%d)' % ((v0,) + self.peer))
        v = self._request(START, v0, self.start_timeout_s)
        if v is None:                       # only with a StartTimeout
            raise RigError('rig %s:%d did not answer START in %.0f s'
                           % (self.peer + (self.start_timeout_s,)))
        self.started, self.startSpeed = True, v
        self._log('[rig] ready at %.2f m/s' % v)
        return v

    def exchange(self, vref, aref, dt):
        """Send one command and return the speed the rig reached, or None if it did not answer."""
        if not self.started:
            self.start(0.0)
        self._drain()
        self._send(max(0.0, float(vref)), float(aref))
        deadline = self._clock() + self.step_timeout_s if self.step_timeout_s > 0 else float('inf')
        while True:
            got = self._recv(deadline - self._clock())
            if got is None:
                break
            c = code(got[0])
            if c is None:                   # a speed, slightly negative ones included
                self._missRun = 0
                return got[0]
            if c == FAULT:
                raise RigError('rig %s:%d reported a fault (%g)' % (self.peer + (got[1],)))
            if c == HELLO:
                raise RigError('rig %s:%d restarted mid-run' % self.peer)
            # a late START/STOP acknowledgement: keep waiting for this step's answer
        self.misses += 1
        self._missRun += 1
        if self.max_misses and self._missRun >= self.max_misses:
            raise RigError('rig %s:%d did not answer %d steps in a row'
                           % (self.peer + (self._missRun,)))
        return None

    def stop(self):
        """Hand the car back to the rig; the speed it answered at, or None. Never raises."""
        if not self.started:
            return None
        self.started = False
        self._log('[rig] STOP: the car goes back to the rig')
        try:
            v = self._request(STOP, 0.0, self.stop_timeout_s)
        except RigError as exc:
            self._log('[rig] %s' % exc)
            return None
        if v is None:
            self._log('[rig] no STOP answer in %.0f s; the rig must ramp down on its own timeout'
                      % self.stop_timeout_s)
        elif abs(v) < AT_REST_MPS:
            self._log('[rig] at rest (%.2f m/s)' % v)
        else:
            self._log('[rig] released at %.2f m/s: the rig has the car' % v)
        return v

    def close(self):
        self._sock.close()
