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

UDP is free-running with no handshake. The newest datagram wins and nothing
blocks. TCP costs a connection and gives back delivery and ordering; it never
blocks either, so a run starts before the far end is listening. A caller that
has gone stale decides for itself what to do about it -- "keep the last value"
and "fall back to your own reference" are both right somewhere, so the link does
not choose.
"""

import socket
import struct
import time

_PACKET = struct.Struct('<2f')
PACKET_SIZE = _PACKET.size                  # 8
REFERENCE_PORT = 5010                       # simulator -> dyno
MEASUREMENT_PORT = 5011                     # dyno -> simulator


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
    """

    def __init__(self, clock=time.monotonic):
        self._clock = clock
        self._to_dyno = None
        self._to_sim = None

    def _put(self, which, value, stamp):
        setattr(self, which, (value, stamp))

    # simulator end -------------------------------------------------------
    def send_reference(self, speed, steer=0.0):
        self._to_dyno = ((float(speed), float(steer)), self._clock())

    def recv_measurement(self):
        return self._to_sim[0] if self._to_sim else None

    def measurement_age(self):
        return None if self._to_sim is None else self._clock() - self._to_sim[1]

    # dyno end ------------------------------------------------------------
    def send_measurement(self, speed, steer=0.0):
        self._to_sim = ((float(speed), float(steer)), self._clock())

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
