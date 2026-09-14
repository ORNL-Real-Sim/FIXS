"""The wire between the simulator and the dyno cell (#323).

Two endpoints and two transports. The simulator side sends a speed reference and
reads back what the vehicle achieved; the dyno side does the reverse. Which
transport is underneath is the only thing that changes between running the whole
thing in one process on a laptop and running it against hardware across a
network -- which is the point, because then a difference in results is the
transport and not the model.

    simulator side                              dyno side
    --------------                              ---------
    send_reference(v_ref, steer)  ───────────▶  latest_reference()
    latest_measurement()          ◀───────────  send_measurement(v, steer)

WIRE FORMAT
-----------
Two little-endian float32, 8 bytes, matching the ORNL dyno cell so our software
can talk to it without a translation layer::

    :5010   [v_ref_mps,  steer_deg]     simulator -> dyno
    :5011   [v_meas_mps, steer_deg]     dyno -> simulator

Steer is carried because the packet carries it. Nothing here acts on it: the
bench in this package is longitudinal, and a lateral dyno is a different machine.
It is relayed so that the field exists when somebody wants it.

FRESHNESS, NOT DELIVERY
-----------------------
UDP has no handshake and this does not add one. Both sides are free-running, the
newest datagram wins, and a reader asks ``is_fresh()`` rather than blocking. A
caller that has gone stale has to decide what to do about it -- there is no
sensible default, because "keep using the last value" and "fall back to your own
reference" are both right in different places.
"""

from __future__ import annotations

import socket
import struct
import time
from typing import Optional, Tuple

#: Two little-endian float32. Matches the ORNL cell.
PACKET = struct.Struct('<2f')
PACKET_SIZE = PACKET.size                       # 8

DEFAULT_REFERENCE_PORT = 5010                   # simulator -> dyno
DEFAULT_MEASUREMENT_PORT = 5011                 # dyno -> simulator
DEFAULT_STALE_S = 0.15                          # ORNL's DYNO_RX_TIMEOUT_S

Sample = Tuple[float, float]                    # (speed_mps, steer_deg)


def pack(speed_mps: float, steer_deg: float = 0.0) -> bytes:
    return PACKET.pack(float(speed_mps), float(steer_deg))


def unpack(data: bytes) -> Sample:
    if len(data) < PACKET_SIZE:
        raise ValueError('packet is %d bytes, expected %d' % (len(data), PACKET_SIZE))
    v, s = PACKET.unpack(data[:PACKET_SIZE])
    return float(v), float(s)


class _Endpoint:
    """Shared freshness bookkeeping. Subclasses supply the transport."""

    def __init__(self, stale_after_s: float = DEFAULT_STALE_S,
                 clock=time.monotonic):
        self.stale_after_s = stale_after_s
        self._clock = clock
        self._last: Optional[Sample] = None
        self._last_t: Optional[float] = None

    def _accept(self, sample: Sample) -> None:
        self._last = sample
        self._last_t = self._clock()

    @property
    def latest(self) -> Optional[Sample]:
        """Newest sample received, or None if nothing has arrived yet."""
        return self._last

    def age_s(self) -> Optional[float]:
        if self._last_t is None:
            return None
        return self._clock() - self._last_t

    def is_fresh(self) -> bool:
        age = self.age_s()
        return age is not None and age <= self.stale_after_s

    def close(self) -> None:
        pass


# ------------------------------------------------------------- in process

class InProcessPair:
    """Both endpoints sharing memory. Zero latency, no loss, no jitter.

    The baseline: whatever a UDP run does differently from this is transport,
    not model.
    """

    def __init__(self, stale_after_s: float = DEFAULT_STALE_S,
                 clock=time.monotonic):
        self.simulator = InProcessSimulatorSide(self, stale_after_s, clock)
        self.dyno = InProcessDynoSide(self, stale_after_s, clock)


class InProcessSimulatorSide(_Endpoint):
    def __init__(self, pair: InProcessPair, stale_after_s, clock):
        super().__init__(stale_after_s, clock)
        self._pair = pair

    def send_reference(self, v_ref_mps: float, steer_deg: float = 0.0) -> None:
        self._pair.dyno._accept((float(v_ref_mps), float(steer_deg)))

    def latest_measurement(self) -> Optional[Sample]:
        return self.latest


class InProcessDynoSide(_Endpoint):
    def __init__(self, pair: InProcessPair, stale_after_s, clock):
        super().__init__(stale_after_s, clock)
        self._pair = pair

    def send_measurement(self, v_mps: float, steer_deg: float = 0.0) -> None:
        self._pair.simulator._accept((float(v_mps), float(steer_deg)))

    def latest_reference(self) -> Optional[Sample]:
        return self.latest


# -------------------------------------------------------------------- udp

class _UdpEndpoint(_Endpoint):
    def __init__(self, peer_ip: str, tx_port: int, rx_port: int,
                 stale_after_s: float = DEFAULT_STALE_S, clock=time.monotonic):
        super().__init__(stale_after_s, clock)
        self._tx_addr = (peer_ip, tx_port)
        self._tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._rx.bind(('0.0.0.0', rx_port))
        self._rx.setblocking(False)

    def _send(self, v: float, steer: float) -> None:
        try:
            self._tx.sendto(pack(v, steer), self._tx_addr)
        except OSError:
            pass                                # free-running: a lost send is a lost send

    def poll(self) -> Optional[Sample]:
        """Drain the socket, keep the newest. Never blocks."""
        got = None
        while True:
            try:
                data, _ = self._rx.recvfrom(64)
            except (BlockingIOError, OSError):
                break
            try:
                got = unpack(data)
            except ValueError:
                continue                        # short packet, not ours
        if got is not None:
            self._accept(got)
        return got

    def close(self) -> None:
        self._tx.close()
        self._rx.close()


class UdpSimulatorSide(_UdpEndpoint):
    """Sends the reference, receives the measurement."""

    def __init__(self, dyno_ip: str,
                 tx_port: int = DEFAULT_REFERENCE_PORT,
                 rx_port: int = DEFAULT_MEASUREMENT_PORT, **kw):
        super().__init__(dyno_ip, tx_port, rx_port, **kw)

    def send_reference(self, v_ref_mps: float, steer_deg: float = 0.0) -> None:
        self._send(v_ref_mps, steer_deg)

    def latest_measurement(self) -> Optional[Sample]:
        self.poll()
        return self.latest


class UdpDynoSide(_UdpEndpoint):
    """Receives the reference, sends the measurement."""

    def __init__(self, simulator_ip: str,
                 tx_port: int = DEFAULT_MEASUREMENT_PORT,
                 rx_port: int = DEFAULT_REFERENCE_PORT, **kw):
        super().__init__(simulator_ip, tx_port, rx_port, **kw)

    def send_measurement(self, v_mps: float, steer_deg: float = 0.0) -> None:
        self._send(v_mps, steer_deg)

    def latest_reference(self) -> Optional[Sample]:
        self.poll()
        return self.latest
