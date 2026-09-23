"""EgoDriver -- backend-agnostic geometric ego driver (pure pursuit + speed hold).

Python peer of ``CommonLib/EgoDriver.{h,cpp}``. Deliberately NOT fused with any
simulator SDK: the caller supplies the ego pose and the route in ONE consistent
planar frame (x, y, heading in radians) and applies the returned
:class:`DriveCommand` through whatever actuation its backend owns.

This is the map-agnostic L0 fallback, used when the simulator's native driver is
unavailable or undesirable -- CARLA's Traffic Manager on a map whose OpenDRIVE
topology it cannot route, for instance. When the native driver works, prefer it
(``CarlaBackend.enableEgoTM``); this module stays selectable through
``CarlaSetup.EgoL0Driver``.

Defaults reproduce the C++ driver exactly, so the two produce the same command
for the same pose and route.
"""

import math

__all__ = ['DriveCommand', 'Params', 'EgoDriver']


class DriveCommand:
    """Neutral actuation command.

    :param throttle: [0, 1].
    :param brake: [0, 1].
    :param steer: [-1, 1]; -1 full left, +1 full right. The backend maps it to the
        simulator's own convention.
    """

    __slots__ = ('throttle', 'brake', 'steer')

    def __init__(self, throttle=0.0, brake=0.0, steer=0.0):
        self.throttle = throttle
        self.brake = brake
        self.steer = steer

    def __repr__(self):
        return ('DriveCommand(throttle=%.3f, brake=%.3f, steer=%.3f)'
                % (self.throttle, self.brake, self.steer))


class Params:
    """Tunables; the defaults reproduce the original bridge driver exactly."""

    def __init__(self):
        self.wheelbase = 2.9        # m (~tesla model3)
        self.maxSteerRad = 0.7      # wheel angle at |steer| = 1 (~40 deg)
        self.lookaheadMin = 6.0     # m
        self.lookaheadTime = 1.2    # s   (lookahead = max(min, time * speed))
        self.curveSlowGain = 1.2    # target-speed reduction vs |heading error|
        self.curveSlowMin = 0.4
        self.throttleGain = 0.25    # per (m/s) speed deficit
        self.throttleBias = 0.15
        self.throttleMax = 0.75
        self.brakeGain = 0.30       # per (m/s) speed excess
        self.searchWindow = 40      # nearest-point local window (points)


class EgoDriver:
    """Pure-pursuit steering with a proportional speed hold."""

    def __init__(self):
        self._route = []            # densified planar path
        self._closed = True
        self._cursor = 0            # current nearest index
        self._p = Params()

    def params(self):
        """() -> Params -- the live tunables; mutate in place to retune."""
        return self._p

    def routeSize(self):
        return len(self._route)

    def hasRoute(self):
        return len(self._route) >= 2

    def reset(self):
        self._cursor = 0

    def setRoute(self, pts, closed):
        """(list of (x, y), bool) -> None -- set the route in the caller's frame.

        Densified internally to ~3 m spacing. ``closed=True`` wraps the last ->
        first segment, so a loop scenario drives endlessly.
        """
        self._route = []
        self._closed = closed
        self._cursor = 0
        if len(pts) < 2:
            if len(pts) == 1:
                self._route.append(pts[0])
            return

        segs = len(pts) if closed else len(pts) - 1
        for i in range(segs):
            ax, ay = pts[i]
            bx, by = pts[(i + 1) % len(pts)]
            dx, dy = bx - ax, by - ay
            n = max(1, int(math.sqrt(dx * dx + dy * dy) / 3.0))
            for k in range(n):
                self._route.append((ax + dx * k / n, ay + dy * k / n))
        if not closed:
            self._route.append(pts[-1])

    def computeControl(self, x, y, headingRad, speed, targetSpeed):
        """(float, float, float, float, float) -> DriveCommand -- one control step.

        ``(x, y)`` and ``headingRad`` are the ego pose in the route frame; speeds
        in m/s. Advances the internal path cursor.
        """
        c = DriveCommand()
        n = len(self._route)
        if n < 2:
            return c
        p = self._p

        # Advance the cursor to the nearest point ahead within a local window --
        # local, not global, so the driver cannot jump to the far side of a loop
        # that passes close to itself. Closed route wraps; open stops at the end.
        best = 1e18
        bestIx = self._cursor
        for k in range(p.searchWindow):
            ix = self._cursor + k
            if ix >= n:
                if not self._closed:
                    break
                ix %= n
            dx = self._route[ix][0] - x
            dy = self._route[ix][1] - y
            d = dx * dx + dy * dy
            if d < best:
                best = d
                bestIx = ix
        self._cursor = bestIx

        # lookahead point: ~max(6 m, 1.2 s of travel)
        lookahead = max(p.lookaheadMin, p.lookaheadTime * speed)
        tgtIx = self._cursor
        acc = 0.0
        while acc < lookahead:
            nxt = tgtIx + 1
            if nxt >= n:
                if self._closed:
                    nxt = 0
                else:
                    tgtIx = n - 1
                    break
            dx = self._route[nxt][0] - self._route[tgtIx][0]
            dy = self._route[nxt][1] - self._route[tgtIx][1]
            acc += math.sqrt(dx * dx + dy * dy)
            tgtIx = nxt

        tx = self._route[tgtIx][0] - x
        ty = self._route[tgtIx][1] - y
        alpha = math.atan2(ty, tx) - headingRad          # heading error to lookahead
        while alpha > math.pi:
            alpha -= 2.0 * math.pi
        while alpha < -math.pi:
            alpha += 2.0 * math.pi
        delta = math.atan2(2.0 * p.wheelbase * math.sin(alpha), lookahead)

        c.steer = max(-1.0, min(1.0, delta / p.maxSteerRad))
        # Curvature-aware speed: slow toward corners so pure pursuit tracks the arc
        # rather than cutting it.
        curveSlow = max(p.curveSlowMin, 1.0 - p.curveSlowGain * abs(alpha))
        dv = targetSpeed * curveSlow - speed
        if dv >= 0:
            c.throttle = min(p.throttleMax, p.throttleGain * dv + p.throttleBias)
            c.brake = 0.0
        else:
            c.throttle = 0.0
            c.brake = min(1.0, -p.brakeGain * dv)
        return c
