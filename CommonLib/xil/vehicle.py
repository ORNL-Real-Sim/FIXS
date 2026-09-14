"""The vehicle under test.

    Powertrain   pedal -> axle torque
    Driveline    wheels, brakes, and how fast torque actually arrives
    Vehicle      a mass with one of each

Nothing here knows about a dyno. A vehicle is the same vehicle on the road, on
rollers, or on hub units.
"""

import math

__all__ = ['Powertrain', 'Driveline', 'Vehicle']


class Powertrain(object):
    """Pedal to axle torque: constant torque to base speed, then constant power.

    Torque is AT THE WHEEL, so the gear ratio is already inside these numbers.
    The defaults are manufacturer-derived for a dual-motor EV and are not
    measured -- the instrumented cycles we hold only reached 41 kW of a rated
    239, so that data bounds demand and cannot confirm capability.
    """

    def __init__(self, front_peak_torque_Nm=2700.0, front_peak_power_W=74.0e3,
                 rear_peak_torque_Nm=3700.0, rear_peak_power_W=165.0e3,
                 front_share=0.42, max_speed_mps=51.4, limiter_taper_mps=2.0):
        if not 0.0 <= front_share <= 1.0:
            raise ValueError('front_share must be in [0, 1]')
        self.front_peak_torque_Nm = front_peak_torque_Nm
        self.front_peak_power_W = front_peak_power_W
        self.rear_peak_torque_Nm = rear_peak_torque_Nm
        self.rear_peak_power_W = rear_peak_power_W
        self.front_share = front_share
        self.max_speed_mps = max_speed_mps
        self.limiter_taper_mps = limiter_taper_mps

    def torque(self, throttle, omega_front, omega_rear, wheel_radius_m):
        """(front, rear) axle torque demand, before the delivery lag.

        ``front_share`` splits the DEMAND. It is not a scale on each axle's
        capability -- doing that gave 51 % of the vehicle at full throttle, 3280
        of 6400 Nm. So full throttle delivers the whole car whatever the split,
        and ``front_share=0`` really does mean rear drive only. Where the share
        and the capability ratio disagree the total comes up short, and that
        shortfall is the vehicle saying the split you asked for is not one it
        can deliver.

        Subclass and override this to drive a bench from another simulator's
        torque map instead of this envelope.
        """
        thr = max(0.0, min(1.0, throttle))
        front = self._cap(self.front_peak_torque_Nm, self.front_peak_power_W,
                          abs(omega_front))
        rear = self._cap(self.rear_peak_torque_Nm, self.rear_peak_power_W,
                         abs(omega_rear))
        mean_omega = 0.5 * (abs(omega_front) + abs(omega_rear))
        demand = thr * self._limiter(mean_omega, wheel_radius_m) * (front + rear)
        return (min(front, self.front_share * demand),
                min(rear, (1.0 - self.front_share) * demand))

    @staticmethod
    def _cap(peak_torque, peak_power, omega):
        return peak_torque if omega <= 1e-3 else min(peak_torque,
                                                     peak_power / omega)

    def _limiter(self, omega, wheel_radius_m):
        """1 below the limited speed, tapering to 0 at it.

        Real EVs are held well below what their power would reach; without this
        the bench settled at 229 km/h where an EV6 stops at 185.
        """
        if self.max_speed_mps <= 0.0:
            return 1.0
        speed = abs(omega) * wheel_radius_m
        taper = max(1e-6, self.limiter_taper_mps)
        return max(0.0, min(1.0, (self.max_speed_mps - speed) / taper))


class _Lag(object):
    """Second order: xdd + 2*z*wn*xd + wn^2*x = wn^2*u, RK4."""

    def __init__(self, hz, zeta):
        self.wn = 2.0 * math.pi * hz
        self.zeta = zeta
        self.x = 0.0
        self.xd = 0.0

    def _d(self, x, xd, u):
        return xd, self.wn * self.wn * (u - x) - 2.0 * self.zeta * self.wn * xd

    def step(self, u, dt):
        x, xd = self.x, self.xd
        k1 = self._d(x, xd, u)
        k2 = self._d(x + 0.5 * dt * k1[0], xd + 0.5 * dt * k1[1], u)
        k3 = self._d(x + 0.5 * dt * k2[0], xd + 0.5 * dt * k2[1], u)
        k4 = self._d(x + dt * k3[0], xd + dt * k3[1], u)
        self.x += dt / 6.0 * (k1[0] + 2 * k2[0] + 2 * k3[0] + k4[0])
        self.xd += dt / 6.0 * (k1[1] + 2 * k2[1] + 2 * k3[1] + k4[1])
        return self.x


class Driveline(object):
    """Wheels, brakes, and how fast commanded torque actually arrives.

    ``torque_bandwidth_Hz`` is a second-order lag between commanded and
    delivered wheel torque. Torque is not instantaneous, and on a speed-matched
    coupling that lag is a first-class error source rather than a detail.
    """

    def __init__(self, wheel_radius_m=0.36, wheel_inertia_kgm2=1.4,
                 max_brake_torque_Nm=1500.0, torque_bandwidth_Hz=5.0,
                 torque_damping=0.7):
        if wheel_radius_m <= 0.0:
            raise ValueError('wheel_radius_m must be positive')
        if torque_bandwidth_Hz <= 0.0:
            raise ValueError('torque_bandwidth_Hz must be positive')
        self.wheel_radius_m = wheel_radius_m
        self.wheel_inertia_kgm2 = wheel_inertia_kgm2
        self.max_brake_torque_Nm = max_brake_torque_Nm
        self.torque_bandwidth_Hz = torque_bandwidth_Hz
        self.torque_damping = torque_damping

    def lag(self):
        """A fresh torque-delivery lag for one axle."""
        return _Lag(self.torque_bandwidth_Hz, self.torque_damping)


class Vehicle(object):
    """A mass with a powertrain and a driveline.

    Keywords are forwarded to whichever of the two owns them, so a caller writes
    ``Vehicle(mass_kg=1800, front_share=0.5)`` without having to know which.
    """

    def __init__(self, mass_kg=2100.0, powertrain=None, driveline=None, **kw):
        self.mass_kg = mass_kg
        self.powertrain = powertrain or Powertrain(**_take(kw, Powertrain))
        self.driveline = driveline or Driveline(**_take(kw, Driveline))
        if kw:
            raise TypeError('unknown vehicle parameters: %s'
                            % ', '.join(sorted(kw)))

    # A few driveline facts read naturally as vehicle facts, and a caller that
    # wants the wheel radius should not have to know which half stores it.
    @property
    def wheel_radius_m(self):
        return self.driveline.wheel_radius_m

    @property
    def max_brake_torque_Nm(self):
        return self.driveline.max_brake_torque_Nm

    @property
    def torque_bandwidth_Hz(self):
        return self.driveline.torque_bandwidth_Hz

    def axle_torque(self, throttle, omega_front, omega_rear):
        return self.powertrain.torque(throttle, omega_front, omega_rear,
                                      self.driveline.wheel_radius_m)


def _take(kw, cls):
    """Pull the keywords belonging to cls out of kw, by its own field names."""
    fields = set(cls().__dict__)
    return dict((k, kw.pop(k)) for k in list(kw) if k in fields)
