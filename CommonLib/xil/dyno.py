"""A vehicle on a dynamometer, simulated (#323).

Two things to configure and one thing that runs them:

    Vehicle   the car on the bench: mass, wheels, powertrain, brakes
    Dyno      the bench: what it absorbs, what it weighs
    DynoSim   step(throttle, brake, dt) -> State

The dyno applies road resistance ``A + B*v + C*v^2`` and nothing else, so speed
is an output and whoever works the pedal is the only authority over it. There is
no speed-controlled mode on purpose: a dyno servo holding a speed fights a driver
tracking the same speed, and the pedal then parks on whatever the driver's
integrator happened to hold rather than on what the physics require.

``mode='chassis'`` puts the vehicle on rollers, so the rotating parts refer to
the road through r^2. ``mode='axle'`` bolts hub units to the hubs; nothing moves,
so the body's inertia has to be added electrically.

Standard library only. No CARLA, no FIXS, no numpy.
"""

import math
from collections import namedtuple

FL, FR, RL, RR = 0, 1, 2, 3
NWHEEL = 4
G = 9.80665

#: One tick of output. ``axle_torque`` is the measured one -- torque on the wheel
#: axis, which is what the powertrain produced less what went into spinning the
#: wheel up rather than reaching the road. ``drive_torque`` and ``brake_torque``
#: are upstream of that.
State = namedtuple('State', 'time speed omega axle_torque drive_torque '
                            'brake_torque resistance force standstill substeps')


class Vehicle(object):
    """The car bolted to the bench.

    Torque figures are AT THE WHEEL, so the gear ratio is already inside them.
    The defaults are manufacturer-derived for a dual-motor EV and are not
    measured; the instrumented cycles we hold only ever reached 41 kW of a rated
    239, so that data bounds demand and cannot confirm capability.
    """

    def __init__(self, mass_kg=2100.0, wheel_radius_m=0.36,
                 wheel_inertia_kgm2=1.4, max_brake_torque_Nm=1500.0,
                 front_peak_torque_Nm=2700.0, front_peak_power_W=74.0e3,
                 rear_peak_torque_Nm=3700.0, rear_peak_power_W=165.0e3,
                 front_share=0.42, max_speed_mps=51.4, limiter_taper_mps=2.0,
                 torque_bandwidth_Hz=5.0, torque_damping=0.7):
        if wheel_radius_m <= 0.0:
            raise ValueError('wheel_radius_m must be positive')
        if not 0.0 <= front_share <= 1.0:
            raise ValueError('front_share must be in [0, 1]')
        if torque_bandwidth_Hz <= 0.0:
            raise ValueError('torque_bandwidth_Hz must be positive')
        self.mass_kg = mass_kg
        self.wheel_radius_m = wheel_radius_m
        self.wheel_inertia_kgm2 = wheel_inertia_kgm2
        self.max_brake_torque_Nm = max_brake_torque_Nm
        self.front_peak_torque_Nm = front_peak_torque_Nm
        self.front_peak_power_W = front_peak_power_W
        self.rear_peak_torque_Nm = rear_peak_torque_Nm
        self.rear_peak_power_W = rear_peak_power_W
        self.front_share = front_share
        self.max_speed_mps = max_speed_mps
        self.limiter_taper_mps = limiter_taper_mps
        self.torque_bandwidth_Hz = torque_bandwidth_Hz
        self.torque_damping = torque_damping

    def axle_torque(self, throttle, omega_front, omega_rear):
        """Pedal to (front, rear) axle torque demand, before the delivery lag.

        Constant torque up to base speed then constant power, which is the right
        shape for an EV. ``front_share`` splits the DEMAND; it is not a scale on
        each axle's capability, so full throttle delivers the whole vehicle
        whatever the split, and ``front_share=0`` really does mean rear drive
        only. Where the share and the capability ratio disagree the total comes
        up short, and that shortfall is the vehicle saying the split you asked
        for is not one it can deliver.

        Override this method to drive the bench from another simulator's torque
        map instead of this envelope.
        """
        thr = max(0.0, min(1.0, throttle))
        cf = self._cap(self.front_peak_torque_Nm, self.front_peak_power_W,
                       abs(omega_front))
        cr = self._cap(self.rear_peak_torque_Nm, self.rear_peak_power_W,
                       abs(omega_rear))
        demand = thr * self._limiter(0.5 * (abs(omega_front) + abs(omega_rear))) \
            * (cf + cr)
        return (min(cf, self.front_share * demand),
                min(cr, (1.0 - self.front_share) * demand))

    @staticmethod
    def _cap(peak_torque, peak_power, omega):
        return peak_torque if omega <= 1e-3 else min(peak_torque,
                                                     peak_power / omega)

    def _limiter(self, omega):
        """1 below the limited speed, tapering to 0 at it.

        Real EVs are held well below what their power would reach -- without
        this the bench settled at 229 km/h where an EV6 stops at 185.
        """
        if self.max_speed_mps <= 0.0:
            return 1.0
        v = abs(omega) * self.wheel_radius_m
        taper = max(1e-6, self.limiter_taper_mps)
        return max(0.0, min(1.0, (self.max_speed_mps - v) / taper))


class Dyno(object):
    """The bench: what it absorbs, and what it weighs."""

    def __init__(self, mode='chassis', road_A_N=111.0, road_B_Npms=0.99,
                 road_C_Npms2=0.45, roller_inertia_kgm2=40.0,
                 hub_inertia_kgm2=0.9, grade_rad=0.0):
        if mode not in ('chassis', 'axle'):
            raise ValueError("mode must be 'chassis' or 'axle', got %r" % (mode,))
        self.mode = mode
        self.road_A_N = road_A_N
        self.road_B_Npms = road_B_Npms
        self.road_C_Npms2 = road_C_Npms2
        self.roller_inertia_kgm2 = roller_inertia_kgm2
        self.hub_inertia_kgm2 = hub_inertia_kgm2
        self.grade_rad = grade_rad

    def resistance(self, speed):
        """Road resistance, signed so it always opposes motion."""
        s = 1.0 if speed >= 0.0 else -1.0
        return s * (self.road_A_N + self.road_B_Npms * abs(speed)
                    + self.road_C_Npms2 * speed * speed)


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


def _oppose(cap, motion, applied, inertia, dt):
    """Resistive effort: opposes the motion that exists, and can at most stop it.

    Applied efforts push either way; resistive ones do not. Summing both as
    signed terms is how a parked vehicle rolls backwards under its own rolling
    resistance. Returns (effort, held).
    """
    if abs(motion) > 1e-12:
        arresting = abs(motion) * inertia / dt + abs(applied)
        return math.copysign(min(cap, arresting), motion), False
    if abs(applied) <= cap:
        return applied, True                    # static, exactly balances
    return math.copysign(cap, applied), False


class DynoSim(object):
    """Run a Vehicle on a Dyno. One step per tick, no threads and no clock."""

    def __init__(self, vehicle=None, dyno=None):
        self.vehicle = vehicle or Vehicle()
        self.dyno = dyno or Dyno()
        v, d = self.vehicle, self.dyno
        r = v.wheel_radius_m

        # Rotating parts referred to the road through r^2, so they add to mass.
        self.effective_mass_kg = (v.mass_kg + d.roller_inertia_kgm2 / (r * r)
                                  + NWHEEL * v.wheel_inertia_kgm2 / (r * r))
        # An axle dyno has no body, so the hubs must emulate one.
        self.axle_inertia_kgm2 = (v.wheel_inertia_kgm2 + d.hub_inertia_kgm2
                                  + v.mass_kg * r * r / NWHEEL)
        # Inner step, from the fastest dynamic here. A 0.1 s step integrated
        # whole diverged to 1e81 and the clamps then read it back as a stopped
        # vehicle, so step() chops whatever it is handed down to this.
        self.inner_dt = 0.1 / v.torque_bandwidth_Hz

        self.time = 0.0
        self.speed = 0.0
        self.omega = [0.0] * NWHEEL
        self._lag_f = _Lag(v.torque_bandwidth_Hz, v.torque_damping)
        self._lag_r = _Lag(v.torque_bandwidth_Hz, v.torque_damping)

    def step(self, throttle, brake, dt):
        """Advance dt, chopped into steps the bench can integrate."""
        if dt <= 0.0:
            raise ValueError('dt must be positive')
        n = max(1, int(math.ceil(dt / self.inner_dt - 1e-12)))
        h = dt / n
        advance = self._chassis if self.dyno.mode == 'chassis' else self._axle
        for _ in range(n):
            self.time += h
            state = advance(throttle, brake, h)
        return state._replace(substeps=n)

    def reset(self, speed=0.0):
        v = self.vehicle
        self.time = 0.0
        self.speed = speed
        self.omega = [speed / v.wheel_radius_m] * NWHEEL
        self._lag_f = _Lag(v.torque_bandwidth_Hz, v.torque_damping)
        self._lag_r = _Lag(v.torque_bandwidth_Hz, v.torque_damping)

    # -- internals ---------------------------------------------------------

    def _torques(self, throttle, brake, h):
        """Delivered drive torque and brake capacity, per wheel."""
        v = self.vehicle
        t_f, t_r = v.axle_torque(throttle,
                                 0.5 * (self.omega[FL] + self.omega[FR]),
                                 0.5 * (self.omega[RL] + self.omega[RR]))
        t_f = self._lag_f.step(t_f, h)
        t_r = self._lag_r.step(t_r, h)
        drive = [t_f * 0.5, t_f * 0.5, t_r * 0.5, t_r * 0.5]    # open diff
        brk = max(0.0, min(1.0, brake)) * v.max_brake_torque_Nm
        return drive, [brk] * NWHEEL

    def _chassis(self, throttle, brake, h):
        v, d = self.vehicle, self.dyno
        r = v.wheel_radius_m
        m = self.effective_mass_kg
        drive, brake_cap = self._torques(throttle, brake, h)

        applied = sum(drive) / r - v.mass_kg * G * math.sin(d.grade_rad)
        cap_road = abs(d.resistance(self.speed))
        cap_brake = sum(brake_cap) / r
        resist, held = _oppose(cap_road + cap_brake, self.speed, applied, m, h)

        was = self.speed
        self.speed = 0.0 if held else self.speed + (applied - resist) / m * h
        if not held and was != 0.0 and self.speed * was < 0.0:
            self.speed, held = 0.0, True        # decelerated onto rest

        # Split the resistance that ACTED by capacity: rolling, that is the full
        # road load and the full brake; held at rest, only the share that
        # balanced the applied force.
        total = cap_road + cap_brake
        f_road = resist * (cap_road / total) if total > 0.0 else 0.0
        f_brake = resist - f_road

        prev = self.omega
        self.omega = [self.speed / r] * NWHEEL   # rigid on the roller
        axle = [drive[j] - f_brake * r / NWHEEL
                - v.wheel_inertia_kgm2 * (self.omega[j] - prev[j]) / h
                for j in range(NWHEEL)]
        return State(self.time, self.speed, tuple(self.omega), tuple(axle),
                     tuple(drive), tuple(f_brake * r / NWHEEL
                                         for _ in range(NWHEEL)),
                     f_road, applied - resist, held, 1)

    def _axle(self, throttle, brake, h):
        v, d = self.vehicle, self.dyno
        r = v.wheel_radius_m
        inertia = self.axle_inertia_kgm2
        drive, brake_cap = self._torques(throttle, brake, h)

        implied = sum(self.omega) / NWHEEL * r
        road = d.resistance(implied)
        share = abs(road) * r / NWHEEL

        axle, brakes, held = [], [], True
        for j in range(NWHEEL):
            cap = brake_cap[j] + share
            resist, stopped = _oppose(cap, self.omega[j], drive[j], inertia, h)
            was = self.omega[j]
            new = 0.0 if stopped else was + (drive[j] - resist) / inertia * h
            if not stopped and was != 0.0 and new * was < 0.0:
                new, stopped = 0.0, True
            self.omega[j] = new
            held = held and stopped
            brk = resist * (brake_cap[j] / cap) if cap > 0.0 else 0.0
            axle.append(drive[j] - brk - v.wheel_inertia_kgm2 * (new - was) / h)
            brakes.append(brk)

        self.speed = sum(self.omega) / NWHEEL * r
        return State(self.time, self.speed, tuple(self.omega), tuple(axle),
                     tuple(drive), tuple(brakes), road, 0.0, held, 1)
