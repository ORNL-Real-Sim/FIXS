"""The dynamometer, and the simulation of a vehicle on it.

The other three modules are the rest of the setup -- the car, whoever works the
pedal, the wire out to CARLA. This one is the dyno side: the machine itself, and
the equations of motion that only exist once a car is bolted to it. Those belong
to neither the car nor the dyno alone, and the effective mass they turn on is
the car plus the roller plus four wheels referred to the road through the tyre
radius.

    sim.step(v_ref, dt)                   the driver chases a speed reference
    sim.step_pedals(throttle, brake, dt)  you work the pedal yourself

Both return a State. The first is what the wire carries -- a speed reference in,
the speed achieved out -- and the gap between those two is the point: a
pass-through always delivers what was asked, a dyno does not.
"""

import math
from collections import namedtuple

from .driver import RobotDriver
from .vehicle import Vehicle

__all__ = ['Dyno', 'DynoSim', 'State', 'FL', 'FR', 'RL', 'RR', 'NWHEEL']

FL, FR, RL, RR = 0, 1, 2, 3
NWHEEL = 4
G = 9.80665


class Dyno(object):
    """The machine: what it absorbs, what it weighs, where it couples.

    ``mode='chassis'`` puts the vehicle on rollers, so its rotating parts refer
    to the road through ``r^2``. ``mode='axle'`` bolts hub units to the hubs --
    nothing moves, so the body's inertia has to be added electrically.

    It applies road resistance ``A + B*v + C*v^2`` and nothing else, so speed is
    an output and whoever works the pedal is the only authority over it. There
    is deliberately no speed-controlled mode: a dyno servo holding a speed
    fights a driver tracking the same speed, and the pedal then parks on
    whatever the driver's integrator happened to hold rather than on what the
    physics require. Measured, reaching 15 m/s three different ways parked the
    throttle at 0.040, 0.206 and 0.696; under road resistance it is 0.0283 every
    time.
    """

    def __init__(self, mode='chassis', road_A_N=111.0, road_B_Npms=0.99,
                 road_C_Npms2=0.45, roller_inertia_kgm2=40.0,
                 hub_inertia_kgm2=0.9, grade_rad=0.0):
        if mode not in ('chassis', 'axle'):
            raise ValueError("mode must be 'chassis' or 'axle', got %r" % (mode,))
        self.mode = mode
        self.road_A_N = road_A_N
        self.road_B_Npms = road_B_Npms
        self.road_C_Npms2 = road_C_Npms2
        #: A real dyno's rollers are heavy and this term is not small.
        self.roller_inertia_kgm2 = roller_inertia_kgm2
        self.hub_inertia_kgm2 = hub_inertia_kgm2
        self.grade_rad = grade_rad

    def resistance(self, speed):
        """Road resistance, signed so it always opposes motion."""
        sign = 1.0 if speed >= 0.0 else -1.0
        return sign * (self.road_A_N + self.road_B_Npms * abs(speed)
                       + self.road_C_Npms2 * speed * speed)


#: One tick of output. ``axle_torque`` is the MEASURED one -- torque on the wheel
#: axis, which is what the powertrain produced less what went into spinning the
#: wheel up rather than reaching the road. ``drive_torque`` and ``brake_torque``
#: are upstream of that and are not what a transducer reads.
State = namedtuple('State', 'time speed omega axle_torque drive_torque '
                            'brake_torque resistance force standstill substeps')


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
    """A Vehicle on a Dyno, driven by a RobotDriver. No threads, no clock."""

    def __init__(self, vehicle=None, dyno=None, driver=None):
        self.vehicle = vehicle or Vehicle()
        self.dyno = dyno or Dyno()
        self.driver = driver or RobotDriver()

        v, d = self.vehicle, self.dyno
        r = v.driveline.wheel_radius_m
        #: Rotating parts referred to the road through r^2, so they add to mass.
        self.effective_mass_kg = (
            v.mass_kg + d.roller_inertia_kgm2 / (r * r)
            + NWHEEL * v.driveline.wheel_inertia_kgm2 / (r * r))
        #: An axle dyno has no body, so the hubs must emulate one.
        self.axle_inertia_kgm2 = (v.driveline.wheel_inertia_kgm2
                                  + d.hub_inertia_kgm2
                                  + v.mass_kg * r * r / NWHEEL)
        #: Inner step, from the fastest dynamic here. A 0.1 s step integrated
        #: whole diverged to 1e81 and the clamps then read it back as a stopped
        #: vehicle, so step chops whatever it is handed down to this.
        self.inner_dt = 0.1 / v.driveline.torque_bandwidth_Hz

        self.time = 0.0
        self.speed = 0.0
        self.omega = [0.0] * NWHEEL
        self.throttle = 0.0
        self.brake = 0.0
        self._lag_f = v.driveline.lag()
        self._lag_r = v.driveline.lag()

    # -- running -----------------------------------------------------------

    def step(self, v_ref, dt):
        """The driver chases v_ref. Read ``throttle``/``brake`` for what it did."""
        throttle, brake = self.driver.step(v_ref, self.speed, dt)
        return self.step_pedals(throttle, brake, dt)

    def step_pedals(self, throttle, brake, dt):
        """Work the pedal directly, no driver. Chopped into steps it can
        integrate, so a caller sets the outer rate without knowing what is
        inside."""
        if dt <= 0.0:
            raise ValueError('dt must be positive')
        self.throttle, self.brake = throttle, brake
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
        self.omega = [speed / v.driveline.wheel_radius_m] * NWHEEL
        self.throttle = self.brake = 0.0
        self.driver.reset()
        self._lag_f = v.driveline.lag()
        self._lag_r = v.driveline.lag()

    # -- physics -----------------------------------------------------------

    def _torques(self, throttle, brake, h):
        """Delivered drive torque and brake capacity, per wheel."""
        v = self.vehicle
        t_f, t_r = v.axle_torque(throttle,
                                 0.5 * (self.omega[FL] + self.omega[FR]),
                                 0.5 * (self.omega[RL] + self.omega[RR]))
        t_f = self._lag_f.step(t_f, h)
        t_r = self._lag_r.step(t_r, h)
        drive = [t_f * 0.5, t_f * 0.5, t_r * 0.5, t_r * 0.5]    # open diff
        brk = max(0.0, min(1.0, brake)) * v.driveline.max_brake_torque_Nm
        return drive, [brk] * NWHEEL

    def _chassis(self, throttle, brake, h):
        v, d = self.vehicle, self.dyno
        r = v.driveline.wheel_radius_m
        mass = self.effective_mass_kg
        drive, brake_cap = self._torques(throttle, brake, h)

        applied = sum(drive) / r - v.mass_kg * G * math.sin(d.grade_rad)
        cap_road = abs(d.resistance(self.speed))
        cap_brake = sum(brake_cap) / r
        resist, held = _oppose(cap_road + cap_brake, self.speed, applied, mass, h)

        was = self.speed
        self.speed = 0.0 if held else self.speed + (applied - resist) / mass * h
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
                - v.driveline.wheel_inertia_kgm2 * (self.omega[j] - prev[j]) / h
                for j in range(NWHEEL)]
        return State(self.time, self.speed, tuple(self.omega), tuple(axle),
                     tuple(drive),
                     tuple(f_brake * r / NWHEEL for _ in range(NWHEEL)),
                     f_road, applied - resist, held, 1)

    def _axle(self, throttle, brake, h):
        v, d = self.vehicle, self.dyno
        r = v.driveline.wheel_radius_m
        inertia = self.axle_inertia_kgm2
        drive, brake_cap = self._torques(throttle, brake, h)

        road = d.resistance(sum(self.omega) / NWHEEL * r)
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
            axle.append(drive[j] - brk
                        - v.driveline.wheel_inertia_kgm2 * (new - was) / h)
            brakes.append(brk)

        self.speed = sum(self.omega) / NWHEEL * r
        return State(self.time, self.speed, tuple(self.omega), tuple(axle),
                     tuple(drive), tuple(brakes), road, 0.0, held, 1)
