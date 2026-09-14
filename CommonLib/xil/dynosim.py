"""The simulation: a vehicle on a dyno, with somebody working the pedal.

Not a part of the setup -- the other four modules are that. This one owns the
equations of motion, because the acceleration of a car on a dyno belongs to
neither the car nor the dyno alone. It emerges once they are coupled, and the
effective mass it turns on is the car plus the roller plus four wheels referred
to the road through the tyre radius.

    sim.step(v_ref, dt)                   the driver chases a speed reference
    sim.step_pedals(throttle, brake, dt)  you work the pedal yourself

Both return a State. The first is what the wire carries -- a speed reference in,
the speed achieved out -- and the gap between those two is the point: a
pass-through always delivers what was asked, a dyno does not.
"""
import math
from collections import namedtuple

from .driver import RobotDriver
from .dyno import Dyno
from .vehicle import Vehicle

__all__ = ['DynoSim', 'State', 'FL', 'FR', 'RL', 'RR', 'NWHEEL']

FL, FR, RL, RR = 0, 1, 2, 3
NWHEEL = 4
G = 9.80665

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
