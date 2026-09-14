"""A dynamometer bench, simulated (#323).

A stand-in for the XIL hardware, so a coupling can be built and argued about
before the bench exists. It simulates the bench as a whole -- the dynamometer AND
the vehicle bolted to it -- because that pair is what the rest of the system
talks to. Nothing here imports CARLA, FIXS or numpy; it is standard library, so
it runs anywhere and can be unit-tested without a simulator.

THE PEDAL IS ALWAYS AN INPUT
-----------------------------
The bench never decides throttle. A real bench has a real vehicle on it whose
pedal is worked by somebody -- a driver robot following a trace, or in our case
the same command the CARLA ego receives. ``step()`` therefore always takes
``throttle`` and ``brake``, in every mode. What changes between modes is what the
DYNO does, not who drives the car.

TWO INDEPENDENT CHOICES
-----------------------
``mode`` -- where the dyno couples to the vehicle::

    'chassis'   the vehicle sits on rollers; the dyno acts at the roller surface
    'axle'      the wheels come off and hub units bolt to the hubs; the vehicle
                never moves, so there is no body here at all

``control`` -- what the dyno's servo is doing::

    'road_load'  the dyno pretends to be the road: it absorbs A + B*v + C*v^2 and
                 lets the vehicle accelerate against simulated inertia. Speed is
                 an OUTPUT. This is a coastdown/emissions-cycle bench.
    'speed'      the dyno FORCES the commanded speed and measures the effort
                 needed to hold it. Speed is an INPUT, torque is the measurement.
                 Road load is NOT applied -- the servo has replaced it, and
                 whoever owns the vehicle dynamics owns the road load too.

All four combinations are real benches, and they sit in a loop differently::

    chassis + road_load   pedals ─────────────▶ speed, wheel omega
    chassis + speed       pedals, speed_cmd ──▶ measured tractive force
    axle    + road_load   pedals ─────────────▶ wheel omega  (inertia simulated)
    axle    + speed       pedals, omega_cmd ──▶ measured axle torque

For coupling to a simulator that already owns the vehicle dynamics -- CARLA, or
CarMaker over CM4SL -- ``control='speed'`` is the one you want: the simulator says
what the speed should be, the bench holds it, and the torque it took is the
measurement that goes back. ``'road_load'`` is for running the bench standalone.

WHAT IS MODELLED
----------------
* A powertrain envelope, pedal to wheel torque, per axle: constant torque up to
  base speed then constant power, the right shape for an EV. Replace it wholesale
  with ``powertrain=`` -- that is the hook for driving the bench with another
  simulator's torque map so the two agree by construction instead of by luck.
* A second-order lag on delivered torque. Torque is not instantaneous, and on a
  speed-matched coupling that lag is a first-class error source.
* Friction brake torque per wheel, summing with the powertrain the way PhysX does.
* Rotating inertia, kept separate throughout: wheel/driveline, roller, hub rotor.
  A chassis roller refers to the road through r^2 and adds to the vehicle mass; a
  hub rotor adds directly to its wheel.
* Servos with finite bandwidth and an effort ceiling, gains DERIVED from the
  bandwidth and the inertia they act on.

WHAT IS NOT
-----------
* Tyre slip. Wheel couples to roller (or rotor) rigidly, so ``omega = v/r``
  exactly in chassis mode. A longitudinal bench study does not need a tyre; a
  traction-limit study does, and this is the wrong tool for that.
* Lateral anything. Thermal derate, battery limits, state of charge.
* Transport: no sockets, no threads, no clock. One ``step(dt)``, you own the loop.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Callable, Optional, Sequence

G = 9.80665

#: Wheel order, matching CARLA's ``physics_control.wheels``.
FL, FR, RL, RR = 0, 1, 2, 3
NWHEEL = 4

MODES = ('chassis', 'axle')
CONTROLS = ('road_load', 'speed')


# --------------------------------------------------------------------- config

@dataclass
class PowertrainParams:
    """Pedal to wheel torque, per axle, as a torque/power envelope.

    Torque is AT THE WHEEL, not at the motor, because that is what a dyno
    measures and what a coupling exchanges. Gear ratio is already inside these
    numbers.

    The peaks are manufacturer-derived for a dual-motor EV and are NOT measured.
    Overwrite them, or replace the whole map with ``powertrain=``.
    """

    front_peak_torque_Nm: float = 2700.0
    front_peak_power_W: float = 74.0e3
    rear_peak_torque_Nm: float = 3700.0
    rear_peak_power_W: float = 165.0e3
    #: Fraction of the pedal reaching the front axle; the rest goes rear.
    front_share: float = 0.42
    #: Regen commanded through the brake pedal, appearing as negative wheel
    #: torque split like drive torque. 0 disables it.
    regen_fraction: float = 0.0


@dataclass
class DrivelineParams:
    wheel_radius_m: float = 0.36
    #: One wheel plus its share of the driveline, referred to the wheel.
    wheel_inertia_kgm2: float = 1.4
    #: Per-wheel friction brake capacity.
    max_brake_torque_Nm: float = 1500.0
    #: Second-order lag between commanded and delivered wheel torque.
    torque_bandwidth_Hz: float = 5.0
    torque_damping: float = 0.7


@dataclass
class RoadLoadParams:
    """EPA-style coastdown, already in SI. Used by ``control='road_load'``."""

    A_N: float = 111.0
    B_Npms: float = 0.99
    C_Npms2: float = 0.45


@dataclass
class ServoParams:
    """A dyno servo. Gains are derived, never quoted.

    For a plant ``I * dx/dt = u`` a PI crosses over at ``kp/I``, so a gain quoted
    without the inertia it acts on is meaningless -- and is the easy way to build
    an unstable bench by accident. Ask for a bandwidth instead and the gains
    follow from whatever inertia this mode presents.
    """

    #: Closed-loop bandwidth of the outer loop. This decides how faithfully the
    #: bench holds its command, and therefore how much of a measurement is real.
    loop_Hz: float = 20.0
    #: The drive's inner torque/current loop, as a second-order lag. Must sit
    #: well above loop_Hz or the outer loop has no phase margin.
    actuator_Hz: float = 200.0
    actuator_damping: float = 0.9
    #: PI zero placement as a fraction of crossover. 1/5 is the textbook choice
    #: and contributes about 79 degrees of phase.
    zero_ratio: float = 0.2

    def gains(self, inertia: float):
        wc = 2.0 * math.pi * self.loop_Hz
        kp = inertia * wc
        return kp, kp * (wc * self.zero_ratio)

    def phase_margin_deg(self) -> float:
        """Approximate, PI zero and actuator lag included. Inertia cancels."""
        wc = 2.0 * math.pi * self.loop_Hz
        wa = 2.0 * math.pi * self.actuator_Hz
        return (math.degrees(math.atan(1.0 / self.zero_ratio))
                - 2.0 * math.degrees(math.atan(wc / wa)))


@dataclass
class ChassisDynoParams:
    vehicle_mass_kg: float = 2100.0
    #: Roller rotating inertia. Referred to the road through r^2 when added to
    #: the vehicle mass; a real dyno's rollers are heavy and this is not small.
    roller_inertia_kgm2: float = 40.0
    grade_rad: float = 0.0
    #: Ceiling on the speed servo's force. control='speed' only.
    max_force_N: float = 30000.0
    servo: ServoParams = field(default_factory=ServoParams)


@dataclass
class AxleDynoParams:
    """One hub unit per wheel."""

    #: Rotor inertia of one hub unit, adding directly to that wheel's inertia.
    hub_inertia_kgm2: float = 0.9
    #: Absorber ceiling, both directions.
    max_torque_Nm: float = 4000.0
    #: control='road_load' only: the vehicle mass the hubs must emulate, since
    #: there is no real body. Split evenly and referred to each wheel through r^2.
    simulated_mass_kg: float = 2100.0
    servo: ServoParams = field(default_factory=ServoParams)


@dataclass
class DynoConfig:
    """``mode`` picks where the dyno couples, ``control`` picks what it does."""

    mode: str = 'chassis'
    control: str = 'road_load'
    powertrain: PowertrainParams = field(default_factory=PowertrainParams)
    driveline: DrivelineParams = field(default_factory=DrivelineParams)
    road_load: RoadLoadParams = field(default_factory=RoadLoadParams)
    chassis: ChassisDynoParams = field(default_factory=ChassisDynoParams)
    axle: AxleDynoParams = field(default_factory=AxleDynoParams)

    def validate(self) -> None:
        if self.mode not in MODES:
            raise ValueError('mode must be one of %s, got %r' % (MODES, self.mode))
        if self.control not in CONTROLS:
            raise ValueError('control must be one of %s, got %r'
                             % (CONTROLS, self.control))
        if self.driveline.wheel_radius_m <= 0.0:
            raise ValueError('wheel_radius_m must be positive')
        if not 0.0 <= self.powertrain.front_share <= 1.0:
            raise ValueError('front_share must be in [0, 1]')
        if self.control == 'speed':
            # A servo asked to cross over near its own actuator has no phase
            # margin, and the bench then oscillates instead of holding its
            # command -- which reads as a physics result and is not one.
            s = (self.chassis.servo if self.mode == 'chassis' else self.axle.servo)
            pm = s.phase_margin_deg()
            if pm < 30.0:
                raise ValueError(
                    'the %s speed servo has %.0f deg phase margin: loop_Hz=%g is '
                    'too close to actuator_Hz=%g. Lower the first or raise the '
                    'second.' % (self.mode, pm, s.loop_Hz, s.actuator_Hz))


# ---------------------------------------------------------------------- state

@dataclass
class DynoState:
    """One tick of bench output. Every mode fills what it can.

    Which field is the MEASUREMENT depends on ``control``. Under ``'road_load'``
    the bench is telling you what the vehicle did, so read ``speed_mps`` or
    ``wheel_omega_radps``. Under ``'speed'`` the bench was told what to do, so
    read ``axle_torque_Nm`` or ``tractive_force_N`` -- the effort it took.
    """

    t_s: float = 0.0
    #: Vehicle speed. Real in chassis mode; in axle mode there is no body, so it
    #: is the speed the wheels IMPLY, ``mean(omega) * r``, offered as a
    #: convenience and not as a measurement.
    speed_mps: Optional[float] = None
    wheel_omega_radps: Sequence[float] = (0.0,) * NWHEEL
    #: Torque at the hub coupling, per wheel. Positive drives the vehicle. This
    #: is the powertrain's net torque less what accelerated the vehicle-side
    #: inertia, which is what a load cell there actually reads.
    axle_torque_Nm: Sequence[float] = (0.0,) * NWHEEL
    drive_torque_Nm: Sequence[float] = (0.0,) * NWHEEL
    brake_torque_Nm: Sequence[float] = (0.0,) * NWHEEL
    #: What the dyno itself applied, and how well it is holding its command.
    dyno_torque_Nm: Sequence[float] = (0.0,) * NWHEEL
    dyno_force_N: float = 0.0
    omega_error_radps: Sequence[float] = (0.0,) * NWHEEL
    speed_error_mps: float = 0.0
    #: Road load actually acting. Zero under control='speed', which does not
    #: impose one.
    road_load_N: float = 0.0
    #: Net longitudinal force on the vehicle: what actually accelerated it.
    tractive_force_N: float = 0.0
    at_standstill: bool = False


# ----------------------------------------------------------------- components

class _SecondOrder:
    """xdd + 2*z*wn*xd + wn^2*x = wn^2*u, RK4."""

    def __init__(self, f_hz: float, zeta: float, x0: float = 0.0):
        self.wn = 2.0 * math.pi * f_hz
        self.zeta = zeta
        self.x = x0
        self.xd = 0.0

    def _d(self, x, xd, u):
        return xd, self.wn * self.wn * (u - x) - 2.0 * self.zeta * self.wn * xd

    def step(self, u: float, dt: float) -> float:
        x, xd = self.x, self.xd
        k1 = self._d(x, xd, u)
        k2 = self._d(x + 0.5 * dt * k1[0], xd + 0.5 * dt * k1[1], u)
        k3 = self._d(x + 0.5 * dt * k2[0], xd + 0.5 * dt * k2[1], u)
        k4 = self._d(x + dt * k3[0], xd + dt * k3[1], u)
        self.x += dt / 6.0 * (k1[0] + 2 * k2[0] + 2 * k3[0] + k4[0])
        self.xd += dt / 6.0 * (k1[1] + 2 * k2[1] + 2 * k3[1] + k4[1])
        return self.x


class _PI:
    """PI with the integral frozen while the output is saturated."""

    def __init__(self, kp: float, ki: float, limit: float):
        self.kp, self.ki, self.limit = kp, ki, limit
        self.i = 0.0

    def step(self, err: float, dt: float) -> float:
        raw = self.kp * err + self.ki * self.i
        if abs(raw) < self.limit:
            self.i += err * dt
        out = self.kp * err + self.ki * self.i
        return max(-self.limit, min(self.limit, out))


class _Servo:
    """A dyno axis: PI on the tracking error through a second-order drive."""

    def __init__(self, p: ServoParams, inertia: float, limit: float):
        kp, ki = p.gains(inertia)
        self.pi = _PI(kp, ki, limit)
        self.lag = _SecondOrder(p.actuator_Hz, p.actuator_damping)

    def step(self, err: float, dt: float) -> float:
        return self.lag.step(self.pi.step(err, dt), dt)

    def reset(self) -> None:
        self.pi.i = 0.0
        self.lag.x = self.lag.xd = 0.0


def envelope_powertrain(p: PowertrainParams):
    """Default pedal-to-axle-torque map: constant torque, then constant power.

    Returns ``f(throttle, brake, omega_front, omega_rear) -> (T_front, T_rear)``
    as axle torque demand in Nm, before the delivery lag.
    """

    def cap(peak_T: float, peak_P: float, omega: float) -> float:
        return peak_T if omega <= 1e-3 else min(peak_T, peak_P / omega)

    def f(throttle: float, brake: float, w_f: float, w_r: float):
        thr = max(0.0, min(1.0, throttle))
        brk = max(0.0, min(1.0, brake))
        cf = cap(p.front_peak_torque_Nm, p.front_peak_power_W, abs(w_f))
        cr = cap(p.rear_peak_torque_Nm, p.rear_peak_power_W, abs(w_r))
        T_f = thr * p.front_share * cf
        T_r = thr * (1.0 - p.front_share) * cr
        if p.regen_fraction > 0.0:
            T_f -= brk * p.regen_fraction * p.front_share * cf
            T_r -= brk * p.regen_fraction * (1.0 - p.front_share) * cr
        return T_f, T_r

    return f


# -------------------------------------------------------------------- the sim

class DynoSimulator:
    """The bench. One object, one ``step`` per tick, no threads and no clock."""

    def __init__(self, config: Optional[DynoConfig] = None,
                 powertrain: Optional[Callable] = None):
        self.cfg = config or DynoConfig()
        self.cfg.validate()
        self.powertrain = powertrain or envelope_powertrain(self.cfg.powertrain)

        d, c, a = self.cfg.driveline, self.cfg.chassis, self.cfg.axle
        r = d.wheel_radius_m

        #: Translational inertia the chassis roller presents, rotating parts
        #: referred to the road through r^2.
        self.chassis_effective_mass_kg = (
            c.vehicle_mass_kg + c.roller_inertia_kgm2 / (r * r)
            + NWHEEL * d.wheel_inertia_kgm2 / (r * r))
        #: Rotational inertia one hub axis presents. Under road_load the hubs
        #: must also emulate the body, which has no other representation there.
        self.axle_inertia_kgm2 = d.wheel_inertia_kgm2 + a.hub_inertia_kgm2
        if self.cfg.control == 'road_load':
            self.axle_inertia_kgm2 += a.simulated_mass_kg * r * r / NWHEEL

        self._lagF = _SecondOrder(d.torque_bandwidth_Hz, d.torque_damping)
        self._lagR = _SecondOrder(d.torque_bandwidth_Hz, d.torque_damping)
        self._chassis_servo = _Servo(c.servo, self.chassis_effective_mass_kg,
                                     c.max_force_N)
        self._axle_servo = [_Servo(a.servo, self.axle_inertia_kgm2, a.max_torque_Nm)
                            for _ in range(NWHEEL)]

        self.t = 0.0
        self.v = 0.0
        self.omega = [0.0] * NWHEEL

    # -- shared -----------------------------------------------------------

    def road_load_N(self, v: float) -> float:
        """Signed so it always opposes motion."""
        p = self.cfg.road_load
        s = 1.0 if v >= 0.0 else -1.0
        return s * (p.A_N + p.B_Npms * abs(v) + p.C_Npms2 * v * v)

    def _torques(self, throttle: float, brake: float, dt: float):
        """Delivered drive torque and friction brake capacity, per wheel."""
        d = self.cfg.driveline
        w_f = 0.5 * (self.omega[FL] + self.omega[FR])
        w_r = 0.5 * (self.omega[RL] + self.omega[RR])
        T_f_cmd, T_r_cmd = self.powertrain(throttle, brake, w_f, w_r)
        T_f = self._lagF.step(T_f_cmd, dt)
        T_r = self._lagR.step(T_r_cmd, dt)
        drive = [T_f * 0.5, T_f * 0.5, T_r * 0.5, T_r * 0.5]     # open diff
        brk = max(0.0, min(1.0, brake)) * d.max_brake_torque_Nm
        return drive, [brk] * NWHEEL

    @staticmethod
    def _oppose(effort_cap: float, motion: float, applied: float, inertia: float,
                dt: float):
        """Resistive effort that opposes motion and can at most arrest it.

        Applied forces can push either way; resistive ones only ever oppose the
        motion that exists. Summing both as signed terms is how a parked vehicle
        ends up rolling backwards under its own rolling resistance.

        Returns ``(effort, held)``.
        """
        if abs(motion) > 1e-12:
            arresting = abs(motion) * inertia / dt + abs(applied)
            return math.copysign(min(effort_cap, arresting), motion), False
        if abs(applied) <= effort_cap:
            return applied, True                    # static, exactly balances
        return math.copysign(effort_cap, applied), False

    # -- chassis ----------------------------------------------------------

    def _step_chassis(self, throttle: float, brake: float, dt: float,
                      speed_cmd: Optional[float]) -> DynoState:
        c, d = self.cfg.chassis, self.cfg.driveline
        r = d.wheel_radius_m
        m_eff = self.chassis_effective_mass_kg
        drive, brake_cap = self._torques(throttle, brake, dt)

        F_applied = sum(drive) / r - c.vehicle_mass_kg * G * math.sin(c.grade_rad)
        F_brake_cap = sum(brake_cap) / r

        if self.cfg.control == 'speed':
            # The servo replaces the road: it holds the commanded speed and the
            # measurement is the effort that took. No road load is imposed.
            err = speed_cmd - self.v
            F_servo = self._chassis_servo.step(err, dt)
            F_brake, held = self._oppose(F_brake_cap, self.v,
                                         F_applied + F_servo, m_eff, dt)
            F_road = 0.0
            F_net = F_applied + F_servo - F_brake
        else:
            F_road_cap = abs(self.road_load_N(self.v))
            F_resist, held = self._oppose(F_road_cap + F_brake_cap, self.v,
                                          F_applied, m_eff, dt)
            total = F_road_cap + F_brake_cap
            F_road = F_resist * (F_road_cap / total) if total > 0.0 else 0.0
            F_brake = F_resist - F_road
            F_servo = 0.0
            err = 0.0
            F_net = F_applied - F_resist

        v_prev = self.v
        self.v = 0.0 if held else self.v + F_net / m_eff * dt
        if not held and v_prev != 0.0 and self.v * v_prev < 0.0:
            self.v = 0.0                            # decelerated onto rest
            held = True

        omega_prev = list(self.omega)
        self.omega = [self.v / r] * NWHEEL
        alpha = [(self.omega[j] - omega_prev[j]) / dt for j in range(NWHEEL)]
        axle = [drive[j] - F_brake * r / NWHEEL - d.wheel_inertia_kgm2 * alpha[j]
                for j in range(NWHEEL)]

        return DynoState(
            t_s=self.t, speed_mps=self.v,
            wheel_omega_radps=tuple(self.omega),
            axle_torque_Nm=tuple(axle),
            drive_torque_Nm=tuple(drive),
            brake_torque_Nm=tuple(F_brake * r / NWHEEL for _ in range(NWHEEL)),
            dyno_force_N=F_servo,
            speed_error_mps=err,
            road_load_N=F_road,
            tractive_force_N=F_net,
            at_standstill=held)

    # -- axle -------------------------------------------------------------

    def _step_axle(self, throttle: float, brake: float, dt: float,
                   omega_cmd: Optional[Sequence[float]]) -> DynoState:
        d = self.cfg.driveline
        r = d.wheel_radius_m
        J = self.axle_inertia_kgm2
        drive, brake_cap = self._torques(throttle, brake, dt)

        v_implied = sum(self.omega) / NWHEEL * r
        road_share = (abs(self.road_load_N(v_implied)) * r / NWHEEL
                      if self.cfg.control == 'road_load' else 0.0)

        dyno_T, err, axle, brk_out = [], [], [], []
        held = True
        for j in range(NWHEEL):
            if self.cfg.control == 'speed':
                e = omega_cmd[j] - self.omega[j]
                T_servo = self._axle_servo[j].step(e, dt)
                resist_cap = brake_cap[j]
            else:
                e = 0.0
                T_servo = 0.0
                resist_cap = brake_cap[j] + road_share

            applied = drive[j] + T_servo
            resist, wheel_held = self._oppose(resist_cap, self.omega[j], applied,
                                              J, dt)
            w_prev = self.omega[j]
            w_new = 0.0 if wheel_held else w_prev + (applied - resist) / J * dt
            if not wheel_held and w_prev != 0.0 and w_new * w_prev < 0.0:
                w_new = 0.0
                wheel_held = True
            alpha = (w_new - w_prev) / dt
            self.omega[j] = w_new
            held = held and wheel_held

            # A load cell between vehicle and rotor reads the vehicle's net
            # torque less what accelerated the vehicle-side inertia.
            brake_part = resist if self.cfg.control == 'speed' else \
                resist * (brake_cap[j] / resist_cap if resist_cap > 0.0 else 0.0)
            axle.append(drive[j] - brake_part - d.wheel_inertia_kgm2 * alpha)
            dyno_T.append(T_servo)
            err.append(e)
            brk_out.append(brake_part)

        return DynoState(
            t_s=self.t,
            speed_mps=sum(self.omega) / NWHEEL * r,
            wheel_omega_radps=tuple(self.omega),
            axle_torque_Nm=tuple(axle),
            drive_torque_Nm=tuple(drive),
            brake_torque_Nm=tuple(brk_out),
            dyno_torque_Nm=tuple(dyno_T),
            omega_error_radps=tuple(err),
            road_load_N=(self.road_load_N(v_implied)
                         if self.cfg.control == 'road_load' else 0.0),
            at_standstill=held)

    # -- entry point -------------------------------------------------------

    def step(self, throttle: float, brake: float, dt: float,
             speed_cmd: Optional[float] = None,
             omega_cmd: Optional[Sequence[float]] = None) -> DynoState:
        """Advance one tick. Throttle and brake are ALWAYS inputs.

        =====================  ==========================  ===================
        mode / control         extra argument              read back
        =====================  ==========================  ===================
        chassis / road_load    --                          speed_mps
        chassis / speed        speed_cmd (m/s)             tractive_force_N
        axle    / road_load    --                          wheel_omega_radps
        axle    / speed        omega_cmd (4x rad/s)        axle_torque_Nm
        =====================  ==========================  ===================
        """
        if dt <= 0.0:
            raise ValueError('dt must be positive')
        want_speed = self.cfg.control == 'speed'
        if self.cfg.mode == 'chassis':
            if omega_cmd is not None:
                raise ValueError('chassis mode takes speed_cmd, not omega_cmd')
            if want_speed and speed_cmd is None:
                raise ValueError("control='speed' needs speed_cmd")
            if not want_speed and speed_cmd is not None:
                raise ValueError("control='road_load' produces the speed; "
                                 'speed_cmd is not accepted')
        else:
            if speed_cmd is not None:
                raise ValueError('axle mode takes omega_cmd, not speed_cmd')
            if want_speed and (omega_cmd is None or len(omega_cmd) != NWHEEL):
                raise ValueError("control='speed' needs omega_cmd with %d entries"
                                 % NWHEEL)
            if not want_speed and omega_cmd is not None:
                raise ValueError("control='road_load' produces the wheel speeds; "
                                 'omega_cmd is not accepted')

        self.t += dt
        if self.cfg.mode == 'chassis':
            return self._step_chassis(throttle, brake, dt, speed_cmd)
        return self._step_axle(throttle, brake, dt, omega_cmd)

    def reset(self, speed_mps: float = 0.0) -> None:
        self.t = 0.0
        self.v = speed_mps
        self.omega = [speed_mps / self.cfg.driveline.wheel_radius_m] * NWHEEL
        self._chassis_servo.reset()
        for s in self._axle_servo:
            s.reset()
        d = self.cfg.driveline
        self._lagF = _SecondOrder(d.torque_bandwidth_Hz, d.torque_damping)
        self._lagR = _SecondOrder(d.torque_bandwidth_Hz, d.torque_damping)


# ------------------------------------------------------------------ self-test

def _demo() -> int:
    print('1. chassis / road_load -- pedals in, speed out')
    d = DynoSimulator(DynoConfig(mode='chassis', control='road_load'))
    for k in range(12000):
        s = d.step(1.0 if k < 8000 else 0.0, 0.0 if k < 8000 else 1.0, 0.001)
        if k % 3000 == 0 or k == 11999:
            print('   t=%5.2f  v=%6.2f  road=%7.1f N  Fx=%9.1f N' %
                  (s.t_s, s.speed_mps, s.road_load_N, s.tractive_force_N))

    print('\n2. chassis / speed -- speed in, force out')
    d = DynoSimulator(DynoConfig(mode='chassis', control='speed'))
    for k in range(6000):
        s = d.step(0.3, 0.0, 0.001, speed_cmd=15.0)
        if k % 1500 == 0 or k == 5999:
            print('   t=%5.2f  v=%7.3f  err=%9.2e  F_dyno=%9.1f N  Fx=%8.1f N' %
                  (s.t_s, s.speed_mps, s.speed_error_mps, s.dyno_force_N,
                   s.tractive_force_N))

    print('\n3. axle / speed -- wheel speed in, axle torque out')
    d = DynoSimulator(DynoConfig(mode='axle', control='speed'))
    for k in range(4000):
        s = d.step(0.3, 0.0, 0.001, omega_cmd=[20.0] * NWHEEL)
        if k % 1000 == 0 or k == 3999:
            print('   t=%5.2f  omega=%7.3f  err=%9.2e  T_axle=%8.1f  T_dyno=%8.1f' %
                  (s.t_s, s.wheel_omega_radps[RL], s.omega_error_radps[RL],
                   s.axle_torque_Nm[RL], s.dyno_torque_Nm[RL]))

    print('\n4. axle / road_load -- pedals in, wheel speed out (inertia simulated)')
    d = DynoSimulator(DynoConfig(mode='axle', control='road_load'))
    for k in range(8000):
        s = d.step(0.5, 0.0, 0.001)
        if k % 2000 == 0 or k == 7999:
            print('   t=%5.2f  omega=%7.3f  v_implied=%6.2f  road=%7.1f N' %
                  (s.t_s, s.wheel_omega_radps[RL], s.speed_mps, s.road_load_N))
    return 0


if __name__ == '__main__':
    raise SystemExit(_demo())
