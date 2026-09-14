"""A dynamometer bench, simulated (#323).

A stand-in for the XIL hardware, so a coupling can be built and argued about
before the bench exists. It simulates the bench as a whole -- the dynamometer AND
the vehicle bolted to it -- because that pair is what the rest of the system
talks to. Nothing here imports CARLA, FIXS or numpy; it is standard library, so
it runs anywhere and can be unit-tested without a simulator.

THE PEDAL IS THE INPUT
----------------------
The bench never decides throttle. A real bench has a real vehicle on it whose
pedal is worked by somebody -- a driver robot following a speed trace, which is
what ``RobotDriver`` in this package stands in for. ``step()`` takes ``throttle``
and ``brake``, and gives back what the vehicle did.

THE DYNO APPLIES ROAD RESISTANCE
--------------------------------
That is the only thing it does. It pretends to be the road: it absorbs
``A + B*v + C*v^2`` and lets the vehicle accelerate against simulated inertia.
Speed is therefore an OUTPUT, and the driver is the only authority over it.

An earlier revision also offered a speed-controlled mode, where the dyno servo
forced a commanded speed and the measurement was the force it took. That is a
real bench, but it does not compose with a speed-tracking driver: both would be
steering the same degree of freedom, the driver's error would sit at zero, and
its pedal would park on whatever its integrator happened to hold. Measured: the
same 15 m/s setpoint reached three ways parked the throttle at 0.040, 0.206 and
0.696. Under road resistance the pedal is pinned by physics instead -- 0.0283
every time, because that is the value whose tractive force balances the
resistance. Git has the speed mode if a torque-coupled bench ever needs it.

WHERE THE DYNO COUPLES
----------------------
``mode='chassis'``
    The vehicle sits on rollers. The dyno acts at the roller surface and the
    rotating parts refer to the road through ``r^2``.
``mode='axle'``
    The wheels come off and hub units bolt to the hubs. The vehicle never moves,
    so the body has no physical representation and its translational inertia is
    added electrically -- ``simulated_mass_kg``, split across the hubs.

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

WHAT IS NOT
-----------
* Tyre slip. Wheel couples to roller (or rotor) rigidly, so ``omega = v/r``
  exactly in chassis mode. A longitudinal bench study does not need a tyre; a
  traction-limit study does, and this is the wrong tool for that.
* Lateral anything. Thermal derate, battery limits, state of charge.
* Transport: no sockets, no threads, no clock. See ``link.py`` for the wire.
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
    #: How the demand is SPLIT between the axles -- not a scale on each axle's
    #: capability. An earlier revision multiplied each axle's cap by its share,
    #: which meant full throttle delivered 51 % of the vehicle (3280 of 6400 Nm,
    #: 127 of 239 kW) and terminal speed was wrong for a reason that had nothing
    #: to do with the road. A split decides who gets what; it must not throw half
    #: the car away. Demand above an axle's cap spills to the other one.
    front_share: float = 0.42
    #: Electronically limited top speed. Real EVs have one and it is usually well
    #: below what the power would reach: an EV6 stops at ~185 km/h against a road
    #: load it could push through to nearly 290. Drive torque tapers to zero over
    #: the last ``limiter_taper_mps`` rather than cutting, which would chatter.
    max_speed_mps: float = 51.4                 # 185 km/h
    limiter_taper_mps: float = 2.0
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
class RoadResistanceParams:
    """EPA-style coastdown, already in SI: ``F = A + B*v + C*v^2``."""

    A_N: float = 111.0
    B_Npms: float = 0.99
    C_Npms2: float = 0.45


@dataclass
class ChassisDynoParams:
    vehicle_mass_kg: float = 2100.0
    #: Roller rotating inertia. Referred to the road through r^2 when added to
    #: the vehicle mass; a real dyno's rollers are heavy and this is not small.
    roller_inertia_kgm2: float = 40.0
    grade_rad: float = 0.0


@dataclass
class AxleDynoParams:
    """One hub unit per wheel, absorbing the road-resistance share."""

    #: Rotor inertia of one hub unit, adding directly to that wheel's inertia.
    hub_inertia_kgm2: float = 0.9
    #: The vehicle mass the hubs must emulate, since there is no real body.
    #: Split evenly and referred to each wheel through r^2.
    simulated_mass_kg: float = 2100.0


@dataclass
class DynoConfig:
    mode: str = 'chassis'
    powertrain: PowertrainParams = field(default_factory=PowertrainParams)
    driveline: DrivelineParams = field(default_factory=DrivelineParams)
    road_resistance: RoadResistanceParams = field(
        default_factory=RoadResistanceParams)
    chassis: ChassisDynoParams = field(default_factory=ChassisDynoParams)
    axle: AxleDynoParams = field(default_factory=AxleDynoParams)
    #: Largest step the bench will integrate internally. Whatever ``dt`` a caller
    #: hands ``step()`` is chopped into pieces no larger than this, so the caller
    #: never has to know the bench's time constants. 0 derives it from the
    #: torque bandwidth, which is the fastest thing in here.
    max_internal_dt_s: float = 0.0

    def validate(self) -> None:
        if self.mode not in MODES:
            raise ValueError('mode must be one of %s, got %r' % (MODES, self.mode))
        if self.driveline.wheel_radius_m <= 0.0:
            raise ValueError('wheel_radius_m must be positive')
        if not 0.0 <= self.powertrain.front_share <= 1.0:
            raise ValueError('front_share must be in [0, 1]')
        if self.driveline.torque_bandwidth_Hz <= 0.0:
            raise ValueError('torque_bandwidth_Hz must be positive')

    def internal_dt(self) -> float:
        """The inner step, derived unless it was set explicitly.

        The torque lag is the fastest dynamic here, and integrating it with a
        step comparable to its own period diverges -- at 0.1 s the speed reached
        1e81 before the clamps snapped it to zero, which reads as a stopped
        vehicle rather than a broken integration. A tenth of the period keeps
        that comfortably away.
        """
        if self.max_internal_dt_s > 0.0:
            return self.max_internal_dt_s
        return 0.1 / self.driveline.torque_bandwidth_Hz


# ---------------------------------------------------------------------- state

@dataclass
class DynoState:
    """One tick of bench output."""

    t_s: float = 0.0
    #: Vehicle speed. Real in chassis mode; in axle mode there is no body, so it
    #: is the speed the wheels IMPLY, ``mean(omega) * r``.
    speed_mps: float = 0.0
    wheel_omega_radps: Sequence[float] = (0.0,) * NWHEEL
    #: Torque on the WHEEL AXIS, per wheel, positive driving the vehicle:
    #: the powertrain's net torque less what accelerated the wheel rather than
    #: reaching the road, ``T_drive - T_brake - J_wheel * domega/dt``.
    #:
    #: This is the measured quantity in both modes. On a chassis dyno it is the
    #: contact force the roller senses times the radius; on an axle dyno it is
    #: the shaft torque the hub transducer reads. Use this rather than
    #: ``drive_torque_Nm - brake_torque_Nm``, which is one term upstream and is
    #: what the powertrain produced rather than what the bench measures.
    axle_torque_Nm: Sequence[float] = (0.0,) * NWHEEL
    drive_torque_Nm: Sequence[float] = (0.0,) * NWHEEL
    brake_torque_Nm: Sequence[float] = (0.0,) * NWHEEL
    #: Road resistance actually acting.
    road_resistance_N: float = 0.0
    #: Net longitudinal force on the vehicle: what actually accelerated it.
    tractive_force_N: float = 0.0
    at_standstill: bool = False
    #: How many inner steps this call took. 1 unless the caller's dt was large.
    substeps: int = 1


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


def envelope_powertrain(p: PowertrainParams, wheel_radius_m: float = 0.36):
    """Default pedal-to-axle-torque map: constant torque, then constant power.

    Returns ``f(throttle, brake, omega_front, omega_rear) -> (T_front, T_rear)``
    as axle torque demand in Nm, before the delivery lag.

    The pedal asks for a fraction of what the vehicle can do; ``front_share``
    then decides how that demand is divided, and anything an axle cannot take
    spills to the other. So full throttle delivers the whole vehicle whatever the
    split, which is the property an earlier revision got wrong.
    """

    def cap(peak_T: float, peak_P: float, omega: float) -> float:
        return peak_T if omega <= 1e-3 else min(peak_T, peak_P / omega)

    def limiter(omega: float) -> float:
        """1 below the limited speed, tapering to 0 at it."""
        if p.max_speed_mps <= 0.0:
            return 1.0
        v = abs(omega) * wheel_radius_m
        taper = max(1e-6, p.limiter_taper_mps)
        return max(0.0, min(1.0, (p.max_speed_mps - v) / taper))

    def split(demand: float, cf: float, cr: float):
        """Divide demand by share, each axle clipped at its own capability.

        Deliberately no spill-over. An earlier attempt let an axle's unmet share
        flow to the other one, which sounds generous but makes ``front_share=0``
        -- rear-drive only, a real vehicle -- quietly send torque to the front.
        Where the share and the capability ratio disagree the total comes up
        short, and that shortfall is information: it is the vehicle telling you
        the split you asked for is not one it can deliver.
        """
        return min(cf, p.front_share * demand), min(cr, (1.0 - p.front_share) * demand)

    def f(throttle: float, brake: float, w_f: float, w_r: float):
        thr = max(0.0, min(1.0, throttle))
        brk = max(0.0, min(1.0, brake))
        cf = cap(p.front_peak_torque_Nm, p.front_peak_power_W, abs(w_f))
        cr = cap(p.rear_peak_torque_Nm, p.rear_peak_power_W, abs(w_r))

        drive = thr * limiter(0.5 * (abs(w_f) + abs(w_r))) * (cf + cr)
        T_f, T_r = split(drive, cf, cr)
        if p.regen_fraction > 0.0:
            regen = brk * p.regen_fraction * (cf + cr)
            R_f, R_r = split(regen, cf, cr)
            T_f -= R_f
            T_r -= R_r
        return T_f, T_r

    return f


# -------------------------------------------------------------------- the sim

class DynoSimulator:
    """The bench. One ``step`` per tick, no threads and no clock."""

    def __init__(self, config: Optional[DynoConfig] = None,
                 powertrain: Optional[Callable] = None):
        self.cfg = config or DynoConfig()
        self.cfg.validate()
        self.powertrain = powertrain or envelope_powertrain(self.cfg.powertrain,
                                                    self.cfg.driveline.wheel_radius_m)
        self.inner_dt = self.cfg.internal_dt()

        d, c, a = self.cfg.driveline, self.cfg.chassis, self.cfg.axle
        r = d.wheel_radius_m

        #: Translational inertia the chassis roller presents, rotating parts
        #: referred to the road through r^2.
        self.chassis_effective_mass_kg = (
            c.vehicle_mass_kg + c.roller_inertia_kgm2 / (r * r)
            + NWHEEL * d.wheel_inertia_kgm2 / (r * r))
        #: Rotational inertia one hub axis presents, body included since there
        #: is no other representation of it on an axle dyno.
        self.axle_inertia_kgm2 = (d.wheel_inertia_kgm2 + a.hub_inertia_kgm2
                                  + a.simulated_mass_kg * r * r / NWHEEL)

        self._lagF = _SecondOrder(d.torque_bandwidth_Hz, d.torque_damping)
        self._lagR = _SecondOrder(d.torque_bandwidth_Hz, d.torque_damping)

        self.t = 0.0
        self.v = 0.0
        self.omega = [0.0] * NWHEEL

    # -- shared -----------------------------------------------------------

    def road_resistance_N(self, v: float) -> float:
        """Signed so it always opposes motion."""
        p = self.cfg.road_resistance
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
    def _oppose(cap: float, motion: float, applied: float, inertia: float,
                dt: float):
        """Resistive effort that opposes motion and can at most arrest it.

        Applied efforts push either way; resistive ones only ever oppose the
        motion that exists. Summing both as signed terms is how a parked vehicle
        ends up rolling backwards under its own rolling resistance.

        Returns ``(effort, held)``.
        """
        if abs(motion) > 1e-12:
            arresting = abs(motion) * inertia / dt + abs(applied)
            return math.copysign(min(cap, arresting), motion), False
        if abs(applied) <= cap:
            return applied, True                    # static, exactly balances
        return math.copysign(cap, applied), False

    # -- chassis ----------------------------------------------------------

    def _advance_chassis(self, throttle: float, brake: float, h: float) -> DynoState:
        c, d = self.cfg.chassis, self.cfg.driveline
        r = d.wheel_radius_m
        m_eff = self.chassis_effective_mass_kg
        drive, brake_cap = self._torques(throttle, brake, h)

        F_applied = sum(drive) / r - c.vehicle_mass_kg * G * math.sin(c.grade_rad)
        cap_road = abs(self.road_resistance_N(self.v))
        cap_brake = sum(brake_cap) / r

        F_resist, held = self._oppose(cap_road + cap_brake, self.v, F_applied,
                                      m_eff, h)
        v_prev = self.v
        self.v = 0.0 if held else self.v + (F_applied - F_resist) / m_eff * h
        if not held and v_prev != 0.0 and self.v * v_prev < 0.0:
            self.v = 0.0                            # decelerated onto rest
            held = True

        # Attribute the resistance that acted, split by capacity: rolling, that
        # is the full road resistance and the full brake; held at standstill, it
        # is only the share that balanced the applied force.
        total = cap_road + cap_brake
        F_road = F_resist * (cap_road / total) if total > 0.0 else 0.0
        F_brake = F_resist - F_road

        omega_prev = list(self.omega)
        self.omega = [self.v / r] * NWHEEL
        alpha = [(self.omega[j] - omega_prev[j]) / h for j in range(NWHEEL)]
        axle = [drive[j] - F_brake * r / NWHEEL - d.wheel_inertia_kgm2 * alpha[j]
                for j in range(NWHEEL)]

        return DynoState(
            t_s=self.t, speed_mps=self.v,
            wheel_omega_radps=tuple(self.omega),
            axle_torque_Nm=tuple(axle),
            drive_torque_Nm=tuple(drive),
            brake_torque_Nm=tuple(F_brake * r / NWHEEL for _ in range(NWHEEL)),
            road_resistance_N=F_road,
            tractive_force_N=F_applied - F_resist,
            at_standstill=held)

    # -- axle -------------------------------------------------------------

    def _advance_axle(self, throttle: float, brake: float, h: float) -> DynoState:
        d = self.cfg.driveline
        r = d.wheel_radius_m
        J = self.axle_inertia_kgm2
        drive, brake_cap = self._torques(throttle, brake, h)

        v_implied = sum(self.omega) / NWHEEL * r
        road_total = self.road_resistance_N(v_implied)
        road_share = abs(road_total) * r / NWHEEL

        axle, brk_out = [], []
        held = True
        for j in range(NWHEEL):
            cap = brake_cap[j] + road_share
            resist, wheel_held = self._oppose(cap, self.omega[j], drive[j], J, h)
            w_prev = self.omega[j]
            w_new = 0.0 if wheel_held else w_prev + (drive[j] - resist) / J * h
            if not wheel_held and w_prev != 0.0 and w_new * w_prev < 0.0:
                w_new = 0.0
                wheel_held = True
            alpha = (w_new - w_prev) / h
            self.omega[j] = w_new
            held = held and wheel_held

            brake_part = resist * (brake_cap[j] / cap) if cap > 0.0 else 0.0
            axle.append(drive[j] - brake_part - d.wheel_inertia_kgm2 * alpha)
            brk_out.append(brake_part)

        return DynoState(
            t_s=self.t,
            speed_mps=sum(self.omega) / NWHEEL * r,
            wheel_omega_radps=tuple(self.omega),
            axle_torque_Nm=tuple(axle),
            drive_torque_Nm=tuple(drive),
            brake_torque_Nm=tuple(brk_out),
            road_resistance_N=road_total,
            at_standstill=held)

    # -- entry point -------------------------------------------------------

    def step(self, throttle: float, brake: float, dt: float) -> DynoState:
        """Advance ``dt``, chopped into inner steps the bench can integrate.

        The caller sets the outer rate -- CARLA's tick, the feed period, a
        bench loop -- and does not have to know what is inside here.
        """
        if dt <= 0.0:
            raise ValueError('dt must be positive')
        n = max(1, int(math.ceil(dt / self.inner_dt - 1e-12)))
        h = dt / n
        advance = (self._advance_chassis if self.cfg.mode == 'chassis'
                   else self._advance_axle)
        state = None
        for _ in range(n):
            self.t += h
            state = advance(throttle, brake, h)
        state.substeps = n
        return state

    def reset(self, speed_mps: float = 0.0) -> None:
        self.t = 0.0
        self.v = speed_mps
        self.omega = [speed_mps / self.cfg.driveline.wheel_radius_m] * NWHEEL
        d = self.cfg.driveline
        self._lagF = _SecondOrder(d.torque_bandwidth_Hz, d.torque_damping)
        self._lagR = _SecondOrder(d.torque_bandwidth_Hz, d.torque_damping)
