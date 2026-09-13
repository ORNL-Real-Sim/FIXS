"""A dynamometer bench, simulated (#323).

A stand-in for the XIL hardware, so a coupling can be built and argued about
before the bench exists. It simulates the bench as a whole -- the dynamometer
AND the vehicle bolted to it -- because that pair is what the rest of the system
talks to. Nothing here imports CARLA, FIXS or numpy; it is standard library, so
it runs anywhere and can be unit-tested without a simulator.

TWO BENCHES, AND WHY THEY ARE NOT THE SAME OBJECT
--------------------------------------------------
``mode='chassis'``
    The vehicle sits on rollers. The dyno emulates the road: it applies the road
    load ``A + B*v + C*v^2`` at the roller surface and lets the vehicle's own
    powertrain accelerate the combined inertia. **The bench owns the body.** You
    give it pedals; it hands you back a vehicle speed.

``mode='axle'``
    The wheels are removed and hub units bolt to the hubs. There is no road and
    no body motion -- the vehicle never goes anywhere. Each hub unit is a servo
    that HOLDS a commanded wheel speed and measures the torque needed to do it.
    **Something else owns the body**: a vehicle model computes what the wheel
    speed should be and sends it here, exactly as CarMaker does over CM4SL. You
    give it pedals AND a wheel-speed command; it hands you back measured axle
    torque.

So the two modes sit in the loop differently, and that is physical rather than a
software choice:

    chassis   pedals ─────────────────▶ [bench] ─────────▶ speed, wheel omega
    axle      pedals ──┐
              omega_cmd ┴────────────▶ [bench] ─────────▶ axle torque, omega

Read ``DynoState`` for what comes back; both modes fill every field they can.

WHAT IS MODELLED
----------------
* A powertrain envelope, pedal to wheel torque, per axle. Constant torque up to
  base speed then constant power, which is the right shape for an EV. Replace it
  wholesale by passing your own ``powertrain=`` callable -- that is the hook for
  driving the bench with another simulator's torque map so the two agree by
  construction.
* A second-order lag on delivered torque. Torque is not instantaneous, and on a
  speed-matched coupling the lag is a first-class error source, not a detail.
* Friction brake torque per wheel, summing with the powertrain the way PhysX
  does.
* Rotating inertia, kept separate everywhere: wheel/driveline, dyno roller or
  hub rotor. On a chassis dyno the roller is referred to the road through r^2
  and adds to the vehicle mass; on an axle dyno the hub rotor adds to the wheel.
* An axle-dyno speed servo with finite bandwidth and a torque ceiling.

WHAT IS NOT
-----------
* Tyre slip. Both modes couple wheel to road (or wheel to rotor) rigidly, so
  ``omega = v / r`` in chassis mode exactly. A longitudinal bench study does not
  need a tyre; a traction-limit study does, and this is the wrong tool for it.
* Lateral anything.
* Thermal derate, battery limits, state of charge.
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


# --------------------------------------------------------------------- config

@dataclass
class PowertrainParams:
    """Pedal to wheel torque, per axle, as a torque/power envelope.

    Torque is expressed AT THE WHEEL, not at the motor, because that is what a
    dyno measures and what the coupling exchanges. Gear ratio is therefore
    already inside these numbers.

    The peak figures are manufacturer-derived for a dual-motor EV6 and are NOT
    measured. The instrumented drive cycles we hold reached 41 kW at the wheels
    against a rated 239 kW, so that data bounds demand and cannot confirm
    capability. Treat them as placeholders and overwrite them for real work.
    """

    front_peak_torque_Nm: float = 2700.0
    front_peak_power_W: float = 74.0e3
    rear_peak_torque_Nm: float = 3700.0
    rear_peak_power_W: float = 165.0e3
    #: Fraction of the pedal that reaches the front axle. The rest goes rear.
    #: 0.42 mirrors the measured EV6 split at high demand; at low demand the real
    #: car runs rear-only, which a constant cannot express -- see the module note
    #: in the FIXS issue on per-axle injection.
    front_share: float = 0.42
    #: Regenerative braking is commanded through the brake pedal and appears as
    #: negative wheel torque, split like drive torque. 0 disables it.
    regen_fraction: float = 0.0


@dataclass
class DrivelineParams:
    wheel_radius_m: float = 0.3596          # measured: median(v / omega) on the EV6 logs
    #: Rotating inertia of one wheel plus its share of the driveline, at the wheel.
    wheel_inertia_kgm2: float = 1.4
    #: Per-wheel friction brake capacity.
    max_brake_torque_Nm: float = 1500.0
    #: Second-order lag between commanded and delivered wheel torque.
    torque_bandwidth_Hz: float = 5.0
    torque_damping: float = 0.7


@dataclass
class RoadLoadParams:
    """EPA-style coastdown, already in SI. Chassis mode only.

    On an axle dyno the vehicle does not move, so road load belongs to whatever
    owns the body and is not applied here.
    """

    A_N: float = 111.0
    B_Npms: float = 0.99
    C_Npms2: float = 0.45


@dataclass
class ChassisDynoParams:
    vehicle_mass_kg: float = 2100.0
    #: Roller rotating inertia, referred to the road through r^2 when it is added
    #: to the vehicle mass. A real chassis dyno's rollers are heavy and this term
    #: is not small.
    roller_inertia_kgm2: float = 40.0
    grade_rad: float = 0.0


@dataclass
class AxleDynoParams:
    """One speed-controlled hub unit per wheel.

    The two bandwidths are the knobs; the PI gains are DERIVED from them and the
    inertia, because gains quoted as raw numbers are meaningless without the
    inertia they act on and are the easy way to build an unstable bench by
    accident. For ``J*dw/dt = T`` with a PI, the crossover is ``kp/J``, so
    ``kp = J * 2*pi*speed_loop_Hz`` puts it exactly where asked.
    """

    #: Rotor inertia of one hub unit, adding directly to that wheel's inertia.
    hub_inertia_kgm2: float = 0.9
    #: Absorber ceiling, both directions.
    max_torque_Nm: float = 4000.0
    #: Closed-loop bandwidth of the speed servo. This is the number that decides
    #: how faithfully the bench holds a commanded wheel speed, and therefore how
    #: much of a measured torque is real.
    speed_loop_Hz: float = 20.0
    #: The drive's inner torque/current loop, modelled as a second-order lag.
    #: Must sit well above speed_loop_Hz or the speed loop has no phase margin.
    current_loop_Hz: float = 200.0
    current_loop_damping: float = 0.9
    #: PI zero placement, as a fraction of the crossover. 1/5 is the textbook
    #: choice and gives about 79 degrees of phase from the PI itself.
    zero_ratio: float = 0.2

    def gains(self, inertia_kgm2: float):
        """(kp, ki) for the requested bandwidth against this inertia."""
        wc = 2.0 * math.pi * self.speed_loop_Hz
        kp = inertia_kgm2 * wc
        return kp, kp * (wc * self.zero_ratio)

    def phase_margin_deg(self, inertia_kgm2: float) -> float:
        """Approximate phase margin, PI zero and current loop included."""
        wc = 2.0 * math.pi * self.speed_loop_Hz
        wz = wc * self.zero_ratio
        wa = 2.0 * math.pi * self.current_loop_Hz
        return (math.degrees(math.atan(wc / wz))
                - 2.0 * math.degrees(math.atan(wc / wa)))


@dataclass
class DynoConfig:
    """Everything the bench needs. ``mode`` picks which half is used."""

    mode: str = 'chassis'                   # 'chassis' | 'axle'
    powertrain: PowertrainParams = field(default_factory=PowertrainParams)
    driveline: DrivelineParams = field(default_factory=DrivelineParams)
    road_load: RoadLoadParams = field(default_factory=RoadLoadParams)
    chassis: ChassisDynoParams = field(default_factory=ChassisDynoParams)
    axle: AxleDynoParams = field(default_factory=AxleDynoParams)

    def validate(self) -> None:
        if self.mode not in ('chassis', 'axle'):
            raise ValueError("mode must be 'chassis' or 'axle', got %r" % (self.mode,))
        if self.driveline.wheel_radius_m <= 0.0:
            raise ValueError('wheel_radius_m must be positive')
        if not 0.0 <= self.powertrain.front_share <= 1.0:
            raise ValueError('front_share must be in [0, 1]')
        if self.mode == 'axle':
            # A speed loop asked to cross over near or above its own actuator has
            # no phase margin, and the bench then oscillates instead of holding
            # speed -- which looks like a physics result and is not one.
            J = self.driveline.wheel_inertia_kgm2 + self.axle.hub_inertia_kgm2
            pm = self.axle.phase_margin_deg(J)
            if pm < 30.0:
                raise ValueError(
                    'axle speed loop has %.0f deg phase margin: speed_loop_Hz=%g '
                    'is too close to current_loop_Hz=%g. Lower the first or raise '
                    'the second.' % (pm, self.axle.speed_loop_Hz,
                                     self.axle.current_loop_Hz))


# ---------------------------------------------------------------------- state

@dataclass
class DynoState:
    """One tick of bench output. Both modes fill everything they can.

    ``axle_torque_Nm`` is the headline in axle mode: it is what a load cell at
    the hub coupling reads, which is the powertrain torque minus what went into
    spinning the wheel up. That distinction is the whole reason wheel inertia is
    a parameter.
    """

    t_s: float = 0.0
    #: Vehicle speed. Chassis mode measures it; axle mode has no body, so this
    #: stays None and the caller's vehicle model owns it.
    speed_mps: Optional[float] = None
    wheel_omega_radps: Sequence[float] = (0.0,) * NWHEEL
    #: Torque at the hub coupling, per wheel. Positive drives the vehicle.
    axle_torque_Nm: Sequence[float] = (0.0,) * NWHEEL
    drive_torque_Nm: Sequence[float] = (0.0,) * NWHEEL
    brake_torque_Nm: Sequence[float] = (0.0,) * NWHEEL
    #: Axle-dyno only: what the absorber commanded, and how well it is holding.
    dyno_torque_Nm: Sequence[float] = (0.0,) * NWHEEL
    omega_error_radps: Sequence[float] = (0.0,) * NWHEEL
    #: Chassis-mode only.
    road_load_N: float = 0.0
    tractive_force_N: float = 0.0
    #: True while the bench is holding a stopped vehicle against the brake.
    at_standstill: bool = False


# ----------------------------------------------------------------- components

class _SecondOrder:
    """xdd + 2*z*wn*xd + wn^2*x = wn^2*u, RK4.

    Used for both the powertrain's torque response and the servo's current loop.
    It is the same equation; only the bandwidth differs.
    """

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


def envelope_powertrain(p: PowertrainParams):
    """Default pedal-to-axle-torque map: constant torque, then constant power.

    Returns ``f(throttle, brake, omega_front, omega_rear) -> (T_front, T_rear)``
    as AXLE torque demand in Nm, before the delivery lag.
    """

    def cap(peak_T: float, peak_P: float, omega: float) -> float:
        if omega <= 1e-3:
            return peak_T
        return min(peak_T, peak_P / omega)

    def f(throttle: float, brake: float, w_f: float, w_r: float):
        thr = max(0.0, min(1.0, throttle))
        brk = max(0.0, min(1.0, brake))
        demand_f = thr * p.front_share
        demand_r = thr * (1.0 - p.front_share)
        T_f = demand_f * cap(p.front_peak_torque_Nm, p.front_peak_power_W, abs(w_f))
        T_r = demand_r * cap(p.rear_peak_torque_Nm, p.rear_peak_power_W, abs(w_r))
        if p.regen_fraction > 0.0:
            regen = brk * p.regen_fraction
            T_f -= regen * p.front_share * cap(p.front_peak_torque_Nm,
                                               p.front_peak_power_W, abs(w_f))
            T_r -= regen * (1.0 - p.front_share) * cap(p.rear_peak_torque_Nm,
                                                       p.rear_peak_power_W, abs(w_r))
        return T_f, T_r

    return f


# -------------------------------------------------------------------- the sim

class DynoSimulator:
    """The bench. One object, one ``step`` per tick, no threads and no clock.

    ``powertrain`` overrides the default envelope with any callable of the same
    shape, which is how you make the bench's powertrain agree with another
    simulator's instead of guessing at it.
    """

    def __init__(self, config: Optional[DynoConfig] = None,
                 powertrain: Optional[Callable] = None):
        self.cfg = config or DynoConfig()
        self.cfg.validate()
        self.powertrain = powertrain or envelope_powertrain(self.cfg.powertrain)

        d = self.cfg.driveline
        self._lagF = _SecondOrder(d.torque_bandwidth_Hz, d.torque_damping)
        self._lagR = _SecondOrder(d.torque_bandwidth_Hz, d.torque_damping)

        self.t = 0.0
        self.v = 0.0                                   # chassis mode only
        self.omega = [0.0] * NWHEEL
        self._prev_omega = [0.0] * NWHEEL

        a = self.cfg.axle
        self._J_axle = d.wheel_inertia_kgm2 + a.hub_inertia_kgm2
        kp, ki = a.gains(self._J_axle)
        self._servo = [_PI(kp, ki, a.max_torque_Nm) for _ in range(NWHEEL)]
        self._servo_lag = [_SecondOrder(a.current_loop_Hz, a.current_loop_damping)
                           for _ in range(NWHEEL)]

    # -- shared -----------------------------------------------------------

    def road_load_N(self, v: float) -> float:
        """Signed so it always opposes motion."""
        r = self.cfg.road_load
        s = 1.0 if v >= 0.0 else -1.0
        return s * (r.A_N + r.B_Npms * abs(v) + r.C_Npms2 * v * v)

    def _torques(self, throttle: float, brake: float, dt: float):
        """Delivered drive torque and friction brake torque, per wheel."""
        d = self.cfg.driveline
        w_f = 0.5 * (self.omega[FL] + self.omega[FR])
        w_r = 0.5 * (self.omega[RL] + self.omega[RR])
        T_f_cmd, T_r_cmd = self.powertrain(throttle, brake, w_f, w_r)
        T_f = self._lagF.step(T_f_cmd, dt)
        T_r = self._lagR.step(T_r_cmd, dt)

        drive = [T_f * 0.5, T_f * 0.5, T_r * 0.5, T_r * 0.5]   # open diff, 50/50
        brk = max(0.0, min(1.0, brake)) * d.max_brake_torque_Nm
        return drive, [brk] * NWHEEL

    # -- chassis ----------------------------------------------------------

    def _step_chassis(self, throttle: float, brake: float, dt: float) -> DynoState:
        c, d = self.cfg.chassis, self.cfg.driveline
        r = d.wheel_radius_m

        drive, brake_cap = self._torques(throttle, brake, dt)

        # Rigid roller coupling, so every wheel turns at v/r and the rotating
        # inertias refer to the road through r^2 and simply add to the mass.
        m_eff = (c.vehicle_mass_kg
                 + c.roller_inertia_kgm2 / (r * r)
                 + NWHEEL * d.wheel_inertia_kgm2 / (r * r))

        # Applied forces can drive the vehicle in either direction; resistive
        # ones can only oppose whatever motion there is. Keeping them apart is
        # what stops road load from accelerating a parked car backwards, which a
        # single signed sum silently does.
        F_applied = sum(drive) / r - c.vehicle_mass_kg * G * math.sin(c.grade_rad)
        F_resist_cap = abs(self.road_load_N(self.v)) + sum(brake_cap) / r

        eps = 1e-9
        standstill = False
        if abs(self.v) < eps:
            if abs(F_applied) <= F_resist_cap:
                v_new = 0.0                        # static friction holds it
                F_resist = F_applied               # exactly balances, no more
                standstill = True
            else:
                F_resist = math.copysign(F_resist_cap, F_applied)
                v_new = (F_applied - F_resist) / m_eff * dt
        else:
            F_resist = math.copysign(F_resist_cap, self.v)
            v_new = self.v + (F_applied - F_resist) / m_eff * dt
            if v_new * self.v < 0.0 and abs(F_applied) <= F_resist_cap:
                v_new = 0.0                        # decelerated onto rest, stays
                F_resist = F_applied
                standstill = True
        self.v = v_new
        # Attribute the resistance that actually acted, split by capacity. While
        # rolling that returns the full road load and the full brake; while held
        # at standstill it returns only the share that balanced the applied
        # force, rather than the capacity neither of them used.
        cap_road = abs(self.road_load_N(self.v))
        cap_brake = sum(brake_cap) / r
        cap_total = cap_road + cap_brake
        if cap_total > 0.0:
            F_road = F_resist * cap_road / cap_total
            F_brake = F_resist * cap_brake / cap_total
        else:
            F_road = F_brake = 0.0

        self._prev_omega = list(self.omega)
        self.omega = [self.v / r] * NWHEEL
        alpha = [(self.omega[j] - self._prev_omega[j]) / dt for j in range(NWHEEL)]
        axle = [drive[j] - (F_brake * r / NWHEEL) - d.wheel_inertia_kgm2 * alpha[j]
                for j in range(NWHEEL)]

        return DynoState(
            t_s=self.t, speed_mps=self.v,
            wheel_omega_radps=tuple(self.omega),
            axle_torque_Nm=tuple(axle),
            drive_torque_Nm=tuple(drive),
            brake_torque_Nm=tuple(F_brake * r / NWHEEL for _ in range(NWHEEL)),
            road_load_N=F_road,
            #: Net longitudinal force on the vehicle: what actually accelerated it.
            tractive_force_N=F_applied - F_resist,
            at_standstill=standstill)

    # -- axle -------------------------------------------------------------

    def _step_axle(self, throttle: float, brake: float,
                   omega_cmd: Sequence[float], dt: float) -> DynoState:
        a, d = self.cfg.axle, self.cfg.driveline
        if omega_cmd is None or len(omega_cmd) != NWHEEL:
            raise ValueError('axle mode needs omega_cmd with %d entries' % NWHEEL)

        drive, brake_cap = self._torques(throttle, brake, dt)
        J = d.wheel_inertia_kgm2 + a.hub_inertia_kgm2

        self._prev_omega = list(self.omega)
        dyno_T, err, axle, brk_out = [], [], [], []
        for j in range(NWHEEL):
            e = omega_cmd[j] - self.omega[j]
            T_servo = self._servo_lag[j].step(self._servo[j].step(e, dt), dt)

            # The brake opposes rotation and can at most bring the wheel to rest
            # within this step; it must never drive it the other way.
            other = drive[j] + T_servo
            if abs(self.omega[j]) > 1e-9:
                stopping = abs(self.omega[j]) * J / dt + abs(other)
                b = math.copysign(min(brake_cap[j], stopping), self.omega[j])
            elif other != 0.0:
                b = math.copysign(min(brake_cap[j], abs(other)), other)
            else:
                b = 0.0

            net = other - b
            w_new = self.omega[j] + net / J * dt

            alpha = (w_new - self.omega[j]) / dt
            self.omega[j] = w_new

            # What the load cell between vehicle and rotor reads: the vehicle's
            # net torque less what accelerated the vehicle-side inertia.
            axle.append(drive[j] - b - d.wheel_inertia_kgm2 * alpha)
            dyno_T.append(T_servo)
            err.append(e)
            brk_out.append(b)

        return DynoState(
            t_s=self.t, speed_mps=None,
            wheel_omega_radps=tuple(self.omega),
            axle_torque_Nm=tuple(axle),
            drive_torque_Nm=tuple(drive),
            brake_torque_Nm=tuple(brk_out),
            dyno_torque_Nm=tuple(dyno_T),
            omega_error_radps=tuple(err),
            at_standstill=all(abs(w) < 1e-6 for w in self.omega))

    # -- entry point -------------------------------------------------------

    def step(self, throttle: float, brake: float, dt: float,
             omega_cmd: Optional[Sequence[float]] = None) -> DynoState:
        """Advance one tick.

        chassis: ``step(throttle, brake, dt)`` and read ``speed_mps``.
        axle:    ``step(throttle, brake, dt, omega_cmd=[FL, FR, RL, RR])`` and
                 read ``axle_torque_Nm``.
        """
        if dt <= 0.0:
            raise ValueError('dt must be positive')
        self.t += dt
        if self.cfg.mode == 'chassis':
            if omega_cmd is not None:
                raise ValueError('chassis mode owns the speed; omega_cmd is not accepted')
            return self._step_chassis(throttle, brake, dt)
        return self._step_axle(throttle, brake, omega_cmd, dt)

    def reset(self, speed_mps: float = 0.0) -> None:
        self.t = 0.0
        self.v = speed_mps
        w = speed_mps / self.cfg.driveline.wheel_radius_m
        self.omega = [w] * NWHEEL
        self._prev_omega = list(self.omega)
        for pi in self._servo:
            pi.i = 0.0
        d = self.cfg.driveline
        self._lagF = _SecondOrder(d.torque_bandwidth_Hz, d.torque_damping)
        self._lagR = _SecondOrder(d.torque_bandwidth_Hz, d.torque_damping)


# ------------------------------------------------------------------ self-test

def _demo() -> int:
    print('chassis: full throttle from rest, then full brake')
    dyno = DynoSimulator(DynoConfig(mode='chassis'))
    for k in range(12000):
        s = dyno.step(1.0 if k < 8000 else 0.0, 0.0 if k < 8000 else 1.0, 0.001)
        if k % 2000 == 0 or k == 11999:
            print('  t=%5.2f  v=%6.2f m/s  omega=%6.2f  Fx=%8.1f N  road=%7.1f N'
                  % (s.t_s, s.speed_mps, s.wheel_omega_radps[0],
                     s.tractive_force_N, s.road_load_N))

    print('\naxle: hubs told to hold 20 rad/s while the vehicle pushes')
    dyno = DynoSimulator(DynoConfig(mode='axle'))
    for k in range(4000):
        s = dyno.step(0.3, 0.0, 0.001, omega_cmd=[20.0] * NWHEEL)
        if k % 800 == 0 or k == 3999:
            print('  t=%5.2f  omega=%7.3f  err=%8.4f  T_axle=%8.1f  T_dyno=%8.1f'
                  % (s.t_s, s.wheel_omega_radps[RL], s.omega_error_radps[RL],
                     s.axle_torque_Nm[RL], s.dyno_torque_Nm[RL]))
    return 0


if __name__ == '__main__':
    raise SystemExit(_demo())
