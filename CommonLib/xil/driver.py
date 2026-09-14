"""The robot driver on the dyno, and the self-driving dyno car (#323).

A dyno bench does not drive itself. Somebody works the pedal: on a real
emissions cell that is a driver robot chasing a speed trace, and on ours it is
whatever chases the reference the simulator sends. ``RobotDriver`` is that, and
``DynoVehicle`` bolts it to the bench so the pair presents one interface --
speed reference in, achieved speed out -- which is the interface the wire
carries.

WHY THE DRIVER IS A SEPARATE CLASS
----------------------------------
Because it is a separate thing, and the two get swapped independently. The bench
is physics and is the same whoever is driving; the driver is a control law and
will eventually want to be a characterisation of the real vehicle's own
controller rather than a PI. Keeping them apart also lets the bench be driven
from a recorded pedal trace with no driver at all.

THE DRIVER'S JOB IS TO DRIVE THE SPEED ERROR TO ZERO
-----------------------------------------------------
That is the whole specification, and it is why the integral term is not
optional. Under road resistance a steady speed needs a steady pedal, and a
proportional-only law can only produce one from a standing error -- so it would
sit permanently below the reference by however much error generates the pedal it
needs. The integrator is what removes that.

A PI is a placeholder and is meant to be. It stands in for a real vehicle's ACC
or driver robot, and it will flatter one: a real controller has rate limits, an
acceleration envelope and regen blending that this does not. That matters when
the bench is asked to PREDICT the hardware. It does not matter for building the
coupling, which is what this is for now.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional, Tuple

from .dyno import DynoConfig, DynoSimulator, DynoState


@dataclass
class RobotDriverParams:
    """Gains are in pedal units per speed error, so they are dimensionless over
    m/s. A pedal of 1.0 is full throttle; -1.0 is full brake."""

    kp: float = 0.45                # pedal per (m/s)
    ki: float = 0.25                # pedal per (m/s * s)
    #: Separate ceilings, because a vehicle's brake authority is not its drive
    #: authority and pinning both to 1.0 hides that.
    max_throttle: float = 1.0
    max_brake: float = 1.0
    #: Below this reference the driver commands a standstill brake rather than
    #: chasing zero with a shrinking pedal. Real drivers stop; PIs dither.
    standstill_ref_mps: float = 0.05
    standstill_brake: float = 0.3


class RobotDriver:
    """PI on speed error, producing a pedal split into throttle and brake.

    One pedal axis, not two independent ones: positive is throttle, negative is
    brake, and they are never both non-zero. That is what a driver does and what
    a ``VehicleControl`` expects.
    """

    def __init__(self, params: Optional[RobotDriverParams] = None):
        self.p = params or RobotDriverParams()
        self.integral = 0.0
        self.last_pedal = 0.0

    def reset(self) -> None:
        self.integral = 0.0
        self.last_pedal = 0.0

    def step(self, v_ref: float, v_measured: float, dt: float) -> Tuple[float, float]:
        """Returns ``(throttle, brake)``, each in [0, 1] and never both positive."""
        if dt <= 0.0:
            raise ValueError('dt must be positive')
        p = self.p

        if v_ref <= p.standstill_ref_mps and v_measured <= p.standstill_ref_mps:
            # Asked to stand still and standing still. Hold the brake and stop
            # integrating, or the integral winds on an error that cannot close.
            self.integral = 0.0
            self.last_pedal = -p.standstill_brake
            return 0.0, min(p.max_brake, p.standstill_brake)

        err = v_ref - v_measured
        raw = p.kp * err + p.ki * self.integral
        if -p.max_brake < raw < p.max_throttle:
            self.integral += err * dt          # anti-windup: freeze at the rail
        pedal = max(-p.max_brake,
                    min(p.max_throttle, p.kp * err + p.ki * self.integral))
        self.last_pedal = pedal
        return (pedal, 0.0) if pedal >= 0.0 else (0.0, -pedal)


@dataclass
class DynoVehicleConfig:
    dyno: DynoConfig = field(default_factory=DynoConfig)
    driver: RobotDriverParams = field(default_factory=RobotDriverParams)


class DynoVehicle:
    """A driver and a bench, presenting the interface the wire carries.

    ``step(v_ref, dt) -> DynoState``. What comes back is what the vehicle
    achieved, which may be well short of what was asked -- that gap is the whole
    point of having a plant here rather than a pass-through.
    """

    def __init__(self, config: Optional[DynoVehicleConfig] = None,
                 dyno: Optional[DynoSimulator] = None,
                 driver: Optional[RobotDriver] = None):
        cfg = config or DynoVehicleConfig()
        self.dyno = dyno or DynoSimulator(cfg.dyno)
        self.driver = driver or RobotDriver(cfg.driver)
        self.last_throttle = 0.0
        self.last_brake = 0.0

    @property
    def speed_mps(self) -> float:
        return self.dyno.v if self.dyno.cfg.mode == 'chassis' else \
            sum(self.dyno.omega) / len(self.dyno.omega) \
            * self.dyno.cfg.driveline.wheel_radius_m

    def step(self, v_ref: float, dt: float) -> DynoState:
        thr, brk = self.driver.step(v_ref, self.speed_mps, dt)
        self.last_throttle, self.last_brake = thr, brk
        return self.dyno.step(thr, brk, dt)

    def reset(self, speed_mps: float = 0.0) -> None:
        self.dyno.reset(speed_mps)
        self.driver.reset()
        self.last_throttle = self.last_brake = 0.0
