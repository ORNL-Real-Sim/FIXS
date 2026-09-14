"""Driver plus dyno: the self-driving bench (#323).

The two belong apart -- the dyno is physics and is the same whoever drives, the
driver is a control law and will eventually want to be a characterisation of the
real vehicle's own controller. This ties them together so the pair presents the
interface the wire carries: a speed reference in, the speed achieved out.

The gap between those two is the point. A pass-through would always deliver what
was asked; a bench does not.
"""

from .driver import RobotDriver
from .dyno import DynoSim


class Bench(object):
    """step(v_ref, dt) -> State. Read ``throttle`` and ``brake`` for what the
    driver did to get there."""

    def __init__(self, sim=None, driver=None):
        self.sim = sim or DynoSim()
        self.driver = driver or RobotDriver()
        self.throttle = 0.0
        self.brake = 0.0

    @property
    def speed(self):
        return self.sim.speed

    def step(self, v_ref, dt):
        self.throttle, self.brake = self.driver.step(v_ref, self.sim.speed, dt)
        return self.sim.step(self.throttle, self.brake, dt)

    def reset(self, speed=0.0):
        self.sim.reset(speed)
        self.driver.reset()
        self.throttle = self.brake = 0.0
