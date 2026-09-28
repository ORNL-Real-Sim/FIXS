"""fixs.xil -- the simulated dynamometer, when the scenario asks for one.

    import fixs.xil

    dyno = fixs.xil.dynosim()       # None unless XilSetup.EnableXil
    if dyno is not None:
        vRef = dyno.exchange(vRef, dt)

FIXS ships the simulated dyno and nothing else. Which dyno answers a run is
the application's choice: ``Transport: udp`` means it talks to a rig of its
own, at ``fixs.config.get('xil')['ip']`` and ``['port']``.
"""

from . import config

__all__ = ['enabled', 'dynosim']


def enabled(configPath=None):
    """Is a dynamometer in this run's loop? -> bool

    True for every transport, including a rig the application talks to itself.
    """
    return bool(config.get('xil', configPath)['enable_xil'])


def dynosim(configPath=None, vehicle=None, dyno=None, driver=None):
    """The simulated dyno this scenario describes, or None unless EnableXil.

    Vehicle, dyno and robot-driver parameters come from ``XilSetup.Vehicle``,
    ``Dyno`` and ``Driver``; any given here win over the yaml. CommonLib.xil
    refuses a parameter it does not know, so a typo fails the run.
    """
    xil = config.get('xil', configPath)
    if not xil['enable_xil']:
        return None
    return _Dyno(vehicle=dict(xil['vehicle'] or {}, **(vehicle or {})),
                 dyno=dict(xil['dyno'] or {}, **(dyno or {})),
                 driver=dict(xil['driver'] or {}, **(driver or {})))


class _Dyno:
    """The simulated bench. ``exchange`` steps it once and returns the speed reached."""

    def __init__(self, vehicle=None, dyno=None, driver=None):
        from CommonLib.xil.driver import RobotDriver
        from CommonLib.xil.dynosim import Dyno, DynoSim
        from CommonLib.xil.link import LocalLink
        from CommonLib.xil.vehicle import Vehicle

        self.sim = DynoSim(Vehicle(**(vehicle or {})), Dyno(**(dyno or {})),
                           RobotDriver(**(driver or {})))
        self.link = LocalLink()
        self.misses = 0

    def exchange(self, speed, dt, steer=0.0):
        """(float, float) -> float -- step the bench once by dt, return its speed."""
        self.link.send_reference(speed, steer)
        reference = self.link.recv_reference()
        self.sim.step(reference[0] if reference else 0.0, dt)
        self.link.send_measurement(self.sim.speed)
        got = self.link.recv_measurement()
        if got is None:
            self.misses += 1
            return speed
        return got[0]

    def age(self):
        """Seconds since the bench last answered, or None if it never has."""
        return self.link.measurement_age()

    def close(self):
        self.link.close()
