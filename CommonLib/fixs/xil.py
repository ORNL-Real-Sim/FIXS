"""fixs.xil -- the simulated dynamometer, when the scenario asks for one.

    import fixs.xil

    dyno = fixs.xil.dyno()          # a DynoSim for Transport: inprocess, else None
    if dyno is not None:
        vRef = dyno.exchange(vRef, dt)

FIXS ships the simulated dyno and nothing else. ``Transport: udp`` or ``tcp``
names a wire the application speaks itself, to the address in
``fixs.config.get('xil')['ip']``; FIXS opens no socket for it.
"""

from . import config

__all__ = ['enabled', 'dyno']


def enabled(configPath=None):
    """Is a dynamometer in this run's loop? -> bool

    True for every transport, including a rig the application talks to itself.
    """
    return bool(config.get('xil', configPath)['enable_xil'])


def dyno(configPath=None, vehicle=None, dyno=None, driver=None):
    """The simulated dyno for ``Transport: inprocess``, else None.

    Vehicle, dyno and robot-driver parameters come from ``XilSetup.Vehicle``,
    ``Dyno`` and ``Driver``; any given here win over the yaml. CommonLib.xil
    refuses a parameter it does not know, so a typo fails the run.
    """
    xil = config.get('xil', configPath)
    if not xil['enable_xil'] or xil['transport'] != 'inprocess':
        return None
    return _Dyno(vehicle=dict(xil['vehicle'] or {}, **(vehicle or {})),
                 dyno=dict(xil['dyno'] or {}, **(dyno or {})),
                 driver=dict(xil['driver'] or {}, **(driver or {})))


class _Dyno:
    """The simulated bench. ``exchange`` sends a speed and returns the one reached."""

    def __init__(self, vehicle=None, dyno=None, driver=None):
        from CommonLib.xil.driver import RobotDriver
        from CommonLib.xil.dynosim import Dyno, DynoSim
        from CommonLib.xil.link import LocalLink
        from CommonLib.xil.vehicle import Vehicle

        self.transport = 'inprocess'
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
