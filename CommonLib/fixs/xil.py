"""fixs.xil -- the simulated dynamometer FIXS ships.

    import fixs.xil

    if fixs.xil.enabled():
        dyno = fixs.xil.dynosim(vehicle={'mass_kg': 2100.0})
        vRef = dyno.exchange(vRef, aRef, dt)

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


def dynosim(vehicle=None, dyno=None, driver=None):
    """The simulated dyno, built from the vehicle, dyno and robot-driver
    parameters given. Reads no scenario: whether one is in the loop is the
    caller's question (:func:`enabled`, ``fixs.config.get('xil')``).
    CommonLib.xil refuses a parameter it does not know, so a typo fails.
    """
    return _Dyno(vehicle=vehicle, dyno=dyno, driver=driver)


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

    def exchange(self, vref, aref, dt):
        """Step the bench once by dt and return the speed it reached.

        vref is the speed command (m/s); aref the acceleration command (m/s^2),
        read only by the robot driver's hold (``stop_speed_mps``/``stop_accel_mps2``).
        """
        self.link.send_reference(vref)
        reference = self.link.recv_reference()
        self.sim.step(reference[0] if reference else 0.0, dt, a_ref=aref)
        self.link.send_measurement(self.sim.speed)
        got = self.link.recv_measurement()
        if got is None:
            self.misses += 1
            return vref
        return got[0]

    def age(self):
        """Seconds since the bench last answered, or None if it never has."""
        return self.link.measurement_age()

    def close(self):
        self.link.close()
