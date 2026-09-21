"""fixs.xil -- the dynamometer this scenario declares, if it declares one.

A submodule, like fixs.carla, because a plant is not part of the wire API and a
controller that is not driving one has no reason to load it.

    from fixs import xil

    self.dyno = xil.dyno()                 # None unless XilSetup.EnableXil
    ...
    if self.dyno is not None:
        vRef = self.dyno.exchange(vRef, dt)

What this saves a controller is reading XilSetup itself -- which yaml, which
transport, which endpoint, and the bench's own parameters. Everything it builds
is CommonLib.xil, which a controller with its own ideas can build directly.
"""

import os

from CommonLib.ConfigHelper import ConfigHelper

from . import FixsError

__all__ = ['enabled', 'dyno']


def _scenario(configPath):
    """The scenario this run is using, read the same way fixs.connect() reads it."""
    configPath = configPath or os.environ.get('FIXS_CONFIG_YAML')
    if not configPath or not os.path.exists(configPath):
        raise FixsError(
            'cannot tell whether this scenario declares a dynamometer: '
            + ('$FIXS_CONFIG_YAML is not set' if not configPath
               else '%s does not exist' % configPath)
            + '. Pass the scenario yaml, or set $FIXS_CONFIG_YAML to the one '
              'this run is using.')
    config = ConfigHelper()
    config.getConfig(configPath)
    return config.Xil_setup


def enabled(configPath=None):
    """Is a dynamometer in this run's loop? -> bool

    Ask this rather than testing whether :func:`dyno` returned something. The
    two are different questions: this one is about the SCENARIO, and it stays
    true for a controller that brings its own cell client instead of the one
    below. A rig with its own dyno software still answers the same flag, so the
    control flow that depends on a bench being present does not have to care
    who talks to it.
    """
    return bool(_scenario(configPath)['EnableXil'])


def dyno(configPath=None, vehicle=None, dyno=None):
    """The dynamometer this scenario declares, or None when it declares none.

    A bench is not a plant that owns the ego. The virtual environment still
    integrates position, heading and everything lateral; the bench answers one
    question, and it is the longitudinal one: *you asked for this speed -- here
    is what a real vehicle did with it*. A controller sends its command through
    the bench and commands the simulator with what came back, so the vehicle's
    mass and the time its torque takes to arrive are in the loop rather than
    beside it.

    Reads ``XilSetup`` from the yaml TrafficLayer is running -- ``EnableXil``
    switches it on, ``Transport`` picks inprocess/udp/tcp, and the endpoint is
    the ``VehicleSubscription``'s ip and port. Same source as :func:`connect`,
    for the same reason: two yamls that disagree are two different scenarios.

        self.dyno = fixs.dyno()                    # None unless EnableXil
        ...
        if self.dyno is not None:
            vRef = self.dyno.exchange(vRef, dt)

    ``inprocess`` runs CommonLib.xil here in this process, so the coupling can
    be exercised with no hardware and over the same call the cell takes. What
    vehicle is on the bench, and what the dyno does, come from the yaml too:

        XilSetup:
          Vehicle: {mass_kg: 2100.0, torque_bandwidth_Hz: 5.0}
          Dyno:    {road_A_N: 111.0, roller_inertia_kgm2: 40.0}

    Forwarded by name, so the yaml names the parameter rather than restating
    a list -- and an unknown one fails the run rather than leaving a bench
    silently on its defaults.
    """
    xil = _scenario(configPath)
    if not xil['EnableXil']:
        return None

    subs = xil['VehicleSubscription'] or []
    host = (subs[0].get('ip') or ['127.0.0.1'])[0] if subs else '127.0.0.1'
    port = (subs[0].get('port') or [None])[0] if subs else None
    # Stated in code wins over the yaml: a caller who writes the mass down
    # is saying what is on the bench, and should not have to edit a scenario
    # as well to be believed.
    return _Dyno(xil['Transport'], host, port,
                 vehicle=dict(xil['Vehicle'], **(vehicle or {})),
                 dyno=dict(xil['Dyno'], **(dyno or {})))


class _Dyno:
    """The bench, as the simulator sees it. Get one from :func:`dyno`.

    One call, because there is only one thing to ask. ``exchange`` is a round
    trip: the reference goes out, the achieved speed comes back.
    """

    def __init__(self, transport, host='127.0.0.1', port=None,
                 vehicle=None, dyno=None):
        from CommonLib.xil.dynosim import Dyno, DynoSim
        from CommonLib.xil.link import LocalLink, TcpLink, UdpLink
        from CommonLib.xil.vehicle import Vehicle

        self.transport = transport
        self.sim = None
        self.misses = 0
        if transport == 'inprocess':
            # Named parameters, straight through. CommonLib.xil refuses one it
            # does not know rather than ignoring it, so a typo in the yaml is a
            # failed run and not a bench quietly running on its defaults.
            self.sim = DynoSim(Vehicle(**(vehicle or {})), Dyno(**(dyno or {})))
            self.link = LocalLink()
        elif transport == 'tcp':
            self.link = (TcpLink('simulator', peer_ip=host, port=port)
                         if port else TcpLink('simulator', peer_ip=host))
        elif transport == 'udp':
            self.link = UdpLink('simulator', peer_ip=host)
        else:
            raise FixsError(
                f'unknown XilSetup.Transport {transport!r}. '
                f'Use one of: inprocess, udp, tcp.')

    def exchange(self, speed, dt, steer=0.0):
        """(float, float) -> float -- ask for a speed, get what the bench did.

        The bench's answer whenever there is one. When there is not -- the cell
        has not answered yet, or has stopped -- the reference comes straight
        back, which is the run behaving as though no bench were attached. That
        is the only fallback that cannot invent motion; ``misses`` counts how
        often it was taken, and a run that ends with a large one did not test
        what it claims to have tested.
        """
        self.link.send_reference(speed, steer)
        if self.sim is not None:
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
