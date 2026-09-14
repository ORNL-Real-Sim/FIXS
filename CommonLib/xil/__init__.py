"""xil -- a simulated XIL dyno bench.

    v_ref -> [RobotDriver] -> pedals -> [DynoSim] -> speed
             Bench ties those two together
             LocalLink / UdpLink carry v_ref out and the speed back

Standard library only, so it loads and unit-tests on a machine with neither
CARLA nor CarMaker. See dyno.py for the physics.
"""

from .bench import Bench
from .driver import RobotDriver
from .dyno import Dyno, DynoSim, State, Vehicle
from .link import LocalLink, UdpLink

__all__ = ['Vehicle', 'Dyno', 'DynoSim', 'State',
           'RobotDriver', 'Bench', 'LocalLink', 'UdpLink']
