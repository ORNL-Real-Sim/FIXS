"""xil -- a simulated XIL dynamometer bench.

A stand-in for the hardware, so a coupling can be built and argued about before
the bench exists. Standard library only: it loads and unit-tests on a machine
with neither CARLA nor CarMaker.

    vehicle.py   Powertrain, Driveline, Vehicle    the car under test
    dyno.py      Dyno                              the dynamometer
    driver.py    RobotDriver                       whoever works the pedal
    bench.py     Bench, State                      the three of them running
    link.py      LocalLink, UdpLink                the wire to a simulator

    bench = Bench()
    state = bench.step(v_ref, dt)          # driver chases a speed reference
    state = bench.step_pedals(thr, brk, dt)  # or work the pedal yourself
"""

from .bench import NWHEEL, Bench, State
from .driver import RobotDriver
from .dyno import Dyno
from .link import LocalLink, UdpLink
from .vehicle import Driveline, Powertrain, Vehicle

__all__ = ['Bench', 'State', 'NWHEEL',
           'Vehicle', 'Powertrain', 'Driveline', 'Dyno',
           'RobotDriver', 'LocalLink', 'UdpLink']
