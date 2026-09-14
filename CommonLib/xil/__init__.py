"""xil -- a simulated XIL dynamometer bench.

A stand-in for the hardware, so a coupling can be built and argued about before
the bench exists. Standard library only: it loads and unit-tests on a machine
with neither CARLA nor CarMaker.

Four modules are parts of the setup, one is the simulation that couples them:

    vehicle.py   Vehicle, Powertrain, Driveline    the car under test
    dyno.py      Dyno                              the dynamometer
    driver.py    RobotDriver                       who works the pedal
    link.py      LocalLink, UdpLink                the wire out to CARLA
    dynosim.py   DynoSim, State                    those three, running

DynoSim holds the parts and owns the equations of motion, because the
acceleration of a car on a dyno belongs to neither the car nor the dyno alone.

    sim = DynoSim(Vehicle(mass_kg=2100), Dyno(mode='axle'))
    state = sim.step(v_ref, dt)               # the driver chases a reference
    state = sim.step_pedals(thr, brk, dt)     # or work the pedal yourself
    sim.vehicle.powertrain.front_share
    sim.dyno.resistance(v)
"""

from .driver import RobotDriver
from .dyno import Dyno
from .dynosim import NWHEEL, DynoSim, State
from .link import LocalLink, UdpLink
from .vehicle import Driveline, Powertrain, Vehicle

__all__ = ['DynoSim', 'State', 'NWHEEL',
           'Vehicle', 'Powertrain', 'Driveline', 'Dyno',
           'RobotDriver', 'LocalLink', 'UdpLink']
