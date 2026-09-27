"""xil -- a simulated XIL dynamometer bench.

A stand-in for the hardware, so a coupling can be built and argued about before
the bench exists. Standard library only: it loads and unit-tests on a machine
with neither CARLA nor CarMaker.

    vehicle.py   Vehicle, Powertrain, Driveline    the car under test
    driver.py    RobotDriver                       who works the pedal
    link.py      LocalLink, UdpLink                the wire out to CARLA
    dynosim.py   Dyno, DynoSim, State              the dyno, and a car on it

DynoSim owns the equations of motion, because the acceleration of a car on a
dyno belongs to neither the car nor the dyno alone. It holds the parts by name.

    from CommonLib.xil.dynosim import Dyno, DynoSim
    from CommonLib.xil.vehicle import Vehicle

    sim = DynoSim(Vehicle(mass_kg=2100), Dyno(mode='axle'))
    state = sim.step(v_ref, dt)               # the driver chases a reference
    state = sim.step_pedals(thr, brk, dt)     # or work the pedal yourself
    sim.vehicle.powertrain.front_share
    sim.dyno.resistance(v)

Nothing is re-exported here on purpose. Import from the module that defines the
thing, so the import line says where it lives.
"""
