"""xil -- simulated hardware-in-the-loop benches (#323).

Stand-ins for XIL hardware, so a coupling can be built and argued about before
the bench exists. Nothing in this package imports a simulator SDK or numpy: it
is standard library only, so it loads on a machine with neither CARLA nor
CarMaker and can be unit-tested without either.

``DynoSimulator`` is a dynamometer plus the vehicle bolted to it, in one of two
shapes. ``mode='chassis'`` puts the vehicle on rollers, applies road load, and
gives you back a speed. ``mode='axle'`` replaces the wheels with speed-controlled
hub units, owns no body at all, and gives you back measured axle torque for a
wheel speed you command. Which one you want depends on who owns the vehicle
dynamics -- see ``dyno.py`` for the picture.
"""

from .dyno import (
    CONTROLS, FL, FR, MODES, NWHEEL, RL, RR,
    AxleDynoParams, ChassisDynoParams, DrivelineParams, DynoConfig,
    DynoSimulator, DynoState, PowertrainParams, RoadLoadParams, ServoParams,
    envelope_powertrain,
)

__all__ = [
    'DynoSimulator', 'DynoConfig', 'DynoState',
    'PowertrainParams', 'DrivelineParams', 'RoadLoadParams', 'ServoParams',
    'ChassisDynoParams', 'AxleDynoParams',
    'envelope_powertrain',
    'MODES', 'CONTROLS', 'FL', 'FR', 'RL', 'RR', 'NWHEEL',
]
