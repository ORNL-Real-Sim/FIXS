"""xil -- a simulated hardware-in-the-loop dyno bench (#323).

A stand-in for the XIL hardware, so a coupling can be built and argued about
before the bench exists. Nothing in this package imports a simulator SDK or
numpy: it is standard library only, so it loads on a machine with neither CARLA
nor CarMaker and can be unit-tested without either.

Three pieces, which is the same three pieces the real bench has::

    v_ref ─▶ [RobotDriver] ─▶ pedals ─▶ [DynoSimulator] ─▶ v_measured
             the driving                 the physics

    ... and [link] carries v_ref out and v_measured back, either in one
        process or over UDP in the format the ORNL cell speaks.

``DynoVehicle`` bolts the first two together, so the pair presents the interface
the wire carries: a speed reference in, the speed actually achieved out. The gap
between those two is the whole reason there is a plant here.

The dyno applies road resistance and nothing else; the driver is the only
authority over speed. See ``dyno.py`` for why it does not also offer a
speed-controlled mode.
"""

from .driver import (DynoVehicle, DynoVehicleConfig, RobotDriver,
                     RobotDriverParams)
from .dyno import (FL, FR, MODES, NWHEEL, RL, RR,
                   AxleDynoParams, ChassisDynoParams, DrivelineParams,
                   DynoConfig, DynoSimulator, DynoState, PowertrainParams,
                   RoadResistanceParams, envelope_powertrain)
from .link import (DEFAULT_MEASUREMENT_PORT, DEFAULT_REFERENCE_PORT,
                   DEFAULT_STALE_S, PACKET_SIZE,
                   InProcessDynoSide, InProcessPair, InProcessSimulatorSide,
                   UdpDynoSide, UdpSimulatorSide, pack, unpack)

__all__ = [
    # the bench
    'DynoSimulator', 'DynoConfig', 'DynoState',
    'PowertrainParams', 'DrivelineParams', 'RoadResistanceParams',
    'ChassisDynoParams', 'AxleDynoParams', 'envelope_powertrain',
    'MODES', 'FL', 'FR', 'RL', 'RR', 'NWHEEL',
    # the driving
    'RobotDriver', 'RobotDriverParams', 'DynoVehicle', 'DynoVehicleConfig',
    # the wire
    'InProcessPair', 'InProcessSimulatorSide', 'InProcessDynoSide',
    'UdpSimulatorSide', 'UdpDynoSide',
    'pack', 'unpack', 'PACKET_SIZE',
    'DEFAULT_REFERENCE_PORT', 'DEFAULT_MEASUREMENT_PORT', 'DEFAULT_STALE_S',
]
