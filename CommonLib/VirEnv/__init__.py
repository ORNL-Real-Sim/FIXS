"""VirEnv -- the backend-agnostic virtual-environment bridge core (#174, #325).

The Python peer of the C++ ``CommonLib/VirEnvCore.{h,cpp}`` +
``CommonLib/IVirEnvBackend.h`` pair. Nothing in this package imports a simulator
SDK: a backend does that, and backends live with their host (the Carla one under
``Carla/VirEnv/``), so this package loads on a machine with neither CARLA nor
CarMaker installed.
"""

from .FixsProtocol import kFeedHz, kFeedPeriodS, onFeedBoundary
from .IVirEnvBackend import (EgoState, IVirEnvBackend, Pose, VehClass, VehHandle,
                             kNoHandle)
from .VirEnvCore import InitErr, StepErr, VirEnvCore, lerpHeadingDeg

__all__ = [
    'kFeedPeriodS', 'kFeedHz', 'onFeedBoundary',
    'VehClass', 'VehHandle', 'kNoHandle', 'Pose', 'EgoState', 'IVirEnvBackend',
    'VirEnvCore', 'InitErr', 'StepErr', 'lerpHeadingDeg',
]
