from dataclasses import dataclass, field
import ctypes
from typing import List

@dataclass
class VehData:
    # Fixed-size arrays for string-like fields (50 bytes each)
    id: str = field(default_factory=lambda: ' ' * 50)  # char[50]
    type: str = field(default_factory=lambda: ' ' * 50)  # char[50]
    vehicleClass: str = field(default_factory=lambda: ' ' * 50)  # char[50]
    
    # Floating point and integer fields
    speed: float = 0.0
    acceleration: float = 0.0
    positionX: float = 0.0
    positionY: float = 0.0
    positionZ: float = 0.0
    heading: float = 0.0
    color: int = 0  # uint32_t
    
    linkId: str = field(default_factory=lambda: ' ' * 50)  # char[50]
    laneId: int = 0 # uint32_t
    distanceTravel: float = 0.0
    speedDesired: float = 0.0
    accelerationDesired: float = 0.0
    
    hasPrecedingVehicle: int = 0  # Boolean-like integer (0 or 1)
    precedingVehicleId: str = field(default_factory=lambda: ' ' * 50)  # char[50]
    precedingVehicleDistance: float = 0.0
    precedingVehicleSpeed: float = 0.0
    
    signalLightId: str = field(default_factory=lambda: ' ' * 50)  # char[50]
    signalLightHeadId: int = 0
    signalLightDistance: float = 0.0
    signalLightColor: int = 0  # int8_t
    
    speedLimit: float = 0.0
    speedLimitNext: float = 0.0
    speedLimitChangeDistance: float = 0.0
    
    linkIdNext: str = field(default_factory=lambda: ' ' * 50)  # char[50]
    grade: float = 0.0
    
    length: float = 0.0
    width: float = 0.0
    height: float = 0.0
    
    activeLaneChange: int = 0  # Boolean-like integer (-1, 0, or 1)

    # #174 EgoDriver command channel (L2/L4). Serialized at the END, gated by
    # VehicleMessageField. steer is a physical angle; pedals are unitless positions.
    steerAngleDesired: float = 0.0        # rad, desired front road-wheel steer angle
    acceleratorPedalDesired: float = 0.0  # [0,1] accelerator pedal position
    brakePedalDesired: float = 0.0        # [0,1] brake pedal position

    def get(self, field_name, default=None):
        return getattr(self, field_name, default)


@dataclass
class TrafficLightData:
    id: int # uint16_t
    name: str
    state: str

    def get(self, field_name, default=None):
        return getattr(self, field_name, default)
    
@dataclass
class DetectorData:
    id: int # uint8_t
    name: str
    state: int # uint8_t