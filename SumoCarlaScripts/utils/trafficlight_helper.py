#!/usr/bin/env python

# Copyright (c) 2020 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
""" This module provides a helper for the co-simulation between sumo and carla ."""

# ==================================================================================================
# -- imports ---------------------------------------------------------------------------------------
# ==================================================================================================

import collections
import math

try:
    import carla  # pylint: disable=import-error
except ModuleNotFoundError:
    from dataclasses import dataclass

    @dataclass
    class _Location:
        x: float
        y: float
        z: float

    @dataclass
    class _Rotation:
        pitch: float = 0.0
        yaw: float = 0.0
        roll: float = 0.0

    class _CarlaShim:
        Location = _Location
        Rotation = _Rotation

    carla = _CarlaShim()
    
# Conditional import for unreal module
try:
    import unreal
    UNREAL_AVAILABLE = True
except ImportError:
    UNREAL_AVAILABLE = False
    

SumoTrafficLight = collections.namedtuple('SumoTrafficLight', 'junction_id link_id x y z heading')


# ==================================================================================================
# -- Bridge helper (SUMO <=> CARLA) ----------------------------------------------------------------
# ==================================================================================================


class TrafficLightHelper(object):
    """
    BridgeHelper provides methos to ease the co-simulation between sumo and carla.
    """
    offset = (0, 0)

    @staticmethod
    def set_offset(offset):
        TrafficLightHelper.offset = offset
    
    @staticmethod
    def create_sumo_traffic_light(junction_id, link_id, x, y, z, heading):
        return SumoTrafficLight(junction_id, link_id, x, y, z, heading)
    
    @staticmethod
    def create_sumo_transform(x, y, z, heading):
        # heading is in radians, convert to degrees
        x = float(x)
        y = float(y)
        z = float(z)
        heading = float(heading)
        heading = math.degrees(heading)
        heading = (heading + 360) % 360
        location = carla.Location(x, y, z)
        rotation = carla.Rotation(0, heading, 0)
        return location, rotation
        
    
    @staticmethod
    def sumo_transform_to_carla_transform(in_sumo_location, in_sumo_rotation):
        """
        Returns carla transform based on sumo transform.
        """
        offset = TrafficLightHelper.offset
        in_location = in_sumo_location
        in_rotation = in_sumo_rotation

        out_location = (in_location.x,
                        in_location.y,
                        in_location.z)
        out_rotation = (in_rotation.pitch, in_rotation.yaw, in_rotation.roll)

        # Applying offset sumo-carla net.
        out_location = (out_location[0] - offset[0], out_location[1] - offset[1], out_location[2])

        # Transform to carla reference system (left-handed system).

        return carla.Location(out_location[0], -out_location[1], out_location[2]), carla.Rotation(out_rotation[0], out_rotation[1] - 90, out_rotation[2])


    @staticmethod
    def is_unreal_available():
        """
        Check if the unreal module is available.
        """
        return UNREAL_AVAILABLE

    @staticmethod
    def carla_transform_to_unreal_transform(carla_location, carla_rotation):
        """
        Convert a CARLA transform (meters) to an Unreal Engine transform (centimeters).
        """
        if not UNREAL_AVAILABLE:
            raise ImportError("Unreal module is not available. This method requires the unreal module to be installed.")
        
        loc = carla_location
        rot = carla_rotation

        # Convert meters to centimeters
        unreal_location = unreal.Vector(loc.x * 100, loc.y * 100, loc.z * 100)
        unreal_rotation = unreal.Rotator(rot.roll, rot.pitch, -rot.yaw + 180)

        return unreal.Vector(unreal_location.x, unreal_location.y, unreal_location.z), unreal.Rotator(unreal_rotation.roll, unreal_rotation.pitch, unreal_rotation.yaw)
    
    @staticmethod
    def get_trafficlight_group_transform_from_trafficlight_unreal_transforms(trafficlight_unreal_transforms):
        """
        Get the transform of a traffic light group from a list of traffic lights, by calculating the center of the traffic lights.
        Returns the unreal transform of the traffic light group.
        """
        if not UNREAL_AVAILABLE:
            raise ImportError("Unreal module is not available. This method requires the unreal module to be installed.")
        
        # calculate the center of the traffic lights
        center_x = sum(trafficlight_unreal_location.x for trafficlight_unreal_location, _ in trafficlight_unreal_transforms) / len(trafficlight_unreal_transforms)
        center_y = sum(trafficlight_unreal_location.y for trafficlight_unreal_location, _ in trafficlight_unreal_transforms) / len(trafficlight_unreal_transforms)
        center_z = sum(trafficlight_unreal_location.z for trafficlight_unreal_location, _ in trafficlight_unreal_transforms) / len(trafficlight_unreal_transforms)
        
        
        # create the unreal transform of the traffic light group
        return unreal.Vector(center_x, center_y, center_z), unreal.Rotator(0, 0, 0)
        
    @staticmethod
    def carla_location_to_sumo_location(carla_location):
        """
        Convert a CARLA location (meters) to a SUMO location (meters).
        """
        offset = TrafficLightHelper.offset
        in_location = carla_location

        out_location = (in_location.x,
                        -in_location.y,
                        in_location.z)

        # Applying offset sumo-carla net.
        out_location = (out_location[0] + offset[0], out_location[1] + offset[1], out_location[2])
        return carla.Location(out_location[0], out_location[1], out_location[2])
    