#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Mar 20 13:43:01 2025

@author: arms
"""

import carla
import math

def get_carla_transform(offset=0, in_location=(0,0,0), in_rotation=(0,0,0)):
    """
    Returns carla transform based on sumo transform.
    """
    # offset = (801.49,264.53)
    # in_location = (1223.40,331.17,181)
    # in_rotation = (0,0,90)

    # From front-center-bumper to center (sumo reference system).
    # (http://sumo.sourceforge.net/userdoc/Purgatory/Vehicle_Values.html#angle)
    yaw = -1 * in_rotation[2] + 90
    pitch = in_rotation[1]
    out_location = (in_location[0] - math.cos(math.radians(yaw)) * 1,
                    in_location[1] - math.sin(math.radians(yaw)) * 1,
                    in_location[2] - math.sin(math.radians(pitch)) * 1)
    out_rotation = (in_rotation[0], in_rotation[1], in_rotation[2])

    # Applying offset sumo-carla net.
    out_location = (out_location[0] - offset[0], out_location[1] - offset[1], out_location[2])

    ## Transform to carla reference system (left-handed system).
    # out_transform = carla.Transform(
    #     carla.Location(out_location[0], -out_location[1], out_location[2]),
    #     carla.Rotation(out_rotation[0], out_rotation[1] - 90, out_rotation[2]))

    out_transform = (out_location[0], -out_location[1], out_location[2], out_rotation[0], out_rotation[1], out_rotation[2]-90)

    return out_transform
