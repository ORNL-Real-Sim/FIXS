"""The ego's route keeps the lanes the traffic simulator chose.

THE ROUTE IS THE AUTHORITY. SUMO already decided which lane the ego drives, edge
by edge, and sumo_route_points emits that choice as geometry -- lane.getShape(),
via-lanes included. So a plan only has to read the lane back, which CARLA's own
get_waypoint does: measured, the median distance from a route point to the lane
CARLA returns is 0.06 m over 2561 points, with none unmatched.

What this must NOT do is ask CARLA's route planner. GlobalRoutePlanner adds
lane changes as ZERO-COST graph edges (_lane_change_link), so the shortest-path
search takes one whenever it shortens the route at all:

    24  lane changes in a traced 5.39 km loop
    14  in a plan stock CARLA built for itself with set_destination
     2  when the lane is read off the route instead, both one flicker

Executing one saturates the steer at 0.800 for about a second -- the swing at an
intersection approach.

    python -m pytest tests/Python/unit/test_route_lanes.py
"""
from __future__ import annotations

import math

import pytest

from CommonLib.fixs import carla as relay

realCarla = pytest.importorskip("carla", reason="needs the CARLA PythonAPI")


class _Wp:
    def __init__(self, road_id, lane_id, left=None, right=None):
        self.road_id, self.lane_id = road_id, lane_id
        self._left, self._right = left, right

    def get_left_lane(self):
        return self._left

    def get_right_lane(self):
        return self._right


def test_a_lane_hop_and_back_is_undone():
    """Consecutive lane-centre points occasionally snap to the neighbour and
    straight back -- once in 2561 points, measured. A plan must not carry a lane
    change the route never asked for."""
    held = _Wp(1211, -2)
    strayed = _Wp(1211, -1, left=_Wp(1211, -2))
    kept = relay._holdLane(strayed, held)
    assert kept.lane_id == -2


def test_a_genuine_road_change_is_left_alone():
    held = _Wp(1211, -2)
    nxt = _Wp(1315, -1)
    assert relay._holdLane(nxt, held) is nxt


def test_no_previous_lane_means_nothing_to_hold():
    wp = _Wp(1211, -1)
    assert relay._holdLane(wp, None) is wp


def test_a_straight_route_is_lanefollow():
    from agents.navigation.local_planner import RoadOption
    pts = [(float(i) * 2.0, 0.0) for i in range(60)]
    assert relay._turnAt(pts, 30) == RoadOption.LANEFOLLOW


def test_a_turn_is_read_off_the_route_geometry():
    from agents.navigation.local_planner import RoadOption
    pts = [(float(i) * 2.0, 0.0) for i in range(30)]
    pts += [(58.0, float(i) * 2.0) for i in range(1, 30)]   # a left turn
    opt = relay._turnAt(pts, 30)
    assert opt in (RoadOption.LEFT, RoadOption.RIGHT)
    assert opt != RoadOption.LANEFOLLOW
