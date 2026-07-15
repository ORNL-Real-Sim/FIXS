"""
CI-safe unit tests for the World Model translator primitives (#189).

Exercises the FIXS-side plumbing -- BEV crop, rolling history,
constant-velocity backend, heading conversion -- with fabricated VehData. No
sockets, no SUMO, no torch/jax; runs in the plain ``realsim`` env:

    python -m pytest tests/WorldModels/test_wm_translator.py -v
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np
import pytest

HERE = pathlib.Path(__file__).resolve().parent
REPO = HERE.parents[1]
sys.path.insert(0, str(REPO))
sys.path.insert(0, str(HERE))

from CommonLib.VehDataMsgDefs import VehData  # noqa: E402

from bev import (  # noqa: E402
    RollingHistory,
    crop_square_bev,
    make_ego_future,
    math_rad_to_sumo_deg,
    push_frame,
    sumo_deg_to_math_rad,
)
from backends import make_backend  # noqa: E402


def veh(vid, x, y, heading=0.0, speed=10.0, length=5.0, width=2.0, height=1.5):
    v = VehData()
    v.id = vid.ljust(50)
    v.vehicleClass = "passenger".ljust(50)
    v.positionX, v.positionY, v.heading, v.speed = x, y, heading, speed
    v.length, v.width, v.height = length, width, height
    return v


# --------------------------------------------------------------------------- #
# Heading conversion round-trips
# --------------------------------------------------------------------------- #
@pytest.mark.parametrize("deg", [0.0, 90.0, 180.0, 270.0, 45.0, 123.4])
def test_heading_roundtrip(deg):
    assert math_rad_to_sumo_deg(sumo_deg_to_math_rad(deg)) == pytest.approx(deg, abs=1e-9)


def test_heading_convention():
    # SUMO 0 deg = north (+Y) = math pi/2 ; SUMO 90 deg = east (+X) = math 0
    assert sumo_deg_to_math_rad(0.0) == pytest.approx(math.pi / 2)
    assert sumo_deg_to_math_rad(90.0) == pytest.approx(0.0)


# --------------------------------------------------------------------------- #
# Square BEV crop
# --------------------------------------------------------------------------- #
def test_crop_square_bev_membership_and_ego():
    vehicles = [
        veh("ego", 100.0, 100.0),
        veh("near", 120.0, 90.0),     # inside +/-50 box
        veh("edge", 150.0, 100.0),    # exactly on the boundary -> inside
        veh("far_x", 151.0, 100.0),   # outside in X
        veh("far_y", 100.0, 155.0),   # outside in Y
    ]
    hist = RollingHistory(depth=4)
    bev = crop_square_bev(vehicles, "ego", half_extent=50.0, history=hist)
    assert bev is not None
    assert set(bev.ids) == {"ego", "near", "edge"}
    assert bev.ids[bev.ego_idx] == "ego"
    # non_ego_mask drops exactly the ego row
    assert bev.non_ego_mask.sum() == len(bev.ids) - 1


def test_crop_returns_none_without_ego():
    hist = RollingHistory(depth=4)
    bev = crop_square_bev([veh("other", 0, 0)], "ego", 50.0, hist)
    assert bev is None


# --------------------------------------------------------------------------- #
# Rolling history
# --------------------------------------------------------------------------- #
def test_rolling_history_warm_and_prev():
    hist = RollingHistory(depth=3)
    assert not hist.is_warm
    for t in range(3):
        bev = crop_square_bev([veh("ego", 0, 0), veh("a", 10 + t, 0)], "ego", 50.0, hist)
        push_frame(bev)
    assert hist.is_warm
    prev = hist.prev_state("a")          # second-to-last frame: a at x=11
    assert prev is not None and prev[0] == pytest.approx(11.0)
    win = hist.window(["ego", "a"])
    assert win.shape == (2, 3, 4)


# --------------------------------------------------------------------------- #
# constant_velocity backend: advance by reported speed+heading; ego untouched.
# It must ADVANCE the position every step (not echo it) -- echoing is the
# moveToXY feedback deadlock that froze vehicles.
# --------------------------------------------------------------------------- #
def test_constant_velocity_advances_and_ego_skipped():
    backend = make_backend("constant_velocity")
    backend.reset({"StepLength": 0.1})

    hist = RollingHistory(depth=4)
    # a at x=140, moving east (SUMO heading 90 = +X) at 10 m/s
    b0 = crop_square_bev([veh("ego", 100, 100), veh("a", 140, 100, heading=90, speed=10)],
                         "ego", 60.0, hist)
    out0 = backend.step(b0)
    assert "ego" not in out0                       # ego never moved by the WM
    # advances by speed*dt = 10*0.1 = 1.0 m along +X, even on the first frame
    assert out0["a"].x == pytest.approx(141.0, abs=1e-6)
    assert out0["a"].y == pytest.approx(100.0, abs=1e-6)
    assert out0["a"].speed == pytest.approx(10.0)

    # a moving north (SUMO heading 0 = +Y): advances in +Y, not +X
    b1 = crop_square_bev([veh("ego", 100, 100), veh("a", 140, 100, heading=0, speed=10)],
                         "ego", 60.0, hist)
    out1 = backend.step(b1)
    assert out1["a"].x == pytest.approx(140.0, abs=1e-6)
    assert out1["a"].y == pytest.approx(101.0, abs=1e-6)


# --------------------------------------------------------------------------- #
# Ego-alone bubble (background traffic exited on the teardrop): zero neighbours.
# The integration must handle this gracefully -- empty neighbour set, no crash.
# --------------------------------------------------------------------------- #
def test_ego_alone_zero_neighbours():
    hist = RollingHistory(depth=4)
    backend = make_backend("constant_velocity")
    backend.reset({"StepLength": 0.1})

    # only the ego in the bubble (on the loop turnaround, traffic has exited)
    bev = crop_square_bev([veh("egoCm123", 500, 0, heading=90, speed=8)],
                          "egoCm123", 200.0, hist)
    assert bev is not None
    assert bev.ids == ["egoCm123"]
    assert bev.non_ego_mask.sum() == 0        # zero neighbours
    push_frame(bev)
    out = backend.step(bev)
    assert out == {}                           # nothing to write back -- no crash


# --------------------------------------------------------------------------- #
# Ego future (CATK conditioning) -- const velocity in usage 1
# --------------------------------------------------------------------------- #
def test_make_ego_future_const_velocity():
    hist = RollingHistory(depth=4)
    # ego heading east (SUMO 90) at 10 m/s from (0,0); dt=0.1, horizon=5
    bev = crop_square_bev([veh("ego", 0, 0, heading=90, speed=10)], "ego", 50.0, hist)
    fut = make_ego_future(bev, horizon=5, dt=0.1)
    assert fut.shape == (5, 3)
    # first waypoint 0.1s ahead: x = 0 + 10*0.1 = 1.0, y ~ 0
    assert fut[0, 0] == pytest.approx(1.0, abs=1e-6)
    assert fut[0, 1] == pytest.approx(0.0, abs=1e-6)
    # last waypoint 0.5s ahead: x = 5.0
    assert fut[-1, 0] == pytest.approx(5.0, abs=1e-6)
