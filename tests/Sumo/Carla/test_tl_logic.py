"""
Tier-1 logic tests for the SUMO <-> CARLA traffic-light sync.

These run on ANY computer with the `realsim` env (SUMO + traci/sumolib + the
CARLA *client* wheel from PyPI). They need NO CARLA server, NO GPU, and NO map
asset - only the tiny grid fixture in fixtures/. Run with:

    pytest test_tl_logic.py

The heavy, CARLA-dependent end-to-end check lives in verify_demo.py (gated on
CARLA_ROOT).
"""
import csv
import os
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
FIX = os.path.join(HERE, "fixtures")
# the co-sim runtime lives at the repo root: FIXS_root/Carla
CARLA = os.path.normpath(os.path.join(HERE, "..", "..", "..", "Carla"))
sys.path.insert(0, os.path.join(CARLA, "sumo"))
sys.path.insert(0, os.path.join(CARLA, "sumo", "run_synchronization"))

# The modules under test import `carla` (the client wheel - no server needed).
carla = pytest.importorskip("carla", reason="needs the carla client wheel (realsim env)")
traci = pytest.importorskip("traci", reason="needs SUMO tools on PYTHONPATH (realsim env)")

import sumo_carla_tl_sync as tlsync  # noqa: E402
from sumo_integration.bridge_helper import BridgeHelper  # noqa: E402

TABLE = os.path.join(FIX, "traffic_light_table.csv")
SUMOCFG = os.path.join(FIX, "grid_tls.sumocfg")


# ---------------------------------------------------------------- pure logic

def test_char_to_state_mapping():
    """SUMO RYG chars map to the right CARLA traffic-light states."""
    m = tlsync.map_sumo_char_to_carla_state
    assert m("r") == carla.TrafficLightState.Red
    assert m("u") == carla.TrafficLightState.Red
    assert m("y") == carla.TrafficLightState.Yellow
    assert m("G") == carla.TrafficLightState.Green
    assert m("g") == carla.TrafficLightState.Green
    assert m("s") == carla.TrafficLightState.Green
    assert m("O") == carla.TrafficLightState.Off
    assert m("?") == carla.TrafficLightState.Unknown


def test_load_tl_table():
    """The table parses into the expected rows/junctions."""
    rows, table = tlsync.load_tl_table(TABLE)
    assert len(rows) > 0
    assert len(table) == 5  # the grid fixture has 5 signalized junctions
    # every row is (junction_id:str, link_id:int, x:float, y:float)
    j, k, x, y = rows[0]
    assert isinstance(k, int) and isinstance(x, float) and isinstance(y, float)


def test_carla_transform_zero_offset():
    """With net_offset (0,0), a SUMO (x, y) maps to CARLA (x, -y) (the
    RoadRunner-local / --no-net-offset case)."""
    BridgeHelper.offset = (0.0, 0.0)
    sumo_t = carla.Transform(carla.Location(x=100.0, y=50.0, z=0.0),
                             carla.Rotation(pitch=0.0, yaw=0.0, roll=0.0))
    out = BridgeHelper.get_carla_transform(sumo_t, carla.Vector3D(0.0, 0.0, 0.0))
    assert abs(out.location.x - 100.0) < 1e-3
    assert abs(out.location.y - (-50.0)) < 1e-3


# ----------------------------------------------- table <-> net consistency

def test_table_junctions_match_net():
    """Every table junction is a real SUMO TL junction, and every table
    link_id is within that junction's state-string length. This is the check
    that catches a table generated for a *different* net."""
    table_juncs = set()
    max_link = {}
    with open(TABLE, newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            j = row["junction_id"]
            table_juncs.add(j)
            max_link[j] = max(max_link.get(j, 0), int(row["link_id"]))

    from sumolib import checkBinary
    traci.start([checkBinary("sumo"), "-c", SUMOCFG, "--no-warnings", "true"])
    try:
        sumo_juncs = set(traci.trafficlight.getIDList())
        out_of_range = []
        for j in table_juncs & sumo_juncs:
            state = traci.trafficlight.getRedYellowGreenState(j)
            if max_link[j] >= len(state):
                out_of_range.append((j, max_link[j], len(state)))
    finally:
        traci.close()

    assert table_juncs == sumo_juncs, f"junction mismatch: {table_juncs ^ sumo_juncs}"
    assert not out_of_range, f"link_id out of range: {out_of_range}"
