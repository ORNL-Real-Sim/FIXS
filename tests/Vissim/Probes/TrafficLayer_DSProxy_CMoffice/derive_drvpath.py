"""
Derive the closed-loop DrvPath for the authored junction-based simple_loop.rd5.

The loop is authored by gen_junction_loop_xodr.py (tests/Vissim/SimpleEcho) as 4
straight roads (odr ids 1-4 = bottom, right, top, left) + 4 corner JUNCTIONS
whose connecting-roads are odr ids 5-8 (br, tr, tl, bl), each a smooth R15 arc.
osc2cm maps the straights to rd5 Links and the connectors to Junction.<n>.Link.0;
the driving lane (LaneR.0) of each carries a LanePath.

DrvPath = the LanePaths in driving order
    1 -> 5 -> 2 -> 6 -> 3 -> 7 -> 4 -> 8
(bottom, br, right, tr, top, tl, left, bl) -- roadutil confirms this is the full
~774 m loop. The odr road ids are fixed by the authoring step, so the loop order
is a constant; only the LanePath ids (which osc2cm assigns) are looked up.

This replaces the earlier Node-chaining deriver, which assumed netconvert's
connector-as-Link layout; the authored xodr folds connectors into junctions.
"""
from __future__ import annotations
import pathlib
import re

# authored odr road ids in driving order (straight, connector, straight, ...)
LOOP = [1, 5, 2, 6, 3, 7, 4, 8]


def derive_drvpath(rd5: pathlib.Path):
    s = rd5.read_text(encoding="utf-8", errors="ignore")
    lane_lp = {int(m.group(2)): int(m.group(1))
               for m in re.finditer(r"LanePath\.\d+ = (\d+) (\d+)", s)}
    # straight roads -> rd5 Links (Tag + driving lane id)
    lt = {int(m.group(1)): int(m.group(2))
          for m in re.finditer(r"Link\.(\d+)\.Tag = odrRoadId:(\d+)", s)}
    ll = {int(m.group(1)): int(m.group(2))
          for m in re.finditer(r"Link\.(\d+)\.LaneSection\.0\.LaneR\.0\.ID = (\d+)", s)}
    # connector roads -> Junction.<n>.Link.0 (Tag + driving lane id)
    jt = {int(m.group(1)): int(m.group(2))
          for m in re.finditer(r"Junction\.(\d+)\.Link\.0\.Tag = odrRoadId:(\d+)", s)}
    jl = {int(m.group(1)): int(m.group(2))
          for m in re.finditer(r"Junction\.(\d+)\.Link\.0\.LaneSection\.0\.LaneR\.0\.ID = (\d+)", s)}

    road_lp = {}
    for n, road in lt.items():
        if ll.get(n) in lane_lp:
            road_lp[road] = lane_lp[ll[n]]
    for n, road in jt.items():
        if jl.get(n) in lane_lp:
            road_lp[road] = lane_lp[jl[n]]

    drvpath = [road_lp[r] for r in LOOP if r in road_lp]
    closed = len(drvpath) == len(LOOP)
    return drvpath, LOOP, len(drvpath), len(LOOP), closed


if __name__ == "__main__":
    here = pathlib.Path(__file__).resolve().parents[4]
    rd5 = here / "ProprietaryFiles" / "CM13_proj" / "Data" / "Road" / "simple_loop.rd5"
    drv, order, n, total, closed = derive_drvpath(rd5)
    print(f"chained {n}/{total}; closed loop: {closed}")
    print(f"odr road order: {order}")
    print(f"DRVPATH = {drv}")
