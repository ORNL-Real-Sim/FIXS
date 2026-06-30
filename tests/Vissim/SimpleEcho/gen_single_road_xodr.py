"""
Author the SimpleEcho loop as a SINGLE continuous OpenDRIVE road (no junctions).

WHY: netconvert's `--opendrive-output` turns the rounded SUMO loop into 8 lane
edges + 8 junction connectors, and exports those connectors degenerate
(near-zero-length, reversed reference lines, only a backward-direction lane).
Both VISSIM (ImportOpenDrive) and CarMaker (osc2cm) then import DISCONNECTED
links -- visible gaps and no through-route. The SUMO network itself is fine
(vehicles run the loop); the break is purely in the OpenDRIVE *export*.

So we bypass netconvert for the xodr and write one road whose reference line
traces the rounded rectangle (4 straight + 4 quarter-arc segments, R metres),
with one driving lane all the way around and the road's predecessor/successor
pointing to itself so the loop closes. Geometry matches gen_rounded_loop.py
(SimpleEchoClient) so VISSIM/CarMaker stay consistent with the SUMO source.

Outputs simple_loop.xodr here; copy to the CarMaker project + probe for osc2cm.

Run:  python gen_single_road_xodr.py [R]      (R defaults to 15 m)
"""
from __future__ import annotations
import math
import pathlib
import sys

HERE = pathlib.Path(__file__).resolve().parent
L = 200.0           # outer box side (matches the SUMO loop)
LANE_WIDTH = 3.2    # one driving lane


def main() -> int:
    R = float(sys.argv[1]) if len(sys.argv) > 1 else 15.0
    straight = L - 2.0 * R
    arc_len = (math.pi / 2.0) * R
    curv = 1.0 / R                      # +curvature = left turn (loop is CCW)

    # 8 segments CCW from (R,0) heading East: line, arc, line, arc, ...
    segs = []
    for _ in range(4):
        segs.append(("line", straight, 0.0))
        segs.append(("arc", arc_len, curv))

    # integrate to get each segment's start (x, y, hdg)
    x, y, hdg, s = R, 0.0, 0.0, 0.0
    geoms = []
    for typ, length, k in segs:
        geoms.append((s, x, y, hdg, length, typ, k))
        if typ == "line":
            x += length * math.cos(hdg)
            y += length * math.sin(hdg)
        else:
            eh = hdg + length * k
            x += (math.sin(eh) - math.sin(hdg)) / k
            y += (math.cos(hdg) - math.cos(eh)) / k
            hdg = eh
        s += length
    total = s

    pv = ""
    for s0, x0, y0, h0, length, typ, k in geoms:
        g = "<line/>" if typ == "line" else f'<arc curvature="{k:.10f}"/>'
        pv += (f'        <geometry s="{s0:.6f}" x="{x0:.6f}" y="{y0:.6f}" '
               f'hdg="{h0:.10f}" length="{length:.6f}">{g}</geometry>\n')

    xodr = f'''<?xml version="1.0" standalone="yes"?>
<OpenDRIVE>
    <header revMajor="1" revMinor="4" name="simple_loop" version="1.00" north="0" south="0" east="0" west="0"/>
    <road name="loop" length="{total:.6f}" id="1" junction="-1">
        <link>
            <predecessor elementType="road" elementId="1" contactPoint="end"/>
            <successor elementType="road" elementId="1" contactPoint="start"/>
        </link>
        <type s="0" type="town"/>
        <planView>
{pv}        </planView>
        <elevationProfile>
            <elevation s="0" a="0" b="0" c="0" d="0"/>
        </elevationProfile>
        <lanes>
            <laneSection s="0">
                <center>
                    <lane id="0" type="none" level="false"/>
                </center>
                <right>
                    <lane id="-1" type="driving" level="false">
                        <link/>
                        <width sOffset="0" a="{LANE_WIDTH}" b="0" c="0" d="0"/>
                        <roadMark sOffset="0" type="solid" weight="standard" color="standard" width="0.12"/>
                    </lane>
                </right>
            </laneSection>
        </lanes>
    </road>
</OpenDRIVE>
'''
    out = HERE / "simple_loop.xodr"
    out.write_text(xodr, encoding="utf-8")
    print(f"[gen] single-road loop R={R} m, length={total:.1f} m, "
          f"{len(geoms)} geometries -> {out.name}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
