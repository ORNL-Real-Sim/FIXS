"""
Author the SimpleEcho loop as an OpenDRIVE network: 4 straight roads + 4 corner
JUNCTIONS whose connecting-roads are real R-metre arcs.

WHY (not netconvert): SUMO's netconvert can build the topology, but its
OpenDRIVE *export* of every corner connector is fixed-tight (~R6.6, exported as
a short line/polyline) no matter the junction radius, a custom connection shape,
or junction-joining -- verified exhaustively. So a smooth drivable loop can't
come out of the export. (And making the corners arc *edges* instead gives
zero-angle tangent junctions that export as degenerate reversed stubs -- the gap
RoadRunner showed.)

So we keep the junction structure the user wants -- 4 corner junctions, like the
original square -- but write the connector geometry ourselves as proper R arcs.
The road/junction/laneLink structure mirrors exactly what netconvert emits for
the square loop (4 straight roads junction=-1, 4 connecting-roads each in a
junction with one incoming->connecting connection and laneLink -1->-1), so
osc2cm and VISSIM's ImportOpenDrive consume it the same way -- only the corner
geometry is smooth instead of tight.

Layout (CCW), reference line = lane centerline of the rounded rectangle:
  straights  1=bottom 2=right 3=top 4=left   (each L-2R long, ending at tangents)
  connectors 5=br 6=tr 7=tl 8=bl             (quarter R arcs, one per junction)
  junctions  1=br 2=tr 3=tl 4=bl

Run:  python gen_junction_loop_xodr.py [R] [LANE_W]   (defaults: R 15 m, lane 3.2 m)
"""
from __future__ import annotations
import math
import pathlib
import sys

HERE = pathlib.Path(__file__).resolve().parent
L = 200.0
LANE_W = 3.2

NAMES = {1: "bottom", 2: "right", 3: "top", 4: "left",
         5: "conn_br", 6: "conn_tr", 7: "conn_tl", 8: "conn_bl"}
# straight pred/succ are junctions; connector pred/succ are roads (with contact)
LINKS = {
    1: (("junction", 4), ("junction", 1)),
    2: (("junction", 1), ("junction", 2)),
    3: (("junction", 2), ("junction", 3)),
    4: (("junction", 3), ("junction", 4)),
    5: (("road", 1, "end"), ("road", 2, "start")),
    6: (("road", 2, "end"), ("road", 3, "start")),
    7: (("road", 3, "end"), ("road", 4, "start")),
    8: (("road", 4, "end"), ("road", 1, "start")),
}
JUNC_OF = {5: 1, 6: 2, 7: 3, 8: 4}            # connector road -> its junction
JUNC_CONN = {1: (1, 5), 2: (2, 6), 3: (3, 7), 4: (4, 8)}  # junction -> (incoming, connecting)


def main() -> int:
    R = float(sys.argv[1]) if len(sys.argv) > 1 else 15.0
    lane_w = float(sys.argv[2]) if len(sys.argv) > 2 else LANE_W
    straight = L - 2.0 * R
    arc_len = (math.pi / 2.0) * R
    curv = 1.0 / R

    # walk the loop in driving order, integrating each segment's start pose
    walk = [(1, "line", straight, 0.0), (5, "arc", arc_len, curv),
            (2, "line", straight, 0.0), (6, "arc", arc_len, curv),
            (3, "line", straight, 0.0), (7, "arc", arc_len, curv),
            (4, "line", straight, 0.0), (8, "arc", arc_len, curv)]
    x, y, hdg = R, 0.0, 0.0
    geom = {}
    for rid, typ, length, k in walk:
        geom[rid] = (x, y, math.atan2(math.sin(hdg), math.cos(hdg)), length, typ, k)
        if typ == "line":
            x += length * math.cos(hdg)
            y += length * math.sin(hdg)
        else:
            eh = hdg + length * k
            x += (math.sin(eh) - math.sin(hdg)) / k
            y += (math.cos(hdg) - math.cos(eh)) / k
            hdg = eh

    def el(side, spec):
        if spec[0] == "junction":
            return f'<{side} elementType="junction" elementId="{spec[1]}"/>'
        return f'<{side} elementType="road" elementId="{spec[1]}" contactPoint="{spec[2]}"/>'

    roads = ""
    for rid in range(1, 9):
        x0, y0, h0, length, typ, k = geom[rid]
        junc = JUNC_OF.get(rid, -1)
        pre, suc = LINKS[rid]
        g = "<line/>" if typ == "line" else f'<arc curvature="{k:.12f}"/>'
        # connecting roads carry the lane link; roads touching a junction don't
        lane_link = ('<link><predecessor id="-1"/><successor id="-1"/></link>'
                     if rid >= 5 else "<link/>")
        roads += f'''  <road name="{NAMES[rid]}" length="{length:.6f}" id="{rid}" junction="{junc}">
    <link>{el("predecessor", pre)}{el("successor", suc)}</link>
    <type s="0" type="town"/>
    <planView>
      <geometry s="0" x="{x0:.6f}" y="{y0:.6f}" hdg="{h0:.12f}" length="{length:.6f}">{g}</geometry>
    </planView>
    <elevationProfile><elevation s="0" a="0" b="0" c="0" d="0"/></elevationProfile>
    <lanes>
      <laneSection s="0">
        <center><lane id="0" type="none" level="false"/></center>
        <right><lane id="-1" type="driving" level="false">{lane_link}<width sOffset="0" a="{lane_w}" b="0" c="0" d="0"/><roadMark sOffset="0" type="solid" weight="standard" color="standard" width="0.12"/></lane></right>
      </laneSection>
    </lanes>
  </road>
'''

    juncs = ""
    for jid, (inc, conn) in JUNC_CONN.items():
        juncs += f'''  <junction id="{jid}" name="j{jid}">
    <connection id="0" incomingRoad="{inc}" connectingRoad="{conn}" contactPoint="start">
      <laneLink from="-1" to="-1"/>
    </connection>
  </junction>
'''

    xodr = (f'<?xml version="1.0" standalone="yes"?>\n<OpenDRIVE>\n'
            f'  <header revMajor="1" revMinor="4" name="simple_loop" version="1.00" '
            f'north="0" south="0" east="0" west="0"/>\n{roads}{juncs}</OpenDRIVE>\n')
    out = HERE / "simple_loop.xodr"
    out.write_text(xodr, encoding="utf-8")
    print(f"[gen] junction loop R={R} m, lane width {lane_w} m: 4 straights "
          f"({straight:.0f} m) + 4 R{R:.0f} arc connectors + 4 junctions -> {out.name}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
