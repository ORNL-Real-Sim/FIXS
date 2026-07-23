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

Run:  python gen_junction_loop_xodr.py [R] [LANE_W] [GRADE_AMP] [N_HILLS]
      R 15 m, lane 3.2 m, grade 0 by default.  GRADE_AMP > 0 adds a closed rolling
      elevation of N_HILLS (default 3) up-and-down hills of that amplitude (m) and
      writes simple_loop_elevation.xodr (the flat simple_loop.xodr is unchanged).
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
# CARLA decides "is a road's successor a junction?" purely by !ContainsRoad(id)
# (LibCarla road/MapBuilder.cpp GetLaneNext). If a junction id collides with a
# road id, the road's junction-successor is misread as a same-numbered road (a
# self-reference) -> the lane dead-ends at the corner, so get_topology drops every
# straight->corner edge and TM/agents drive straight off the loop. Junction ids
# MUST therefore stay clear of every road id (1..8). Offset them by JID_BASE.
JID_BASE = 100
JUNC_OF = {5: 1, 6: 2, 7: 3, 8: 4}            # connector road -> its junction (logical 1..4)
JUNC_CONN = {1: (1, 5), 2: (2, 6), 3: (3, 7), 4: (4, 8)}  # junction -> (incoming, connecting)


def main() -> int:
    R = float(sys.argv[1]) if len(sys.argv) > 1 else 15.0
    lane_w = float(sys.argv[2]) if len(sys.argv) > 2 else LANE_W
    grade_amp = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0   # elevation amplitude [m]; 0 = flat
    n_hills   = int(sys.argv[4]) if len(sys.argv) > 4 else 3       # up-and-down hills per lap
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

    # CLOSED rolling elevation (grade_amp > 0): n_hills up-and-down hills per lap.
    #   z(S) = A * sin(2*pi*n*S/L) ,  S = arc length around the loop, L = perimeter.
    # Integer n keeps it periodic -- z AND slope match at S=0 and S=L, so the loop
    # closes seamlessly (no step, no kink). The profile is sampled into many short
    # Hermite-cubic <elevation> segments per road (>= ~12 per wavelength) so a crest
    # sitting mid-straight is captured, not smoothed away by one cubic per 170 m road.
    # grade_amp = 0 -> a single flat record (keeps the flat xodr byte-identical).
    perim = 0.0
    S_start = {}
    for rid_, typ_, len_, k_ in walk:
        S_start[rid_] = perim
        perim += len_
    w = 2.0 * math.pi * n_hills / perim
    def hill(S):  return grade_amp * math.sin(w * S)
    def dhill(S): return grade_amp * w * math.cos(w * S)
    def hermite(z0, z1, m0, m1, length):
        a = z0
        b = m0
        c = 3.0 * (z1 - z0) / length**2 - (2.0 * m0 + m1) / length
        d = 2.0 * (z0 - z1) / length**3 + (m0 + m1) / length**2
        return a + 0.0, b + 0.0, c + 0.0, d + 0.0
    def elevation_profile(rid, length):
        if grade_amp == 0.0:                    # flat -> one zero record (byte-identical)
            return '<elevation s="0" a="0" b="0" c="0" d="0"/>'
        S0 = S_start[rid]                        # road s == arc length, so global S = S0 + s
        seg_len = perim / (n_hills * 12.0)       # >= 12 samples per wavelength
        nseg = max(1, int(math.ceil(length / seg_len)))
        out = []
        for i in range(nseg):
            sa, sb = length * i / nseg, length * (i + 1) / nseg
            a, b, c, d = hermite(hill(S0 + sa), hill(S0 + sb), dhill(S0 + sa), dhill(S0 + sb), sb - sa)
            out.append(f'<elevation s="{sa:.6g}" a="{a:.10g}" b="{b:.10g}" c="{c:.10g}" d="{d:.10g}"/>')
        return "".join(out)

    def el(side, spec):
        if spec[0] == "junction":
            return f'<{side} elementType="junction" elementId="{JID_BASE + spec[1]}"/>'
        return f'<{side} elementType="road" elementId="{spec[1]}" contactPoint="{spec[2]}"/>'

    roads = ""
    for rid in range(1, 9):
        x0, y0, h0, length, typ, k = geom[rid]
        junc = JID_BASE + JUNC_OF[rid] if rid in JUNC_OF else -1
        pre, suc = LINKS[rid]
        g = "<line/>" if typ == "line" else f'<arc curvature="{k:.12f}"/>'
        elevprof = elevation_profile(rid, length)
        # connecting roads carry the lane link; roads touching a junction don't
        lane_link = ('<link><predecessor id="-1"/><successor id="-1"/></link>'
                     if rid >= 5 else "<link/>")
        roads += f'''  <road name="{NAMES[rid]}" length="{length:.6f}" id="{rid}" junction="{junc}">
    <link>{el("predecessor", pre)}{el("successor", suc)}</link>
    <type s="0" type="town"/>
    <planView>
      <geometry s="0" x="{x0:.6f}" y="{y0:.6f}" hdg="{h0:.12f}" length="{length:.6f}">{g}</geometry>
    </planView>
    <elevationProfile>{elevprof}</elevationProfile>
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
        juncs += f'''  <junction id="{JID_BASE + jid}" name="j{jid}">
    <connection id="0" incomingRoad="{inc}" connectingRoad="{conn}" contactPoint="start">
      <laneLink from="-1" to="-1"/>
    </connection>
  </junction>
'''

    xodr = (f'<?xml version="1.0" standalone="yes"?>\n<OpenDRIVE>\n'
            f'  <header revMajor="1" revMinor="4" name="simple_loop" version="1.00" '
            f'north="0" south="0" east="0" west="0"/>\n{roads}{juncs}</OpenDRIVE>\n')
    fname = "simple_loop_elevation.xodr" if grade_amp > 0 else "simple_loop.xodr"
    out = HERE / fname
    out.write_text(xodr, encoding="utf-8")
    max_grade = (grade_amp * w * 100.0) if grade_amp > 0 else 0.0
    print(f"[gen] junction loop R={R} m, lane {lane_w} m, {n_hills} hills x amp {grade_amp} m "
          f"(peak grade {max_grade:.1f}%): 4 straights ({straight:.0f} m) + 4 R{R:.0f} arcs "
          f"+ 4 junctions -> {out.name}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
