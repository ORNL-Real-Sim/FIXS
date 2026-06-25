"""
FIXS #172: generate SUMO node/edge sources for the SimpleTrafficLight corridor
with a COMPACT, CURVED turnaround loop at each end (per the user's spec):

  straight corridor  ->  CURVED ramp (peels off, turns up)  ->  LOOP (car goes
  ~300 deg around the circle)  ->  CURVED ramp (turns back down)  ->  straight
  corridor, reversed.

No long straight tangent ramps. The shape is defined by a CURVATURE profile and
integrated, so it is smooth (tangent-continuous) everywhere and provably closes
back onto the corridor centreline (y=0) heading the opposite way:
  entry ramp turns +T1 (one way),  loop turns -(2*T1+180) (the other way, the big
  loop),  exit ramp turns +T1.  Net heading change = -180 deg (a U-turn).
The ramp angle T1 is solved numerically so the path lands back on y=0.

The main corridor (between the 3 signals) stays dead straight; the loop is past
the last signal. Writes nodes.nod.xml + edges.edg.xml; run netconvert afterwards.
"""
from __future__ import annotations
import math, pathlib

HERE = pathlib.Path(__file__).resolve().parent
STRAIGHT_PAST = 150.0   # straight corridor past the last intersection before the loop (m)
RAMP_R = 45.0           # curved-ramp radius (m)
LOOP_R = 45.0           # loop radius (m)
DS = 2.0                # polyline sample step (m)
LOOP_SPEED = 8.33       # m/s (~30 km/h) on ramps + loop
RAMP_TURN = -1          # entry/exit ramp turn sign (-1 = eastbound peels SOUTH -> loop runs CCW)
LOOP_LANES = 1          # single-lane loop+ramps -> clean merge (no multi-lane tangle at the neck)
NECK_RADIUS = 30.0      # big junction radius at the loop neck -> long/gentle internal connectors
                        # (avoids the sharp lane-change area CM/IPGDriver choke on)
T_RADIUS = 12.0         # modest radius at the ramp<->loop tangent nodes

BASE_NODES = [
    ("int_west", -400, 0, "traffic_light"), ("int_center", 0, 0, "traffic_light"),
    ("int_east", 400, 0, "traffic_light"),
    ("west_north", -400, 300, "priority"), ("west_south", -400, -300, "priority"),
    ("center_north", 0, 300, "priority"), ("center_south", 0, -300, "priority"),
    ("east_north", 400, 300, "priority"), ("east_south", 400, -300, "priority"),
]
BASE_EDGES = [
    ("corridor_wc_e", "int_west", "int_center", 3), ("corridor_wc_w", "int_center", "int_west", 2),
    ("corridor_ce_e", "int_center", "int_east", 3), ("corridor_ce_w", "int_east", "int_center", 2),
    ("west_N_in", "west_north", "int_west", 1), ("west_N_out", "int_west", "west_north", 1),
    ("west_S_in", "west_south", "int_west", 1), ("west_S_out", "int_west", "west_south", 1),
    ("center_N_in", "center_north", "int_center", 1), ("center_N_out", "int_center", "center_north", 1),
    ("center_S_in", "center_south", "int_center", 1), ("center_S_out", "int_center", "center_south", 1),
    ("east_N_in", "east_north", "int_east", 1), ("east_N_out", "int_east", "east_north", 1),
    ("east_S_in", "east_south", "int_east", 1), ("east_S_out", "int_east", "east_south", 1),
]


def arc(x, y, h, radius, turn_sign, total_ang, ds=DS):
    """Integrate a constant-curvature arc. turn_sign +1 = left/CCW, -1 = right/CW.
    Returns (points, x, y, h_end). Heading h in radians."""
    n = max(2, int(round(radius * total_ang / ds)))
    dphi = turn_sign * total_ang / n
    pts = []
    for _ in range(n):
        h += dphi / 2.0
        x += ds_step(radius, dphi) * math.cos(h)
        y += ds_step(radius, dphi) * math.sin(h)
        h += dphi / 2.0
        pts.append((x, y))
    return pts, x, y, h


def ds_step(radius, dphi):
    return radius * abs(dphi)


def turnaround(x0, y0, h0, t1_deg):
    """3 phases from (x0,y0,h0): entry ramp +t1, loop -(2*t1+180), exit ramp +t1.
    Returns (ramp_in_pts, loop_pts, ramp_out_pts, end_x, end_y, end_h)."""
    t1 = math.radians(t1_deg)
    t2 = 2 * t1 + math.pi                      # loop sweep so net = 180 deg (a U-turn)
    p1, x, y, h = arc(x0, y0, h0, RAMP_R, RAMP_TURN, t1)        # ramp peels off
    p2, x, y, h = arc(x, y, h, LOOP_R, -RAMP_TURN, t2)         # loop (opposite turn = the big loop)
    p3, x, y, h = arc(x, y, h, RAMP_R, RAMP_TURN, t1)          # ramp back to corridor
    return p1, p2, p3, x, y, h


def solve_t1(x0, y0, h0):
    """Find ramp angle t1 (deg) so the turnaround lands back on the corridor (end y == y0)."""
    def endy(t1d):
        return turnaround(x0, y0, h0, t1d)[4] - y0
    lo, hi = 20.0, 89.0
    flo, fhi = endy(lo), endy(hi)
    for _ in range(60):
        mid = (lo + hi) / 2
        fm = endy(mid)
        if (flo < 0) == (fm < 0):
            lo, flo = mid, fm
        else:
            hi, fhi = mid, fm
    return (lo + hi) / 2


def shp(pts):
    return " ".join(f"{x:.2f},{y:.2f}" for x, y in pts)


def loop_at(prefix, sign):
    """Build a curved turnaround past the intersection at x = sign*400.
    sign +1 = east end (car arrives heading +x), -1 = west end (heading -x)."""
    x0 = sign * (400 + STRAIGHT_PAST)
    h0 = 0.0 if sign > 0 else math.pi          # heading into the loop
    t1 = solve_t1(x0, 0.0, h0)
    p1, p2, p3, ex, ey, eh = turnaround(x0, 0.0, h0, t1)
    # entry A and exit B coincide (solver closes the loop) -> one shared "neck" junction
    A = (x0, 0.0); T1 = p1[-1]; T2 = p2[-1]
    neck = f"{prefix}_neck"
    nodes = [
        (neck, round(A[0], 2), 0.0, "priority"),
        (f"{prefix}_t1", round(T1[0], 2), round(T1[1], 2), "priority"),
        (f"{prefix}_t2", round(T2[0], 2), round(T2[1], 2), "priority"),
    ]
    edges = [
        (f"{prefix}_ramp_in", neck, f"{prefix}_t1", LOOP_LANES, shp([A] + p1)),
        (f"{prefix}_loop", f"{prefix}_t1", f"{prefix}_t2", LOOP_LANES, shp(p2)),
        (f"{prefix}_ramp_out", f"{prefix}_t2", neck, LOOP_LANES, shp(p3 + [A])),
    ]
    return nodes, edges, neck, neck, t1, math.degrees(eh)


def main():
    e_nodes, e_edges, e_a, e_b, e_t1, e_eh = loop_at("rbe", +1)
    w_nodes, w_edges, w_a, w_b, w_t1, w_eh = loop_at("rbw", -1)
    nodes = BASE_NODES + e_nodes + w_nodes

    # corridor connects straight (y=0) to the loop entry (A) and from the loop exit (B)
    end_edges = [
        ("corridor_e_out", "int_east", e_a, 2, None), ("corridor_e_in", e_b, "int_east", 3, None),
        ("corridor_w_out", "int_west", w_a, 2, None),  ("corridor_w_in", w_b, "int_west", 3, None),
    ]

    nl = ['<?xml version="1.0" encoding="UTF-8"?>', "<nodes>"]
    for n, x, y, t in nodes:
        # enlarge the loop junctions so their internal connectors are long + gentle
        r = NECK_RADIUS if n.endswith("_neck") else (T_RADIUS if (n.endswith("_t1") or n.endswith("_t2")) else None)
        extra = f' radius="{r}" keepClear="false"' if r else ""
        nl.append(f'    <node id="{n}" x="{x}" y="{y}" type="{t}"{extra}/>')
    nl.append("</nodes>")
    (HERE / "nodes.nod.xml").write_text("\n".join(nl) + "\n", encoding="utf-8")

    el = ['<?xml version="1.0" encoding="UTF-8"?>', "<edges>"]
    for eid, fr, to, nlanes in [(e[0], e[1], e[2], e[3]) for e in BASE_EDGES]:
        el.append(f'    <edge id="{eid}" from="{fr}" to="{to}" numLanes="{nlanes}" speed="13.89" priority="3"/>')
    for eid, fr, to, nlanes, sh in end_edges + e_edges + w_edges:
        spd = "13.89" if (sh is None and eid.startswith("corridor")) else f"{LOOP_SPEED}"
        shape = f' shape="{sh}"' if sh else ""
        el.append(f'    <edge id="{eid}" from="{fr}" to="{to}" numLanes="{nlanes}" speed="{spd}" priority="3"{shape}/>')
    el.append("</edges>")
    (HERE / "edges.edg.xml").write_text("\n".join(el) + "\n", encoding="utf-8")

    print(f"[gen_loop_net] curved turnaround: ramp_R={RAMP_R} loop_R={LOOP_R} straight_past={STRAIGHT_PAST}")
    print(f"  east: ramp_angle={e_t1:.1f} deg, A={e_a}, B={e_b}, end_heading={e_eh:.1f} deg")
    print(f"  west: ramp_angle={w_t1:.1f} deg, A={w_a}, B={w_b}, end_heading={w_eh:.1f} deg")


if __name__ == "__main__":
    main()
