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

The turnaround geometry itself now lives in Carla/utils/sumo_uturn.py (#327),
which also puts the same loop on a network that ALREADY exists (MLK and other
real corridors, where there are no plain-XML sources to regenerate from). This
script keeps only what is specific to the synthetic SimpleTrafficLight corridor;
importing the shared core is what stops the two copies of the curvature
integration from drifting apart.
"""
from __future__ import annotations
import pathlib, sys

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[3] / "Carla" / "utils"))
from sumo_uturn import curved_uturn  # noqa: E402
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


def shp(pts):
    return " ".join(f"{x:.2f},{y:.2f}" for x, y in pts)


def loop_at(prefix, sign):
    """Build a curved turnaround past the intersection at x = sign*400.
    sign +1 = east end (car arrives heading +x), -1 = west end (heading -x)."""
    x0 = sign * (400 + STRAIGHT_PAST)
    # curved_uturn works in a local frame whose +x points away from the network and
    # whose origin is where the corridor meets the loop. Here the corridor edge already
    # runs all the way to the neck, so there is no stub (straight_past=0) and no lane
    # offset to blend out; mirror the local frame for the west end.
    g = curved_uturn(RAMP_R, LOOP_R, straight_past=0.0,
                     side=("right" if RAMP_TURN < 0 else "left"), ds=DS)
    to_world = lambda pts: [(x0 + sign * x, sign * y) for x, y in pts]
    p1, p2, p3 = (to_world(g["in_pts"][1:]), to_world(g["loop_pts"][1:]),
                  to_world(g["out_pts"][1:]))
    t1 = g["ramp_angle_deg"]
    # local end heading is 180 deg; rotate it back into world for the west end
    eh_deg = (g["end_heading_deg"] + (0.0 if sign > 0 else 180.0) + 180.0) % 360.0 - 180.0
    # entry A and exit B coincide (the solver closes the loop) -> one shared "neck" junction
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
    return nodes, edges, neck, neck, t1, eh_deg


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
    SUMO_NET = HERE.parents[3] / "tests" / "Sumo" / "networks" / "simple_traffic_light"
    SUMO_NET.mkdir(parents=True, exist_ok=True)
    (SUMO_NET / "nodes.nod.xml").write_text("\n".join(nl) + "\n", encoding="utf-8")

    el = ['<?xml version="1.0" encoding="UTF-8"?>', "<edges>"]
    for eid, fr, to, nlanes in [(e[0], e[1], e[2], e[3]) for e in BASE_EDGES]:
        el.append(f'    <edge id="{eid}" from="{fr}" to="{to}" numLanes="{nlanes}" speed="13.89" priority="3"/>')
    for eid, fr, to, nlanes, sh in end_edges + e_edges + w_edges:
        spd = "13.89" if (sh is None and eid.startswith("corridor")) else f"{LOOP_SPEED}"
        shape = f' shape="{sh}"' if sh else ""
        el.append(f'    <edge id="{eid}" from="{fr}" to="{to}" numLanes="{nlanes}" speed="{spd}" priority="3"{shape}/>')
    el.append("</edges>")
    (SUMO_NET / "edges.edg.xml").write_text("\n".join(el) + "\n", encoding="utf-8")

    print(f"[gen_loop_net] curved turnaround: ramp_R={RAMP_R} loop_R={LOOP_R} straight_past={STRAIGHT_PAST}")
    print(f"  east: ramp_angle={e_t1:.1f} deg, A={e_a}, B={e_b}, end_heading={e_eh:.1f} deg")
    print(f"  west: ramp_angle={w_t1:.1f} deg, A={w_a}, B={w_b}, end_heading={w_eh:.1f} deg")


if __name__ == "__main__":
    main()
