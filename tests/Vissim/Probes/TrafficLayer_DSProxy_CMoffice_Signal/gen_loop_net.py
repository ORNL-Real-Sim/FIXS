"""
FIXS #172: generate SUMO node/edge sources for the SimpleTrafficLight corridor
with a big CIRCULAR loop (roundabout-style balloon) at each end.

The main E-W corridor (3 signalized intersections + cross-streets) stays straight
and divided. Each end's corridor connects to a one-way circular loop (R=LOOP_R);
a vehicle enters from the in-bound corridor, circulates ~360 deg around the
circle, and exits onto the out-bound corridor reversed -- a clean round U-turn
loop like the reference figure, with no bending of the main corridor.

Writes nodes.nod.xml + edges.edg.xml; run netconvert afterwards.
"""
from __future__ import annotations
import math, pathlib

HERE = pathlib.Path(__file__).resolve().parent
LOOP_R = 45.0          # loop radius (m) -> ~90 m circle
ARC_PTS = 7            # points per quarter arc (smoothness)

# corridor + cross-street nodes (original SimpleTrafficLight coords, y=0 centerline)
BASE_NODES = [
    ("int_west", -400, 0, "traffic_light"), ("int_center", 0, 0, "traffic_light"),
    ("int_east", 400, 0, "traffic_light"),
    ("west_north", -400, 300, "priority"), ("west_south", -400, -300, "priority"),
    ("center_north", 0, 300, "priority"), ("center_south", 0, -300, "priority"),
    ("east_north", 400, 300, "priority"), ("east_south", 400, -300, "priority"),
]
# corridor + cross edges (the straight, divided corridor -- unchanged)
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


def arc_shape(cx, cy, r, a0_deg, a1_deg):
    """polyline points (SUMO 'x,y x,y ...') along the arc from a0 to a1 (CCW if a1>a0)."""
    pts = []
    for k in range(ARC_PTS + 1):
        a = math.radians(a0_deg + (a1_deg - a0_deg) * k / ARC_PTS)
        pts.append(f"{cx + r*math.cos(a):.2f},{cy + r*math.sin(a):.2f}")
    return " ".join(pts)


def loop(prefix, cx, cy, conn_angle_deg):
    """4 circle nodes + 4 CCW arc edges around (cx,cy). conn_angle = where the
    corridor connects (the W point for east loop=180, the E point for west=0)."""
    # node at each cardinal angle
    cards = {0: "E", 90: "N", 180: "W", 270: "S"}
    nodes = {}
    for ang, tag in cards.items():
        nodes[ang] = (f"{prefix}_{tag}", cx + LOOP_R*math.cos(math.radians(ang)),
                      cy + LOOP_R*math.sin(math.radians(ang)))
    node_lines = [(n, round(x, 2), round(y, 2), "priority") for (n, x, y) in nodes.values()]
    # CCW arc edges: 0->90->180->270->360
    seq = [0, 90, 180, 270, 0]
    edge_lines = []
    for i in range(4):
        a0, a1 = seq[i], seq[i+1] if seq[i+1] != 0 else 360
        fr, to = nodes[a0 % 360][0], nodes[a1 % 360][0]
        sh = arc_shape(cx, cy, LOOP_R, a0, a1)
        edge_lines.append((f"{prefix}_arc{i}", fr, to, 1, sh))
    return node_lines, edge_lines, nodes[conn_angle_deg][0]


def main():
    e_nodes, e_edges, e_conn = loop("rbe", 400 + 600, 0, 180)   # east loop center (1000,0), connect at W
    w_nodes, w_edges, w_conn = loop("rbw", -400 - 600, 0, 0)     # west loop center (-1000,0), connect at E

    nodes = BASE_NODES + e_nodes + w_nodes
    # corridor ends connect to the loop connection nodes
    end_edges = [
        ("corridor_e_out", "int_east", e_conn, 2), ("corridor_e_in", e_conn, "int_east", 3),
        ("corridor_w_in", w_conn, "int_west", 3),  ("corridor_w_out", "int_west", w_conn, 2),
    ]

    nl = ['<?xml version="1.0" encoding="UTF-8"?>', "<nodes>"]
    for n, x, y, t in nodes:
        nl.append(f'    <node id="{n}" x="{x}" y="{y}" type="{t}"/>')
    nl.append("</nodes>")
    (HERE / "nodes.nod.xml").write_text("\n".join(nl) + "\n", encoding="utf-8")

    el = ['<?xml version="1.0" encoding="UTF-8"?>', "<edges>"]
    for e in BASE_EDGES + end_edges:
        eid, fr, to, nl_ = e
        el.append(f'    <edge id="{eid}" from="{fr}" to="{to}" numLanes="{nl_}" speed="13.89" priority="3"/>')
    for e in e_edges + w_edges:
        eid, fr, to, nl_, sh = e
        el.append(f'    <edge id="{eid}" from="{fr}" to="{to}" numLanes="{nl_}" speed="8.33" priority="3" shape="{sh}"/>')
    el.append("</edges>")
    (HERE / "edges.edg.xml").write_text("\n".join(el) + "\n", encoding="utf-8")

    print(f"[gen_loop_net] wrote nodes.nod.xml + edges.edg.xml; loop R={LOOP_R} m")
    print(f"  east loop conn node={e_conn}, west loop conn node={w_conn}")


if __name__ == "__main__":
    main()
