"""
Regenerate the SimpleEcho SUMO loop with ROUNDED corners.

The original loop is a 200x200 square: 4 straight edges meeting at 4 priority
junctions, i.e. 90-degree point corners. netconvert turns each into a ~3 m
connector in the exported .xodr, which is below a car's ~5-6 m minimum turning
radius -- so the CarMaker IPGDriver ego drives off the road at every corner.

Fix at the SOURCE: replace the sharp corners with quarter-circle ARC edges of
radius R, keeping the four straights (shortened to 200-2R). The loop centerline
becomes a rounded rectangle inscribed in the 200x200 box (corners cut inward by
R). One-way, single lane, CCW (bottom->right->top->left), same 13.89 m/s limit
and the same single red ego vehicle as before.

Outputs (overwrites): simple_loop.nod.xml, simple_loop.edg.xml,
simple_loop.net.xml (via netconvert), and updates simple_loop.rou.xml to route
through the new corner edges. Downstream (xodr via netconvert --opendrive-output,
then VISSIM import + CarMaker osc2cm) is regenerated from this net.

Run:  python gen_rounded_loop.py [R]      (R defaults to 15 m)
"""
from __future__ import annotations
import math
import pathlib
import subprocess
import sys

HERE = pathlib.Path(__file__).resolve().parent
NETCONVERT = r"C:\Program Files (x86)\Eclipse\Sumo\bin\netconvert.exe"

L = 200.0          # outer side length (matches the original square)
SPEED = 13.89      # m/s lane speed limit (unchanged)
N = 16             # polyline points per quarter-arc (smoothness)


def arc(cx: float, cy: float, r: float, a0: float, a1: float, n: int):
    """N+1 points along the arc from angle a0 to a1 (degrees), CCW."""
    return [(cx + r * math.cos(math.radians(a0 + (a1 - a0) * i / n)),
             cy + r * math.sin(math.radians(a0 + (a1 - a0) * i / n)))
            for i in range(n + 1)]


def shp(pts) -> str:
    return " ".join(f"{x:.3f},{y:.3f}" for x, y in pts)


def main() -> int:
    R = float(sys.argv[1]) if len(sys.argv) > 1 else 15.0

    # 8 nodes = the straight/arc tangent points around the rounded rectangle.
    nodes = [
        ("nb1", R, 0),     ("nb2", L - R, 0),       # bottom straight ends
        ("nr1", L, R),     ("nr2", L, L - R),       # right straight ends
        ("nt1", L - R, L), ("nt2", R, L),           # top straight ends
        ("nl1", 0, L - R), ("nl2", 0, R),           # left straight ends
    ]

    # 8 edges = 4 straights (no shape -> straight line) + 4 corner arcs (CCW).
    edges = [
        ("bottom",    "nb1", "nb2", None),
        ("corner_br", "nb2", "nr1", arc(L - R, R,     R, 270, 360, N)),
        ("right",     "nr1", "nr2", None),
        ("corner_tr", "nr2", "nt1", arc(L - R, L - R, R,   0,  90, N)),
        ("top",       "nt1", "nt2", None),
        ("corner_tl", "nt2", "nl1", arc(R,     L - R, R,  90, 180, N)),
        ("left",      "nl1", "nl2", None),
        ("corner_bl", "nl2", "nb1", arc(R,     R,     R, 180, 270, N)),
    ]

    nod = ("<nodes>\n" + "\n".join(
        f'    <node id="{i}" x="{x:.3f}" y="{y:.3f}" type="priority"/>'
        for i, x, y in nodes) + "\n</nodes>\n")
    (HERE / "simple_loop.nod.xml").write_text(nod, encoding="utf-8")

    edg = "<edges>\n"
    for eid, f, t, shape in edges:
        sh = f' shape="{shp(shape)}"' if shape else ""
        edg += (f'    <edge id="{eid}" from="{f}" to="{t}" numLanes="1" '
                f'speed="{SPEED}" spreadType="center"{sh}/>\n')
    edg += "</edges>\n"
    (HERE / "simple_loop.edg.xml").write_text(edg, encoding="utf-8")

    print(f"[gen] R={R} m, straights={L - 2 * R:.0f} m, "
          f"perimeter={4 * (L - 2 * R) + 4 * (math.pi * R / 2):.0f} m")

    subprocess.run([
        NETCONVERT,
        "--node-files", str(HERE / "simple_loop.nod.xml"),
        "--edge-files", str(HERE / "simple_loop.edg.xml"),
        "--output-file", str(HERE / "simple_loop.net.xml"),
        "--no-turnarounds",
        "--offset.disable-normalization", "true",
    ], check=True)
    print("[gen] netconvert -> simple_loop.net.xml")

    rou_path = HERE / "simple_loop.rou.xml"
    rou = rou_path.read_text(encoding="utf-8")
    new_route = "bottom corner_br right corner_tr top corner_tl left corner_bl"
    rou = rou.replace('edges="bottom right top left"', f'edges="{new_route}"')
    rou_path.write_text(rou, encoding="utf-8")
    print(f"[gen] route -> {new_route}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
