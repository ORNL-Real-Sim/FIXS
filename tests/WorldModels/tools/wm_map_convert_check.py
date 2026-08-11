"""Verify SUMO net -> OpenDRIVE -> trajdata -> AlpaSim-native map conversion.

Pipeline under test (issue #189, the fix for the lane_centers-only map that
collapses CATK queue launch):

    SUMO .net.xml --netconvert--> .xodr --trajdata parse--> VectorMap
        --> AlpaSim 8-layer env_map (same extraction as CATKSceneAdapter)

Checks:
  1. which layers are populated + polyline/segment counts
  2. geometric alignment against the ORIGINAL SUMO net lane shapes
     (max nearest-point distance -- must be sub-metre, no coordinate offset)
  3. junction coverage at int_east (through/turn lanes present?)
  4. writes an overlay HTML (SUMO net gray, converted layers colored)

Run (CATK venv):
    python wm_map_convert_check.py --net ../Demo/simple_traffic_light.net.xml
"""
from __future__ import annotations

import argparse
import pathlib
import re
import subprocess
import sys

import numpy as np

NETCONVERT = pathlib.Path("C:/Program Files (x86)/Eclipse/Sumo/bin/netconvert.exe")

# same layer extraction as wm_native_reference._build_map / AlpaSim map_adapter
LAYERS = ["lanelines", "road_boundaries", "waitlines", "crosswalks",
          "lane_boundaries", "lane_centers", "intersection_areas", "road_islands"]


def sumo_lanes(net_file: pathlib.Path) -> list[np.ndarray]:
    txt = net_file.read_text(encoding="utf-8")
    lanes = []
    for m in re.finditer(r'<lane id="([^"]*)"[^>]*shape="([^"]+)"', txt):
        pts = np.array([tuple(map(float, p.split(","))) for p in m.group(2).split()])
        if len(pts) >= 2:
            lanes.append(pts)
    return lanes


def convert(net_file: pathlib.Path, xodr_file: pathlib.Path) -> None:
    res = subprocess.run(
        [str(NETCONVERT), "-s", str(net_file), "--opendrive-output", str(xodr_file)],
        capture_output=True, text=True)
    if res.returncode != 0 or not xodr_file.exists():
        raise RuntimeError(f"netconvert failed:\n{res.stderr[-2000:]}")


def build_layers(xodr_file: pathlib.Path) -> dict[str, list[np.ndarray]]:
    from trajdata.dataset_specific.xodr.vector_map_export import populate_vector_map_from_xodr
    from trajdata.maps import VectorMap
    from trajdata.maps.vec_map_elements import MapElementType

    vmap = VectorMap(map_id="sumo_xodr:demo")
    populate_vector_map_from_xodr(vmap, xodr_file.read_text(encoding="utf-8"),
                                  resolution=0.5, t_xodr_enu_to_sim=None)
    out: dict[str, list[np.ndarray]] = {k: [] for k in LAYERS}
    for lane in vmap.elements.get(MapElementType.ROAD_LANE, {}).values():
        out["lane_centers"].append(np.asarray(lane.center.xyz))
        for edge in (lane.left_edge, lane.right_edge):
            if edge is not None:
                out["lane_boundaries"].append(np.asarray(edge.xyz))
                out["lanelines"].append(np.asarray(edge.xyz))
    for e in vmap.elements.get(MapElementType.ROAD_EDGE, {}).values():
        out["road_boundaries"].append(np.asarray(e.polyline.xyz))
    for w in vmap.elements.get(MapElementType.WAIT_LINE, {}).values():
        out["waitlines"].append(np.asarray(w.polyline.xyz))
    for c in vmap.elements.get(MapElementType.PED_CROSSWALK, {}).values():
        out["crosswalks"].append(np.asarray(c.polygon.xyz))
    for a in vmap.elements.get(MapElementType.ROAD_AREA, {}).values():
        out["intersection_areas"].append(np.asarray(a.exterior_polygon.xyz))
        out["road_islands"].extend(np.asarray(h.xyz) for h in a.interior_holes)
    return out


def alignment(sumo: list[np.ndarray], centers: list[np.ndarray]) -> tuple[float, float]:
    """For sampled converted lane-center points, distance to nearest SUMO
    lane segment (and vice versa is not needed -- offset shows up either way)."""
    segs = []
    for p in sumo:
        for i in range(len(p) - 1):
            segs.append((p[i], p[i + 1]))
    S0 = np.array([s[0] for s in segs]); S1 = np.array([s[1] for s in segs])
    d = S1 - S0
    dd = (d * d).sum(1) + 1e-9

    def dist(pt):
        t = np.clip(((pt - S0) * d).sum(1) / dd, 0, 1)
        return np.sqrt((((S0 + t[:, None] * d) - pt) ** 2).sum(1).min())

    pts = np.vstack([c[::10, :2] for c in centers if len(c) >= 2])
    dists = np.array([dist(p) for p in pts])
    return float(np.median(dists)), float(dists.max())


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--net", type=pathlib.Path,
                    default=pathlib.Path(__file__).parent / "../Demo/simple_traffic_light.net.xml")
    ap.add_argument("--out", type=pathlib.Path, default=pathlib.Path("map_convert_check.html"))
    args = ap.parse_args()
    net = args.net.resolve()
    xodr = net.with_suffix(".xodr")

    convert(net, xodr)
    print(f"netconvert OK -> {xodr.name} ({xodr.stat().st_size//1024} KB)")

    layers = build_layers(xodr)
    print("\nlayer                 polylines   points")
    for k in LAYERS:
        npts = sum(len(p) for p in layers[k])
        print(f"  {k:20s} {len(layers[k]):8d} {npts:8d}")

    sumo = sumo_lanes(net)
    med, mx = alignment(sumo, layers["lane_centers"])
    print(f"\nalignment lane_centers vs SUMO net: median={med:.2f} m  max={mx:.2f} m")
    print("  (median must be ~half-lane or less and max ~2 m -- a constant offset "
        "would show as median >> 1 m)")

    # junction coverage at int_east (x 1065-1080, y 285-320)
    def count_in(box, polys):
        x0, x1, y0, y1 = box
        n = 0
        for p in polys:
            q = np.asarray(p)
            if ((q[:, 0] > x0) & (q[:, 0] < x1) & (q[:, 1] > y0) & (q[:, 1] < y1)).any():
                n += 1
        return n
    box = (1065, 1081, 285, 320)
    print(f"\nint_east junction coverage (x{box[0]}-{box[1]}):")
    for k in ("lane_centers", "lane_boundaries", "road_boundaries", "intersection_areas"):
        print(f"  {k:20s} {count_in(box, layers[k])} polylines")

    # overlay plot
    import plotly.graph_objects as go
    fig = go.Figure()
    for p in sumo:
        fig.add_trace(go.Scattergl(x=p[:, 0], y=p[:, 1], mode="lines",
                                   line=dict(color="#d1d5db", width=5),
                                   showlegend=False, hoverinfo="skip"))
    style = {"lane_centers": ("#2563eb", 1.5), "lane_boundaries": ("#16a34a", 1),
             "road_boundaries": ("#dc2626", 2), "waitlines": ("#f59e0b", 4),
             "crosswalks": ("#9333ea", 2), "intersection_areas": ("#0891b2", 2),
             "lanelines": ("#a3a3a3", 0.5), "road_islands": ("#78350f", 2)}
    for k in LAYERS:
        col, w = style[k]
        first = True
        for p in layers[k]:
            q = np.asarray(p)
            fig.add_trace(go.Scattergl(x=q[:, 0], y=q[:, 1], mode="lines",
                                       line=dict(color=col, width=w),
                                       name=k, legendgroup=k,
                                       showlegend=first))
            first = False
    fig.update_layout(title="SUMO net (gray) vs converted AlpaSim layers -- "
                            "netconvert xodr -> trajdata -> 8-layer map",
                      width=1300, height=700,
                      yaxis=dict(scaleanchor="x"))
    fig.write_html(args.out, include_plotlyjs=True)
    print(f"\nwrote {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
