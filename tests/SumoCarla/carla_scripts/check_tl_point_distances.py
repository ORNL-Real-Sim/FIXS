#!/usr/bin/env python3
"""
check_tl_point_distances.py

Compute nearest-neighbor distances from SUMO traffic light table points
to CARLA traffic light actors.

Usage examples:
  python check_tl_point_distances.py --tls-table traffic_light_table.csv
  python check_tl_point_distances.py --tls-table traffic_light_table.csv --carla-host 127.0.0.1 --carla-port 2000
  python check_tl_point_distances.py --tls-table traffic_light_table.csv --use-3d
  python check_tl_point_distances.py --tls-table traffic_light_table.csv --out report.csv --top 30
"""

import argparse
import csv
import math
import statistics
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import carla


@dataclass(frozen=True)
class TLEntry:
    junction_id: str
    link_id: int
    x: float
    y: float
    z: float
    heading: float


def _get(row: Dict[str, str], *keys: str) -> Optional[str]:
    for k in keys:
        if k in row and row[k] != "":
            return row[k]
    return None


def load_tl_table(csv_path: str) -> List[TLEntry]:
    entries: List[TLEntry] = []
    with open(csv_path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            jid = _get(row, "junction_id", "junctionId", "junction")
            lid = _get(row, "link_id", "linkId", "link")
            x = _get(row, "x", "X")
            y = _get(row, "y", "Y")
            z = _get(row, "z", "Z")
            h = _get(row, "heading", "Heading", "yaw", "Yaw")

            if jid is None or lid is None or x is None or y is None:
                raise ValueError(
                    f"CSV row missing required fields. "
                    f"Need junction_id/junctionId, link_id/linkId, x, y. Row: {row}"
                )

            entries.append(
                TLEntry(
                    junction_id=jid.strip(),
                    link_id=int(lid),
                    x=float(x),
                    y=float(y),
                    z=float(z) if z is not None else 0.0,
                    heading=float(h) if h is not None else 0.0,
                )
            )
    return entries


def dist_sq_2d(ax: float, ay: float, bx: float, by: float) -> float:
    dx = ax - bx
    dy = ay - by
    return dx * dx + dy * dy


def dist_sq_3d(ax: float, ay: float, az: float, bx: float, by: float, bz: float) -> float:
    dx = ax - bx
    dy = ay - by
    dz = az - bz
    return dx * dx + dy * dy + dz * dz


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--tls-table", required=True, help="Path to traffic_light_table.csv")
    ap.add_argument("--carla-host", default="127.0.0.1")
    ap.add_argument("--carla-port", type=int, default=2000)
    ap.add_argument("--carla-timeout", type=float, default=10.0)
    ap.add_argument("--use-3d", action="store_true", help="Use 3D distance instead of 2D")
    ap.add_argument("--top", type=int, default=20, help="Show top-N worst distances")
    ap.add_argument("--out", help="Optional output CSV report")
    args = ap.parse_args()

    entries = load_tl_table(args.tls_table)
    if not entries:
        print("No entries found in TLS table.")
        return 2

    client = carla.Client(args.carla_host, args.carla_port)
    client.set_timeout(args.carla_timeout)
    world = client.get_world()

    print(client.get_available_maps())
    world = client.load_world("Town01")

    world = client.get_world()
    print("Map:", world.get_map().name)
    tls = world.get_actors().filter("traffic.traffic_light*")
    print("TrafficLight actors:", len(tls))

    actors = world.get_actors().filter("traffic.traffic_light*")
    if len(actors) == 0:
        print("ERROR: No CARLA traffic light actors found. Is the correct map loaded?")
        return 2

    carla_pts: List[Tuple[int, float, float, float]] = []
    for a in actors:
        tf = a.get_transform()
        loc = tf.location
        carla_pts.append((a.id, float(loc.x), float(loc.y), float(loc.z)))

    results = []
    dists = []

    # Brute-force nearest neighbor (fine for hundreds of points)
    for e in entries:
        best_id = None
        best_d2 = float("inf")
        best_pt = None

        for aid, cx, cy, cz in carla_pts:
            if args.use_3d:
                d2 = dist_sq_3d(e.x, e.y, e.z, cx, cy, cz)
            else:
                d2 = dist_sq_2d(e.x, e.y, cx, cy)

            if d2 < best_d2:
                best_d2 = d2
                best_id = aid
                best_pt = (cx, cy, cz)

        dist = math.sqrt(best_d2)
        dists.append(dist)
        results.append(
            {
                "junction_id": e.junction_id,
                "link_id": e.link_id,
                "sumo_x": e.x,
                "sumo_y": e.y,
                "sumo_z": e.z,
                "heading": e.heading,
                "nearest_carla_actor_id": best_id,
                "carla_x": best_pt[0] if best_pt else None,
                "carla_y": best_pt[1] if best_pt else None,
                "carla_z": best_pt[2] if best_pt else None,
                "distance_m": dist,
            }
        )

    dists_sorted = sorted(dists)
    n = len(dists_sorted)

    def pct(p: float) -> float:
        # p in [0,100]
        if n == 1:
            return dists_sorted[0]
        k = (p / 100.0) * (n - 1)
        lo = int(math.floor(k))
        hi = int(math.ceil(k))
        if lo == hi:
            return dists_sorted[lo]
        w = k - lo
        return dists_sorted[lo] * (1 - w) + dists_sorted[hi] * w

    mean = statistics.fmean(dists)
    median = pct(50)
    p90 = pct(90)
    p95 = pct(95)
    p99 = pct(99)
    mx = dists_sorted[-1]
    mn = dists_sorted[0]

    print("=== CARLA TL nearest-distance report ===")
    print(f"SUMO points: {len(entries)}")
    print(f"CARLA TL actors: {len(carla_pts)}")
    print(f"Distance metric: {'3D' if args.use_3d else '2D'}")
    print("")
    print(f"min:    {mn:8.3f} m")
    print(f"mean:   {mean:8.3f} m")
    print(f"median: {median:8.3f} m")
    print(f"p90:    {p90:8.3f} m")
    print(f"p95:    {p95:8.3f} m")
    print(f"p99:    {p99:8.3f} m")
    print(f"max:    {mx:8.3f} m")
    print("")

    # Show worst matches
    results_sorted = sorted(results, key=lambda r: r["distance_m"], reverse=True)
    topn = min(args.top, len(results_sorted))
    print(f"=== Worst {topn} matches ===")
    for r in results_sorted[:topn]:
        print(
            f"jid={r['junction_id']}, lid={r['link_id']}, "
            f"d={r['distance_m']:.3f} m, "
            f"carla_actor={r['nearest_carla_actor_id']}, "
            f"sumo=({r['sumo_x']:.2f},{r['sumo_y']:.2f},{r['sumo_z']:.2f}), "
            f"carla=({r['carla_x']:.2f},{r['carla_y']:.2f},{r['carla_z']:.2f})"
        )

    if args.out:
        fieldnames = list(results_sorted[0].keys())
        with open(args.out, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=fieldnames)
            w.writeheader()
            w.writerows(results_sorted)
        print("")
        print(f"Wrote report: {args.out}")

    # Heuristic hint
    print("")
    if median < 10.0 and p95 < 20.0:
        print("Interpretation: frames likely already aligned; no global transform needed.")
    elif median > 50.0:
        print("Interpretation: large systematic mismatch; consider estimating a rigid transform (R,t) or using a different ID strategy.")
    else:
        print("Interpretation: mixed; may be partial placement/missing actors or modest offset. Inspect worst matches and consider local-per-junction matching.")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())