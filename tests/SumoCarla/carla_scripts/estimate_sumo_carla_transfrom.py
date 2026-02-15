#!/usr/bin/env python3
"""
estimate_sumo_carla_transform.py

Estimate a 2D rigid transform (R,t) mapping SUMO (x,y) points into CARLA (x,y) points.

Uses iterative closest point (ICP) with Kabsch (SVD) rigid fitting + trimming.

Inputs:
  - SUMO points from traffic_light_table.csv (recommended), OR
  - SUMO points from map.net.xml (junction positions or lane shape points)
  - CARLA points from traffic.traffic_light actors, OR
  - CARLA points from map waypoints (optional mode)

Usage examples:
  # Using traffic light table (SUMO) and CARLA traffic lights:
  python estimate_sumo_to_carla_transform.py --sumo-csv tests/.../traffic_light_table.csv --carla-mode traffic_lights

  # Using net.xml lane shape sample points (SUMO) and CARLA waypoints (more robust):
  python estimate_sumo_to_carla_transform.py --sumo-net map.net.xml --sumo-net-mode lanes --carla-mode waypoints

Notes:
  - Requires CARLA server running and correct map loaded.
  - If coordinate frames already match, theta ~ 0 and translation small.
"""

import argparse
import csv
import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from typing import List, Tuple, Optional

import numpy as np
import carla


Point2 = Tuple[float, float]


def load_sumo_points_from_csv(csv_path: str, max_points: Optional[int] = None) -> List[Point2]:
    pts: List[Point2] = []
    with open(csv_path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            # Support both snake_case and camelCase headers
            x = row.get("x") or row.get("X")
            y = row.get("y") or row.get("Y")
            if x is None or y is None:
                raise ValueError(f"CSV missing x/y columns. Got headers: {reader.fieldnames}")
            pts.append((float(x), float(y)))
            if max_points and len(pts) >= max_points:
                break
    return pts


def _parse_shape(shape_str: str) -> List[Point2]:
    # shape format: "x,y x,y x,y"
    out: List[Point2] = []
    for token in shape_str.strip().split():
        xs, ys = token.split(",")
        out.append((float(xs), float(ys)))
    return out


def load_sumo_points_from_netxml(
    net_path: str,
    mode: str,
    sample_every_n: int = 5,
    max_points: Optional[int] = None
) -> List[Point2]:
    """
    mode:
      - 'junctions': use <junction x= y=> points (good for coarse alignment)
      - 'lanes': sample points from <lane shape="..."> (best density)
    """
    tree = ET.parse(net_path)
    root = tree.getroot()

    pts: List[Point2] = []

    if mode == "junctions":
        for j in root.findall("junction"):
            # skip internal junctions if any (id starts with ':')
            jid = j.get("id", "")
            if jid.startswith(":"):
                continue
            x = j.get("x")
            y = j.get("y")
            if x is None or y is None:
                continue
            pts.append((float(x), float(y)))
            if max_points and len(pts) >= max_points:
                break

    elif mode == "lanes":
        # iterate edges/lanes
        for edge in root.findall("edge"):
            # skip internal edges if function="internal"
            if edge.get("function") == "internal":
                continue
            for lane in edge.findall("lane"):
                shape = lane.get("shape")
                if not shape:
                    continue
                shape_pts = _parse_shape(shape)
                # subsample to reduce size
                for i in range(0, len(shape_pts), max(1, sample_every_n)):
                    pts.append(shape_pts[i])
                    if max_points and len(pts) >= max_points:
                        return pts
    else:
        raise ValueError("sumo-net-mode must be 'junctions' or 'lanes'")

    return pts


def get_carla_points_from_traffic_lights(world: carla.World, max_points: Optional[int] = None) -> List[Point2]:
    actors = world.get_actors().filter("traffic.traffic_light*")
    pts: List[Point2] = []
    for a in actors:
        loc = a.get_transform().location
        pts.append((float(loc.x), float(loc.y)))
        if max_points and len(pts) >= max_points:
            break
    return pts


def get_carla_points_from_waypoints(world: carla.World, resolution: float = 5.0, max_points: Optional[int] = None) -> List[Point2]:
    m = world.get_map()
    wps = m.generate_waypoints(resolution)
    pts: List[Point2] = []
    for wp in wps:
        loc = wp.transform.location
        pts.append((float(loc.x), float(loc.y)))
        if max_points and len(pts) >= max_points:
            break
    return pts


def kabsch_2d(A: np.ndarray, B: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """
    Solve for R,t minimizing || (A R^T + t) - B ||^2
    A, B: Nx2 arrays of matched points.
    Returns R (2x2), t (2,)
    """
    assert A.shape == B.shape and A.shape[1] == 2
    ca = A.mean(axis=0)
    cb = B.mean(axis=0)
    A0 = A - ca
    B0 = B - cb

    H = A0.T @ B0
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T

    # reflection fix
    if np.linalg.det(R) < 0:
        Vt[1, :] *= -1
        R = Vt.T @ U.T

    t = cb - (R @ ca)
    return R, t


def apply_transform(pts: np.ndarray, R: np.ndarray, t: np.ndarray) -> np.ndarray:
    # pts Nx2
    return (pts @ R.T) + t


def nearest_neighbors_bruteforce(src: np.ndarray, dst: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """
    For each src point, find nearest dst point.
    Returns (idx, d2):
      idx: N indices into dst
      d2:  N squared distances
    """
    # src: Nx2, dst: Mx2
    # brute force, OK for a few thousand points
    d2 = ((src[:, None, :] - dst[None, :, :]) ** 2).sum(axis=2)  # NxM
    idx = d2.argmin(axis=1)
    mind2 = d2[np.arange(d2.shape[0]), idx]
    return idx, mind2


@dataclass
class ICPResult:
    R: np.ndarray
    t: np.ndarray
    theta_deg: float
    tx: float
    ty: float
    rmse: float
    median: float
    p95: float
    inliers: int
    total: int


def icp_fit(
    sumo_pts: List[Point2],
    carla_pts: List[Point2],
    iters: int = 30,
    trim_fraction: float = 0.7,
    max_pair_dist: float = 50.0,
    verbose: bool = True,
) -> ICPResult:
    A = np.array(sumo_pts, dtype=float)
    B = np.array(carla_pts, dtype=float)

    if A.shape[0] < 3 or B.shape[0] < 3:
        raise ValueError("Need at least 3 points in each set.")

    # Initial transform
    R = np.eye(2)
    t = np.zeros(2)

    max_pair_d2 = max_pair_dist * max_pair_dist

    for k in range(iters):
        A2 = apply_transform(A, R, t)

        idx, d2 = nearest_neighbors_bruteforce(A2, B)

        # reject very large matches
        ok = d2 <= max_pair_d2
        if ok.sum() < 3:
            raise RuntimeError(
                f"Too few correspondences under max_pair_dist={max_pair_dist}m. "
                f"Try increasing max_pair_dist or verify correct CARLA map."
            )

        A_ok = A[ok]
        B_ok = B[idx[ok]]
        d2_ok = d2[ok]

        # trimming: keep best trim_fraction correspondences
        n = A_ok.shape[0]
        keep = max(3, int(math.floor(trim_fraction * n)))
        order = np.argsort(d2_ok)
        sel = order[:keep]
        A_sel = A_ok[sel]
        B_sel = B_ok[sel]

        R_new, t_new = kabsch_2d(A_sel, B_sel)

        # compute update convergence
        dR = np.linalg.norm(R_new - R)
        dt = np.linalg.norm(t_new - t)

        R, t = R_new, t_new

        if verbose:
            A_fit = apply_transform(A_sel, R, t)
            err = np.sqrt(((A_fit - B_sel) ** 2).sum(axis=1))
            rmse = float(np.sqrt((err ** 2).mean()))
            print(f"[iter {k+1:02d}] inliers={keep}/{len(sumo_pts)} rmse={rmse:.3f} dR={dR:.6f} dt={dt:.6f}")

        if dR < 1e-6 and dt < 1e-4:
            break

    # Final error stats using all points with nearest neighbor
    A2 = apply_transform(A, R, t)
    idx, d2 = nearest_neighbors_bruteforce(A2, B)
    err = np.sqrt(d2)
    err_sorted = np.sort(err)

    def percentile(p: float) -> float:
        if len(err_sorted) == 1:
            return float(err_sorted[0])
        k = (p / 100.0) * (len(err_sorted) - 1)
        lo = int(math.floor(k))
        hi = int(math.ceil(k))
        if lo == hi:
            return float(err_sorted[lo])
        w = k - lo
        return float(err_sorted[lo] * (1 - w) + err_sorted[hi] * w)

    rmse = float(np.sqrt((err ** 2).mean()))
    theta = math.degrees(math.atan2(R[1, 0], R[0, 0]))

    # Inliers under max_pair_dist for reporting
    inliers = int((err <= max_pair_dist).sum())

    return ICPResult(
        R=R, t=t,
        theta_deg=theta, tx=float(t[0]), ty=float(t[1]),
        rmse=rmse, median=percentile(50), p95=percentile(95),
        inliers=inliers, total=len(sumo_pts)
    )


def main() -> int:
    ap = argparse.ArgumentParser()
    # SUMO sources
    ap.add_argument("--sumo-csv", help="Path to traffic_light_table.csv (uses x,y columns)")
    ap.add_argument("--sumo-net", help="Path to map.net.xml")
    ap.add_argument("--sumo-net-mode", choices=["junctions", "lanes"], default="junctions",
                    help="If --sumo-net is used: take points from junctions or lane shapes")
    ap.add_argument("--sumo-sample-every-n", type=int, default=5,
                    help="If using net lanes: subsample every N points along lane shapes")
    ap.add_argument("--sumo-max-points", type=int, default=2000)

    # CARLA
    ap.add_argument("--carla-host", default="127.0.0.1")
    ap.add_argument("--carla-port", type=int, default=2000)
    ap.add_argument("--carla-timeout", type=float, default=20.0)
    ap.add_argument("--load-world", help="Optional: client.load_world('Town01') etc before extracting points")
    ap.add_argument("--carla-mode", choices=["traffic_lights", "waypoints"], default="traffic_lights")
    ap.add_argument("--waypoint-resolution", type=float, default=5.0)
    ap.add_argument("--carla-max-points", type=int, default=2000)

    # ICP settings
    ap.add_argument("--iters", type=int, default=30)
    ap.add_argument("--trim", type=float, default=0.7, help="Keep best fraction of matches each iter (0.3-0.9 typical)")
    ap.add_argument("--max-pair-dist", type=float, default=150.0, help="Reject correspondences above this distance (m)")
    ap.add_argument("--quiet", action="store_true")

    args = ap.parse_args()

    if not args.sumo_csv and not args.sumo_net:
        raise SystemExit("Provide --sumo-csv or --sumo-net")

    # Load SUMO points
    if args.sumo_csv:
        sumo_pts = load_sumo_points_from_csv(args.sumo_csv, max_points=args.sumo_max_points)
    else:
        sumo_pts = load_sumo_points_from_netxml(
            args.sumo_net,
            mode=args.sumo_net_mode,
            sample_every_n=args.sumo_sample_every_n,
            max_points=args.sumo_max_points
        )

    if len(sumo_pts) < 10:
        raise SystemExit(f"Too few SUMO points loaded: {len(sumo_pts)}")

    # Connect CARLA
    client = carla.Client(args.carla_host, args.carla_port)
    client.set_timeout(args.carla_timeout)

    if args.load_world:
        # Warning: load_world resets the world; do before using world
        world = client.load_world(args.load_world)
    else:
        world = client.get_world()

    # Load CARLA points
    if args.carla_mode == "traffic_lights":
        carla_pts = get_carla_points_from_traffic_lights(world, max_points=args.carla_max_points)
    else:
        carla_pts = get_carla_points_from_waypoints(world, resolution=args.waypoint_resolution, max_points=args.carla_max_points)

    if len(carla_pts) < 10:
        raise SystemExit(f"Too few CARLA points loaded ({len(carla_pts)}). Are you on the right map / actors present?")

    print(f"SUMO points:  {len(sumo_pts)}")
    print(f"CARLA points: {len(carla_pts)}  (mode={args.carla_mode})")

    res = icp_fit(
        sumo_pts=sumo_pts,
        carla_pts=carla_pts,
        iters=args.iters,
        trim_fraction=args.trim,
        max_pair_dist=args.max_pair_dist,
        verbose=(not args.quiet),
    )

    print("\n=== Estimated SUMO -> CARLA transform (2D rigid) ===")
    print(f"theta_deg: {res.theta_deg:.6f}")
    print(f"tx:        {res.tx:.6f}")
    print(f"ty:        {res.ty:.6f}")
    print(f"rmse:      {res.rmse:.3f} m")
    print(f"median:    {res.median:.3f} m")
    print(f"p95:       {res.p95:.3f} m")
    print(f"inliers:   {res.inliers}/{res.total} under max_pair_dist")

    print("\nR =")
    print(res.R)
    print("t =")
    print(res.t)

    print("\nApply to a SUMO point (x,y):")
    print("  [x_carla, y_carla]^T = R * [x_sumo, y_sumo]^T + t")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())