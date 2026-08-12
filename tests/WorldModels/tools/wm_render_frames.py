"""Render CATK rollout vs vanilla ground truth as a PNG frame grid (#189).

The plotly clip is for scrubbing; this is for LOOKING -- side-by-side frames of
what CATK produced and what vanilla SUMO actually did at the same times, on the
same map, so the output can be judged visually instead of by summary statistics.

Run (CATK venv, from tests/WorldModels/tools):
    python wm_render_frames.py --at 175.4 --seconds 25 --seed 0 --out frames.png
"""
from __future__ import annotations

import argparse
import pathlib
import re
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import wm_snapshot_rollout as SR


def lane_polylines(net_file: str):
    txt = open(net_file, encoding="utf-8").read()
    return [np.array([tuple(map(float, q.split(","))) for q in m.group(1).split()])
            for m in re.finditer(r'<lane id="[^"]*"[^>]*shape="([^"]+)"', txt)]


def draw_box(ax, x, y, yaw, color, lw=1.0, alpha=1.0):
    L, W = SR.CAR_L, SR.CAR_W
    c, s = np.cos(yaw), np.sin(yaw)
    corn = np.array([[L / 2, W / 2], [L / 2, -W / 2], [-L / 2, -W / 2], [-L / 2, W / 2]])
    p = corn @ np.array([[c, -s], [s, c]]).T + np.array([x, y])
    ax.fill(p[:, 0], p[:, 1], color=color, alpha=alpha, zorder=3)
    ax.plot([x, x + c * L * 0.75], [y, y + s * L * 0.75], color="k", lw=lw, zorder=4)


def main() -> int:
    ap = argparse.ArgumentParser()
    demo = pathlib.Path(__file__).resolve().parents[1] / "Demo"
    ap.add_argument("--fcd", default=str(demo / "fcd_van320.xml"))
    ap.add_argument("--tls", default=str(demo / "tls_state.xml"))
    ap.add_argument("--net", default=str(demo / "simple_traffic_light.net.xml"))
    ap.add_argument("--junction", default="int_east")
    ap.add_argument("--at", type=float, default=175.4)
    ap.add_argument("--ego", default="ego")
    ap.add_argument("--half-extent", type=float, default=200.0)
    ap.add_argument("--crop-radius", type=float, default=25.0)
    ap.add_argument("--seconds", type=float, default=25.0)
    ap.add_argument("--stride", type=int, default=3)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--creep", type=float, default=0.0, metavar="V",
                    help="rewrite STOPPED vehicles' history as a constant creep of V m/s "
                         "ending at the observed pose (ClipGT queues that launch show "
                         "0.2-0.3 m/s residual; SUMO stops at exactly 0)")
    ap.add_argument("--jitter", type=float, default=0.0)
    ap.add_argument("--obs-noise", type=float, default=0.0)
    ap.add_argument("--light", action="store_true")
    ap.add_argument("--span", type=float, default=90.0, help="half-width of the view (m)")
    ap.add_argument("--dx", type=float, default=0.0)
    ap.add_argument("--dy", type=float, default=0.0)
    ap.add_argument("--times", default="0,2,5,9,15,22", help="offsets from t0 to render")
    ap.add_argument("--out", default="frames.png")
    args = ap.parse_args()

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    sys.path.insert(0, SR.ALPASIM_SRC)
    import torch
    from alpasim_trafficsim.catk.model_adapter import CATK
    from catk_map import CatkMap

    fcd = SR.read_fcd(args.fcd)
    tls = SR.read_tls(args.tls, args.junction)
    t0 = args.at
    ids, hxyz, hhead, _ = SR.build_snapshot(fcd, t0, args.ego, args.half_extent)
    ego_idx = ids.index(args.ego)
    if getattr(args, "creep", 0.0) > 0:
        moved = np.linalg.norm(hxyz[:, -1, :2] - hxyz[:, 0, :2], axis=1)
        stopped = [i for i in np.flatnonzero(moved < 0.5) if i != ego_idx]
        back = args.creep * SR.DT * np.arange(SR.HISTORY - 1, -1, -1)     # distance still to travel
        for i in stopped:
            hd = hhead[i, -1]
            hxyz[i, :, 0] = (hxyz[i, -1, 0] - back * np.cos(hd)).astype(np.float32)
            hxyz[i, :, 1] = (hxyz[i, -1, 1] - back * np.sin(hd)).astype(np.float32)
        print(f"creep {args.creep} m/s applied to {len(stopped)} stopped vehicles")
    print(f"snapshot t={t0}: {len(ids)} vehicles")

    cm = CatkMap.build(args.net)
    model = CATK(config_path=f"{SR.MODELS}/catk_v120/config.yaml",
                 ckpt_path=f"{SR.MODELS}/catk_v120/latest.ckpt",
                 token_pkl_dir=f"{SR.MODELS}/tokens",
                 disable_sub_plyline_type=True, device="cuda")
    tls_pts = SR.controlled_lanes(args.net, args.junction) if args.light else None
    rec, _ = SR.rollout(ids, hxyz.copy(), hhead.copy(), ego_idx, fcd, t0, cm, model, torch,
                        args.seconds, args.seed, args.stride, args.crop_radius, args.ego,
                        tls_pts, tls, obs_noise=args.obs_noise)

    lanes = lane_polylines(args.net)
    jx, jy = SR.JUNCTIONS[args.junction]
    S = args.span

    show_dt = [float(x) for x in args.times.split(",")]
    fig, axes = plt.subplots(2, len(show_dt), figsize=(4.0 * len(show_dt), 8.4))
    for col, dt in enumerate(show_dt):
        t = round(t0 + dt, 1)
        k = int(np.argmin(np.abs(rec["t"] - t)))
        st = SR.tls_at(tls, t)
        lit = st[4] if len(st) > 4 else "?"
        litcol = {"r": "red", "y": "orange", "G": "green", "g": "green"}.get(lit, "gray")

        for row, kind in enumerate(("CATK", "vanilla")):
            ax = axes[row, col]
            for l in lanes:
                if (np.abs(l[:, 0] - jx) < S + 40).any() and (np.abs(l[:, 1] - jy) < S + 40).any():
                    ax.plot(l[:, 0], l[:, 1], color="#d0d0d0", lw=2.0, zorder=1, solid_capstyle="round")
            if kind == "CATK":
                for i, vid in enumerate(ids):
                    draw_box(ax, rec["xy"][k, i, 0], rec["xy"][k, i, 1], rec["head"][k, i],
                             "#dc2626" if i == ego_idx else "#2563eb")
            else:
                snap = fcd.get(t, {})
                for vid, s in snap.items():
                    if abs(s[0] - jx) < S + 20 and abs(s[1] - jy) < S + 20:
                        draw_box(ax, s[0], s[1], SR.sumo_deg_to_math_rad(s[2]),
                                 "#dc2626" if vid == args.ego else "#16a34a")
            ax.set_xlim(jx - S + args.dx, jx + S + args.dx)
            ax.set_ylim(jy - S * 0.55 + args.dy, jy + S * 0.55 + args.dy)
            ax.set_aspect("equal")
            ax.set_xticks([]); ax.set_yticks([])
            if col == 0:
                ax.set_ylabel(kind + ("  (blue)" if kind == "CATK" else "  (green)"), fontsize=12)
            ax.set_title(f"t={t:.1f}s  (+{dt:.0f}s)  WB light={lit}", fontsize=10, color=litcol)
    fig.suptitle(f"CATK rollout vs vanilla SUMO, snapshot t={t0}s at {args.junction} "
                 f"(seed {args.seed}, jitter {args.jitter})", fontsize=13)
    fig.tight_layout()
    fig.savefig(args.out, dpi=85)
    print(f"wrote {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
