"""Cross-check: run OUR SUMO snapshot through the REFERENCE rollout loop (#189).

wm_native_reference.rollout() demonstrably launches a stopped queue on AlpaSim's
own ClipGT scene (8/10). wm_snapshot_rollout.rollout() freezes the queue on our
SUMO snapshot (0/N). Those differ in BOTH the loop and the data, so this script
holds the loop fixed and swaps only the data: it packs our SUMO snapshot into the
env_data layout the reference expects and calls the reference rollout.

  launches -> our rollout loop has a bug
  freezes  -> the loop is fine; the difference is our data/map

Run (CATK venv, from tests/WorldModels/tools):
    python wm_crosscheck.py --at 175.4 --seconds 20 --seed 0
"""
from __future__ import annotations

import argparse
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
import wm_native_reference as R
import wm_snapshot_rollout as SR

DT = SR.DT
HISTORY = SR.HISTORY


def build_env_from_sumo(fcd, net_file, ego_id, t_lo, t_hi, half_extent, ego_at):
    """Pack a SUMO FCD window into the reference's env_data layout."""
    import torch
    from catk_map import CatkMap

    steps = [round(t_lo + k * DT, 1) for k in range(int(round((t_hi - t_lo) / DT)) + 1)]
    steps = [t for t in steps if t in fcd]
    n_t = len(steps)
    ex, ey = fcd[ego_at][ego_id][0], fcd[ego_at][ego_id][1]
    ids = sorted({vid for t in steps for vid in fcd[t]
                  if abs(fcd[t][vid][0] - ex) <= half_extent
                  and abs(fcd[t][vid][1] - ey) <= half_extent})
    ids = [v for v in ids if v != ego_id]

    xyz = np.zeros((len(ids), n_t, 3), np.float32)
    head = np.zeros((len(ids), n_t), np.float32)
    valid = np.zeros((len(ids), n_t), bool)
    for i, vid in enumerate(ids):
        for k, t in enumerate(steps):
            s = fcd[t].get(vid)
            if s is None:
                continue
            xyz[i, k, 0], xyz[i, k, 1] = s[0], s[1]
            head[i, k] = SR.sumo_deg_to_math_rad(s[2])
            valid[i, k] = True
    ego_xyz = np.zeros((n_t, 3), np.float32)
    ego_head = np.zeros(n_t, np.float32)
    for k, t in enumerate(steps):
        s = fcd[t].get(ego_id) or fcd[ego_at][ego_id]
        ego_xyz[k, 0], ego_xyz[k, 1] = s[0], s[1]
        ego_head[k] = SR.sumo_deg_to_math_rad(s[2])

    cm = CatkMap.build(net_file)
    env_map = {name: {"polylines": torch.from_numpy(seg.copy()),
                      "label": torch.full((len(seg),),
                                          __import__("catk_map").LAYER_IDS[name], dtype=torch.long)}
               for name, seg in cm.segments.items()}

    env = {
        "ego": {"xyz": torch.from_numpy(ego_xyz), "heading": torch.from_numpy(ego_head),
                "lwh": torch.tensor([SR.CAR_L, SR.CAR_W, 1.6])},
        "agents": {"xyz": torch.from_numpy(xyz), "heading": torch.from_numpy(head),
                   "valid_mask": torch.from_numpy(valid),
                   "lwh": torch.tensor([[SR.CAR_L, SR.CAR_W, 1.6]] * len(ids)),
                   "track_ids": torch.arange(len(ids)),
                   "class_ids": torch.zeros(len(ids), dtype=torch.long),   # 0 = car
                   "num_obstacles": len(ids)},
        "map": env_map,
        "env": {"curr_t": HISTORY - 1,
                "agent_is_static": [False] * len(ids),
                "agent_object_ids": ids},
        "metadata": {"frame_rate": 10},
    }
    return env, ids, steps


def main() -> int:
    ap = argparse.ArgumentParser()
    demo = pathlib.Path(__file__).resolve().parents[1] / "Demo"
    ap.add_argument("--fcd", default=str(demo / "fcd_van320.xml"))
    ap.add_argument("--net", default=str(demo / "simple_traffic_light.net.xml"))
    ap.add_argument("--ego", default="ego")
    ap.add_argument("--at", type=float, default=175.4)
    ap.add_argument("--window", type=float, default=45.0)
    ap.add_argument("--half-extent", type=float, default=200.0)
    ap.add_argument("--seconds", type=float, default=20.0)
    ap.add_argument("--topk", type=int, default=5)
    ap.add_argument("--seeds", type=int, nargs="+", default=[0, 1, 2])
    args = ap.parse_args()

    sys.path.insert(0, SR.ALPASIM_SRC)
    import torch  # noqa: F401
    from alpasim_trafficsim.catk.model_adapter import CATK

    fcd = SR.read_fcd(args.fcd)
    t_lo = round(args.at - (HISTORY - 1) * DT, 1)
    env, ids, steps = build_env_from_sumo(fcd, args.net, args.ego, t_lo,
                                          t_lo + args.window, args.half_extent, args.at)
    start_s = (args.at - t_lo)                     # reference indexes from the window start
    print(f"packed {len(ids)} agents over {len(steps)} frames; "
          f"reference start_s={start_s:.1f}s (= sim t={args.at})")

    model = CATK(config_path=f"{R.MODELS}/catk_v120/config.yaml",
                 ckpt_path=f"{R.MODELS}/catk_v120/latest.ckpt",
                 token_pkl_dir=f"{R.MODELS}/tokens",
                 disable_sub_plyline_type=True, device="cuda")

    for seed in args.seeds:
        rec = R.rollout(env, seconds=args.seconds, topk=args.topk,
                        start_s=start_s, seed=seed)
        sel = list(rec["selected"])
        names = [ids[i] for i in sel]
        sp = rec["speed"]
        q = [k for k, nm in enumerate(names) if nm.startswith("flow_EW")]
        allv = sp[rec["valid"]]
        qv = sp[:, q][rec["valid"][:, q]] if q else np.array([0.0])
        print(f"  seed {seed}: {len(sel)} agents driven | ALL medV={np.median(allv):5.2f} "
              f"| EW-queue medV={np.median(qv):5.2f} maxV={qv.max():5.2f} "
              f"-> {'LAUNCHED' if qv.max() > 3 else 'frozen'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
