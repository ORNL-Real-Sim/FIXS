"""A/B probe: native AlpaSim env_data vs our SUMO-adapter formatting, same scene.

Loads the SAME ClipGT queue scene (026d6a39, 3-car signalized queue) and rolls
CATK closed-loop from the same stopped-queue moment with the SAME loop -- only
the env_data FORMATTING changes between variants:

  native       : as wm_native_reference.rollout: absolute coords, all 8 map
                 layers as stride-2 3-point segments with tensor labels, logged
                 ego future (perfect plan).
  sumo         : as our FIXS adapter (backends/alpasim.py): ego-centered coords,
                 lane_centers ONLY (1 m resample, 30 m agent crop, NaN-padded,
                 PYTHON LIST labels), constant-velocity ego plan,
                 create_model_input(filter_map_by_ego=True, 200 m).
  sumo+logfut  : sumo, but with the logged (true) ego future   -> tests ego plan
  sumo+abs     : sumo, but in absolute coords (no ego-centering)-> tests centering
  sumo+natmap  : sumo, but with the native 8-layer map          -> tests map format

Result headline (stride=3, topk=5, seeds 0-9): native 8/10 launches vs sumo
2/10 -- the bisection variants identify which ingredient causes the collapse.

Run (CATK venv):
  python wm_sumo_format_probe.py --variants native sumo --seeds 0 1 2 3 4 --stride 3
"""
from __future__ import annotations

import argparse
import pathlib
import sys

import numpy as np

import wm_native_reference as R

DT = R.DT
HISTORY = R.HISTORY
PREDICT = R.PREDICT
QUEUE_USDZ = pathlib.Path(
    "C:/src_git/WM_AlpaSim/src/utils/tests/data/xodr_artifact/"
    "026d6a39-bd8f-4175-bc61-fe50ed0403a3.usdz"
)

# variant -> (ego_center, map_kind, ego_future_kind, light)
# light: None = no signal tokens (light_type stays 0 = NO_LANE_STATE);
#        "go"/"stop" = paint the queue's WHOLE approach lane with the upstream
#        CATK training enum (data_preprocess.py:51): STOP=2, GO=3 -- the state
#        is stamped once and held, matching training (tf_current_light).
VARIANTS = {
    "native":      (False, "native", "log", None),
    "sumo":        (True,  "ours",   "cv",  None),
    "sumo+logfut": (True,  "ours",   "log", None),
    "sumo+abs":    (False, "ours",   "cv",  None),
    "sumo+natmap": (True,  "native", "cv",  None),
    "sumo+go":     (True,  "ours",   "cv",  "go"),
    "sumo+stop":   (True,  "ours",   "cv",  "stop"),
    "native+go":   (False, "native", "log", "go"),
    # history bisection: native everything, but the 16-frame history is
    # synthetically FROZEN (current pose repeated) or a smooth creep ramp --
    # isolates whether launch needs real stopped-history microstructure.
    "native+frozen": (False, "native", "log", None),
    "native+creep":  (False, "native", "log", None),
}
LIGHT_IDX = {"stop": 2, "go": 3}   # upstream _polygon_light_type indices


# ------------------------------------------------------------------ #
#  our SUMO-adapter map formatting (mirrors backends/alpasim.py)
# ------------------------------------------------------------------ #
def sumo_style_map(lane_lines: list[np.ndarray], ox: float, oy: float,
                   anchors_xy_rel: np.ndarray, torch, crop_r: float = 30.0,
                   interval: float = 1.0) -> dict:
    """lane_centers-only, 1 m resample, 30 m agent crop, NaN pad, LIST labels."""

    def resample_by_interval(pts, step_m):
        pts = np.asarray(pts, dtype=np.float64)[:, :2]
        seg = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(pts, axis=0), axis=1))]
        if seg[-1] == 0:
            return pts[:1]
        n = max(2, int(np.ceil(seg[-1] / step_m)) + 1)
        t = np.linspace(0.0, seg[-1], n)
        return np.column_stack([np.interp(t, seg, pts[:, 0]),
                                np.interp(t, seg, pts[:, 1])])

    anchors = np.vstack([[0.0, 0.0], anchors_xy_rel])   # ego frame origin + agents
    r2 = crop_r * crop_r
    subpolys = []
    for p in lane_lines:
        rp = resample_by_interval(p, interval) - np.array([ox, oy])
        d2 = ((rp[:, None, :] - anchors[None, :, :]) ** 2).sum(-1).min(1)
        m = d2 < r2
        if not m.any():
            continue
        keep = np.where(m)[0]
        for run in np.split(keep, np.where(np.diff(keep) > 1)[0] + 1):
            if len(run) >= 3:
                subpolys.append(rp[run])
    if not subpolys:
        return {}
    vmax = max(len(s) for s in subpolys)
    arr = np.full((len(subpolys), vmax, 3), np.nan, dtype=np.float64)
    for i, s in enumerate(subpolys):
        arr[i, :len(s), 0:2] = s
        arr[i, :len(s), 2] = 0.0
    return {"lane_centers": {"polylines": torch.tensor(arr, dtype=torch.float32),
                             "label": [5] * len(subpolys)}}


def rollout(env, lane_lines, model, variant: str, seconds: float, topk: int,
            start_s: float, seed: int, stride: int) -> dict:
    """Same loop for every variant; env_data format per VARIANTS[variant]."""
    import torch
    from alpasim_trafficsim.catk.obstacle_classes import obstacle_class_metadata

    ego_center, map_kind, egofut, light = VARIANTS[variant]
    torch.manual_seed(seed)
    np.random.seed(seed)
    model.model.validation_rollout_sampling.num_k = topk

    start = int(round(start_s / DT))
    selected = R._nearby_dynamic_agents(env, radius=80.0, current=start)
    source_xyz = env["agents"]["xyz"][selected].clone()
    source_heading = env["agents"]["heading"][selected].clone()
    source_valid = env["agents"]["valid_mask"][selected].clone()
    history_start = start - HISTORY + 1
    hxyz = source_xyz[:, history_start: start + 1].numpy().copy()
    hhead = source_heading[:, history_start: start + 1].numpy().copy()
    hvalid = source_valid[:, history_start: start + 1].numpy().copy()
    if variant in ("native+frozen", "native+creep"):
        # replace real history with synthetic: pose held (frozen) or a linear
        # decel ramp ending at the current pose (creep), along each heading
        hvalid[:] = True
        cur = hxyz[:, -1].copy()
        hxyz[:] = cur[:, None, :]
        hhead[:] = hhead[:, -1:][:, [0]]
        if variant == "native+creep":
            v = np.linspace(0.5, 0.1, HISTORY)
            back = np.cumsum((v[::-1] * DT))[::-1]
            for a in range(hxyz.shape[0]):
                hd = hhead[a, -1]
                hxyz[a, :-1, 0] = cur[a, 0] - back[:-1] * np.cos(hd)
                hxyz[a, :-1, 1] = cur[a, 1] - back[:-1] * np.sin(hd)
    lwh = env["agents"]["lwh"][selected]
    class_ids = env["agents"]["class_ids"][selected]
    A = len(selected)

    # --- queue approach-lane polyline (for whole-lane light painting) ---
    # queue agents = those stopped at rollout start; the lane centerline is
    # approximated by their positions plus the LEAD car's logged future path
    # (the lane continues through the junction along it).
    queue_line = None
    if light is not None:
        v0 = np.linalg.norm(
            source_xyz[:, start, :2].numpy() - source_xyz[:, start - 5, :2].numpy(),
            axis=1) / (5 * DT)
        stopped_i = np.flatnonzero(v0 < 0.5)
        if len(stopped_i):
            # union of the stopped cars' logged paths = the controlled lane's
            # travelled extent (queue positions + through-junction continuation)
            pts = [source_xyz[stopped_i, start, :2].numpy()]
            for i in stopped_i:
                fut = source_xyz[i, start::5, :2].numpy()
                pts.append(fut[source_valid[i, start::5].numpy()])
            queue_line = np.vstack(pts)

    max_steps = env["ego"]["xyz"].shape[0] - start - 1
    n_step = min(int(round(seconds / DT)), max_steps)
    rec = {"t": [], "ego": [], "xyz": [], "heading": [], "valid": [], "speed": [],
           "gt": [], "gt_valid": []}
    step = 0
    while step < n_step:
        current = start + step
        ego_hist = env["ego"]["xyz"][current - HISTORY + 1: current + 1]
        ego_hist_head = env["ego"]["heading"][current - HISTORY + 1: current + 1]

        # --- ego future: logged truth vs our const-velocity plan ---
        if egofut == "log":
            ego_slice = env["ego"]["xyz"][current - HISTORY + 1: current + PREDICT + 1]
            ego_head = env["ego"]["heading"][current - HISTORY + 1: current + PREDICT + 1]
            if len(ego_slice) < HISTORY + PREDICT:
                break
        else:
            e_now = ego_hist[-1].numpy()
            e_prev = ego_hist[-2].numpy()
            v_vec = (e_now - e_prev) / DT
            plan = np.stack([e_now + v_vec * DT * (k + 1) for k in range(PREDICT)])
            ego_slice = torch.cat([ego_hist, torch.from_numpy(plan.astype(np.float32))])
            ego_head = torch.cat([ego_hist_head, ego_hist_head[-1].repeat(PREDICT)])

        # --- frame: ego-centered vs absolute ---
        e_ref = ego_hist[-1].numpy()
        ox, oy = (float(e_ref[0]), float(e_ref[1])) if ego_center else (0.0, 0.0)
        off = torch.tensor([ox, oy, 0.0], dtype=torch.float32)

        future_xyz = torch.from_numpy(np.repeat(hxyz[:, -1:, :], PREDICT, axis=1))
        future_head = torch.from_numpy(np.repeat(hhead[:, -1:], PREDICT, axis=1))
        agents_xyz = torch.cat([torch.from_numpy(hxyz), future_xyz], dim=1) - off
        agents_head = torch.cat([torch.from_numpy(hhead), future_head], dim=1)
        agents_valid = torch.cat(
            [torch.from_numpy(hvalid), torch.zeros((A, PREDICT), dtype=torch.bool)], dim=1)
        ego_slice = ego_slice - off

        # --- map: our lane_centers-only crop vs native 8-layer segments ---
        anchors_abs = np.vstack((e_ref[None, :2], hxyz[:, -1, :2]))
        if map_kind == "native":
            step_map = {k: {"polylines": v["polylines"] - off,
                            "label": v["label"]}
                        for k, v in R._map_near(env["map"], anchors_abs, 25.0).items()}
            filter_by_ego, filter_th = False, 0.0
        else:
            step_map = sumo_style_map(lane_lines, ox, oy,
                                      hxyz[:, -1, :2] - np.array([ox, oy]), torch)
            filter_by_ego, filter_th = True, 200.0

        step_env = {
            "ego": {"xyz": ego_slice, "heading": ego_head, "lwh": env["ego"]["lwh"]},
            "agents": {"xyz": agents_xyz, "heading": agents_head,
                       "valid_mask": agents_valid, "lwh": lwh,
                       "track_ids": torch.arange(A), "class_ids": class_ids,
                       "num_obstacles": A},
            "map": step_map,
            "env": {"curr_t": HISTORY - 1},    # agent_is_static absent == all-False
            "metadata": {"frame_rate": 10, **obstacle_class_metadata()},
        }
        result = model.create_model_input(step_env, filter_map_by_ego=filter_by_ego,
                                          filter_distance_th=filter_th)
        if light is not None and queue_line is not None:
            # upstream convention: EVERY map token of the controlled lane gets
            # the state (split_light_type repeats per token). Tag triplets whose
            # center lies within a lane half-width of the queue lane polyline.
            pe = result["input_data"]["map"]["polyline_extras"]
            tri = result["input_data"]["map"]["triplets"]          # (M, 3, 2)
            centers = tri.float().mean(dim=1)                      # (M, 2)
            ql = torch.tensor(queue_line - np.array([ox, oy]),
                              dtype=centers.dtype, device=centers.device)
            d2 = ((centers[:, None, :] - ql[None, :, :]) ** 2).sum(-1).min(1).values
            near = d2 < (2.0 ** 2)
            lt = pe["light_type"].clone()
            lt[near] = LIGHT_IDX[light]
            pe["light_type"] = lt
        actions = model.inference(result["input_data"])
        action_xyz = actions["agent_future_xyz"].cpu().numpy() + off.numpy()
        action_head = actions["agent_future_heading"].cpu().numpy()
        action_valid = actions["agent_future_valid_mask"].cpu().numpy().astype(bool)

        n_substep = min(stride, PREDICT, n_step - step)
        previous_xyz = hxyz[:, -1].copy()
        for substep in range(n_substep):
            xyz = action_xyz[:, substep].copy()
            heading = action_head[:, substep].copy()
            valid = action_valid[:, substep].copy()
            speed = np.linalg.norm(xyz[:, :2] - previous_xyz[:, :2], axis=1) / DT
            xyz[~valid] = previous_xyz[~valid]
            heading[~valid] = hhead[~valid, -1]
            hxyz = np.concatenate([hxyz[:, 1:], xyz[:, None]], axis=1)
            hhead = np.concatenate([hhead[:, 1:], heading[:, None]], axis=1)
            hvalid = np.concatenate([hvalid[:, 1:], valid[:, None]], axis=1)
            previous_xyz = xyz
            target = current + substep + 1
            rec["t"].append((target - start) * DT)
            rec["ego"].append(env["ego"]["xyz"][target].numpy())
            rec["xyz"].append(xyz.copy())
            rec["heading"].append(heading.copy())
            rec["valid"].append(valid.copy())
            rec["speed"].append(speed.copy())
            rec["gt"].append(source_xyz[:, target].numpy())
            rec["gt_valid"].append(source_valid[:, target].numpy())
        step += n_substep

    rec = {k: np.asarray(v) for k, v in rec.items()}
    rec["selected"] = selected
    rec["lwh"] = lwh.numpy()
    rec["start_s"] = np.asarray(start_s)
    return rec


def metrics(rec) -> tuple[float, float]:
    valid = rec["valid"]
    speeds = rec["speed"][valid]
    return float(np.median(speeds)), float(np.mean(speeds <= 0.5))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--usdz", type=pathlib.Path, default=QUEUE_USDZ)
    parser.add_argument("--start", type=float, default=10.5)
    parser.add_argument("--seconds", type=float, default=9.0)
    parser.add_argument("--topk", type=int, default=5)
    parser.add_argument("--seeds", type=int, nargs="+", default=[0, 1, 2, 3, 4])
    parser.add_argument("--stride", type=int, default=3,
                        help="sub-steps consumed per inference (our co-sim: 3; native: 5)")
    parser.add_argument("--variants", nargs="+", default=list(VARIANTS),
                        choices=list(VARIANTS))
    parser.add_argument("--save-clips", action="store_true",
                        help="save npz per variant for the first seed")
    args = parser.parse_args()

    sys.path.insert(0, str(R.TRAFFICSIM_SRC))
    import torch  # noqa: F401
    from alpasim_trafficsim.catk.model_adapter import CATK

    env, lane_lines, duration = R.load_scene(args.usdz)
    model = CATK(config_path=str(R.MODELS / "catk_v120/config.yaml"),
                 ckpt_path=str(R.MODELS / "catk_v120/latest.ckpt"),
                 token_pkl_dir=str(R.MODELS / "tokens"),
                 disable_sub_plyline_type=True, device="cuda")

    print(f"scene={args.usdz.name} start={args.start}s topk={args.topk} "
          f"stride={args.stride} seeds={args.seeds}")
    print(f"{'variant':12s} {'seed':>4s} {'medV':>6s} {'stopped%':>9s}  verdict")
    table = {}
    for variant in args.variants:
        launches = 0
        for seed in args.seeds:
            rec = rollout(env, lane_lines, model, variant, args.seconds,
                          args.topk, args.start, seed, args.stride)
            med, stopped = metrics(rec)
            launched = stopped < 0.6
            launches += launched
            print(f"{variant:12s} {seed:4d} {med:6.2f} {100*stopped:8.1f}%  "
                  f"{'LAUNCH' if launched else 'lock-up'}")
            if args.save_clips and seed == args.seeds[0]:
                np.savez(f"formatprobe_{variant.replace('+','_')}_seed{seed}.npz", **rec)
        table[variant] = launches
        print(f"  -> {variant}: {launches}/{len(args.seeds)} launched")
    print("\nlaunch rates: " + "  ".join(f"{k}={v}/{len(args.seeds)}"
                                         for k, v in table.items()))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
