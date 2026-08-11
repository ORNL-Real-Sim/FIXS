"""Behavioral gate: does the converted SUMO map restore CATK queue launch?

Places a synthetic 3-car stopped queue at int_east's westbound stop line on OUR
demo net and rolls CATK closed-loop (native-format env_data, same loop as the
ClipGT probes). Map variants:

  centers : lane_centers only, our old broken format's content (control --
            expected to lock up like the co-sim did)
  conv    : netconvert-xodr -> trajdata layers (lane centers + lane/road
            boundaries; waitlines/areas empty in the xodr)
  full    : conv + waitlines and intersection_areas synthesized exactly from
            the SUMO net (signalized incoming-lane endpoints; junction shapes)

Pass criterion: 'full' (and ideally 'conv') launch at ~the ClipGT native rate
(~8/10); 'centers' locking up reproduces the co-sim failure on this geometry.

Run (CATK venv):
    python wm_queue_gate.py --variants centers conv full --seeds 0 1 2 3 4 5 6 7 8 9
"""
from __future__ import annotations

import argparse
import pathlib
import re
import sys

import numpy as np

import wm_native_reference as R
from wm_map_convert_check import build_layers, convert, sumo_lanes

DT = R.DT
HISTORY = R.HISTORY
PREDICT = R.PREDICT
LAYER_IDS = {"lanelines": 0, "road_boundaries": 1, "waitlines": 2, "crosswalks": 3,
             "lane_boundaries": 4, "lane_centers": 5, "intersection_areas": 6,
             "road_islands": 7}

# westbound approach to int_east on the demo net (from the net geometry)
STOP_X, LANE_Y, HEADING = 1080.4, 304.8, np.pi


def synth_semantic_layers(net_file: pathlib.Path) -> dict[str, list[np.ndarray]]:
    """waitlines + intersection_areas straight from the SUMO net.

    waitline: for every signalized junction, a lane-width segment perpendicular
    to each incoming lane at its end point (SUMO's stop position).
    intersection_area: the junction's own shape polygon.
    """
    txt = net_file.read_text(encoding="utf-8")
    lane_shape = {}
    for m in re.finditer(r'<lane id="([^"]*)"[^>]*shape="([^"]+)"', txt):
        pts = np.array([tuple(map(float, p.split(","))) for p in m.group(2).split()])
        if len(pts) >= 2:
            lane_shape[m.group(1)] = pts
    out = {"waitlines": [], "intersection_areas": []}
    for j in re.finditer(r'<junction id="([^"]+)" type="traffic_light"[^>]*'
                         r'incLanes="([^"]*)"[^>]*shape="([^"]+)"', txt):
        for lid in j.group(2).split():
            p = lane_shape.get(lid)
            if p is None:
                continue
            end, prev = p[-1], p[-2]
            t = end - prev
            t = t / (np.linalg.norm(t) + 1e-9)
            n = np.array([-t[1], t[0]])
            out["waitlines"].append(np.stack([end - 1.6 * n, end + 1.6 * n]))
        poly = np.array([tuple(map(float, p.split(","))) for p in j.group(3).split()])
        if len(poly) >= 3:
            out["intersection_areas"].append(np.vstack([poly, poly[:1]]))
    return out


def build_env_map(layers: dict[str, list[np.ndarray]], torch) -> dict:
    """AlpaSim-native map: stride-2 3-point segments + tensor labels."""
    env_map = {}
    for name, polys in layers.items():
        if not polys:
            continue
        seg = R._segments([np.asarray(p) for p in polys])
        if len(seg):
            env_map[name] = {"polylines": torch.from_numpy(seg),
                             "label": torch.full((len(seg),), LAYER_IDS[name],
                                                 dtype=torch.long)}
    return env_map


def make_scene(torch, n_queue: int = 3, spacing: float = 6.0, n_steps: int = 400,
               creep: tuple[float, float] | None = None,
               jitter: tuple[float, float] | None = None, seed: int = 12345):
    """3 stopped queue cars + an ego passing eastbound on the opposite lane.

    creep=(v0, v1): history speed ramps v0 -> v1 over the 16 frames (ending at
    the queue positions), mimicking a real just-stopped queue (ClipGT stopped
    cars show ~0.5 -> 0.1 m/s residual motion; a SUMO stop is EXACTLY frozen).
    creep=None: perfectly frozen history (identical 16 frames), the SUMO case.
    """
    q_x = STOP_X + spacing * np.arange(n_queue)          # behind the stop line
    agents_xyz = torch.zeros(n_queue, n_steps, 3)
    agents_xyz[:, :, 0] = torch.tensor(q_x, dtype=torch.float32)[:, None]
    agents_xyz[:, :, 1] = LANE_Y
    if creep is not None:
        v = np.linspace(creep[0], creep[1], HISTORY)      # decel toward the stop
        back = np.cumsum(v[::-1] * DT)[::-1]              # distance still to travel
        for k in range(HISTORY - 1):
            agents_xyz[:, k, 0] = torch.tensor(q_x + back[k], dtype=torch.float32)
    agents_heading = torch.full((n_queue, n_steps), float(HEADING))
    if jitter is not None:
        # perception-like micro-noise on the HISTORY frames (ClipGT stopped cars
        # move 2-7 cm/frame with ~0.5 deg heading noise; a SUMO stop is exact)
        rng = np.random.default_rng(seed)
        s_pos, s_deg = jitter
        agents_xyz[:, :HISTORY, 0:2] += torch.tensor(
            rng.normal(0.0, s_pos, (n_queue, HISTORY, 2)), dtype=torch.float32)
        agents_heading[:, :HISTORY] += torch.tensor(
            rng.normal(0.0, np.radians(s_deg), (n_queue, HISTORY)), dtype=torch.float32)
    valid = torch.ones(n_queue, n_steps, dtype=torch.bool)

    ego_xyz = torch.zeros(n_steps, 3)
    ego_xyz[:, 0] = (STOP_X - 80.0) + 10.0 * DT * torch.arange(n_steps)  # eastbound 10 m/s
    ego_xyz[:, 1] = LANE_Y - 9.6                                 # opposite-direction lane
    ego_heading = torch.zeros(n_steps)
    return {
        "ego": {"xyz": ego_xyz, "heading": ego_heading,
                "lwh": torch.tensor([4.5, 2.0, 1.6])},
        "agents": {"xyz": agents_xyz, "heading": agents_heading,
                   "valid_mask": valid,
                   "lwh": torch.tensor([[4.5, 2.0, 1.6]] * n_queue),
                   "track_ids": torch.arange(n_queue),
                   "class_ids": torch.zeros(n_queue, dtype=torch.long),
                   "num_obstacles": n_queue},
        "env": {"curr_t": HISTORY - 1, "agent_is_static": [False] * n_queue},
    }


def rollout(scene, env_map, model, seconds, topk, seed, stride):
    import torch
    from alpasim_trafficsim.catk.obstacle_classes import obstacle_class_metadata

    torch.manual_seed(seed)
    np.random.seed(seed)
    model.model.validation_rollout_sampling.num_k = topk

    A = scene["agents"]["num_obstacles"]
    start = HISTORY - 1
    hxyz = scene["agents"]["xyz"][:, :HISTORY].numpy().copy()
    hhead = scene["agents"]["heading"][:, :HISTORY].numpy().copy()
    n_step = int(round(seconds / DT))
    rec_speed, rec_xyz = [], []
    step = 0
    while step < n_step:
        current = start + step
        ego_slice = scene["ego"]["xyz"][current - HISTORY + 1: current + PREDICT + 1]
        ego_head = scene["ego"]["heading"][current - HISTORY + 1: current + PREDICT + 1]
        future_xyz = torch.from_numpy(np.repeat(hxyz[:, -1:, :], PREDICT, axis=1))
        future_head = torch.from_numpy(np.repeat(hhead[:, -1:], PREDICT, axis=1))
        step_env = {
            "ego": {"xyz": ego_slice, "heading": ego_head, "lwh": scene["ego"]["lwh"]},
            "agents": {"xyz": torch.cat([torch.from_numpy(hxyz), future_xyz], dim=1),
                       "heading": torch.cat([torch.from_numpy(hhead), future_head], dim=1),
                       "valid_mask": torch.cat(
                           [torch.ones(A, HISTORY, dtype=torch.bool),
                            torch.zeros(A, PREDICT, dtype=torch.bool)], dim=1),
                       "lwh": scene["agents"]["lwh"],
                       "track_ids": torch.arange(A),
                       "class_ids": scene["agents"]["class_ids"],
                       "num_obstacles": A},
            "map": R._map_near(env_map,
                               np.vstack((ego_slice[HISTORY - 1].numpy()[None, :2],
                                          hxyz[:, -1, :2])), 25.0),
            "env": {"curr_t": HISTORY - 1, "agent_is_static": [False] * A},
            "metadata": {"frame_rate": 10, **obstacle_class_metadata()},
        }
        result = model.create_model_input(step_env, filter_map_by_ego=False,
                                          filter_distance_th=0.0)
        actions = model.inference(result["input_data"])
        fx = actions["agent_future_xyz"].cpu().numpy()
        fh = actions["agent_future_heading"].cpu().numpy()
        n_substep = min(stride, PREDICT, n_step - step)
        prev = hxyz[:, -1].copy()
        for s in range(n_substep):
            xyz = fx[:, s].copy()
            rec_speed.append(np.linalg.norm(xyz[:, :2] - prev[:, :2], axis=1) / DT)
            rec_xyz.append(xyz.copy())
            hxyz = np.concatenate([hxyz[:, 1:], xyz[:, None]], axis=1)
            hhead = np.concatenate([hhead[:, 1:], fh[:, s][:, None]], axis=1)
            prev = xyz
        step += n_substep
    return np.asarray(rec_speed), np.asarray(rec_xyz)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--net", type=pathlib.Path,
                    default=pathlib.Path(__file__).parent / "../Demo/simple_traffic_light.net.xml")
    ap.add_argument("--seconds", type=float, default=9.0)
    ap.add_argument("--topk", type=int, default=5)
    ap.add_argument("--stride", type=int, default=3)
    ap.add_argument("--seeds", type=int, nargs="+", default=[0, 1, 2, 3, 4, 5, 6, 7, 8, 9])
    ap.add_argument("--variants", nargs="+", default=["centers", "conv", "full"])
    ap.add_argument("--creep", type=float, nargs=2, default=None, metavar=("V0", "V1"),
                    help="history speed ramp v0->v1 m/s (default: exactly frozen)")
    ap.add_argument("--origin", type=float, nargs=2, default=None, metavar=("X", "Y"),
                    help="shift scene+map by -X,-Y (test absolute-coordinate scale; "
                         "the co-sim adapter ego-centers, ClipGT coords are ~1e2)")
    ap.add_argument("--jitter", type=float, nargs=2, default=None, metavar=("POS_M", "HEAD_DEG"),
                    help="per-frame Gaussian history noise, e.g. 0.04 0.5 "
                         "(ClipGT stopped cars: 2-7 cm/frame, ~0.5 deg)")
    args = ap.parse_args()

    sys.path.insert(0, str(R.TRAFFICSIM_SRC))
    import torch
    from alpasim_trafficsim.catk.model_adapter import CATK

    net = args.net.resolve()
    xodr = net.with_suffix(".xodr")
    if not xodr.exists():
        convert(net, xodr)
    conv_layers = build_layers(xodr)
    synth = synth_semantic_layers(net)
    if args.origin is not None:
        offs = np.array([args.origin[0], args.origin[1], 0.0])
        conv_layers = {k: [np.asarray(p, dtype=np.float64) - offs[:np.asarray(p).shape[1]]
                           for p in v] for k, v in conv_layers.items()}
        synth = {k: [np.asarray(p, dtype=np.float64) - offs[:2] for p in v]
                 for k, v in synth.items()}
        global STOP_X, LANE_Y
        STOP_X -= args.origin[0]
        LANE_Y -= args.origin[1]
    print(f"converted layers: " + ", ".join(f"{k}={len(v)}" for k, v in conv_layers.items() if v))
    print(f"synthesized: waitlines={len(synth['waitlines'])} "
          f"intersection_areas={len(synth['intersection_areas'])}")

    variant_layers = {
        "centers": {"lane_centers": conv_layers["lane_centers"]},
        "conv": conv_layers,
        "full": {**conv_layers, "waitlines": synth["waitlines"],
                 "intersection_areas": synth["intersection_areas"]},
    }

    model = CATK(config_path=str(R.MODELS / "catk_v120/config.yaml"),
                 ckpt_path=str(R.MODELS / "catk_v120/latest.ckpt"),
                 token_pkl_dir=str(R.MODELS / "tokens"),
                 disable_sub_plyline_type=True, device="cuda")
    scene = make_scene(torch, creep=tuple(args.creep) if args.creep else None,
                       jitter=tuple(args.jitter) if args.jitter else None)

    print(f"\nqueue @ int_east westbound stop line, {args.seconds:.0f}s rollout, "
          f"topk={args.topk} stride={args.stride} "
          f"history={'creep %.1f->%.1f' % tuple(args.creep) if args.creep else 'frozen'}"
          f"{' +jitter %.2fm/%.1fdeg' % tuple(args.jitter) if args.jitter else ''}")
    print(f"{'variant':8s} {'seed':>4s} {'medV':>6s} {'stopped%':>9s}  verdict")
    for variant in args.variants:
        env_map = build_env_map(variant_layers[variant], torch)
        launches = 0
        for seed in args.seeds:
            speed, xyz = rollout(scene, env_map, model, args.seconds,
                                 args.topk, seed, args.stride)
            med, stopped = float(np.median(speed)), float(np.mean(speed <= 0.5))
            launched = stopped < 0.6
            launches += launched
            print(f"{variant:8s} {seed:4d} {med:6.2f} {100*stopped:8.1f}%  "
                  f"{'LAUNCH' if launched else 'lock-up'}")
        print(f"  -> {variant}: {launches}/{len(args.seeds)} launched")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
