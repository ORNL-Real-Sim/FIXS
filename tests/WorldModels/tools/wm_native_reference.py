"""Run CATK closed-loop on a real ClipGT scene without FIXS or SUMO.

The local ClipGT fixture is a ZIP-based artifact (trajectory JSON + OpenDRIVE),
not a Pixar USD stage.  This script reconstructs the same tensor contract as
``CATKSceneAdapter`` directly so it can run in the CUDA CATK environment without
the unrelated runtime/gRPC/Rust service stack.

The bundled integration fixture is 20 seconds long.  With CATK's required 1.6 s
history and 0.5 s logged ego plan, a start at 1.5 s supports 18.1 s; the default
3.5 s start (which includes a moving car) supports 16.1 s.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import pathlib
import sys
import zipfile

import numpy as np


ALPASIM_ROOT = pathlib.Path(os.environ.get("ALPASIM_ROOT", "C:/src_git/WM_AlpaSim"))
TRAFFICSIM_SRC = ALPASIM_ROOT / "src" / "trafficsim"
MODELS = ALPASIM_ROOT / "data" / "trafficsim-models"
DEFAULT_USDZ = (
    ALPASIM_ROOT
    / "src/runtime/tests/data/integration/9c264c49-82a8-4344-ada1-d93791120c27.usdz"
)
DT = 0.1
HISTORY = 16
PREDICT = 5


def _yaw_from_xyzw(quaternion: np.ndarray) -> np.ndarray:
    x, y, z, w = np.moveaxis(quaternion, -1, 0)
    return np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _interpolate_poses(
    source_t: np.ndarray,
    positions: np.ndarray,
    quaternions_xyzw: np.ndarray,
    target_t: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Match the runtime Trajectory interpolation: linear xyz + quaternion SLERP."""
    from scipy.spatial.transform import Rotation, Slerp

    order = np.argsort(source_t, kind="stable")
    source_t = np.asarray(source_t, dtype=np.float64)[order]
    positions = np.asarray(positions, dtype=np.float64)[order]
    quaternions_xyzw = np.asarray(quaternions_xyzw, dtype=np.float64)[order]
    source_t, unique = np.unique(source_t, return_index=True)
    positions = positions[unique]
    quaternions_xyzw = quaternions_xyzw[unique]

    xyz = np.column_stack(
        [np.interp(target_t, source_t, positions[:, dim]) for dim in range(3)]
    )
    if len(source_t) == 1:
        quat = np.repeat(quaternions_xyzw[:1], len(target_t), axis=0)
    else:
        quat = Slerp(source_t, Rotation.from_quat(quaternions_xyzw))(target_t).as_quat()
    return xyz.astype(np.float32), _yaw_from_xyzw(quat).astype(np.float32)


def _resample(polyline: np.ndarray, spacing: float = 1.0) -> np.ndarray:
    polyline = np.asarray(polyline, dtype=np.float64)
    if polyline.shape[1] == 2:
        polyline = np.column_stack([polyline, np.zeros(len(polyline))])
    distance = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(polyline[:, :3], axis=0), axis=1))]
    keep = np.r_[True, np.diff(distance) > 1e-6]
    polyline, distance = polyline[keep], distance[keep]
    if len(polyline) < 2 or distance[-1] <= 0:
        return polyline.astype(np.float32)
    sample = np.arange(0.0, distance[-1], spacing)
    if not np.isclose(sample[-1], distance[-1]):
        sample = np.r_[sample, distance[-1]]
    return np.column_stack(
        [np.interp(sample, distance, polyline[:, dim]) for dim in range(3)]
    ).astype(np.float32)


def _segments(polylines: list[np.ndarray]) -> np.ndarray:
    """Match CATK map_adapter's overlapping 3-point, stride-2 segmentation."""
    output = []
    for raw in polylines:
        polyline = _resample(raw)
        if len(polyline) < 3:
            continue
        starts = list(range(0, len(polyline) - 2, 2))
        tail = len(polyline) - 3
        if starts[-1] != tail:
            starts.append(tail)
        output.extend(polyline[start : start + 3] for start in starts)
    return np.asarray(output, dtype=np.float32).reshape(-1, 3, 3)


def _build_map(xodr_xml: str, rig_json: dict) -> tuple[dict, list[np.ndarray]]:
    from trajdata.dataset_specific.xodr.geo_transform import get_t_rig_enu_from_ecef
    from trajdata.dataset_specific.xodr.vector_map_export import populate_vector_map_from_xodr
    from trajdata.maps import VectorMap
    from trajdata.maps.vec_map_elements import MapElementType

    transform = np.linalg.inv(
        get_t_rig_enu_from_ecef(np.asarray(rig_json["T_world_base"]), xodr_xml)
    )
    vector_map = VectorMap(map_id="alpasim_usdz:native-reference")
    populate_vector_map_from_xodr(
        vector_map, xodr_xml, resolution=0.5, t_xodr_enu_to_sim=transform
    )

    layers: dict[str, tuple[int, list[np.ndarray]]] = {
        "lanelines": (0, []),
        "road_boundaries": (1, []),
        "waitlines": (2, []),
        "crosswalks": (3, []),
        "lane_boundaries": (4, []),
        "lane_centers": (5, []),
        "intersection_areas": (6, []),
        "road_islands": (7, []),
    }
    for lane in vector_map.elements.get(MapElementType.ROAD_LANE, {}).values():
        layers["lane_centers"][1].append(lane.center.xyz)
        for edge in (lane.left_edge, lane.right_edge):
            if edge is not None:
                layers["lanelines"][1].append(edge.xyz)
                layers["lane_boundaries"][1].append(edge.xyz)
    for edge in vector_map.elements.get(MapElementType.ROAD_EDGE, {}).values():
        layers["road_boundaries"][1].append(edge.polyline.xyz)
    for line in vector_map.elements.get(MapElementType.WAIT_LINE, {}).values():
        layers["waitlines"][1].append(line.polyline.xyz)
    for crossing in vector_map.elements.get(MapElementType.PED_CROSSWALK, {}).values():
        layers["crosswalks"][1].append(crossing.polygon.xyz)
    for area in vector_map.elements.get(MapElementType.ROAD_AREA, {}).values():
        layers["intersection_areas"][1].append(area.exterior_polygon.xyz)
        layers["road_islands"][1].extend(hole.xyz for hole in area.interior_holes)

    import torch

    env_map = {}
    for name, (label, polylines) in layers.items():
        seg = _segments(polylines)
        if len(seg):
            env_map[name] = {
                "polylines": torch.from_numpy(seg),
                "label": torch.full((len(seg),), label, dtype=torch.long),
            }
    return env_map, [np.asarray(x) for x in layers["lane_centers"][1]]


def load_scene(path: pathlib.Path) -> tuple[dict, list[np.ndarray], float]:
    import torch
    from scipy.spatial.transform import Rotation

    with zipfile.ZipFile(path) as archive:
        rig_json = json.load(archive.open("rig_trajectories.json"))
        tracks_json = json.load(archive.open("sequence_tracks.json"))
        xodr_xml = archive.read("map.xodr").decode("utf-8")

    rig = rig_json["rig_trajectories"][0]
    rig_t = np.asarray(rig["T_rig_world_timestamps_us"], dtype=np.int64)
    timestamps = np.arange(rig_t[0], rig_t[-1] + 1, int(DT * 1e6), dtype=np.int64)
    matrices = np.asarray(rig["T_rig_worlds"], dtype=np.float64)
    ego_xyz, ego_heading = _interpolate_poses(
        rig_t.astype(np.float64),
        matrices[:, :3, 3],
        Rotation.from_matrix(matrices[:, :3, :3]).as_quat(),
        timestamps.astype(np.float64),
    )

    scene = next(iter(tracks_json.values()))
    tracks = scene["tracks_data"]
    dims = scene["cuboidtracks_data"]["cuboids_dims"]
    n_agent, n_step = len(tracks["tracks_id"]), len(timestamps)
    xyz = np.zeros((n_agent, n_step, 3), dtype=np.float32)
    heading = np.zeros((n_agent, n_step), dtype=np.float32)
    valid = np.zeros((n_agent, n_step), dtype=bool)
    class_ids = np.full(n_agent, 4, dtype=np.int64)
    static = []
    for idx, (label, flag, raw_t, poses) in enumerate(
        zip(
            tracks["tracks_label_class"],
            tracks["tracks_flags"],
            tracks["tracks_timestamps_us"],
            tracks["tracks_poses"],
        )
    ):
        raw_t = np.asarray(raw_t, dtype=np.int64)
        poses = np.asarray(poses, dtype=np.float64)
        mask = (timestamps >= raw_t.min()) & (timestamps <= raw_t.max())
        if mask.any():
            xyz[idx, mask], heading[idx, mask] = _interpolate_poses(
                raw_t.astype(np.float64), poses[:, :3], poses[:, 3:], timestamps[mask].astype(np.float64)
            )
            valid[idx, mask] = True
        normalized = str(label).lower()
        if normalized in {"automobile", "vehicle", "car"}:
            class_ids[idx] = 0
        elif normalized in {"truck", "bus", "trailer", "other_vehicle"}:
            class_ids[idx] = 1
        elif normalized in {"person", "pedestrian"} or normalized.startswith("person"):
            class_ids[idx] = 2
        elif normalized in {"bicycle", "bike", "cyclist", "cycle"} or normalized.startswith("cycle"):
            class_ids[idx] = 3
        static.append("CONTROLLABLE" not in flag)

    map_data, lane_lines = _build_map(xodr_xml, rig_json)
    ego_dims = rig.get("rig_bbox", {}).get("dim", [5.393, 2.109, 1.503])
    env = {
        "ego": {
            "xyz": torch.from_numpy(ego_xyz),
            "heading": torch.from_numpy(ego_heading),
            "lwh": torch.tensor(ego_dims, dtype=torch.float32),
        },
        "agents": {
            "xyz": torch.from_numpy(xyz),
            "heading": torch.from_numpy(heading),
            "valid_mask": torch.from_numpy(valid),
            "lwh": torch.tensor(dims, dtype=torch.float32),
            "track_ids": torch.arange(n_agent),
            "class_ids": torch.from_numpy(class_ids),
            "num_obstacles": n_agent,
        },
        "map": map_data,
        "env": {
            "curr_t": HISTORY - 1,
            "agent_is_static": static,
            "agent_object_ids": list(tracks["tracks_id"]),
        },
        "metadata": {"frame_rate": 10},
    }
    duration = (timestamps[-1] - timestamps[0]) / 1e6
    return env, lane_lines, duration


def _nearby_dynamic_agents(env: dict, radius: float, current: int) -> np.ndarray:
    valid = env["agents"]["valid_mask"][:, current].numpy()
    history_start = current - HISTORY + 1
    enough_history = (
        env["agents"]["valid_mask"][:, history_start : current + 1].sum(dim=1).numpy() >= 2
    )
    dynamic = ~np.asarray(env["env"]["agent_is_static"], dtype=bool)
    delta = env["agents"]["xyz"][:, current, :2].numpy() - env["ego"]["xyz"][current, :2].numpy()
    distance = np.linalg.norm(delta, axis=1)
    return np.flatnonzero(valid & enough_history & dynamic & (distance <= radius))


def _map_near(env_map: dict, centers: np.ndarray, radius: float) -> dict:
    centers = np.atleast_2d(centers)
    output = {}
    for name, layer in env_map.items():
        polyline = layer["polylines"]
        anchors = polyline.new_tensor(centers[:, :2]).reshape(1, 1, -1, 2)
        delta = polyline[:, :, None, :2] - anchors
        keep = (delta.square().sum(dim=-1) < radius * radius).any(dim=(1, 2))
        if keep.any():
            output[name] = {"polylines": polyline[keep], "label": layer["label"][keep]}
    return output


def rollout(env: dict, seconds: float, topk: int, start_s: float, seed: int | None = None) -> dict:
    import torch
    from alpasim_trafficsim.catk.model_adapter import CATK
    from alpasim_trafficsim.catk.obstacle_classes import obstacle_class_metadata

    if seed is not None:
        # topk>1 sampling is stochastic; seed for a reproducible draw
        torch.manual_seed(seed)
        np.random.seed(seed)

    model = CATK(
        config_path=str(MODELS / "catk_v120/config.yaml"),
        ckpt_path=str(MODELS / "catk_v120/latest.ckpt"),
        token_pkl_dir=str(MODELS / "tokens"),
        disable_sub_plyline_type=True,
        device="cuda",
    )
    model.model.validation_rollout_sampling.num_k = topk

    start = int(round(start_s / DT))
    if start < HISTORY - 1:
        raise ValueError(f"start must be at least {(HISTORY - 1) * DT:.1f}s")
    selected = _nearby_dynamic_agents(env, radius=80.0, current=start)
    if not len(selected):
        raise RuntimeError("No controllable agents are valid near ego at the reference start")
    print(f"native selected agents: {len(selected)} of {env['agents']['num_obstacles']}")

    source_xyz = env["agents"]["xyz"][selected].clone()
    source_heading = env["agents"]["heading"][selected].clone()
    source_valid = env["agents"]["valid_mask"][selected].clone()
    history_start = start - HISTORY + 1
    hxyz = source_xyz[:, history_start : start + 1].numpy().copy()
    hhead = source_heading[:, history_start : start + 1].numpy().copy()
    hvalid = source_valid[:, history_start : start + 1].numpy().copy()
    lwh = env["agents"]["lwh"][selected]
    class_ids = env["agents"]["class_ids"][selected]

    max_steps = env["ego"]["xyz"].shape[0] - start - 1
    n_step = min(int(round(seconds / DT)), max_steps)
    rec = {"t": [], "ego": [], "xyz": [], "heading": [], "valid": [], "speed": [], "gt": [], "gt_valid": []}
    step = 0
    while step < n_step:
        current = start + step
        ego_slice = env["ego"]["xyz"][current - HISTORY + 1 : current + PREDICT + 1]
        ego_head = env["ego"]["heading"][current - HISTORY + 1 : current + PREDICT + 1]
        if len(ego_slice) < HISTORY + PREDICT:
            break
        future_xyz = torch.from_numpy(np.repeat(hxyz[:, -1:, :], PREDICT, axis=1))
        future_head = torch.from_numpy(np.repeat(hhead[:, -1:], PREDICT, axis=1))
        agents_xyz = torch.cat([torch.from_numpy(hxyz), future_xyz], dim=1)
        agents_head = torch.cat([torch.from_numpy(hhead), future_head], dim=1)
        agents_valid = torch.cat(
            [torch.from_numpy(hvalid), torch.zeros((len(selected), PREDICT), dtype=torch.bool)], dim=1
        )
        step_env = {
            "ego": {"xyz": ego_slice, "heading": ego_head, "lwh": env["ego"]["lwh"]},
            "agents": {
                "xyz": agents_xyz,
                "heading": agents_head,
                "valid_mask": agents_valid,
                "lwh": lwh,
                "track_ids": torch.arange(len(selected)),
                "class_ids": class_ids,
                "num_obstacles": len(selected),
            },
            "map": _map_near(
                env["map"],
                np.vstack((ego_slice[HISTORY - 1].numpy(), hxyz[:, -1])),
                25.0,
            ),
            "env": {"curr_t": HISTORY - 1, "agent_is_static": [False] * len(selected)},
            "metadata": {"frame_rate": 10, **obstacle_class_metadata()},
        }
        result = model.create_model_input(step_env, filter_map_by_ego=False, filter_distance_th=0.0)
        actions = model.inference(result["input_data"])
        action_xyz = actions["agent_future_xyz"].cpu().numpy()
        action_head = actions["agent_future_heading"].cpu().numpy()
        action_valid = actions["agent_future_valid_mask"].cpu().numpy().astype(bool)
        if len(action_xyz) != len(selected):
            raise RuntimeError(
                f"CATK returned {len(action_xyz)} agents for {len(selected)} ordered inputs"
            )
        n_substep = min(PREDICT, n_step - step)
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
        if step % 25 == 0:
            valid_speed = speed[valid]
            median_speed = float(np.median(valid_speed)) if len(valid_speed) else float("nan")
            print(
                f"t={rec['t'][-1]:4.1f}s valid={valid.sum():2d} speed50={median_speed:5.2f}"
            )

    rec = {key: np.asarray(value) for key, value in rec.items()}
    rec["selected"] = selected
    rec["lwh"] = lwh.numpy()
    rec["start_s"] = np.asarray(start_s)
    return rec


def report(rec: dict, native_duration: float) -> None:
    valid = rec["valid"]
    speeds = rec["speed"][valid]
    gt_pair = valid & rec["gt_valid"]
    errors = np.linalg.norm(rec["xyz"][:, :, :2] - rec["gt"][:, :, :2], axis=-1)[gt_pair]
    gt_delta = np.linalg.norm(np.diff(rec["gt"][:, :, :2], axis=0), axis=-1) / DT
    gt_delta_valid = rec["gt_valid"][1:] & rec["gt_valid"][:-1]
    gt_speeds = gt_delta[gt_delta_valid]
    print("\n=== native ClipGT closed-loop CATK ===")
    print(
        f"source duration={native_duration:.1f}s; start={float(rec['start_s']):.1f}s; "
        f"rollout={rec['t'][-1]:.1f}s; agents={rec['xyz'].shape[1]}"
    )
    print(
        f"speed all-valid p10/p50/p90={np.percentile(speeds, [10, 50, 90]).round(2).tolist()} "
        f"stopped<=0.5={np.mean(speeds <= 0.5):.1%} slow<3={np.mean(speeds < 3):.1%}"
    )
    if len(gt_speeds):
        print(
            f"logged speed over matching samples p10/p50/p90="
            f"{np.percentile(gt_speeds, [10, 50, 90]).round(2).tolist()} "
            f"stopped<=0.5={np.mean(gt_speeds <= 0.5):.1%} slow<3={np.mean(gt_speeds < 3):.1%}"
        )
    if len(errors):
        print(f"closed-loop position error where log remains valid: median={np.median(errors):.1f}m p90={np.percentile(errors, 90):.1f}m")


def visualize(rec: dict, lane_lines: list[np.ndarray], out: pathlib.Path) -> None:
    import plotly.graph_objects as go

    fig = go.Figure()
    for lane in lane_lines:
        fig.add_trace(
            go.Scattergl(x=lane[:, 0], y=lane[:, 1], mode="lines", line={"color": "#d1d5db", "width": 1}, showlegend=False)
        )
    fig.add_trace(go.Scatter(x=[], y=[], mode="markers", marker={"color": "#dc2626", "size": 10}, name="logged ego"))
    fig.add_trace(go.Scatter(x=[], y=[], mode="markers", marker={"color": "#2563eb", "size": 8}, name="CATK agents"))
    fig.add_trace(go.Scatter(x=[], y=[], mode="markers", marker={"color": "#9ca3af", "size": 6, "symbol": "circle-open"}, name="logged agents"))
    frames = []
    stride = max(1, len(rec["t"]) // 180)
    for idx in range(0, len(rec["t"]), stride):
        valid = rec["valid"][idx]
        gt_valid = rec["gt_valid"][idx]
        frames.append(
            go.Frame(
                name=f"{rec['t'][idx]:.1f}",
                data=[
                    go.Scatter(x=[rec["ego"][idx, 0]], y=[rec["ego"][idx, 1]]),
                    go.Scatter(x=rec["xyz"][idx, valid, 0], y=rec["xyz"][idx, valid, 1]),
                    go.Scatter(x=rec["gt"][idx, gt_valid, 0], y=rec["gt"][idx, gt_valid, 1]),
                ],
                traces=[len(fig.data) - 3, len(fig.data) - 2, len(fig.data) - 1],
                layout=go.Layout(title=f"Native ClipGT: CATK closed-loop t={rec['t'][idx]:.1f}s"),
            )
        )
    fig.frames = frames
    fig.update_layout(
        title="Native ClipGT: CATK closed-loop",
        width=1000,
        height=650,
        xaxis={"range": [-20, 225], "title": "x (m)"},
        yaxis={"range": [-75, 75], "scaleanchor": "x", "title": "y (m)"},
        updatemenus=[{"type": "buttons", "buttons": [{"label": "play", "method": "animate", "args": [None, {"frame": {"duration": 70}, "fromcurrent": True}]}, {"label": "pause", "method": "animate", "args": [[None], {"frame": {"duration": 0}, "mode": "immediate"}]}]}],
        sliders=[{"steps": [{"method": "animate", "args": [[frame.name], {"frame": {"duration": 0}, "mode": "immediate"}], "label": frame.name} for frame in frames]}],
    )
    fig.write_html(out, include_plotlyjs=True)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--usdz", type=pathlib.Path, default=DEFAULT_USDZ)
    parser.add_argument("--seconds", type=float, default=30.0)
    parser.add_argument("--start", type=float, default=3.5)
    parser.add_argument("--topk", type=int, default=1)
    parser.add_argument("--out", type=pathlib.Path, default=pathlib.Path("native_reference.html"))
    parser.add_argument("--seed", type=int, default=None, help="torch/np seed for reproducible topk>1 draws")
    args = parser.parse_args()
    sys.path.insert(0, str(TRAFFICSIM_SRC))
    env, lanes, duration = load_scene(args.usdz)
    available = max(0.0, duration - args.start - (PREDICT - 1) * DT)
    if args.seconds > available:
        print(f"requested {args.seconds:.1f}s, but logged ego supports only {available:.1f}s; truncating")
    rec = rollout(env, min(args.seconds, available), args.topk, args.start, args.seed)
    report(rec, duration)
    np.savez(args.out.with_suffix(".npz"), **rec)
    visualize(rec, lanes, args.out)
    print(f"wrote {args.out.with_suffix('.npz')}")
    print(f"wrote {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
