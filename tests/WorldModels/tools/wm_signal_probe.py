"""Compare CATK queue discharge under STOP->GO and STOP->NONE signals.

This is a native CATK closed-loop probe: no FIXS, SUMO, TraCI, or translator.
The STOP->NONE branch reproduces the signal encoding currently used by the
integration backend; STOP->GO uses the model's explicit green-light token.
"""
from __future__ import annotations

import argparse
import os
import pathlib
import sys

import numpy as np


ALPASIM_ROOT = pathlib.Path(os.environ.get("ALPASIM_ROOT", "C:/src_git/WM_AlpaSim"))
TRAFFICSIM_SRC = ALPASIM_ROOT / "src" / "trafficsim"
MODELS = ALPASIM_ROOT / "data" / "trafficsim-models"
DT = 0.1
HISTORY = 16
PREDICT = 5
LANE = 1.75
CAR_LWH = (4.5, 2.0, 1.6)
STOP_X = -4.0


def _constant_velocity_history(x: float, y: float, heading: float, speed: float) -> np.ndarray:
    t = (np.arange(HISTORY) - HISTORY + 1) * DT
    xyz = np.zeros((HISTORY, 3), dtype=np.float32)
    xyz[:, 0] = x + np.cos(heading) * speed * t
    xyz[:, 1] = y + np.sin(heading) * speed * t
    return xyz


def _make_map(torch, half: float = 400.0) -> tuple[dict, list[np.ndarray]]:
    s = np.arange(-half, half + 1.0, 1.0, dtype=np.float32)
    lanes = [
        np.column_stack((s, np.full_like(s, -LANE), np.zeros_like(s))),
        np.column_stack((s[::-1], np.full_like(s, LANE), np.zeros_like(s))),
        np.column_stack((np.full_like(s, LANE), s, np.zeros_like(s))),
        np.column_stack((np.full_like(s, -LANE), s[::-1], np.zeros_like(s))),
    ]
    polylines = torch.from_numpy(np.stack(lanes))
    return {
        "lane_centers": {
            "polylines": polylines,
            "label": torch.full((len(lanes),), 5, dtype=torch.long),
        }
    }, lanes


def _initial_history(initial_speed: float) -> tuple[np.ndarray, np.ndarray]:
    # The ego has already cleared the junction. Four CATK-controlled vehicles
    # approach the red in a compact 1.5 m/s queue.
    cars = [(25.0, -LANE, 0.0, 10.0)]
    cars.extend((x, -LANE, 0.0, initial_speed) for x in (-8.0, -16.0, -24.0, -32.0))
    xyz = np.stack([_constant_velocity_history(*car) for car in cars])
    heading = np.zeros((len(cars), HISTORY), dtype=np.float32)
    return xyz, heading


def _set_light(result: dict, state: int, torch) -> int:
    """Set light token near the eastbound stop line; return tagged token count."""
    triplets = result["input_data"]["map"]["triplets"]
    extras = result["input_data"]["map"]["polyline_extras"]
    center = triplets.float().mean(dim=1)
    stop = torch.tensor([STOP_X, -LANE], dtype=center.dtype, device=center.device)
    near = ((center - stop) ** 2).sum(dim=1) < 10.0**2
    light = extras["light_type"].clone()
    light[near] = state
    extras["light_type"] = light
    return int(near.sum().item())


def _rollout(
    model,
    torch,
    metadata: dict,
    env_map: dict,
    seconds: float,
    red_s: float,
    initial_light: int,
    green_state: int,
    initial_speed: float,
) -> dict:
    hxyz, hhead = _initial_history(initial_speed)
    ego_i = 0
    agent_i = np.arange(1, len(hxyz))
    n_step = int(round(seconds / DT))
    rec = {"t": [], "xy": [], "heading": [], "speed": [], "valid": [], "light": []}
    step = 0
    tagged = None
    while step < n_step:
        # CATK freezes the ego to these five known future frames. This is the
        # contract used by CATKSceneAdapter and was missing in the old sandbox.
        ego_future = np.repeat(hxyz[ego_i, -1:, :], PREDICT, axis=0)
        ego_future[:, 0] += np.arange(1, PREDICT + 1) * 10.0 * DT
        ego_xyz = np.concatenate((hxyz[ego_i], ego_future), axis=0)
        ego_head = np.concatenate((hhead[ego_i], np.zeros(PREDICT, dtype=np.float32)))

        agent_future = np.repeat(hxyz[agent_i, -1:, :], PREDICT, axis=1)
        agents_xyz = np.concatenate((hxyz[agent_i], agent_future), axis=1)
        agents_head = np.concatenate(
            (hhead[agent_i], np.repeat(hhead[agent_i, -1:], PREDICT, axis=1)), axis=1
        )
        agents_valid = np.concatenate(
            (
                np.ones((len(agent_i), HISTORY), dtype=bool),
                np.zeros((len(agent_i), PREDICT), dtype=bool),
            ),
            axis=1,
        )
        env = {
            "ego": {
                "xyz": torch.from_numpy(ego_xyz),
                "heading": torch.from_numpy(ego_head),
                "lwh": torch.tensor(CAR_LWH),
            },
            "agents": {
                "xyz": torch.from_numpy(agents_xyz),
                "heading": torch.from_numpy(agents_head),
                "valid_mask": torch.from_numpy(agents_valid),
                "lwh": torch.tensor([CAR_LWH] * len(agent_i)),
                "track_ids": torch.arange(len(agent_i)),
                "class_ids": torch.zeros(len(agent_i), dtype=torch.long),
                "num_obstacles": len(agent_i),
            },
            "map": env_map,
            "env": {"curr_t": HISTORY - 1, "agent_is_static": [False] * len(agent_i)},
            "metadata": {"frame_rate": 10, **metadata},
        }
        result = model.create_model_input(env, filter_map_by_ego=False, filter_distance_th=0.0)
        light_state = initial_light if step * DT < red_s else green_state
        tagged_now = _set_light(result, light_state, torch)
        if tagged is None:
            tagged = tagged_now
            print(f"signal tokens tagged per inference: {tagged}")
        actions = model.inference(result["input_data"])
        future_xyz = actions["agent_future_xyz"].cpu().numpy()
        future_heading = actions["agent_future_heading"].cpu().numpy()
        future_valid = actions["agent_future_valid_mask"].cpu().numpy().astype(bool)
        n_substep = min(PREDICT, n_step - step)

        previous = hxyz[:, -1].copy()
        for substep in range(n_substep):
            new_xyz = previous.copy()
            new_heading = hhead[:, -1].copy()
            new_valid = np.ones(len(hxyz), dtype=bool)
            new_xyz[ego_i, 0] += 10.0 * DT
            new_xyz[agent_i] = future_xyz[:, substep]
            new_heading[agent_i] = future_heading[:, substep]
            new_valid[agent_i] = future_valid[:, substep]
            new_xyz[agent_i[~future_valid[:, substep]]] = previous[agent_i[~future_valid[:, substep]]]
            speed = np.linalg.norm(new_xyz[:, :2] - previous[:, :2], axis=1) / DT

            hxyz = np.concatenate((hxyz[:, 1:], new_xyz[:, None]), axis=1)
            hhead = np.concatenate((hhead[:, 1:], new_heading[:, None]), axis=1)
            previous = new_xyz
            rec["t"].append((step + substep + 1) * DT)
            rec["xy"].append(new_xyz[:, :2].copy())
            rec["heading"].append(new_heading.copy())
            rec["speed"].append(speed.copy())
            rec["valid"].append(new_valid.copy())
            rec["light"].append(light_state)
        step += n_substep
    return {key: np.asarray(value) for key, value in rec.items()}


def _report(name: str, rec: dict, red_s: float) -> None:
    t = rec["t"]
    speed = rec["speed"][:, 1:]
    post = speed[t >= red_s]
    late = speed[t >= max(red_s, t[-1] - 5.0)]
    lead = speed[:, 0]
    release = t[(t >= red_s) & (lead >= 3.0)]
    print(f"\n{name}: all post-red speed p10/p50/p90={np.percentile(post, [10, 50, 90]).round(2).tolist()}")
    print(f"{name}: last-5s mean={late.mean():.2f} m/s, stopped<=0.5={np.mean(late <= 0.5):.1%}")
    print(f"{name}: lead first reaches 3 m/s after green={release[0] - red_s:.1f}s" if len(release) else f"{name}: lead never reaches 3 m/s after green")


def _visualize(results: dict[str, dict], lanes: list[np.ndarray], red_s: float, out: pathlib.Path) -> None:
    import plotly.graph_objects as go
    from plotly.subplots import make_subplots

    fig = make_subplots(rows=2, cols=1, row_heights=[0.68, 0.32], vertical_spacing=0.1)
    for lane in lanes:
        fig.add_trace(go.Scattergl(x=lane[:, 0], y=lane[:, 1], mode="lines", line={"color": "#d1d5db", "width": 1}, showlegend=False), row=1, col=1)
    colors = {"STOP->GO": "#2563eb", "STOP->NONE": "#f97316", "NONE-only": "#16a34a"}
    symbols = {"STOP->GO": "circle", "STOP->NONE": "x", "NONE-only": "diamond-open"}
    dynamic_indices = []
    for name, rec in results.items():
        dynamic_indices.append(len(fig.data))
        fig.add_trace(go.Scatter(x=rec["xy"][0, :, 0], y=rec["xy"][0, :, 1], mode="markers", marker={"size": 9, "color": colors[name], "symbol": symbols[name]}, name=name), row=1, col=1)
    for name, rec in results.items():
        fig.add_trace(go.Scatter(x=rec["t"], y=rec["speed"][:, 1:].mean(axis=1), mode="lines", line={"color": colors[name], "width": 2}, name=f"{name} queue mean", showlegend=False), row=2, col=1)
    fig.add_vline(x=red_s, line_dash="dash", line_color="#16a34a", row=2, col=1)
    fig.add_annotation(x=red_s, y=1.0, yref="y2 domain", text="signal changes", showarrow=False, xanchor="left")

    frames = []
    first = next(iter(results.values()))
    stride = max(1, len(first["t"]) // 180)
    for idx in range(0, len(first["t"]), stride):
        state = "RED / STOP" if first["t"][idx] <= red_s else "GREEN: GO versus NONE"
        frames.append(go.Frame(
            name=f"{first['t'][idx]:.1f}",
            data=[go.Scatter(x=rec["xy"][idx, :, 0], y=rec["xy"][idx, :, 1]) for rec in results.values()],
            traces=dynamic_indices,
            layout=go.Layout(title=f"Native CATK signal probe — t={first['t'][idx]:.1f}s — {state}"),
        ))
    fig.frames = frames
    fig.update_xaxes(range=[-60, 300], title="x (m)", row=1, col=1)
    fig.update_yaxes(range=[-22, 22], title="y (m)", scaleanchor="x", row=1, col=1)
    fig.update_xaxes(title="time (s)", row=2, col=1)
    fig.update_yaxes(title="queue mean speed (m/s)", rangemode="tozero", row=2, col=1)
    fig.update_layout(
        title="Native CATK signal probe",
        width=1050,
        height=700,
        updatemenus=[{"type": "buttons", "buttons": [{"label": "play", "method": "animate", "args": [None, {"frame": {"duration": 65}, "fromcurrent": True}]}, {"label": "pause", "method": "animate", "args": [[None], {"frame": {"duration": 0}, "mode": "immediate"}]}]}],
        sliders=[{"steps": [{"method": "animate", "args": [[frame.name], {"frame": {"duration": 0}, "mode": "immediate"}], "label": frame.name} for frame in frames]}],
    )
    fig.write_html(out, include_plotlyjs=True)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--seconds", type=float, default=30.0)
    parser.add_argument("--red-seconds", type=float, default=5.0)
    parser.add_argument("--topk", type=int, default=1)
    parser.add_argument("--initial-speed", type=float, default=1.5)
    parser.add_argument("--out", type=pathlib.Path, default=pathlib.Path("signal_probe.html"))
    args = parser.parse_args()
    sys.path.insert(0, str(TRAFFICSIM_SRC))
    import torch
    from alpasim_trafficsim.catk.model_adapter import CATK
    from alpasim_trafficsim.catk.obstacle_classes import obstacle_class_metadata

    model = CATK(
        config_path=str(MODELS / "catk_v120/config.yaml"),
        ckpt_path=str(MODELS / "catk_v120/latest.ckpt"),
        token_pkl_dir=str(MODELS / "tokens"),
        disable_sub_plyline_type=True,
        device="cuda",
    )
    model.model.validation_rollout_sampling.num_k = args.topk
    env_map, lanes = _make_map(torch)
    metadata = obstacle_class_metadata()
    results = {}
    for name, initial, state in (("STOP->GO", 1, 2), ("STOP->NONE", 1, 0), ("NONE-only", 0, 0)):
        print(f"\nrolling {name}")
        results[name] = _rollout(
            model,
            torch,
            metadata,
            env_map,
            args.seconds,
            args.red_seconds,
            initial,
            state,
            args.initial_speed,
        )
        _report(name, results[name], args.red_seconds)
    payload = {f"{name.lower().replace('->', '_to_').replace('-', '_')}_{key}": value for name, rec in results.items() for key, value in rec.items()}
    np.savez(args.out.with_suffix(".npz"), red_seconds=args.red_seconds, **payload)
    _visualize(results, lanes, args.red_seconds, args.out)
    print(f"wrote {args.out.with_suffix('.npz')}")
    print(f"wrote {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
