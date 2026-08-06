"""Standalone CATK world-model sandbox -- NO FIXS, NO SUMO.

Purpose (issue #189 troubleshooting): isolate the world model from our
TrafficLayer/SUMO plumbing. We build a simple intersection in CATK's OWN native
env_data format, spawn a handful of cars, and roll the model CLOSED-LOOP for a
few seconds -- feeding each step's prediction back in as the next history frame,
exactly like the co-sim does, but without SUMO in the loop.

If cars flow sanely here (hold speed, take turns, no crawl/teleport) then the
model is fine and the crawl we see in the SUMO co-sim is our implementation. If
they crawl/misbehave here too, it is the model.

One car is the "ego" (driven by a fixed constant-velocity policy, like the
SUMO-owned ego); CATK predicts every other car.

Run:
    C:/src_git/WM_AlpaSim/.venv-catk/Scripts/python.exe wm_sandbox.py \
        --seconds 30 --scenario cross --out sandbox.html
"""
from __future__ import annotations

import argparse
import os
import sys

import numpy as np

ALPASIM_SRC = os.environ.get("ALPASIM_SRC", "C:/src_git/WM_AlpaSim/src/trafficsim")
MODELS = os.environ.get("TRAFFICSIM_MODELS_DIR", "C:/src_git/WM_AlpaSim/data/trafficsim-models")
DT = 0.1
THIST = 16
LANE = 1.75          # half lane offset from road centerline
CARL, CARW = 4.5, 2.0


# --------------------------------------------------------------------------- #
#  scene construction (native CATK env_data)
# --------------------------------------------------------------------------- #
def intersection_lanes(half: float, step: float = 1.0, cross: bool = True):
    """Lane-centre polylines for a +/- straight road, optionally a crossing one."""
    lanes = []
    s = np.arange(-half, half + step, step)
    o = np.full_like(s, LANE)
    # EW road: eastbound (y=-LANE, +x) and westbound (y=+LANE, -x)
    lanes.append(np.stack([s, -o], 1))
    lanes.append(np.stack([s[::-1], o], 1))
    if cross:
        # NS road: northbound (x=+LANE, +y) and southbound (x=-LANE, -y)
        lanes.append(np.stack([o, s], 1))
        lanes.append(np.stack([-o, s[::-1]], 1))
    return lanes


def spawn(scenario: str):
    """Return list of (x0, y0, heading, speed, is_ego). Cars head to the origin."""
    v = 10.0
    cars = []
    # eastbound platoon (this lane carries the ego + followers)
    for j, x in enumerate((-35.0, -50.0, -65.0)):
        cars.append((x, -LANE, 0.0, v, j == 0))          # ego = lead eastbound car
    # westbound platoon
    for x in (35.0, 50.0):
        cars.append((x, LANE, np.pi, v, False))
    if scenario == "cross":
        for y in (-35.0, -50.0):
            cars.append((LANE, y, np.pi / 2, v, False))   # northbound
        for y in (35.0, 50.0):
            cars.append((-LANE, y, -np.pi / 2, v, False))  # southbound
    return cars


def const_vel_history(x0, y0, hd, v, n=THIST):
    """n frames of constant-velocity history ending at (x0,y0) at time now."""
    t = (np.arange(n) - (n - 1)) * DT           # ... -0.2,-0.1,0.0
    xs = x0 + np.cos(hd) * v * t
    ys = y0 + np.sin(hd) * v * t
    xyz = np.zeros((n, 3), np.float32)
    xyz[:, 0] = xs
    xyz[:, 1] = ys
    return xyz


# --------------------------------------------------------------------------- #
#  main rollout
# --------------------------------------------------------------------------- #
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=30.0)
    ap.add_argument("--scenario", choices=["straight", "cross"], default="cross")
    ap.add_argument("--half", type=float, default=200.0, help="map half-extent (m)")
    ap.add_argument("--topk", type=int, default=0, help="0=model default, 1=greedy")
    ap.add_argument("--out", default="sandbox.html")
    args = ap.parse_args()

    sys.path.insert(0, ALPASIM_SRC)
    import torch
    from alpasim_trafficsim.catk.model_adapter import CATK
    from alpasim_trafficsim.catk.obstacle_classes import obstacle_class_metadata

    model = CATK(config_path=f"{MODELS}/catk_v120/config.yaml",
                 ckpt_path=f"{MODELS}/catk_v120/latest.ckpt",
                 token_pkl_dir=f"{MODELS}/tokens",
                 disable_sub_plyline_type=True, device="cuda")
    if args.topk > 0:
        try:
            model.validation_rollout_sampling.num_k = args.topk
        except Exception:
            pass
    meta = obstacle_class_metadata()

    cars = spawn(args.scenario)
    ego_i = next(i for i, c in enumerate(cars) if c[4])
    non_ego = [i for i in range(len(cars)) if i != ego_i]
    N = len(cars)

    # rolling THIST-frame history per car: (N, THIST, 3) + heading (N, THIST)
    hxyz = np.stack([const_vel_history(*c[:4]) for c in cars])          # (N,THIST,3)
    hhead = np.stack([np.full(THIST, c[2], np.float32) for c in cars])  # (N,THIST)
    ego_hd, ego_v = cars[ego_i][2], cars[ego_i][3]

    # map (static)
    lanes = intersection_lanes(args.half, 1.0, cross=(args.scenario == "cross"))
    vmax = max(len(l) for l in lanes)
    marr = np.full((len(lanes), vmax, 3), np.nan, np.float32)
    for i, l in enumerate(lanes):
        marr[i, :len(l), 0:2] = l
        marr[i, :len(l), 2] = 0.0
    map_layer = {"lane_centers": {"polylines": torch.tensor(marr), "label": [5] * len(lanes)}}

    nsteps = int(args.seconds / DT)
    rec = {"t": [], "xy": [], "head": [], "spd": []}   # per-step snapshots

    for step in range(nsteps):
        A = len(non_ego)
        ag_xyz = torch.tensor(hxyz[non_ego], dtype=torch.float32)
        ag_head = torch.tensor(hhead[non_ego], dtype=torch.float32)
        env = {
            "ego": {"xyz": torch.tensor(hxyz[ego_i], dtype=torch.float32),
                    "heading": torch.tensor(hhead[ego_i], dtype=torch.float32),
                    "lwh": torch.tensor([CARL, CARW, 1.6])},
            "agents": {"xyz": ag_xyz, "heading": ag_head,
                       "valid_mask": torch.ones(A, THIST, dtype=torch.bool),
                       "lwh": torch.tensor([[CARL, CARW, 1.6]] * A),
                       "track_ids": torch.arange(A),
                       "class_ids": torch.zeros(A, dtype=torch.long), "num_obstacles": A},
            "map": map_layer,
            "env": {"curr_t": THIST - 1},
            "metadata": {"frame_rate": 10, **meta},
        }
        result = model.create_model_input(env, filter_map_by_ego=True, filter_distance_th=args.half)
        newxyz = hxyz[:, -1].copy()
        newhead = hhead[:, -1].copy()
        if result is not None:
            act = model.inference(result["input_data"])
            fx = act["agent_future_xyz"].detach().cpu().numpy()      # (A,5,3)
            fh = act["agent_future_heading"].detach().cpu().numpy()  # (A,5)
            for k, i in enumerate(non_ego):
                newxyz[i, 0:2] = fx[k, 0, 0:2]
                newhead[i] = fh[k, 0]
        # ego: constant-velocity policy
        newxyz[ego_i, 0] = hxyz[ego_i, -1, 0] + np.cos(ego_hd) * ego_v * DT
        newxyz[ego_i, 1] = hxyz[ego_i, -1, 1] + np.sin(ego_hd) * ego_v * DT
        newhead[ego_i] = ego_hd

        # per-car speed from this step's displacement
        spd = np.hypot(newxyz[:, 0] - hxyz[:, -1, 0], newxyz[:, 1] - hxyz[:, -1, 1]) / DT

        # roll the window
        hxyz = np.concatenate([hxyz[:, 1:], newxyz[:, None]], axis=1)
        hhead = np.concatenate([hhead[:, 1:], newhead[:, None]], axis=1)

        rec["t"].append(step * DT)
        rec["xy"].append(newxyz[:, 0:2].copy())
        rec["head"].append(newhead.copy())
        rec["spd"].append(spd.copy())
        if step % 50 == 0:
            print(f"t={step*DT:5.1f}s  mean|v|={spd[non_ego].mean():5.2f}  "
                  f"min={spd[non_ego].min():4.1f} max={spd[non_ego].max():4.1f}", file=sys.stderr)

    npz = os.path.splitext(args.out)[0] + ".npz"
    np.savez(npz, t=np.array(rec["t"]), xy=np.array(rec["xy"]), head=np.array(rec["head"]),
             spd=np.array(rec["spd"]), ego_i=ego_i,
             lanes=np.array(lanes, dtype=object), N=N)
    print(f"wrote {npz}")
    _report(rec, non_ego, ego_i)
    try:
        _visualize(rec, lanes, ego_i, N, args)
    except ModuleNotFoundError as e:
        print(f"[viz skipped: {e}; run tools/wm_sandbox_viz.py on the .npz]")


def _report(rec, non_ego, ego_i):
    sp = np.array(rec["spd"])          # (steps, N)
    nz = sp[:, non_ego]
    mv = nz[nz > 0.5]
    print(f"\n=== WM sandbox speed summary (non-ego, CATK-driven) ===")
    print(f"steps={len(sp)}  agents={len(non_ego)}")
    print(f"moving speed: p50={np.median(mv):.2f}  p90={np.percentile(mv,90):.1f}  "
          f"max={mv.max():.1f}  frac<3m/s={100*(mv<3).mean():.0f}%")
    print(f"(spawned at 10 m/s; healthy flow keeps ~10; crawl collapses toward 1-2)")


def _visualize(rec, lanes, ego_i, N, args):
    import plotly.graph_objects as go
    T = len(rec["t"])
    stride = max(1, T // 150)                # <=150 frames
    frames_idx = list(range(0, T, stride))
    colors = ["red" if i == ego_i else "royalblue" for i in range(N)]

    def boxes(fi):
        xy = rec["xy"][fi]; hd = rec["head"][fi]
        xs, ys = [], []
        for i in range(N):
            c, s = np.cos(hd[i]), np.sin(hd[i])
            corners = np.array([[CARL/2, CARW/2], [CARL/2, -CARW/2],
                                [-CARL/2, -CARW/2], [-CARL/2, CARW/2], [CARL/2, CARW/2]])
            R = np.array([[c, -s], [s, c]])
            pc = corners @ R.T + xy[i]
            xs += list(pc[:, 0]) + [None]; ys += list(pc[:, 1]) + [None]
        return xs, ys

    fig = go.Figure()
    for l in lanes:
        fig.add_trace(go.Scatter(x=l[:, 0], y=l[:, 1], mode="lines",
                                 line=dict(color="lightgray", width=1), showlegend=False, hoverinfo="skip"))
    x0, y0 = boxes(0)
    fig.add_trace(go.Scatter(x=x0, y=y0, mode="lines", fill="toself",
                             line=dict(color="royalblue"), name="cars"))
    frames = []
    for fi in frames_idx:
        xs, ys = boxes(fi)
        frames.append(go.Frame(data=[go.Scatter(x=xs, y=ys, mode="lines", fill="toself",
                               line=dict(color="royalblue"))],
                               name=f"{rec['t'][fi]:.1f}",
                               layout=go.Layout(title=f"t={rec['t'][fi]:.1f}s  (red=ego, blue=CATK)")))
    fig.frames = frames
    R = 70
    fig.update_layout(
        title="WM sandbox (no SUMO): 4-way intersection, CATK closed-loop",
        xaxis=dict(range=[-R, R], title="x (m)"), yaxis=dict(range=[-R, R], scaleanchor="x"),
        width=760, height=780,
        updatemenus=[dict(type="buttons", buttons=[
            dict(label="play", method="animate",
                 args=[None, {"frame": {"duration": 60}, "fromcurrent": True}]),
            dict(label="pause", method="animate",
                 args=[[None], {"frame": {"duration": 0}, "mode": "immediate"}])])],
        sliders=[dict(steps=[dict(method="animate", args=[[f.name],
                 {"frame": {"duration": 0}, "mode": "immediate"}], label=f.name) for f in frames])])
    fig.write_html(args.out, include_plotlyjs=True)
    print(f"wrote {args.out}")


if __name__ == "__main__":
    main()
