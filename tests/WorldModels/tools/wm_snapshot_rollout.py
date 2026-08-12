"""Snapshot vanilla SUMO at a junction, then roll CATK forward with NO SUMO (#189).

Debug loop for the world model itself, decoupled from the co-simulation:

  1. take a SNAPSHOT of a vanilla SUMO run (FCD) at the moment the ego is near a
     chosen junction -- every vehicle's 16-frame history, the ego's logged future
     as the pinned plan, and the true signal state from SUMO's TLS log;
  2. roll CATK CLOSED-LOOP from that snapshot for N seconds, feeding its own
     predictions back as history. SUMO is not running and nothing is written
     back to it;
  3. flag the failure modes directly -- teleports, orientation flips, off-lane
     excursions, red-light violations -- and write a plotly clip with oriented
     boxes, the lane map, and the live signal colour.

Because it replays a fixed snapshot it is deterministic per seed and takes
seconds, so model/input changes can be judged without a co-sim run.

Run (CATK venv, from tests/WorldModels/tools):
    python wm_snapshot_rollout.py --seconds 20 --seed 0 --out snap.html
"""
from __future__ import annotations

import argparse
import os
import pathlib
import re
import sys
import time

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))   # tests/WorldModels
ALPASIM_SRC = os.environ.get("ALPASIM_SRC", "C:/src_git/WM_AlpaSim/src/trafficsim")
MODELS = os.environ.get("TRAFFICSIM_MODELS_DIR", "C:/src_git/WM_AlpaSim/data/trafficsim-models")

DT = 0.1
HISTORY = 16
PREDICT = 5
CAR_L, CAR_W = 5.0, 1.8          # SUMO default passenger vehicle
JUNCTIONS = {"int_east": (1073.0, 300.0), "int_center": (673.0, 300.0), "int_west": (272.9, 300.0)}


def sumo_deg_to_math_rad(deg: np.ndarray | float):
    """SUMO angle (0 = north, clockwise) -> math heading (0 = +x, ccw)."""
    return np.radians(90.0 - np.asarray(deg, dtype=np.float64))


# --------------------------------------------------------------------------- #
#  inputs
# --------------------------------------------------------------------------- #
def read_fcd(path: str) -> dict[float, dict[str, tuple]]:
    """{time: {veh_id: (x, y, angle_deg, speed)}}"""
    ts_re = re.compile(r'<timestep time="([0-9.]+)"')
    veh_re = re.compile(r'<vehicle id="([^"]+)"[^>]*?x="([0-9.\-]+)"[^>]*?y="([0-9.\-]+)"'
                        r'[^>]*?angle="([0-9.\-]+)"[^>]*?speed="([0-9.\-]+)"')
    out: dict[float, dict[str, tuple]] = {}
    t = 0.0
    for line in open(path, encoding="utf-8", errors="ignore"):
        m = ts_re.search(line)
        if m:
            t = round(float(m.group(1)), 1)
            out[t] = {}
            continue
        v = veh_re.search(line)
        if v:
            out[t][v.group(1)] = (float(v.group(2)), float(v.group(3)),
                                  float(v.group(4)), float(v.group(5)))
    return out


def read_tls(path: str, tls_id: str = "int_east") -> list[tuple[float, str]]:
    """[(time, state_string)] for one traffic light."""
    out = []
    for line in open(path, encoding="utf-8", errors="ignore"):
        m = re.search(r'time="([0-9.]+)"[^>]*state="([^"]+)"', line)
        if m and (f'id="{tls_id}"' in line or tls_id in line):
            out.append((float(m.group(1)), m.group(2)))
    return out


def tls_at(states: list[tuple[float, str]], t: float) -> str:
    s = states[0][1] if states else ""
    for tt, st in states:
        if tt <= t:
            s = st
        else:
            break
    return s


def pick_snapshot_time(fcd, junction_xy, ego_id="ego", radius=70.0,
                       after=HISTORY * DT + 1.0) -> float | None:
    """First time the ego is within `radius` of the junction with enough history."""
    jx, jy = junction_xy
    for t in sorted(fcd):
        if t < after:
            continue
        v = fcd[t].get(ego_id)
        if v and np.hypot(v[0] - jx, v[1] - jy) <= radius:
            return t
    return None


def build_snapshot(fcd, t0: float, ego_id: str, half_extent: float):
    """16 frames of history for every vehicle in the bubble + the ego's plan."""
    times = [round(t0 - (HISTORY - 1 - k) * DT, 1) for k in range(HISTORY)]
    if any(t not in fcd for t in times):
        raise SystemExit(f"snapshot at t={t0}: missing history frames in FCD")
    ex, ey = fcd[t0][ego_id][0], fcd[t0][ego_id][1]
    ids = [vid for vid, s in fcd[t0].items()
           if abs(s[0] - ex) <= half_extent and abs(s[1] - ey) <= half_extent
           and all(vid in fcd[t] for t in times)]
    ids.sort()
    xyz = np.zeros((len(ids), HISTORY, 3), np.float32)
    head = np.zeros((len(ids), HISTORY), np.float32)
    for i, vid in enumerate(ids):
        for k, t in enumerate(times):
            x, y, a, _ = fcd[t][vid]
            xyz[i, k, 0], xyz[i, k, 1] = x, y
            head[i, k] = sumo_deg_to_math_rad(a)
    # ego plan: its logged future (what the controller would command)
    plan_t = [round(t0 + (k + 1) * DT, 1) for k in range(PREDICT)]
    plan = np.zeros((PREDICT, 3), np.float32)
    for k, t in enumerate(plan_t):
        src = fcd.get(t, {}).get(ego_id) or fcd[t0][ego_id]
        plan[k, 0], plan[k, 1] = src[0], src[1]
        plan[k, 2] = sumo_deg_to_math_rad(src[2])
    return ids, xyz, head, plan


# --------------------------------------------------------------------------- #
#  rollout
# --------------------------------------------------------------------------- #
def rollout(ids, hxyz, hhead, ego_idx, fcd, t0, catk_map, model, torch,
            seconds, seed, stride, crop_r, ego_id,
            light_pts=None, tls_states=None, light_link=4, apply_caps=True):
    from alpasim_trafficsim.catk.obstacle_classes import obstacle_class_metadata
    torch.manual_seed(seed)
    np.random.seed(seed)
    meta = obstacle_class_metadata()

    others = [i for i in range(len(ids)) if i != ego_idx]
    A = len(others)
    n_step = int(round(seconds / DT))
    rec = {"t": [], "xy": [], "head": [], "spd": []}
    timing = {"crop": 0.0, "prep": 0.0, "infer": 0.0, "n": 0}

    step = 0
    while step < n_step:
        now = round(t0 + step * DT, 1)
        ax = torch.tensor(np.concatenate([hxyz[others],
                          np.repeat(hxyz[others][:, -1:], PREDICT, axis=1)], axis=1))
        ah = torch.tensor(np.concatenate([hhead[others],
                          np.repeat(hhead[others][:, -1:], PREDICT, axis=1)], axis=1))
        # ego: history + logged future plan
        ego_hist = hxyz[ego_idx]
        plan = np.zeros((PREDICT, 3), np.float32)
        ph = np.zeros(PREDICT, np.float32)
        for k in range(PREDICT):
            src = fcd.get(round(now + (k + 1) * DT, 1), {}).get(ego_id)
            if src is None:
                plan[k, :2] = ego_hist[-1, :2]
                ph[k] = hhead[ego_idx, -1]
            else:
                plan[k, 0], plan[k, 1] = src[0], src[1]
                ph[k] = sumo_deg_to_math_rad(src[2])
        ego_xyz = torch.tensor(np.concatenate([ego_hist, plan], axis=0))
        ego_head = torch.tensor(np.concatenate([hhead[ego_idx], ph], axis=0))

        a = time.time()
        anchors = np.vstack([ego_hist[-1:, :2], hxyz[others][:, -1, :2]])
        step_map = catk_map.crop(anchors, crop_r, torch, apply_caps=apply_caps)
        timing["crop"] += time.time() - a

        a = time.time()
        env = {"ego": {"xyz": ego_xyz, "heading": ego_head,
                       "lwh": torch.tensor([CAR_L, CAR_W, 1.6])},
               "agents": {"xyz": ax, "heading": ah,
                          "valid_mask": torch.cat([torch.ones(A, HISTORY, dtype=torch.bool),
                                                   torch.zeros(A, PREDICT, dtype=torch.bool)], 1),
                          "lwh": torch.tensor([[CAR_L, CAR_W, 1.6]] * A),
                          "track_ids": torch.arange(A),
                          "class_ids": torch.zeros(A, dtype=torch.long), "num_obstacles": A},
               "map": step_map, "env": {"curr_t": HISTORY - 1},
               "metadata": {"frame_rate": 10, **meta}}
        res = model.create_model_input(env, filter_map_by_ego=False, filter_distance_th=0.0)
        timing["prep"] += time.time() - a
        if res is None:
            break
        if light_pts:
            # upstream CATK enum: NO_LANE_STATE=0 UNKNOWN=1 STOP=2 GO=3 CAUTION=4
            st = tls_at(tls_states, now)
            pe = res["input_data"]["map"]["polyline_extras"]
            ctr = res["input_data"]["map"]["triplets"].float().mean(dim=1)
            lt = pe["light_type"].clone()
            painted = 0
            for pts, link in light_pts:
                ch = st[link] if len(st) > link else "r"
                state = 2 if ch == "r" else (4 if ch == "y" else 3)
                lp = torch.tensor(pts, dtype=ctr.dtype, device=ctr.device)
                near = ((ctr[:, None, :] - lp[None, :, :]) ** 2).sum(-1).min(1).values < 4.0
                lt[near] = state
                painted += int(near.sum())
            pe["light_type"] = lt
            if step == 0:
                print(f"  light painting: {painted} of {len(ctr)} map tokens tagged "
                      f"(state string '{st}')")
        a = time.time()
        act = model.inference(res["input_data"])
        timing["infer"] += time.time() - a
        timing["n"] += 1

        fx = act["agent_future_xyz"].detach().cpu().numpy()
        fh = act["agent_future_heading"].detach().cpu().numpy()
        nsub = min(stride, PREDICT, n_step - step)
        for s in range(nsub):
            new_xyz = hxyz[:, -1].copy()
            new_head = hhead[:, -1].copy()
            for k, i in enumerate(others):
                new_xyz[i, :2] = fx[k, s, :2]
                new_head[i] = fh[k, s]
            t_ego = round(now + (s + 1) * DT, 1)
            src = fcd.get(t_ego, {}).get(ego_id)
            if src is not None:                       # ego stays on its logged path
                new_xyz[ego_idx, :2] = (src[0], src[1])
                new_head[ego_idx] = sumo_deg_to_math_rad(src[2])
            spd = np.linalg.norm(new_xyz[:, :2] - hxyz[:, -1, :2], axis=1) / DT
            hxyz = np.concatenate([hxyz[:, 1:], new_xyz[:, None]], axis=1)
            hhead = np.concatenate([hhead[:, 1:], new_head[:, None]], axis=1)
            rec["t"].append(round(now + (s + 1) * DT, 1))
            rec["xy"].append(new_xyz[:, :2].copy())
            rec["head"].append(new_head.copy())
            rec["spd"].append(spd.copy())
        step += nsub
    for k in ("t", "xy", "head", "spd"):
        rec[k] = np.asarray(rec[k])
    return rec, timing


# --------------------------------------------------------------------------- #
#  behaviour checks
# --------------------------------------------------------------------------- #
def controlled_lanes(net_file: str, junction: str = "int_east", spacing: float = 1.0):
    """[(points, linkIndex)] for EVERY lane controlled by `junction`.

    Upstream CATK paints the signal state on every map token of the controlled
    lane (data_preprocess.get_map_features), so we need the full lane geometry --
    the approach lane AND the internal lane through the junction -- resampled
    densely enough to cover it. Each entry carries its own linkIndex so the state
    char for that specific movement is used.

    (Earlier version painted only linkIndex 4 and did not resample: 2 points on a
    113 m lane, in a lane the queue was not even in. Nothing got painted.)
    """
    txt = open(net_file, encoding="utf-8").read()
    shapes = {m.group(1): np.array([tuple(map(float, q.split(","))) for q in m.group(2).split()])
              for m in re.finditer(r'<lane id="([^"]*)"[^>]*shape="([^"]+)"', txt)}

    def resample(p):
        p = np.asarray(p, dtype=np.float64)
        d = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(p, axis=0), axis=1))]
        if d[-1] <= 0:
            return p
        n = max(2, int(np.ceil(d[-1] / spacing)) + 1)
        t = np.linspace(0.0, d[-1], n)
        return np.column_stack([np.interp(t, d, p[:, 0]), np.interp(t, d, p[:, 1])])

    out = []
    for m in re.finditer(r'<connection from="([^"]+)" to="([^"]+)" fromLane="(\d+)" toLane="(\d+)"'
                         r'(?: via="([^"]*)")? tl="' + junction + r'" linkIndex="(\d+)"', txt):
        frm, _to, flane, _tlane, via, link = m.groups()
        link = int(link)
        for lid in (f"{frm}_{flane}", via):
            if lid and lid in shapes and len(shapes[lid]) >= 2:
                out.append((resample(shapes[lid]), link))
    return out


def lane_segments(net_file: str):
    txt = open(net_file, encoding="utf-8").read()
    S0, S1 = [], []
    for m in re.finditer(r'<lane id="([^"]*)"[^>]*shape="([^"]+)"', txt):
        p = np.array([tuple(map(float, q.split(","))) for q in m.group(2).split()])
        for i in range(len(p) - 1):
            S0.append(p[i]); S1.append(p[i + 1])
    return np.asarray(S0), np.asarray(S1)


def check(rec, ids, ego_idx, S0, S1, vmax=25.0):
    d = S1 - S0
    dd = (d * d).sum(1) + 1e-9

    def offlane(p):
        t = np.clip(((p - S0) * d).sum(1) / dd, 0, 1)
        return np.sqrt((((S0 + t[:, None] * d) - p) ** 2).sum(1)).min()

    print(f"\n{'vehicle':16s} {'maxJump':>8} {'maxYaw/step':>12} {'offMax':>7} {'meanV':>6} verdict")
    bad = 0
    for i, vid in enumerate(ids):
        if i == ego_idx:
            continue
        xy = rec["xy"][:, i]
        hd = np.unwrap(rec["head"][:, i])
        jump = np.linalg.norm(np.diff(xy, axis=0), axis=1)
        yaw = np.abs(np.degrees(np.diff(hd)))
        off = max(offlane(p) for p in xy[::3])
        flags = []
        if jump.max() > vmax * DT * 1.05:
            flags.append(f"TELEPORT {jump.max():.1f}m")
        if yaw.max() > 25:
            flags.append(f"YAW {yaw.max():.0f}deg/step")
        if off > 2.0:
            flags.append(f"OFFLANE {off:.1f}m")
        bad += bool(flags)
        print(f"{vid:16s} {jump.max():8.2f} {yaw.max():12.1f} {off:7.2f} "
              f"{rec['spd'][:, i].mean():6.2f} {' '.join(flags) if flags else 'ok'}")
    print(f"\n{bad}/{len(ids)-1} vehicles flagged")
    return bad


def red_light_check(rec, ids, ego_idx, tls_states, stop_x=1080.15, lane_y=(299, 308), link=4):
    """Vehicles crossing the int_east westbound stop line while its light is red."""
    viol = []
    for i, vid in enumerate(ids):
        if i == ego_idx:
            continue
        xy = rec["xy"][:, i]
        for k in range(1, len(xy)):
            if lane_y[0] < xy[k, 1] < lane_y[1] and xy[k - 1, 0] >= stop_x > xy[k, 0]:
                st = tls_at(tls_states, rec["t"][k])
                if len(st) > link and st[link] in "ry":
                    viol.append((vid, rec["t"][k], st[link]))
                break
    print(f"\nred-light violations at int_east westbound: {len(viol)}")
    for vid, t, s in viol:
        print(f"   {vid} crossed at t={t:.1f}s while light='{s}'")
    return viol


# --------------------------------------------------------------------------- #
def visualize(rec, ids, ego_idx, net_file, tls_states, out, title):
    import plotly.graph_objects as go
    txt = open(net_file, encoding="utf-8").read()
    lanes = [np.array([tuple(map(float, q.split(","))) for q in m.group(1).split()])
             for m in re.finditer(r'<lane id="[^"]*"[^>]*shape="([^"]+)"', txt)]
    cx, cy = rec["xy"][0].mean(0)
    R = 120.0

    def boxes(k):
        xs, ys = [], []
        for i in range(len(ids)):
            c, s = np.cos(rec["head"][k, i]), np.sin(rec["head"][k, i])
            corn = np.array([[CAR_L/2, CAR_W/2], [CAR_L/2, -CAR_W/2],
                             [-CAR_L/2, -CAR_W/2], [-CAR_L/2, CAR_W/2], [CAR_L/2, CAR_W/2]])
            p = corn @ np.array([[c, -s], [s, c]]).T + rec["xy"][k, i]
            xs += list(p[:, 0]) + [None]; ys += list(p[:, 1]) + [None]
        return xs, ys

    fig = go.Figure()
    for l in lanes:
        if (abs(l[:, 0] - cx) < R + 60).any() and (abs(l[:, 1] - cy) < R + 60).any():
            fig.add_trace(go.Scattergl(x=l[:, 0], y=l[:, 1], mode="lines",
                                       line=dict(color="#d1d5db", width=3),
                                       showlegend=False, hoverinfo="skip"))
    x0, y0 = boxes(0)
    fig.add_trace(go.Scatter(x=x0, y=y0, mode="lines", fill="toself",
                             line=dict(color="#2563eb"), name="CATK vehicles"))
    ex = rec["xy"][:, ego_idx]
    fig.add_trace(go.Scatter(x=[ex[0, 0]], y=[ex[0, 1]], mode="markers",
                             marker=dict(color="#dc2626", size=13, symbol="star"), name="ego (SUMO log)"))
    frames = []
    stride = max(1, len(rec["t"]) // 200)
    for k in range(0, len(rec["t"]), stride):
        xs, ys = boxes(k)
        st = tls_at(tls_states, rec["t"][k])
        lit = st[4] if len(st) > 4 else "?"
        frames.append(go.Frame(name=f"{rec['t'][k]:.1f}",
                               data=[go.Scatter(x=xs, y=ys),
                                     go.Scatter(x=[rec["xy"][k, ego_idx, 0]],
                                                y=[rec["xy"][k, ego_idx, 1]])],
                               traces=[len(fig.data) - 2, len(fig.data) - 1],
                               layout=go.Layout(title=f"{title} — t={rec['t'][k]:.1f}s — "
                                                      f"int_east WB light '{lit}'")))
    fig.frames = frames
    fig.update_layout(title=title, width=1000, height=800,
                      xaxis=dict(range=[cx - R, cx + R], title="x (m)"),
                      yaxis=dict(range=[cy - R, cy + R], scaleanchor="x", title="y (m)"),
                      updatemenus=[dict(type="buttons", buttons=[
                          dict(label="play", method="animate",
                               args=[None, {"frame": {"duration": 60}, "fromcurrent": True}]),
                          dict(label="pause", method="animate",
                               args=[[None], {"frame": {"duration": 0}, "mode": "immediate"}])])],
                      sliders=[dict(steps=[dict(method="animate", args=[[f.name],
                               {"frame": {"duration": 0}, "mode": "immediate"}], label=f.name)
                               for f in frames])])
    fig.write_html(out, include_plotlyjs=True)
    print(f"wrote {out}")


def main() -> int:
    ap = argparse.ArgumentParser()
    demo = pathlib.Path(__file__).resolve().parents[1] / "Demo"
    ap.add_argument("--fcd", default=str(demo / "fcd_van320.xml"))
    ap.add_argument("--tls", default=str(demo / "tls_state.xml"))
    ap.add_argument("--net", default=str(demo / "simple_traffic_light.net.xml"))
    ap.add_argument("--junction", default="int_east", choices=list(JUNCTIONS))
    ap.add_argument("--at", type=float, default=None, help="snapshot time (default: auto)")
    ap.add_argument("--ego", default="ego")
    ap.add_argument("--half-extent", type=float, default=100.0)
    ap.add_argument("--crop-radius", type=float, default=25.0)
    ap.add_argument("--seconds", type=float, default=20.0)
    ap.add_argument("--stride", type=int, default=3)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--no-map-cap", action="store_true",
                    help="disable the per-group map segment caps (reference _map_near has none)")
    ap.add_argument("--creep", type=float, default=0.0, metavar="V",
                    help="rewrite STOPPED vehicles' history as a constant creep of V m/s "
                         "ending at the observed pose (ClipGT queues that launch show "
                         "0.2-0.3 m/s residual; SUMO stops at exactly 0)")
    ap.add_argument("--jitter", type=float, default=0.0, metavar="SIGMA_M",
                    help="correlated micro-motion added to STOPPED vehicles' history "
                         "(real stopped cars move ~0.02-0.07 m/frame; SUMO stops are exactly 0)")
    ap.add_argument("--light", action="store_true",
                    help="paint the true SUMO signal state onto the controlled lane's map tokens")
    ap.add_argument("--out", default="snapshot_rollout.html")
    args = ap.parse_args()

    sys.path.insert(0, ALPASIM_SRC)
    import torch
    from alpasim_trafficsim.catk.model_adapter import CATK
    from catk_map import CatkMap

    fcd = read_fcd(args.fcd)
    tls = read_tls(args.tls, args.junction)
    t0 = args.at if args.at is not None else pick_snapshot_time(fcd, JUNCTIONS[args.junction], args.ego)
    if t0 is None:
        raise SystemExit(f"ego never comes within range of {args.junction} in {args.fcd}")
    ids, hxyz, hhead, _ = build_snapshot(fcd, t0, args.ego, args.half_extent)
    ego_idx = ids.index(args.ego)
    if getattr(args, "creep", 0.0) > 0:
        moved = np.linalg.norm(hxyz[:, -1, :2] - hxyz[:, 0, :2], axis=1)
        stopped = [i for i in np.flatnonzero(moved < 0.5) if i != ego_idx]
        back = args.creep * DT * np.arange(HISTORY - 1, -1, -1)     # distance still to travel
        for i in stopped:
            hd = hhead[i, -1]
            hxyz[i, :, 0] = (hxyz[i, -1, 0] - back * np.cos(hd)).astype(np.float32)
            hxyz[i, :, 1] = (hxyz[i, -1, 1] - back * np.sin(hd)).astype(np.float32)
        print(f"creep {args.creep} m/s applied to {len(stopped)} stopped vehicles")
    if args.jitter > 0:
        rng = np.random.default_rng(args.seed)
        moved = np.linalg.norm(hxyz[:, -1, :2] - hxyz[:, 0, :2], axis=1)
        stopped = np.flatnonzero(moved < 0.5)
        stopped = stopped[stopped != ego_idx]
        for i in stopped:                      # correlated random walk along the heading
            hd = hhead[i, -1]
            step = rng.normal(0.0, args.jitter, HISTORY).cumsum()
            step -= step[-1]                   # end exactly at the observed pose
            hxyz[i, :, 0] += (step * np.cos(hd)).astype(np.float32)
            hxyz[i, :, 1] += (step * np.sin(hd)).astype(np.float32)
            hhead[i, :] += rng.normal(0.0, np.radians(0.5), HISTORY).astype(np.float32)
        print(f"jitter {args.jitter} m applied to {len(stopped)} stopped vehicles")
    print(f"snapshot t={t0:.1f}s at {args.junction}: {len(ids)} vehicles "
          f"(ego + {len(ids)-1} CATK-driven), light='{tls_at(tls, t0)[4] if tls else '?'}'")

    catk_map = CatkMap.build(args.net)
    model = CATK(config_path=f"{MODELS}/catk_v120/config.yaml",
                 ckpt_path=f"{MODELS}/catk_v120/latest.ckpt",
                 token_pkl_dir=f"{MODELS}/tokens",
                 disable_sub_plyline_type=True, device="cuda")
    light_pts = controlled_lanes(args.net, args.junction) if args.light else None
    if args.light:
        print(f"signal painting ON: {len(light_pts)} controlled lanes, "
              f"{sum(len(p) for p, _ in light_pts)} points")
    rec, timing = rollout(ids, hxyz, hhead, ego_idx, fcd, t0, catk_map, model, torch,
                          args.seconds, args.seed, args.stride, args.crop_radius, args.ego,
                          light_pts, tls, apply_caps=not args.no_map_cap)
    n = max(1, timing["n"])
    print(f"rolled {rec['t'][-1]-t0+DT:.1f}s in {timing['n']} inferences  "
          f"(crop {timing['crop']/n*1000:.0f} ms, prep {timing['prep']/n*1000:.0f} ms, "
          f"infer {timing['infer']/n*1000:.0f} ms per call)")

    S0, S1 = lane_segments(args.net)
    check(rec, ids, ego_idx, S0, S1)
    red_light_check(rec, ids, ego_idx, tls)
    visualize(rec, ids, ego_idx, args.net, tls, args.out,
              f"CATK offline rollout from vanilla snapshot t={t0:.1f}s @ {args.junction}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
