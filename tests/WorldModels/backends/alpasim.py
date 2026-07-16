"""
NVIDIA AlpaSim (CATK/SMART) world-model backend (#189).

Maps the generic BEV agent-set <-> CATK's ``env_data`` dict and runs one
tokenized reactive-traffic inference step (PyTorch). Bypasses AlpaSim's
trajdata/map_adapter (and its non-Windows ``deltalake`` dep) by building
``env_data`` directly from the FIXS bubble + a SUMO-derived vector map.

CATK is stateful: it needs >=16 frames of history per agent and the ego's
near-future as a pinned trajectory (it predicts the surrounding agents reacting
to that ego plan, and returns NON-ego agents only). During the ~1.6 s warm-up
(before the history ring is full) we fall back to constant-velocity so nothing
freezes.

Runs in a torch env (see run_with_wm_alpasim.bat); imports nothing heavy at
module load, so the registry stays importable in the plain realsim env.
"""

from __future__ import annotations

import os
import sys

from bev import AgentPose, BevAgentSet, math_rad_to_sumo_deg
from backends.constant_velocity import ConstantVelocityBackend

ALPASIM_SRC = os.environ.get(
    "ALPASIM_SRC", r"C:/src_git/WM_AlpaSim/src/trafficsim"
)


def _backfill(win, dt):
    """Fill NaN (pre-entry) frames in a (N,T,4)=[x,y,yaw,speed] window with a
    *moving* reconstruction: leading NaNs are extrapolated BACKWARD along the
    first valid frame's heading at its speed (a straight const-velocity past),
    internal gaps hold the previous frame. Keeps the history continuous AND
    moving, so CATK sees the agent's real velocity on entry and predicts
    continuation -- hold-stationary backfill made CATK freeze the agent instead.
    """
    import numpy as np
    win = np.asarray(win, dtype=np.float64).copy()
    N, T, _ = win.shape
    for n in range(N):
        valid = ~np.isnan(win[n, :, 0])
        if not valid.any():
            win[n] = 0.0
            continue
        f = int(np.where(valid)[0][0])
        xf, yf, yawf, spdf = win[n, f]
        for t in range(f):                       # leading: backward const-velocity
            back = (f - t) * dt * spdf
            win[n, t, 0] = xf - back * np.cos(yawf)
            win[n, t, 1] = yf - back * np.sin(yawf)
            win[n, t, 2] = yawf
            win[n, t, 3] = spdf
        for t in range(f + 1, T):                # internal gaps: hold previous
            if np.isnan(win[n, t, 0]):
                win[n, t] = win[n, t - 1]
    return win


class AlpaSimBackend:
    """CATK/SMART reactive-traffic backend."""

    def reset(self, cfg: dict, vector_map=None) -> None:
        self.device = cfg.get("Device", "cuda")
        self.dt = float(cfg.get("StepLength", 0.1))
        self.num_history = int(cfg.get("HistoryFrames", 16))
        self.predict = int(cfg.get("EgoFutureHorizon", 5))
        self.half = float(cfg.get("BevHalfExtent", 200.0))
        self._light_radius = float(cfg.get("SignalLightRadius", 10.0))
        # Feed CATK an EGO-CENTERED scene (subtract the ego position). WOMD/SMART
        # is trained SDC-centered; absolute SUMO coords (~600-1300 m) are far
        # out-of-distribution and hurt stability. Outputs are shifted back.
        self.ego_center = bool(cfg.get("EgoCenter", True))
        self.vector_map = vector_map

        model_dir = cfg["ModelDir"]
        if ALPASIM_SRC not in sys.path:
            sys.path.insert(0, ALPASIM_SRC)
        import torch  # noqa: F401
        from alpasim_trafficsim.catk.model_adapter import CATK
        from alpasim_trafficsim.catk.obstacle_classes import obstacle_class_metadata

        self._torch = torch
        self.model = CATK(
            config_path=os.path.join(model_dir, "config.yaml"),
            ckpt_path=os.path.join(model_dir, "latest.ckpt"),
            token_pkl_dir=cfg.get("TokenDir", os.path.join(os.path.dirname(model_dir), "tokens")),
            disable_sub_plyline_type=True,
            device=self.device,
        )
        self.model.model_predict_step_num = self.predict
        self._meta = obstacle_class_metadata()

        # warm-up / degenerate fallback
        self._fallback = ConstantVelocityBackend()
        self._fallback.reset(cfg)

    # ------------------------------------------------------------------ #
    def _full_map_layers(self, ox: float, oy: float, k: int = 20) -> dict:
        """All lane polylines as a single CATK 'lane_centers' layer.

        CATK's filter_map + triplet extraction need FIXED-LENGTH, NaN-free
        polylines (NaN padding corrupts the per-polyline distance filter and the
        3-vertex-window validity check -> empty map). So each lane is resampled
        to ``k`` points by arc length. create_model_input(filter_map_by_ego=True)
        then crops by ego distance, so we hand it the whole (parsed-once) map.
        """
        if self.vector_map is None or not self.vector_map.lanes:
            return {}
        import numpy as np

        def resample(pts, kk):
            pts = np.asarray(pts, dtype=np.float64)
            seg = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(pts, axis=0), axis=1))]
            if seg[-1] == 0:
                return np.repeat(pts[:1], kk, axis=0)
            t = np.linspace(0.0, seg[-1], kk)
            return np.column_stack([np.interp(t, seg, pts[:, 0]),
                                    np.interp(t, seg, pts[:, 1])])

        polys = self.vector_map.lanes
        arr = np.zeros((len(polys), k, 3), dtype=np.float64)
        for i, p in enumerate(polys):
            rp = resample(p, k)
            rp[:, 0] -= ox                  # ego-center the map too
            rp[:, 1] -= oy
            arr[i, :, 0:2] = rp
        return {"lane_centers": {"polylines": self._torch.tensor(arr, dtype=self._torch.float32),
                                 "label": [5] * len(polys)}}

    def _inject_light_type(self, result, bev, ox: float = 0.0, oy: float = 0.0) -> None:
        """Tag CATK map triplets near a red/yellow stop-line with light_type=STOP.

        SMART's map encoder has a 5-state traffic-light embedding but the env_data
        adapter zeros it, so the model never sees signals. We derive stop-lines
        from each bubble vehicle's next-signal (pos + dist along heading) and set
        the STOP index (1, per SMART's ['NONE','STOP','GO','CAUTION','GO_TO_STOP'])
        on map triplets within light_radius of one. Experiment: does CATK then
        slow/stop for the red on its own?
        """
        import numpy as np
        torch = self._torch
        stops = []
        for i in range(len(bev.ids)):
            c = int(bev.sig_color[i])
            d = float(bev.sig_dist[i])
            if c in (1, 2) and d >= 0.0:                 # red or yellow ahead
                yaw = float(bev.yaw[i])
                stops.append((bev.xy[i, 0] + d * np.cos(yaw) - ox,   # ego-centered frame
                              bev.xy[i, 1] + d * np.sin(yaw) - oy))
        if not stops:
            return
        pe = result["input_data"]["map"]["polyline_extras"]
        tri = result["input_data"]["map"]["triplets"]     # (M, 3, 2)
        centers = tri.float().mean(dim=1)                 # (M, 2)
        stops_t = torch.tensor(stops, dtype=centers.dtype, device=centers.device)
        d2 = ((centers[:, None, :] - stops_t[None, :, :]) ** 2).sum(-1)  # (M, S)
        near = d2.min(dim=1).values < (self._light_radius ** 2)
        lt = pe["light_type"].clone()
        lt[near] = 1                                      # STOP
        pe["light_type"] = lt

    def _dump_bev(self, bev) -> None:
        """Append one ego-centered BEV snapshot of EXACTLY the converted CATK
        input, for manual verification. Self-contained (built from bev + its
        history + the vector map), so it fires EVERY step -- including warm-up and
        fallback -- giving a CONTIGUOUS 10 Hz sequence (no skipped frames).

        Per frame: sim time, ego (heading + pinned future plan), each agent's
        history tail (x,y) + heading + speed, the local road lane polylines, and
        the red/yellow stop-line points that get light_type=STOP.
        """
        import json
        import numpy as np

        ego_i = bev.ego_idx
        if ego_i < 0 or len(bev.history) == 0:
            return
        ox, oy = ((float(bev.xy[ego_i, 0]), float(bev.xy[ego_i, 1]))
                  if self.ego_center else (0.0, 0.0))
        win = _backfill(bev.history.window(bev.ids), self.dt)   # (N, Thist, 4), ego-centered below
        win[:, :, 0] -= ox
        win[:, :, 1] -= oy
        idx = [i for i in range(len(bev.ids)) if i != ego_i]

        lanes = []
        if self.vector_map is not None:
            for p in self.vector_map.crop(ox, oy, self.half * 1.3):
                q = np.asarray(p, float).copy()
                q[:, 0] -= ox
                q[:, 1] -= oy
                lanes.append([[round(float(x), 1), round(float(y), 1)] for x, y in q])

        stops = []
        for i in range(len(bev.ids)):
            c = int(bev.sig_color[i])
            d = float(bev.sig_dist[i])
            if c in (1, 2) and d >= 0.0:                          # red/yellow ahead
                yaw = float(bev.yaw[i])
                stops.append([round(float(bev.xy[i, 0] + d * np.cos(yaw) - ox), 1),
                              round(float(bev.xy[i, 1] + d * np.sin(yaw) - oy), 1)])

        ego_future = []
        if bev.ego_future is not None:
            for x, y, yw in bev.ego_future:
                ego_future.append([round(float(x) - ox, 2), round(float(y) - oy, 2)])

        frame = {
            "t": round(float(bev.sim_time), 1),
            "ego": {"yaw": round(float(win[ego_i, -1, 2]), 3), "future": ego_future},
            "agents": [{
                "id": bev.ids[i],
                "traj": [[round(float(x), 1), round(float(y), 1)] for x, y in win[i, :, 0:2]],
                "yaw": round(float(win[i, -1, 2]), 3),
                "speed": round(float(win[i, -1, 3]), 2),
            } for i in idx],
            "lanes": lanes,
            "stops": stops,
        }
        with open(os.environ.get("RS_WM_BEV_DUMP", "bev_dump.jsonl"), "a", encoding="utf-8") as f:
            f.write(json.dumps(frame) + "\n")

    def step(self, bev: BevAgentSet) -> dict[str, AgentPose]:
        if os.environ.get("RS_WM_BEV_DUMP"):
            self._dump_bev(bev)        # contiguous every step (before any early return)

        # Warm-up: not enough history for CATK -> constant velocity.
        if len(bev.history) < self.num_history or bev.non_ego_mask.sum() == 0:
            return self._fallback.step(bev)

        torch = self._torch
        import numpy as np

        T = self.num_history
        P = self.predict
        L = T + P
        win_raw = bev.history.window(bev.ids)[:, -T:, :]  # (N, T, 4) [x,y,yaw,speed]
        valid_all = ~np.isnan(win_raw[:, :, 0])           # (N, T) actually-seen frames
        win = _backfill(win_raw, self.dt)   # moving reconstruction of pre-entry frames
        N = len(bev.ids)
        ego_i = bev.ego_idx

        # ego-center: translate the whole scene so the ego sits at the origin
        ox, oy = ((float(bev.xy[ego_i, 0]), float(bev.xy[ego_i, 1]))
                  if self.ego_center else (0.0, 0.0))
        win[:, :, 0] -= ox
        win[:, :, 1] -= oy

        # --- ego: history + pinned future (the plan) ---
        ego_xyz = torch.zeros(L, 3)
        ego_head = torch.zeros(L)
        ego_xyz[:T, 0] = torch.tensor(win[ego_i, :, 0], dtype=torch.float32)
        ego_xyz[:T, 1] = torch.tensor(win[ego_i, :, 1], dtype=torch.float32)
        ego_head[:T] = torch.tensor(win[ego_i, :, 2], dtype=torch.float32)
        if bev.ego_future is not None and len(bev.ego_future) >= P:
            ego_xyz[T:, 0] = torch.tensor(bev.ego_future[:P, 0] - ox, dtype=torch.float32)
            ego_xyz[T:, 1] = torch.tensor(bev.ego_future[:P, 1] - oy, dtype=torch.float32)
            ego_head[T:] = torch.tensor(bev.ego_future[:P, 2], dtype=torch.float32)
        else:
            ego_xyz[T:] = ego_xyz[T - 1]
            ego_head[T:] = ego_head[T - 1]

        # --- non-ego agents: history, future held + invalid (model predicts them) ---
        idx = [i for i in range(N) if i != ego_i]
        A = len(idx)
        ag_xyz = torch.zeros(A, L, 3)
        ag_head = torch.zeros(A, L)
        ag_valid = torch.zeros(A, L, dtype=torch.bool)
        ag_lwh = torch.zeros(A, 3)
        for k, i in enumerate(idx):
            ag_xyz[k, :T, 0] = torch.tensor(win[i, :, 0], dtype=torch.float32)
            ag_xyz[k, :T, 1] = torch.tensor(win[i, :, 1], dtype=torch.float32)
            ag_head[k, :T] = torch.tensor(win[i, :, 2], dtype=torch.float32)
            ag_valid[k, :T] = torch.tensor(valid_all[i])
            ag_xyz[k, T:] = ag_xyz[k, T - 1]
            ag_head[k, T:] = ag_head[k, T - 1]
            ag_lwh[k] = torch.tensor(bev.lwh[i], dtype=torch.float32)

        env_data = {
            "ego": {"xyz": ego_xyz, "heading": ego_head,
                    "lwh": torch.tensor(bev.lwh[ego_i], dtype=torch.float32)},
            "agents": {"xyz": ag_xyz, "heading": ag_head, "valid_mask": ag_valid,
                       "lwh": ag_lwh, "track_ids": torch.arange(A),
                       "class_ids": torch.zeros(A, dtype=torch.long), "num_obstacles": A},
            "map": self._full_map_layers(ox, oy),
            "env": {"curr_t": T - 1},
            "metadata": {"frame_rate": 10, **self._meta},
        }

        result = self.model.create_model_input(
            env_data, filter_map_by_ego=True, filter_distance_th=self.half)
        if result is None:
            # e.g. no map triplets near the ego this step -> keep moving, no freeze
            return self._fallback.step(bev)
        self._inject_light_type(result, bev, ox, oy)   # feed SUMO signals into the map
        actions = self.model.inference(result["input_data"])

        fx = actions["agent_future_xyz"].detach().cpu().numpy()        # (A, S, 3)
        fh = actions["agent_future_heading"].detach().cpu().numpy()    # (A, S)
        fv = actions["agent_future_velocity"].detach().cpu().numpy()   # (A, S, 2)

        out: dict[str, AgentPose] = {}
        for k, i in enumerate(idx):
            aid = bev.ids[i]
            yaw = float(fh[k, 0])
            out[aid] = AgentPose(
                x=float(fx[k, 0, 0]) + ox,          # shift back to global SUMO frame
                y=float(fx[k, 0, 1]) + oy,
                heading_deg=math_rad_to_sumo_deg(yaw),
                speed=float(np.hypot(fv[k, 0, 0], fv[k, 0, 1])),
            )
        return out
