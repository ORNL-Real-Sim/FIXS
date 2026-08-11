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
        # Paint SUMO signal state onto nearby map tokens. Measured inert on the
        # v120 checkpoint (see _inject_light_type), so off by default.
        self.inject_light = bool(cfg.get("SignalLightInject", False))
        # Feed CATK an EGO-CENTERED scene (subtract the ego position). WOMD/SMART
        # is trained SDC-centered; absolute SUMO coords (~600-1300 m) are far
        # out-of-distribution and hurt stability. Outputs are shifted back.
        self.ego_center = bool(cfg.get("EgoCenter", True))
        # Greedy/top-K token sampling: >0 overrides the model default (=5). num_k=1
        # is greedy (argmax) -> deterministic, removes per-step sampling jitter.
        self.topk = int(cfg.get("SamplingTopK", 0))
        # Inference stride: run CATK every N steps and REPLAY the token's own
        # sub-steps (0..N-1) in between (N<=EgoFutureHorizon). Cuts re-sampling
        # jitter + cost; N=1 = infer every step (fully reactive).
        self.stride = max(1, int(cfg.get("InferenceStride", 1)))
        # Map resample spacing (m). CATK map tokens are 3 consecutive lane points,
        # so this sets the token scale; matches CATK's own 1.0 m default.
        self.map_interval = float(cfg.get("MapResampleInterval", 1.0))
        # Crop the map to segments within this radius of ANY bubble agent (ego +
        # others), mirroring CATK's own v_to_ego_and_obs filter (its config uses
        # map_max_pts_to_ego_distance=25).
        self.map_crop_r = float(cfg.get("MapCropRadius", 25.0))
        # Physical speed ceiling (m/s) for the WM write-back. CATK occasionally
        # emits a garbage token (seen: 85-130 m/s) that moveToXY would apply as a
        # multi-metre teleport, stranding a car mid-junction. Clamp per-substep
        # displacement to vmax*dt and speed to vmax; well above lane speed so
        # normal motion is untouched.
        self.vmax = float(cfg.get("MaxSpeed", 25.0))
        self._istep = 0
        self._cache = {}
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
        if self.topk > 0:
            # NOTE: the sampling scheme lives on the SMART instance INSIDE the
            # CATK wrapper (model_adapter: self.model = SMART(...)). Setting it
            # on the wrapper silently creates a dead attribute -- which is why an
            # earlier "greedy" setting never actually took effect.
            self.model.model.validation_rollout_sampling.num_k = self.topk
            print(f"[alpasim] SamplingTopK={self.topk} "
                  f"({'greedy' if self.topk == 1 else 'top-K'})", file=sys.stderr)
        self._meta = obstacle_class_metadata()

        # Static multi-layer map (net.xml -> netconvert -> trajdata -> CATK
        # layers), parsed once. Lane centrelines ALONE measurably degrade CATK
        # (see catk_map docstring), so the junction layers matter.
        self.catk_map = None
        net_file = cfg.get("NetFile", "")
        if net_file and os.path.exists(net_file):
            from catk_map import CatkMap
            self.catk_map = CatkMap.build(net_file, spacing=self.map_interval,
                                          netconvert=cfg.get("NetconvertExe") or None)
            print(f"[alpasim] CATK map layers: {self.catk_map.summary()}", file=sys.stderr)
        else:
            print(f"[alpasim] WARNING: no NetFile -- CATK will run without a map",
                  file=sys.stderr)

        # warm-up / degenerate fallback
        self._fallback = ConstantVelocityBackend()
        self._fallback.reset(cfg)

    # ------------------------------------------------------------------ #
    def _full_map_layers(self, ox: float, oy: float, agent_xy_c=None) -> dict:
        """Multi-layer CATK map cropped to the bubble agents.

        Delegates to :mod:`catk_map`, which converts the SUMO net through
        netconvert + trajdata into AlpaSim's own 8-layer format (lane centres,
        lane/road boundaries, stop lines, junction areas). Feeding lane centres
        ALONE strips the junction semantics CATK relies on and measurably
        degrades it (2/10 vs 8/10 stopped-queue launches on AlpaSim's own scene),
        which is why this no longer hand-rolls geometry.

        ``agent_xy_c`` are the bubble agents in the EGO-CENTRED frame, so the
        anchors are shifted back to world coords for the crop.
        """
        if self.catk_map is None:
            return {}
        import numpy as np

        anchors = np.array([[ox, oy]], dtype=np.float64)          # the ego
        if agent_xy_c is not None and len(agent_xy_c):
            anchors = np.vstack([anchors,
                                 np.asarray(agent_xy_c, dtype=np.float64) + [ox, oy]])
        return self.catk_map.crop(anchors, self.map_crop_r, self._torch, ox, oy)

    def _inject_light_type(self, result, bev, ox: float = 0.0, oy: float = 0.0) -> None:
        """Tag CATK map triplets near a red/yellow stop-line with light_type=STOP.

        SMART's map encoder has a 5-state traffic-light embedding but the env_data
        adapter zeros it, so the model never sees signals. We derive stop-lines
        from each bubble vehicle's next-signal (pos + dist along heading) and set
        the STOP index (1, per SMART's ['NONE','STOP','GO','CAUTION','GO_TO_STOP'])
        on map triplets within light_radius of one.

        OFF by default (``SignalLightInject: false``): AlpaSim's own runtime
        always feeds light_type=0, and a controlled test on their ClipGT queue
        scene showed painting STOP changes nothing (3/4 launches with STOP held
        vs 3/4 with no light) -- the pathway is effectively inert in the v120
        checkpoint. Kept, corrected and disabled rather than deleted so it can be
        re-tested against a checkpoint that does use it.

        The enum now matches upstream CATK training (data_preprocess.py
        _polygon_light_type): NO_LANE_STATE=0, UNKNOWN=1, STOP=2, GO=3,
        CAUTION=4 -- the previous code wrote 1, which is UNKNOWN, not STOP.
        """
        if not self.inject_light:
            return
        import numpy as np
        torch = self._torch
        stops = []
        for i in range(len(bev.ids)):
            c = int(bev.sig_color[i])
            d = float(bev.sig_dist[i])
            if c in (1, 2) and d >= 0.0:                 # red or yellow ahead
                yaw = float(bev.yaw[i])
                state = 2 if c == 1 else 4               # STOP / CAUTION
                stops.append((bev.xy[i, 0] + d * np.cos(yaw) - ox,   # ego-centered frame
                              bev.xy[i, 1] + d * np.sin(yaw) - oy, state))
        if not stops:
            return
        pe = result["input_data"]["map"]["polyline_extras"]
        tri = result["input_data"]["map"]["triplets"]     # (M, 3, 2)
        centers = tri.float().mean(dim=1)                 # (M, 2)
        st = torch.tensor([[s[0], s[1]] for s in stops],
                          dtype=centers.dtype, device=centers.device)
        states = torch.tensor([s[2] for s in stops], device=centers.device)
        d2 = ((centers[:, None, :] - st[None, :, :]) ** 2).sum(-1)  # (M, S)
        best = d2.min(dim=1)
        near = best.values < (self._light_radius ** 2)
        lt = pe["light_type"].clone()
        lt[near] = states[best.indices[near]].to(lt.dtype)
        pe["light_type"] = lt

    def _dump_bev(self, bev) -> None:
        """Append one ego-centered BEV snapshot of the agent state sent to CATK,
        for manual verification. Fires EVERY step -- including warm-up and
        fallback -- giving a CONTIGUOUS 10 Hz sequence (no skipped frames).

        NOTE: the 'lanes' here are the RAW SUMO net polylines (road reference
        geometry for readability). They are NOT the map CATK receives, which is
        the multi-layer netconvert/trajdata conversion (see catk_map).

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
                "lwh": [round(float(bev.lwh[i, 0]), 1), round(float(bev.lwh[i, 1]), 1)],  # length,width
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

        # Inference stride: on in-between phases, replay the last token's sub-steps
        # (const-velocity for any agent that entered since the last inference).
        phase = self._istep % self.stride
        self._istep += 1
        if self.stride > 1 and phase != 0 and self._cache:
            out = self._fallback.step(bev)
            for aid in list(out.keys()):
                subs = self._cache.get(aid)
                if subs and phase < len(subs):
                    out[aid] = subs[phase]
            return out

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
            "map": self._full_map_layers(ox, oy, win[:, T - 1, 0:2]),
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

        # cache the token's sub-steps (0..stride-1) per agent for stride replay,
        # and return sub-step 0 this step
        S = fx.shape[1]
        nsub = max(1, min(S, self.stride))
        self._cache = {}
        out: dict[str, AgentPose] = {}
        for k, i in enumerate(idx):
            aid = bev.ids[i]
            cx, cy = float(bev.xy[i, 0]), float(bev.xy[i, 1])   # current global pose
            subs = []
            for s in range(nsub):
                tx = float(fx[k, s, 0]) + ox                    # shift back to global SUMO frame
                ty = float(fx[k, s, 1]) + oy
                # clamp a garbage token from teleporting the car: cap displacement
                # from the CURRENT pose to vmax*dt per sub-step (+slack), and speed.
                dx, dy = tx - cx, ty - cy
                dist = float(np.hypot(dx, dy))
                maxd = self.vmax * self.dt * (s + 1)
                if dist > maxd > 0.0:
                    f = maxd / dist
                    tx, ty = cx + dx * f, cy + dy * f
                spd = min(float(np.hypot(fv[k, s, 0], fv[k, s, 1])), self.vmax)
                subs.append(AgentPose(
                    x=tx, y=ty,
                    heading_deg=math_rad_to_sumo_deg(float(fh[k, s])),
                    speed=spd,
                ))
            self._cache[aid] = subs
            out[aid] = subs[0]
        return out
