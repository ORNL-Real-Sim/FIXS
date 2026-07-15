"""
Waymo Waymax world-model backend (#189) -- JAX.

Drives the surrounding bubble vehicles with Waymax's IDM route-following policy
(car-following + collision avoidance). Each step we build a Waymax
``SimulatorState`` from the bubble: every agent gets a straight constant-velocity
"route" (log_trajectory) from its current pose, the ego is the SDC (uncontrolled
by the WM), the neighbours are ``is_controlled``. ``IDMRoutePolicy.update_trajectory``
projects each controlled agent forward along its route at the IDM-adjusted speed
(braking for lead vehicles). We read back the first step.

Global frame, so no ego-centric round-trip. Needs no multi-frame history (unlike
CATK) -- the route encodes intent. JAX has no CUDA build on native Windows, so
this runs CPU-only here (fine for a small bubble); run via run_with_wm_waymax.bat.
Imports nothing heavy at module load, so the registry stays importable in realsim.
"""

from __future__ import annotations

import math

from bev import AgentPose, BevAgentSet, math_rad_to_sumo_deg

_ROUTE_STEPS = 20   # length of the const-velocity route each agent follows


class WaymaxBackend:
    """Waymax IDM reactive-traffic backend."""

    def reset(self, cfg: dict, vector_map=None) -> None:
        self.dt = float(cfg.get("StepLength", 0.1))
        import jax.numpy as jnp
        from waymax import datatypes
        from waymax.agents.waypoint_following_agent import IDMRoutePolicy

        self.jnp = jnp
        self.datatypes = datatypes
        self.policy = IDMRoutePolicy(
            is_controlled_func=lambda s: s.object_metadata.is_controlled
        )

    def step(self, bev: BevAgentSet) -> dict[str, AgentPose]:
        if bev.non_ego_mask.sum() == 0:
            return {}

        import numpy as np
        jnp = self.jnp
        dt = self.dt
        N = len(bev.ids)
        TS = _ROUTE_STEPS
        t = np.arange(TS)

        x0 = bev.xy[:, 0].astype(np.float64)
        y0 = bev.xy[:, 1].astype(np.float64)
        yaw = bev.yaw.astype(np.float64)
        spd = bev.speed.astype(np.float64)
        cos, sin = np.cos(yaw), np.sin(yaw)

        # per-agent straight constant-velocity route (the intent IDM follows)
        x = x0[:, None] + (spd * cos)[:, None] * t[None, :] * dt
        y = y0[:, None] + (spd * sin)[:, None] * t[None, :] * dt
        vx = np.repeat((spd * cos)[:, None], TS, axis=1)
        vy = np.repeat((spd * sin)[:, None], TS, axis=1)
        yawm = np.repeat(yaw[:, None], TS, axis=1)

        traj = self.datatypes.Trajectory(
            x=jnp.asarray(x), y=jnp.asarray(y), z=jnp.zeros((N, TS)),
            vel_x=jnp.asarray(vx), vel_y=jnp.asarray(vy), yaw=jnp.asarray(yawm),
            valid=jnp.ones((N, TS), dtype=bool),
            timestamp_micros=jnp.asarray(np.repeat((t * int(dt * 1e6))[None, :], N, axis=0).astype(np.int64)),
            length=jnp.asarray(np.repeat(bev.lwh[:, 0:1], TS, axis=1)),
            width=jnp.asarray(np.repeat(bev.lwh[:, 1:2], TS, axis=1)),
            height=jnp.asarray(np.repeat(bev.lwh[:, 2:3], TS, axis=1)),
        )
        is_sdc = np.zeros(N, dtype=bool)
        if bev.ego_idx >= 0:
            is_sdc[bev.ego_idx] = True
        meta = self.datatypes.ObjectMetadata(
            ids=jnp.arange(N), object_types=jnp.ones(N, dtype=jnp.int32),
            is_sdc=jnp.asarray(is_sdc), is_modeled=jnp.ones(N, dtype=bool),
            is_valid=jnp.ones(N, dtype=bool), objects_of_interest=jnp.zeros(N, dtype=bool),
            is_controlled=jnp.asarray(bev.non_ego_mask),
        )
        tl = self.datatypes.TrafficLights(
            x=jnp.zeros((1, TS)), y=jnp.zeros((1, TS)), z=jnp.zeros((1, TS)),
            state=jnp.zeros((1, TS), dtype=jnp.int32), lane_ids=jnp.zeros((1, TS), dtype=jnp.int32),
            valid=jnp.zeros((1, TS), dtype=bool),
        )
        state = self.datatypes.SimulatorState(
            sim_trajectory=traj, log_trajectory=traj, log_traffic_light=tl,
            object_metadata=meta, timestep=jnp.asarray(0),
        )

        upd = self.policy.update_trajectory(state)
        ux = np.asarray(upd.x).reshape(N, -1)[:, 0]
        uy = np.asarray(upd.y).reshape(N, -1)[:, 0]
        uyaw = np.asarray(upd.yaw).reshape(N, -1)[:, 0]
        uvx = np.asarray(upd.vel_x).reshape(N, -1)[:, 0]
        uvy = np.asarray(upd.vel_y).reshape(N, -1)[:, 0]

        out: dict[str, AgentPose] = {}
        for i, aid in enumerate(bev.ids):
            if i == bev.ego_idx:
                continue
            out[aid] = AgentPose(
                x=float(ux[i]), y=float(uy[i]),
                heading_deg=math_rad_to_sumo_deg(float(uyaw[i])),
                speed=float(math.hypot(float(uvx[i]), float(uvy[i]))),
            )
        return out
