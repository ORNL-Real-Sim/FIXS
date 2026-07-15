"""
Constant-velocity world-model backend (#189) -- NO torch / jax.

The dependency-free baseline world model the default CI path exercises. It does
**no** neural inference; it advances each non-ego vehicle one step at its own
reported speed along its heading (constant velocity):

    (x, y)_{k+1} = (x, y)_k + speed * dt * (cos yaw, sin yaw)

**Why speed+heading, not position finite-difference.** Once the WM owns a
vehicle it is repositioned every tick via ``moveToXY``, which pins it -- so
consecutive received positions are identical, a finite-difference velocity reads
0, and the model would command the same position forever (a feedback deadlock
that freezes the vehicle). Using the SUMO-reported speed (preserved via
``setSpeed``) breaks that loop: the WM always commands an advancing position, so
``moveToXY`` moves the vehicle forward.

Beyond CI it is a genuine cheap baseline to compare AlpaSim/Waymax against, and
the harness to show a real signal-aware WM stopping at reds (this baseline does
not -- it holds speed through red lights, by design).
"""

from __future__ import annotations

import math

from bev import AgentPose, BevAgentSet, math_rad_to_sumo_deg


class ConstantVelocityBackend:
    """Constant-velocity world model (advance by reported speed along heading)."""

    def __init__(self, dt: float = 0.1):
        self.dt = float(dt)

    def reset(self, cfg: dict, vector_map=None) -> None:
        self.dt = float(cfg.get("StepLength", self.dt))

    def step(self, bev: BevAgentSet) -> dict[str, AgentPose]:
        out: dict[str, AgentPose] = {}
        for i, aid in enumerate(bev.ids):
            if i == bev.ego_idx:
                continue  # ego is externally driven -- never move it

            x, y = float(bev.xy[i, 0]), float(bev.xy[i, 1])
            yaw = float(bev.yaw[i])           # math radians, CCW from +X
            speed = float(bev.speed[i])       # m/s, from SUMO (preserved via setSpeed)

            nx = x + speed * self.dt * math.cos(yaw)
            ny = y + speed * self.dt * math.sin(yaw)

            out[aid] = AgentPose(
                x=nx,
                y=ny,
                heading_deg=math_rad_to_sumo_deg(yaw),
                speed=speed,
            )
        return out
