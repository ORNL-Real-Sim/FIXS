"""
BEV (bird's-eye-view) primitives for the World Model traffic-agent translator (#189).

This module owns the *model-agnostic* FIXS-side geometry: it turns the raw
per-step vehicle list that TrafficLayer delivers into a generic ``BevAgentSet``
(the pivot type every backend translates against), maintains the rolling
history buffer that stateful models such as AlpaSim/CATK require, and builds
the short ego-future lookahead that CATK conditions on.

Nothing in here imports torch or jax -- it is pure numpy and runs in the plain
``realsim`` env. Backends map ``BevAgentSet`` to/from their native I/O.

Coordinate conventions
----------------------
* SUMO / FIXS deliver ``heading`` in **navigational degrees**: 0 = north (+Y),
  increasing **clockwise**, so 90 = east (+X).
* The generic ``BevAgentSet`` carries heading in **math radians** (CCW from +X)
  because both world models (Waymax ``yaw``, CATK ``heading``) use that
  convention. Conversion happens here, once, so backends never re-derive it.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field
from typing import NamedTuple

import numpy as np


# --------------------------------------------------------------------------- #
# Heading conversions (SUMO navigational degrees <-> math radians CCW)
# --------------------------------------------------------------------------- #
def sumo_deg_to_math_rad(heading_deg: float) -> float:
    """SUMO navigational degrees (0=N, CW) -> math radians (0=+X, CCW)."""
    return math.radians(90.0 - heading_deg)


def math_rad_to_sumo_deg(yaw_rad: float) -> float:
    """Math radians (0=+X, CCW) -> SUMO navigational degrees (0=N, CW)."""
    return (90.0 - math.degrees(yaw_rad)) % 360.0


class AgentPose(NamedTuple):
    """One backend-produced pose, in SUMO's own frame, ready for write-back."""
    x: float
    y: float
    heading_deg: float   # SUMO navigational degrees (what moveToXY expects)
    speed: float         # m/s


# --------------------------------------------------------------------------- #
# Rolling per-agent history (stateful models need a short window)
# --------------------------------------------------------------------------- #
class RollingHistory:
    """Fixed-depth ring of per-id state frames.

    Each frame is ``{id: np.array([x, y, yaw_rad, speed])}``. AlpaSim/CATK needs
    >=16 frames (1.6 s @ 10 Hz); Waymax needs a couple; the stub needs one prior
    frame for its finite-difference constant-velocity model.
    """

    def __init__(self, depth: int):
        self.depth = int(depth)
        self._frames: deque[dict[str, np.ndarray]] = deque(maxlen=self.depth)

    def push(self, frame: dict[str, np.ndarray]) -> None:
        self._frames.append(frame)

    def __len__(self) -> int:
        return len(self._frames)

    @property
    def is_warm(self) -> bool:
        """True once the buffer is full enough for real inference."""
        return len(self._frames) >= self.depth

    def prev_state(self, agent_id: str) -> np.ndarray | None:
        """The most recent *previous* frame's state for ``agent_id`` (or None)."""
        if len(self._frames) < 2:
            return None
        return self._frames[-2].get(agent_id)

    def window(self, ids: list[str]) -> np.ndarray:
        """Dense ``(N, T, 4)`` history [x, y, yaw_rad, speed] for ``ids``.

        Missing (agent, t) cells are filled with NaN so a backend can build its
        own validity mask. Ordered oldest -> newest along T.
        """
        T = len(self._frames)
        out = np.full((len(ids), T, 4), np.nan, dtype=np.float64)
        for t, frame in enumerate(self._frames):
            for i, aid in enumerate(ids):
                st = frame.get(aid)
                if st is not None:
                    out[i, t] = st
        return out


# --------------------------------------------------------------------------- #
# The pivot type every backend translates against
# --------------------------------------------------------------------------- #
@dataclass
class BevAgentSet:
    """Generic bird's-eye agent set for one inference step (global frame)."""

    ids: list[str]                       # SUMO ids, ego included
    ego_id: str
    ego_idx: int                         # row of ego in the arrays (-1 if absent)
    xy: np.ndarray                       # (N, 2)  global meters
    yaw: np.ndarray                      # (N,)    math radians CCW
    speed: np.ndarray                    # (N,)    m/s
    lwh: np.ndarray                      # (N, 3)  length, width, height (m)
    veh_class: list[str]                 # per-agent SUMO vClass string
    history: RollingHistory              # rolling window (translator-owned)
    sig_color: np.ndarray                # (N,) next-signal state: 1=red 2=yellow 3=green, 0/-1=none
    sig_dist: np.ndarray                 # (N,) distance to that signal (m); <0 = none
    ego_future: np.ndarray | None = None  # (S, 3) [x, y, yaw] -- CATK only

    @property
    def non_ego_mask(self) -> np.ndarray:
        m = np.ones(len(self.ids), dtype=bool)
        if 0 <= self.ego_idx < len(self.ids):
            m[self.ego_idx] = False
        return m


def crop_square_bev(
    vehicles: list,
    ego_id: str,
    half_extent: float,
    history: RollingHistory,
) -> BevAgentSet | None:
    """Crop a raw VehData list to a square window centred on the ego.

    ``vehicles`` is the list of ``VehData`` from ``recv_data``. Returns None if
    the ego is not present this step (nothing to do). The square box is
    ``[-half_extent, +half_extent]`` in both X and Y around the ego position.
    """
    ego = None
    for v in vehicles:
        if v.id.strip() == ego_id:
            ego = v
            break
    if ego is None:
        return None

    ex, ey = float(ego.positionX), float(ego.positionY)

    ids: list[str] = []
    xy, yaw, speed, lwh, vclass, sigc, sigd = [], [], [], [], [], [], []
    for v in vehicles:
        vx, vy = float(v.positionX), float(v.positionY)
        if abs(vx - ex) > half_extent or abs(vy - ey) > half_extent:
            continue
        vid = v.id.strip()
        ids.append(vid)
        xy.append((vx, vy))
        yaw.append(sumo_deg_to_math_rad(float(v.heading)))
        speed.append(float(v.speed))
        lwh.append((float(v.length), float(v.width), float(v.height)))
        vclass.append(v.vehicleClass.strip() if hasattr(v, "vehicleClass") else "")
        sigc.append(int(getattr(v, "signalLightColor", 0)))
        sigd.append(float(getattr(v, "signalLightDistance", -1.0)))

    ego_idx = ids.index(ego_id) if ego_id in ids else -1

    return BevAgentSet(
        ids=ids,
        ego_id=ego_id,
        ego_idx=ego_idx,
        xy=np.asarray(xy, dtype=np.float64).reshape(-1, 2),
        yaw=np.asarray(yaw, dtype=np.float64),
        speed=np.asarray(speed, dtype=np.float64),
        lwh=np.asarray(lwh, dtype=np.float64).reshape(-1, 3),
        veh_class=vclass,
        history=history,
        sig_color=np.asarray(sigc, dtype=np.int64),
        sig_dist=np.asarray(sigd, dtype=np.float64),
    )


def push_frame(bev: BevAgentSet) -> None:
    """Append the current BEV frame to its rolling history."""
    frame = {
        aid: np.array([bev.xy[i, 0], bev.xy[i, 1], bev.yaw[i], bev.speed[i]])
        for i, aid in enumerate(bev.ids)
    }
    bev.history.push(frame)


def make_ego_future(bev: BevAgentSet, horizon: int, dt: float) -> np.ndarray | None:
    """Constant-velocity ego lookahead: (horizon, 3) [x, y, yaw].

    CATK pins the ego's near-future and predicts the surrounding agents as a
    reaction to it.

    Usage 1 (sumo -> TL -> wm, current): there is no controller; the ego is a
    plain SUMO-driven vehicle, so this const-velocity extrapolation of its
    current state IS the correct conditioning.

    Usage 2 (sumo -> TL -> controller -> wm, later): a controller supplies a
    *planned* ego trajectory; that plan replaces this function's output as the
    ``ego_future`` conditioning -- same field, different source.

    Waymax and the stub ignore this field.
    """
    if bev.ego_idx < 0:
        return None
    x0, y0 = bev.xy[bev.ego_idx]
    yaw = bev.yaw[bev.ego_idx]
    v = bev.speed[bev.ego_idx]
    steps = np.arange(1, horizon + 1) * dt * v
    xs = x0 + steps * math.cos(yaw)
    ys = y0 + steps * math.sin(yaw)
    yaws = np.full(horizon, yaw)
    return np.stack([xs, ys, yaws], axis=1)
