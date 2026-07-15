"""
Backend contract for the World Model traffic-agent translator (#189).

Deliberately *not* an ``abc.ABC``: a backend is any object exposing ``reset``
and ``step``. We document the shape with a ``typing.Protocol`` (structural, not
enforced) so editors and readers see the contract without forcing inheritance
on the AlpaSim/Waymax adapters.
"""

from __future__ import annotations

from typing import Protocol, runtime_checkable

from bev import AgentPose, BevAgentSet


@runtime_checkable
class WorldModelBackend(Protocol):
    """One-step reactive traffic model over a generic BEV agent set."""

    def reset(self, cfg: dict, vector_map=None) -> None:
        """Prepare the backend once, before the loop.

        ``cfg`` is the parsed ``WorldModelSetup`` dict; ``vector_map`` is the
        local polyline map derived once from the SUMO ``.net.xml`` (None for
        map-free backends such as the stub).
        """
        ...

    def step(self, bev: BevAgentSet) -> dict[str, AgentPose]:
        """Advance one 0.1 s step.

        Returns updated poses for the **non-ego** ids only, keyed by SUMO id.
        The ego is driven by SUMO / a planner / an XIL client and must not be
        moved by the world model.
        """
        ...
