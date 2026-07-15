"""
Backend registry for the World Model translator (#189).

Heavy backends (alpasim -> torch, waymax -> jax) are imported **lazily** so that
importing this package -- as the stub smoke test does -- never pulls torch/jax.
"""

from __future__ import annotations


def make_backend(name: str):
    """Return an *instance* of the named backend.

    'stub'    -> constant-velocity, no heavy deps (default CI path)
    'alpasim' -> NVIDIA AlpaSim CATK/SMART (PyTorch)   [lazy import]
    'waymax'  -> Waymo Waymax (JAX)                    [lazy import]
    """
    key = (name or "constant_velocity").lower()
    if key in ("constant_velocity", "constvel", "stub"):
        from .constant_velocity import ConstantVelocityBackend
        return ConstantVelocityBackend()
    if key == "alpasim":
        from .alpasim import AlpaSimBackend
        return AlpaSimBackend()
    if key == "waymax":
        from .waymax import WaymaxBackend
        return WaymaxBackend()
    raise ValueError(f"Unknown world-model backend: {name!r} (have: stub|alpasim|waymax)")
