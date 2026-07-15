"""
Local vector map for the World Model translator (#189).

The road network is static for a whole run, so the SUMO ``.net.xml`` is parsed
ONCE at reset() into lane-centre polylines; per step we only crop to the moving
ego bubble. Both real backends consume this:
  - AlpaSim/CATK -> env_data["map"][layer] = {"polylines": (P,V,3), "label":[..]}
  - Waymax       -> RoadgraphPoints (sampled points + directions)

Pure stdlib + numpy (no torch/jax), so it is importable everywhere.
"""

from __future__ import annotations

import xml.etree.ElementTree as ET

import numpy as np


class VectorMap:
    """Lane-centre polylines in global SUMO XY (metres)."""

    def __init__(self, lanes: list[np.ndarray]):
        # each lane is an (V, 2) float array of polyline vertices
        self.lanes = [np.asarray(p, dtype=np.float64).reshape(-1, 2) for p in lanes if len(p) >= 2]

    # ------------------------------------------------------------------ #
    @classmethod
    def from_sumo_net(cls, net_file: str) -> "VectorMap":
        """Parse lane-centre geometry from a SUMO ``.net.xml``.

        Reads every non-internal ``<lane shape="x,y x,y ...">``. Internal
        (junction) lanes -- edges whose id starts with ':' -- are skipped.
        """
        tree = ET.parse(net_file)
        root = tree.getroot()
        lanes: list[np.ndarray] = []
        for edge in root.findall("edge"):
            if edge.get("function") == "internal" or (edge.get("id", "").startswith(":")):
                continue
            for lane in edge.findall("lane"):
                shape = lane.get("shape")
                if not shape:
                    continue
                pts = [tuple(map(float, xy.split(","))) for xy in shape.split()]
                if len(pts) >= 2:
                    lanes.append(np.asarray(pts, dtype=np.float64))
        return cls(lanes)

    # ------------------------------------------------------------------ #
    def crop(self, cx: float, cy: float, half: float) -> list[np.ndarray]:
        """Lane polylines with any vertex inside the [+/-half] box around (cx, cy)."""
        out = []
        for p in self.lanes:
            inside = (np.abs(p[:, 0] - cx) <= half) & (np.abs(p[:, 1] - cy) <= half)
            if inside.any():
                out.append(p)
        return out

    def crop_catk_layers(self, cx: float, cy: float, half: float,
                         z: float = 0.0) -> dict:
        """CATK ``env_data["map"]`` layers, cropped to the bubble.

        One 'lane_centers' layer: polylines padded to (P, Vmax, 3), label per
        polyline. CATK collapses sub-types when disable_sub_plyline_type=True,
        so lane geometry + the road-boundary flag are what matter.
        """
        polys = self.crop(cx, cy, half)
        if not polys:
            return {}
        vmax = max(len(p) for p in polys)
        arr = np.full((len(polys), vmax, 3), np.nan, dtype=np.float64)
        for i, p in enumerate(polys):
            arr[i, : len(p), 0:2] = p
            arr[i, : len(p), 2] = z
        return {"lane_centers": {"polylines": arr, "label": [5] * len(polys)}}  # 5 = lane_centers


if __name__ == "__main__":
    import sys
    vm = VectorMap.from_sumo_net(sys.argv[1])
    n = len(vm.lanes)
    tot = sum(len(p) for p in vm.lanes)
    print(f"parsed {n} lane polylines, {tot} vertices")
    if n:
        allpts = np.concatenate(vm.lanes)
        print(f"extent x[{allpts[:,0].min():.1f},{allpts[:,0].max():.1f}] "
              f"y[{allpts[:,1].min():.1f},{allpts[:,1].max():.1f}]")
