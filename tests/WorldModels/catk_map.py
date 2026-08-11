"""SUMO net -> AlpaSim/CATK native multi-layer map (#189).

Why this exists
---------------
CATK builds its map tokens from 3 CONSECUTIVE polyline points and embeds a
per-token LAYER identity (``pl_type``). Feeding it lane centrelines alone --
which is what the first cut of this backend did -- strips the junction
semantics (stop lines, junction areas, lane/road boundaries) that tell the model
"this is a signalised intersection", and measurably degrades its behaviour: on
AlpaSim's own ClipGT queue scene, with everything else held fixed, a
lane_centers-only map launched a stopped queue 2/10 times versus 8/10 for the
native multi-layer map.

So we reuse AlpaSim's own ingestion path instead of hand-rolling geometry:

    net.xml --netconvert--> .xodr --trajdata--> VectorMap --> 8 CATK layers

``waitlines`` and ``intersection_areas`` are not emitted by netconvert, so they
are synthesised directly from the SUMO net: a stop-line segment across each
signalised incoming lane's end, and each traffic-light junction's own shape.

The full map is parsed ONCE (``CatkMap.build``) and only cropped per step.
"""

from __future__ import annotations

import os
import re
import subprocess
import xml.etree.ElementTree as ET

import numpy as np

# CATK map-element ids (alpasim_trafficsim.catk.map_adapter.MAP_ELEMENT_NAME2_TYPEID)
LAYER_IDS = {
    "lanelines": 0,
    "road_boundaries": 1,
    "waitlines": 2,
    "crosswalks": 3,
    "lane_boundaries": 4,
    "lane_centers": 5,
    "intersection_areas": 6,
    "road_islands": 7,
}

_DEFAULT_NETCONVERT = os.path.join(
    os.environ.get("SUMO_HOME", r"C:/Program Files (x86)/Eclipse/Sumo"), "bin", "netconvert.exe"
)


# --------------------------------------------------------------------------- #
#  net.xml -> .xodr
# --------------------------------------------------------------------------- #
def ensure_xodr(net_file: str, xodr_file: str | None = None,
                netconvert: str | None = None) -> str:
    """Convert ``net_file`` to OpenDRIVE, skipping the run if already current."""
    xodr_file = xodr_file or os.path.splitext(net_file)[0] + ".xodr"
    if (os.path.exists(xodr_file)
            and os.path.getmtime(xodr_file) >= os.path.getmtime(net_file)):
        return xodr_file
    exe = netconvert or _DEFAULT_NETCONVERT
    if not os.path.exists(exe):
        raise FileNotFoundError(
            f"netconvert not found at {exe} -- set SUMO_HOME or WorldModelSetup.NetconvertExe")
    res = subprocess.run([exe, "-s", net_file, "--opendrive-output", xodr_file],
                         capture_output=True, text=True)
    if res.returncode != 0 or not os.path.exists(xodr_file):
        raise RuntimeError(f"netconvert failed for {net_file}:\n{res.stderr[-2000:]}")
    return xodr_file


# --------------------------------------------------------------------------- #
#  layers
# --------------------------------------------------------------------------- #
def layers_from_xodr(xodr_file: str, resolution: float = 0.5) -> dict[str, list[np.ndarray]]:
    """trajdata's OpenDRIVE importer -> the CATK layer dict (same split as
    AlpaSim's ``map_adapter._vector_map_polylines_by_env_key``)."""
    from trajdata.dataset_specific.xodr.vector_map_export import populate_vector_map_from_xodr
    from trajdata.maps import VectorMap
    from trajdata.maps.vec_map_elements import MapElementType

    with open(xodr_file, "r", encoding="utf-8") as fh:
        xodr = fh.read()
    vmap = VectorMap(map_id="fixs_sumo:" + os.path.basename(xodr_file))
    populate_vector_map_from_xodr(vmap, xodr, resolution=resolution, t_xodr_enu_to_sim=None)

    out: dict[str, list[np.ndarray]] = {k: [] for k in LAYER_IDS}
    for lane in vmap.elements.get(MapElementType.ROAD_LANE, {}).values():
        out["lane_centers"].append(np.asarray(lane.center.xyz))
        for edge in (lane.left_edge, lane.right_edge):
            if edge is not None:
                out["lane_boundaries"].append(np.asarray(edge.xyz))
                out["lanelines"].append(np.asarray(edge.xyz))
    for e in vmap.elements.get(MapElementType.ROAD_EDGE, {}).values():
        out["road_boundaries"].append(np.asarray(e.polyline.xyz))
    for w in vmap.elements.get(MapElementType.WAIT_LINE, {}).values():
        out["waitlines"].append(np.asarray(w.polyline.xyz))
    for c in vmap.elements.get(MapElementType.PED_CROSSWALK, {}).values():
        out["crosswalks"].append(np.asarray(c.polygon.xyz))
    for a in vmap.elements.get(MapElementType.ROAD_AREA, {}).values():
        out["intersection_areas"].append(np.asarray(a.exterior_polygon.xyz))
        out["road_islands"].extend(np.asarray(h.xyz) for h in a.interior_holes)
    return out


def semantic_layers_from_net(net_file: str) -> dict[str, list[np.ndarray]]:
    """``waitlines`` + ``intersection_areas`` straight from the SUMO net.

    netconvert's OpenDRIVE export carries neither, but SUMO states both exactly:
    a signalised junction's incoming lanes end AT the stop line, and the junction
    publishes its own shape polygon.
    """
    root = ET.parse(net_file).getroot()
    lane_shape: dict[str, np.ndarray] = {}
    for lane in root.iter("lane"):
        shape = lane.get("shape")
        lid = lane.get("id")
        if not shape or not lid:
            continue
        pts = np.array([tuple(map(float, xy.split(","))) for xy in shape.split()])
        if len(pts) >= 2:
            lane_shape[lid] = pts

    out: dict[str, list[np.ndarray]] = {"waitlines": [], "intersection_areas": []}
    for j in root.iter("junction"):
        if j.get("type") != "traffic_light":
            continue
        for lid in (j.get("incLanes") or "").split():
            p = lane_shape.get(lid)
            if p is None:
                continue
            end, prev = p[-1], p[-2]
            t = end - prev
            n = np.linalg.norm(t)
            if n < 1e-6:
                continue
            t = t / n
            perp = np.array([-t[1], t[0]])
            out["waitlines"].append(np.stack([end - 1.6 * perp, end + 1.6 * perp]))
        shape = j.get("shape")
        if shape:
            poly = np.array([tuple(map(float, xy.split(","))) for xy in shape.split()])
            if len(poly) >= 3:
                out["intersection_areas"].append(np.vstack([poly, poly[:1]]))
    return out


# --------------------------------------------------------------------------- #
#  polyline -> CATK 3-point segments
# --------------------------------------------------------------------------- #
def _resample(polyline: np.ndarray, spacing: float = 1.0) -> np.ndarray:
    """Uniform arc-length resample; CATK's own adapter uses 1 m."""
    p = np.asarray(polyline, dtype=np.float64)
    if p.shape[1] == 2:
        p = np.column_stack([p, np.zeros(len(p))])
    p = p[:, :3]
    d = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(p, axis=0), axis=1))]
    keep = np.r_[True, np.diff(d) > 1e-6]
    p, d = p[keep], d[keep]
    if len(p) < 2 or d[-1] <= 0:
        return p.astype(np.float32)
    s = np.arange(0.0, d[-1], spacing)
    if not np.isclose(s[-1], d[-1]):
        s = np.r_[s, d[-1]]
    return np.column_stack([np.interp(s, d, p[:, i]) for i in range(3)]).astype(np.float32)


def _segments(polylines: list[np.ndarray], spacing: float = 1.0) -> np.ndarray:
    """Overlapping 3-point, stride-2 windows -- AlpaSim's map_adapter layout."""
    out = []
    for raw in polylines:
        p = _resample(raw, spacing)
        if len(p) < 3:
            continue
        starts = list(range(0, len(p) - 2, 2))
        tail = len(p) - 3
        if starts[-1] != tail:
            starts.append(tail)
        out.extend(p[s: s + 3] for s in starts)
    if not out:
        return np.zeros((0, 3, 3), dtype=np.float32)
    return np.asarray(out, dtype=np.float32).reshape(-1, 3, 3)


class CatkMap:
    """Static multi-layer map, built once; cropped per step."""

    def __init__(self, segments: dict[str, np.ndarray]):
        self.segments = segments                       # layer -> (M, 3, 3) float32
        self._centers = {k: v[:, :, :2].mean(axis=1) for k, v in segments.items()}

    # ------------------------------------------------------------------ #
    @classmethod
    def build(cls, net_file: str, spacing: float = 1.0,
              netconvert: str | None = None) -> "CatkMap":
        xodr = ensure_xodr(net_file, netconvert=netconvert)
        layers = layers_from_xodr(xodr)
        for name, polys in semantic_layers_from_net(net_file).items():
            if polys:                                   # net-derived beats empty
                layers[name] = polys
        segs = {}
        for name, polys in layers.items():
            if not polys:
                continue
            s = _segments(polys, spacing)
            if len(s):
                segs[name] = s
        return cls(segs)

    # ------------------------------------------------------------------ #
    def summary(self) -> str:
        return ", ".join(f"{k}={len(v)}" for k, v in sorted(self.segments.items()))

    #: segment-count caps, mirroring CATK's own ``map_adv_max_*`` config
    #: (scene_adapter._apply_map_adv_count_filter, which the model_adapter path
    #: we use does NOT apply). Without them the map grows with the agent count
    #: and inference time explodes -- measured 74 ms -> 3.8 s as a 200 m bubble
    #: filled up and the map passed ~3000 segments.
    GROUP_CAPS = {
        "lane": (("lane_centers", "lanelines", "lane_boundaries"), 1000),
        "road_boundary": (("road_boundaries",), 750),
        "other": (("waitlines", "crosswalks", "intersection_areas", "road_islands"), 250),
    }

    def crop(self, anchors_xy: np.ndarray, radius: float, torch,
             ox: float = 0.0, oy: float = 0.0) -> dict:
        """CATK ``env_data['map']`` for segments within ``radius`` of any anchor.

        Mirrors CATK's own ``v_to_ego_and_obs`` distance filter (ego + every
        agent) AND its per-group segment caps, keeping the nearest segments when
        a group is over budget. Returns tensor labels like ``map_adapter`` does.
        ``ox/oy`` shift into the ego-centred frame used by the rest of the
        adapter.
        """
        anchors = np.atleast_2d(np.asarray(anchors_xy, dtype=np.float64))
        if not len(anchors):
            return {}
        r2 = radius * radius
        off = np.array([ox, oy, 0.0], dtype=np.float32)

        # distance-filter every layer first, keeping each kept segment's distance
        kept: dict[str, tuple[np.ndarray, np.ndarray]] = {}
        for name, seg in self.segments.items():
            c = self._centers[name]
            d2 = ((c[:, None, :] - anchors[None, :, :]) ** 2).sum(-1).min(1)
            idx = np.flatnonzero(d2 < r2)
            if len(idx):
                kept[name] = (idx, d2[idx])

        # then apply the per-group budget across the layers of that group
        out = {}
        for names, cap in self.GROUP_CAPS.values():
            present = [n for n in names if n in kept]
            total = sum(len(kept[n][0]) for n in present)
            for n in present:
                idx, d2n = kept[n]
                if total > cap:                      # keep this layer's share, nearest first
                    share = max(1, int(round(cap * len(idx) / total)))
                    if share < len(idx):
                        order = np.argsort(d2n)[:share]
                        idx = idx[order]
                seg = self.segments[n][idx] - off
                out[n] = {
                    "polylines": torch.from_numpy(np.ascontiguousarray(seg)),
                    "label": torch.full((len(seg),), LAYER_IDS[n], dtype=torch.long),
                }
        return out


if __name__ == "__main__":
    import sys as _sys
    m = CatkMap.build(_sys.argv[1])
    print(m.summary())
