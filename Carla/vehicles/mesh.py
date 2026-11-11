"""Source model -> vehicle parts (body + four wheels) in the CARLA vehicle frame.

Frame of everything this returns: metres, +X forward, +Y left, +Z up, origin at the
ground under the centre of the bounding box. A "chunk" is the triangles of one mesh node
of the source model in one material: dict(group, path, material, P, N, UV), where group is
the top-level group, path the node names from there down to the mesh, P and N shaped
(n, 3, 3) and UV (n, 3, 2).
"""

import os
import re

import numpy as np

PAINT_SLOT = "Bodywork_Mat"  # the slot name CARLA's BaseVehiclePawn.ApplyColor paints
BONE_OF = {"front_left": "Wheel_Front_Left", "front_right": "Wheel_Front_Right",
           "rear_left": "Wheel_Rear_Left", "rear_right": "Wheel_Rear_Right"}
BASE_BONE = "Vehicle_Base"


class MeshError(Exception):
    pass


# ---------------------------------------------------------------- reading

def read_collada(path):
    """(chunks, materials) from a Collada file, in the file's own axes and metres.

    materials: {material id: {name, color [r,g,b,a] sRGB 0..1, texture basename or None}}.
    """
    import collada
    from collada.scene import GeometryNode, Node, NodeNode

    dae = collada.Collada(path, ignore=[collada.common.DaeUnsupportedError,
                                        collada.common.DaeBrokenRefError])
    unit = float(dae.assetInfo.unitmeter or 1.0)
    materials = {}
    for mat in dae.materials:
        e = mat.effect
        d = {"name": mat.name or mat.id, "color": [0.8, 0.8, 0.8, 1.0], "texture": None}
        if isinstance(e.diffuse, tuple):
            d["color"] = [float(x) for x in e.diffuse]
        elif e.diffuse is not None and hasattr(e.diffuse, "sampler"):
            d["texture"] = os.path.basename(e.diffuse.sampler.surface.image.path)
        materials[mat.id] = d

    chunks = []

    def walk(node, M, top, path):
        if isinstance(node, Node):
            name = node.xmlnode.get("name") or node.id or ""
            # SketchUp wraps everything in one or two root nodes; groups below them are the parts
            t = top if top is not None else (name if name not in ("SketchUp", "group_0") else None)
            for ch in node.children:
                walk(ch, M @ node.matrix, t, path + [name])
        elif isinstance(node, NodeNode):
            walk(node.node, M, top, path)
        elif isinstance(node, GeometryNode):
            matmap = {mn.symbol: (mn.target.id if mn.target is not None else mn.symbol)
                      for mn in node.materials}
            R = M[:3, :3]
            Rn = np.linalg.inv(R).T
            flip = np.linalg.det(R) < 0
            for prim in node.geometry.primitives:
                if isinstance(prim, collada.lineset.LineSet):
                    continue
                ts = prim if isinstance(prim, collada.triangleset.TriangleSet) else prim.triangleset()
                vi = np.asarray(ts.vertex_index).reshape(-1, 3)
                if not len(vi):
                    continue
                P = ts.vertex[vi] @ R.T + M[:3, 3]
                if ts.normal is not None:
                    N = ts.normal[np.asarray(ts.normal_index).reshape(-1, 3)] @ Rn.T
                else:
                    N = np.zeros_like(P)
                if ts.texcoordset and ts.texcoord_indexset:
                    UV = ts.texcoordset[0][np.asarray(ts.texcoord_indexset[0]).reshape(-1, 3)]
                else:
                    UV = np.zeros(P.shape[:2] + (2,))
                if flip:  # a mirroring transform reverses winding
                    P, N, UV = P[:, ::-1], N[:, ::-1], UV[:, ::-1]
                chunks.append({"group": top or "root", "path": path,
                               "material": matmap.get(prim.material, prim.material),
                               "P": P * unit, "N": N, "UV": UV})

    for n in dae.scene.nodes:
        walk(n, np.eye(4), None, [])
    if not chunks:
        raise MeshError("{} holds no triangles".format(path))
    return chunks, materials


def read_npz_dump(npz_path, json_path):
    """(chunks, materials) from the arrays blender_rig.py --dump wrote."""
    import json
    with open(json_path, "r", encoding="utf-8") as fh:
        meta = json.load(fh)
    data = np.load(npz_path)
    chunks = []
    for i, c in enumerate(meta["chunks"]):
        chunks.append({"group": c["group"], "path": c.get("path") or [c["group"]],
                       "material": c["material"],
                       "P": data["p%d" % i].reshape(-1, 3, 3).astype(float),
                       "N": data["n%d" % i].reshape(-1, 3, 3).astype(float),
                       "UV": data["t%d" % i].reshape(-1, 3, 2).astype(float)})
    return chunks, meta["materials"]


# ---------------------------------------------------------------- frame

def orientation(forward, up):
    """3x3 R with new = R @ old: rows are the source axes that become +X (forward),
    +Y (left) and +Z (up)."""
    from .manifest import axis_vector
    f, u = np.array(axis_vector(forward)), np.array(axis_vector(up))
    if abs(f @ u) > 1e-9:
        raise MeshError("forward {} and up {} are not perpendicular".format(forward, up))
    return np.array([f, np.cross(u, f), u])


def to_vehicle_frame(chunks, scale, forward, up):
    """Chunks scaled, rotated into the vehicle frame and re-origined at the ground centre."""
    R = orientation(forward, up)
    out = []
    for c in chunks:
        out.append(dict(c, P=(c["P"] * scale) @ R.T, N=c["N"] @ R.T))
    allP = np.concatenate([c["P"].reshape(-1, 3) for c in out])
    lo, hi = allP.min(0), allP.max(0)
    offset = np.array([-(lo[0] + hi[0]) / 2, -(lo[1] + hi[1]) / 2, -lo[2]])
    for c in out:
        c["P"] = c["P"] + offset
    return out


def bbox(P):
    p = P.reshape(-1, 3)
    return p.min(0), p.max(0)


def group_boxes(chunks):
    boxes = {}
    for c in chunks:
        lo, hi = bbox(c["P"])
        if c["group"] in boxes:
            a = boxes[c["group"]]
            boxes[c["group"]] = (np.minimum(a[0], lo), np.maximum(a[1], hi))
        else:
            boxes[c["group"]] = (lo, hi)
    return boxes


# ---------------------------------------------------------------- wheels

def corner_of(centre):
    return ("front" if centre[0] > 0 else "rear") + "_" + ("left" if centre[1] > 0 else "right")


def find_wheels(boxes, spec):
    """{group: corner} for the four wheel groups, by name (spec.nodes) or by size (spec.detect)."""
    if "nodes" in spec:
        out = {}
        for corner, node in spec["nodes"].items():
            if node not in boxes:
                raise MeshError("wheel node {!r} ({}) is not a top-level group of the model; "
                                "groups are: {}".format(node, corner, sorted(boxes)))
            out[node] = corner
    else:
        dlo, dhi = spec["detect"]["diameter_m"]
        wlo, whi = spec["detect"]["width_m"]
        out = {}
        for g, (lo, hi) in boxes.items():
            sx, sy, sz = hi - lo
            if dlo < sx < dhi and dlo < sz < dhi and wlo < sy < whi:
                out[g] = corner_of((lo + hi) / 2)
        if len(out) != 4:
            raise MeshError(
                "found {} wheel-sized groups ({}), need exactly 4. Adjust "
                "source.wheels.detect, or name them with source.wheels.nodes.".format(
                    len(out), sorted(out)))
    corners = sorted(out.values())
    if corners != sorted(BONE_OF):
        raise MeshError("wheels do not sit one per corner: {}".format(
            {g: c for g, c in out.items()}))
    return out


def _path_of(chunk):
    return chunk.get("path") or [chunk["group"]]


def wheel_of_chunks(chunks, spec):
    """(corner or None per chunk, {corner: (lo, hi, label, node names)}) for the four wheels."""
    if "parts" not in spec:
        boxes = group_boxes(chunks)
        groups = find_wheels(boxes, spec)
        return ([groups.get(c["group"]) for c in chunks],
                {corner: boxes[g] + (g, None) for g, corner in groups.items()})
    hits = {corner: set() for corner in spec["parts"]}
    used = set()
    of = []
    for c in chunks:
        path = _path_of(c)
        mine = set()
        for corner, pats in spec["parts"].items():
            for i, pat in enumerate(pats):
                found = [n for n in path if node_name_matches(pat, n)]
                if found:
                    mine.add(corner)
                    hits[corner].update(found)
                    used.add((corner, i))
        if len(mine) > 1:
            raise MeshError("node {} matches source.wheels.parts of {}".format(
                "/".join(path), sorted(mine)))
        of.append(mine.pop() if mine else None)
    for corner, pats in spec["parts"].items():
        for i, pat in enumerate(pats):
            if (corner, i) not in used:
                raise MeshError("source.wheels.parts.{} {!r} matches no mesh node".format(corner, pat))
    out = {}
    for corner in BONE_OF:
        P = np.concatenate([c["P"].reshape(-1, 3) for c, w in zip(chunks, of) if w == corner])
        lo, hi = bbox(P)
        at = corner_of((lo + hi) / 2)
        if at != corner:
            raise MeshError("source.wheels.parts.{} sits at the {} corner of the model".format(corner, at))
        out[corner] = (lo, hi, corner, sorted(hits[corner]))
    return of, out


# ---------------------------------------------------------------- faces

def dedupe(P, N, UV, keep, cabin_centre):
    """Drop reversed duplicate triangles (SketchUp exports a twin of every face).

    keep: "both" (no change), "first" (one of each twin set), "outward" (the twin facing
    away from `cabin_centre`)."""
    if keep == "both" or not len(P):
        return P, N, UV
    Q = np.round(P * 1e5).astype(np.int64)
    h = Q[..., 0] * 73856093 ^ Q[..., 1] * 19349663 ^ Q[..., 2] * 83492791
    order = np.argsort(h, axis=1, kind="stable")
    key = np.take_along_axis(Q, order[..., None], axis=1).reshape(len(Q), 9)
    if keep == "first":
        _, idx = np.unique(key, axis=0, return_index=True)
        idx.sort()
        return P[idx], N[idx], UV[idx]
    fn = np.cross(P[:, 1] - P[:, 0], P[:, 2] - P[:, 0])
    out = (fn * (P.mean(1) - np.asarray(cabin_centre))).sum(1) > 0
    _, inv = np.unique(key, axis=0, return_inverse=True)
    inv = np.asarray(inv).ravel()
    chosen = {}
    for i in np.argsort(~out, kind="stable"):  # outward-facing candidates first
        chosen.setdefault(int(inv[i]), int(i))
    idx = np.array(sorted(chosen.values()))
    return P[idx], N[idx], UV[idx]


def fit_plates(P, size_m, standoff=0.004):
    """Resize each plate quad (front, rear) in its own plane to `size_m` = [width, height].

    The plate texture spans the quad's UVs, so this sets the size the plate is drawn at.
    `standoff` lifts it off the bumper so the larger quad does not sink into it."""
    P = np.array(P, dtype=float)
    out = P.copy()
    for side in (P[:, :, 0].mean(1) >= 0, P[:, :, 0].mean(1) < 0):
        if not side.any():
            continue
        q = P[side].reshape(-1, 3)
        c0 = (q.min(0) + q.max(0)) / 2
        normal = np.linalg.svd(q - q.mean(0))[2][2]
        if normal[0] * c0[0] < 0:
            normal = -normal
        lat = np.array([0.0, 1.0, 0.0]) - normal[1] * normal
        lat /= np.linalg.norm(lat)
        up = np.cross(normal, lat)
        if up[2] < 0:
            up = -up
        a, b = (q - c0) @ lat, (q - c0) @ up
        sa = size_m[0] / max(np.ptp(a), 1e-9)
        sb = size_m[1] / max(np.ptp(b), 1e-9)
        mid_a, mid_b = (a.max() + a.min()) / 2, (b.max() + b.min()) / 2
        new = (c0 + np.outer((a - mid_a) * sa + mid_a, lat) + np.outer((b - mid_b) * sb + mid_b, up)
               + standoff * normal)
        out[side] = new.reshape(-1, 3, 3)
    return out


# ---------------------------------------------------------------- materials

PLATE_SLOT = "M_LicensePlate"


def _stripped(name, materials_cfg):
    for s in materials_cfg.get("strip") or []:
        name = name.replace(s, "")
    return name.strip("_")


def _as_list(v):
    return v if isinstance(v, list) else ([] if v is None else [v])


def _form_matches(pattern, candidates):
    """{exact: s} / {regex: r} against any of `candidates`."""
    (form, s), = pattern.items()
    if form == "exact":
        return s in candidates
    return any(re.search(s, n) for n in candidates)


def material_matches(pattern, name, materials_cfg, plain="substring"):
    """Whether a manifest material pattern names material `name`.

    A plain string keeps its key's meaning: a substring of the name, or for the plate
    (plain="exact") the name with or without `strip`. {exact} and {regex} test both."""
    candidates = (name, _stripped(name, materials_cfg))
    if isinstance(pattern, dict):
        return _form_matches(pattern, candidates)
    return pattern in candidates if plain == "exact" else pattern in name


def node_name_matches(pattern, name):
    """Whether a node pattern (plain string = exact name) names node `name`.

    Blender's .001 suffix on a duplicate name is ignored."""
    candidates = (name, re.sub(r"\.\d{3}$", "", name))
    if isinstance(pattern, dict):
        return _form_matches(pattern, candidates)
    return pattern in candidates


def role_of(name, materials_cfg):
    plate = materials_cfg.get("plate")
    if plate and any(material_matches(p, name, materials_cfg, "exact") for p in _as_list(plate.get("match"))):
        return "plate"
    for role in ("paint", "light_glass", "glass"):
        if any(material_matches(p, name, materials_cfg) for p in materials_cfg.get(role) or []):
            return role
    return "opaque"


def slot_name(name, role, materials_cfg):
    if role == "paint":
        return PAINT_SLOT
    if role == "plate":
        return PLATE_SLOT
    name = _stripped(name, materials_cfg)
    clean = "".join(ch if ch.isalnum() or ch == "_" else "_" for ch in name).strip("_")
    return "M_" + (clean or "Material")


def _clean_look(look):
    return {k: v for k, v in look.items()
            if k not in ("match", "slot", "texture", "nodes", "role") and v is not None}


def look_for(name, materials_cfg):
    """The first `materials.look` entry whose match names `name`, else {}."""
    for look in materials_cfg.get("look") or []:
        if look.get("match") and material_matches(look["match"], name, materials_cfg):
            return _clean_look(look)
    return {}


def wheel_slot_for(name, materials_cfg):
    """The `materials.wheel_slots` entry for a material on a wheel bone, or None."""
    for ws in materials_cfg.get("wheel_slots") or []:
        if ws.get("match") and material_matches(ws["match"], name, materials_cfg):
            return ws
    return None


def node_slot_for(path, materials_cfg):
    """The first `materials.node_slots` entry naming a node on `path`, or None."""
    for ns in materials_cfg.get("node_slots") or []:
        if any(node_name_matches(p, n) for p in ns["nodes"] for n in path):
            return ns
    return None


# ---------------------------------------------------------------- assemble

def build_parts(chunks, materials, m):
    """(arrays, meta) ready for blender_rig.py, from chunks already in the vehicle frame.

    arrays: npz-able {v<i>, n<i>, t<i>, f<i>} per (bone, slot) part.
    meta: {parts, wheels, slots, slot_info, size}.
    """
    src, mats_cfg = m["source"], m["materials"]
    wheel_of, wheel_boxes = wheel_of_chunks(chunks, src["wheels"])
    allP = np.concatenate([c["P"].reshape(-1, 3) for c in chunks])
    lo, hi = allP.min(0), allP.max(0)
    cabin = np.array([0.0, 0.0, (hi[2] - lo[2]) / 2])

    wheels = {}
    for corner, (wlo, whi, label, nodes) in wheel_boxes.items():
        wheels[BONE_OF[corner]] = {"bone": BONE_OF[corner], "group": label,
                                   "centre": ((wlo + whi) / 2).tolist(), "size": (whi - wlo).tolist()}
        if nodes is not None:
            wheels[BONE_OF[corner]]["nodes"] = nodes

    for i, ns in enumerate(mats_cfg.get("node_slots") or []):
        for pat in ns["nodes"]:
            if not any(node_name_matches(pat, n) for c in chunks for n in _path_of(c)):
                raise MeshError("materials.node_slots[{}] {!r} matches no mesh node".format(i, pat))

    parts, slot_info = {}, {}
    for c, corner in zip(chunks, wheel_of):
        info = materials.get(c["material"]) or {"name": str(c["material"]),
                                                "color": [0.8, 0.8, 0.8, 1.0], "texture": None}
        name = info["name"]
        role = role_of(name, mats_cfg)
        ns = node_slot_for(_path_of(c), mats_cfg)
        if ns is not None and ns.get("role"):
            role = ns["role"]
        slot = slot_name(name, role, mats_cfg)
        keep = "both" if role == "paint" else ("outward" if role in ("glass", "light_glass") else "first")
        if not src["dedupe_twins"]:
            keep = "both"
        P, N, UV = dedupe(c["P"], c["N"], c["UV"], keep, cabin)
        if role == "plate" and (mats_cfg["plate"] or {}).get("size_m"):
            P = fit_plates(P, mats_cfg["plate"]["size_m"])
        bone = BONE_OF[corner] if corner else BASE_BONE
        look = look_for(name, mats_cfg)
        ws = (wheel_slot_for(name, mats_cfg)
              if ns is None and bone != BASE_BONE and role == "opaque" else None)
        if ws is not None:
            slot, look = "M_" + ws["slot"], _clean_look(ws)
        texture = info.get("texture")
        if role == "plate":
            look = _clean_look(mats_cfg["plate"])
            texture = mats_cfg["plate"].get("texture") or texture
        if ns is not None:
            if role != "paint":
                slot = "M_" + ns["slot"]
            look = _clean_look(ns) or look
        k = (bone, slot)
        if k in parts:
            parts[k] = tuple(np.concatenate([a, b]) for a, b in zip(parts[k], (P, N, UV)))
        else:
            parts[k] = (P, N, UV)
        if slot not in slot_info:
            slot_info[slot] = {"slot": slot, "role": role, "source_material": name,
                               "color": list(info.get("color") or [0.8, 0.8, 0.8, 1.0]),
                               "texture": texture, "look": look}

    arrays, out_parts = {}, []
    for i, ((bone, slot), (P, N, UV)) in enumerate(sorted(parts.items())):
        if not len(P):
            continue
        n = np.linalg.norm(N, axis=2, keepdims=True)
        N = np.where(n > 1e-12, N / np.maximum(n, 1e-12), 0)
        corners = np.concatenate([P.reshape(-1, 3), N.reshape(-1, 3), UV.reshape(-1, 2)], 1)
        uniq, inv = np.unique(np.round(corners, 6), axis=0, return_inverse=True)
        arrays["v%d" % i] = uniq[:, :3].astype(np.float32)
        arrays["n%d" % i] = uniq[:, 3:6].astype(np.float32)
        arrays["t%d" % i] = uniq[:, 6:8].astype(np.float32)
        arrays["f%d" % i] = np.asarray(inv).reshape(-1, 3).astype(np.int32)
        out_parts.append({"idx": i, "bone": bone, "slot": slot, "tris": int(len(P))})
    meta = {"parts": out_parts, "wheels": [wheels[b] for b in sorted(wheels)],
            "slots": sorted({p["slot"] for p in out_parts}),
            "slot_info": [slot_info[s] for s in sorted({p["slot"] for p in out_parts})],
            "size": (hi - lo).tolist()}
    return arrays, meta


def measure(meta):
    """Dimensions of the prepared model, in the keys manifest.check_dimensions reads."""
    w = {x["bone"]: np.array(x["centre"]) for x in meta["wheels"]}
    sizes = [x["size"] for x in meta["wheels"]]
    fl, fr = w["Wheel_Front_Left"], w["Wheel_Front_Right"]
    rl, rr = w["Wheel_Rear_Left"], w["Wheel_Rear_Right"]
    return {
        "length_m": meta["size"][0],
        "width_mirrors_m": meta["size"][1],
        "height_m": meta["size"][2],
        "wheelbase_m": float((fl[0] + fr[0]) / 2 - (rl[0] + rr[0]) / 2),
        "track_front_m": float(fl[1] - fr[1]),
        "track_rear_m": float(rl[1] - rr[1]),
        "wheel_radius_m": float(np.mean([(s[0] + s[2]) / 4 for s in sizes])),
        "wheel_width_m": float(np.mean([s[1] for s in sizes])),
        "triangles": int(sum(p["tris"] for p in meta["parts"])),
    }
