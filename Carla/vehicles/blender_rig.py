"""Blender side of prep_vehicle.py. Run headless; never open a downloaded .blend with this.

  blender --background --disable-autoexec --factory-startup --python blender_rig.py -- rig <work_dir> <out.fbx> <decimate>
  blender --background --disable-autoexec --factory-startup --python blender_rig.py -- dump <model> <format> <work_dir>

rig:  reads <work_dir>/parts.npz + parts.json (mesh.build_parts), builds the mesh bound to
      CARLA's 4-wheel skeleton (Vehicle_Base + four wheel bones, rigid weights), exports FBX.
dump: imports an FBX / glTF / OBJ and writes its triangles as dump.npz + dump.json, the
      chunk format mesh.read_npz_dump reads, plus its textures under <work_dir>/textures.
"""
import json
import math
import os
import sys

import bpy
import numpy as np

BONES = ["Vehicle_Base", "Wheel_Front_Left", "Wheel_Front_Right", "Wheel_Rear_Right", "Wheel_Rear_Left"]
CM = 100.0


def rig(work, out_fbx, decimate):
    data = np.load(os.path.join(work, "parts.npz"))
    with open(os.path.join(work, "parts.json"), "r", encoding="utf-8") as fh:
        meta = json.load(fh)
    bpy.ops.wm.read_factory_settings(use_empty=True)
    scene = bpy.context.scene
    # 1 BU = 1 cm, so no bone carries a x100 scale into Unreal
    scene.unit_settings.system, scene.unit_settings.scale_length = "METRIC", 0.01

    slots = meta["slots"]
    V, N, UV, F, M, B = [], [], [], [], [], []
    base = 0
    for p in meta["parts"]:
        i = p["idx"]
        v, n, t, f = data["v%d" % i], data["n%d" % i], data["t%d" % i], data["f%d" % i]
        V.append(v); N.append(n); UV.append(t); F.append(f + base)
        M.append(np.full(len(f), slots.index(p["slot"]), np.int32))
        B.append(np.full(len(v), BONES.index(p["bone"]), np.int32))
        base += len(v)
    V, N, UV, F, M, B = map(np.concatenate, (V, N, UV, F, M, B))
    V = V * CM

    name = os.path.splitext(os.path.basename(out_fbx))[0]
    me = bpy.data.meshes.new(name)
    me.vertices.add(len(V)); me.vertices.foreach_set("co", V.ravel())
    me.loops.add(F.size); me.loops.foreach_set("vertex_index", F.ravel())
    me.polygons.add(len(F)); me.polygons.foreach_set("loop_start", np.arange(0, F.size, 3, dtype=np.int32))
    me.polygons.foreach_set("material_index", M)
    me.uv_layers.new(name="UVMap").data.foreach_set("uv", UV[F.ravel()].ravel())
    me.update(calc_edges=True)
    for s in slots:
        me.materials.append(bpy.data.materials.new(s))
    me.normals_split_custom_set_from_vertices([tuple(x) for x in N])

    ob = bpy.data.objects.new(name, me)
    scene.collection.objects.link(ob)
    for bi, b in enumerate(BONES):
        ob.vertex_groups.new(name=b).add(np.nonzero(B == bi)[0].tolist(), 1.0, "REPLACE")

    if decimate < 1.0:
        bpy.context.view_layer.objects.active = ob
        mod = ob.modifiers.new("Decimate", "DECIMATE")
        mod.ratio = decimate
        bpy.ops.object.modifier_apply(modifier=mod.name)
        # decimation invalidates the imported normals; smooth by angle instead
        if hasattr(me, "set_sharp_from_angle"):
            me.set_sharp_from_angle(angle=math.radians(30.0))
        me.shade_smooth()

    # an armature object named "Armature" is not turned into an extra root bone by UE
    arm = bpy.data.armatures.new("Armature")
    rigob = bpy.data.objects.new("Armature", arm)
    scene.collection.objects.link(rigob)
    bpy.context.view_layer.objects.active = rigob
    bpy.ops.object.mode_set(mode="EDIT")
    centres = {w["bone"]: w["centre"] for w in meta["wheels"]}
    root = arm.edit_bones.new("Vehicle_Base")
    root.head, root.tail = (0, 0, 0), (0, 30.0, 0)
    for b in BONES[1:]:
        e = arm.edit_bones.new(b)
        c = [x * CM for x in centres[b]]
        e.head, e.tail = tuple(c), (c[0], c[1] + 30.0, c[2])
        e.parent = root
    bpy.ops.object.mode_set(mode="OBJECT")
    ob.parent = rigob
    ob.modifiers.new("Armature", "ARMATURE").object = rigob

    bpy.ops.object.select_all(action="DESELECT")
    ob.select_set(True); rigob.select_set(True)
    bpy.context.view_layer.objects.active = rigob
    os.makedirs(os.path.dirname(os.path.abspath(out_fbx)), exist_ok=True)
    bpy.ops.export_scene.fbx(filepath=out_fbx, use_selection=True, object_types={"ARMATURE", "MESH"},
                             apply_scale_options="FBX_SCALE_NONE", axis_forward="-Z", axis_up="Y",
                             mesh_smooth_type="OFF", use_mesh_modifiers=False, add_leaf_bones=False,
                             primary_bone_axis="Y", secondary_bone_axis="X", bake_anim=False,
                             use_armature_deform_only=True, path_mode="STRIP")
    stats = {"fbx": out_fbx, "bytes": os.path.getsize(out_fbx), "vertices": len(me.vertices),
             "triangles": sum(len(p.vertices) - 2 for p in me.polygons), "slots": len(slots),
             "decimate": decimate}
    print("[blender_rig] " + json.dumps(stats))
    with open(os.path.join(work, "rig_stats.json"), "w", encoding="utf-8") as fh:
        json.dump(stats, fh, indent=1)


def _top(ob):
    while ob.parent is not None:
        ob = ob.parent
    return ob.name


def _material_info(mat, tex_dir):
    d = {"name": mat.name if mat else "None", "color": [0.8, 0.8, 0.8, 1.0], "texture": None}
    if mat is None:
        return d
    d["color"] = [float(x) for x in mat.diffuse_color]
    if mat.node_tree:
        for node in mat.node_tree.nodes:
            if node.type != "BSDF_PRINCIPLED":
                continue
            inp = node.inputs.get("Base Color")
            if inp is None:
                continue
            d["color"] = [float(x) for x in inp.default_value]
            for link in inp.links:
                img = getattr(link.from_node, "image", None)
                if img is None:
                    continue
                fname = "".join(ch if ch.isalnum() or ch in "._-" else "_" for ch in img.name)
                if not os.path.splitext(fname)[1]:
                    fname += ".png"
                os.makedirs(tex_dir, exist_ok=True)
                img.save(filepath=os.path.join(tex_dir, fname))
                d["texture"] = fname
            break
    return d


def dump(model, fmt, work):
    bpy.ops.wm.read_factory_settings(use_empty=True)
    if fmt == "fbx":
        bpy.ops.import_scene.fbx(filepath=model)
    elif fmt == "gltf":
        bpy.ops.import_scene.gltf(filepath=model)
    elif fmt == "obj":
        bpy.ops.wm.obj_import(filepath=model)
    else:
        raise SystemExit("blender_rig dump: unsupported format %r" % fmt)
    deps = bpy.context.evaluated_depsgraph_get()
    tex_dir = os.path.join(work, "textures")
    arrays, chunks, materials = {}, [], {}
    for ob in bpy.context.scene.objects:
        if ob.type != "MESH":
            continue
        ev = ob.evaluated_get(deps)
        me = ev.to_mesh()
        me.calc_loop_triangles()
        W = np.array(ob.matrix_world)
        co = np.empty(len(me.vertices) * 3); me.vertices.foreach_get("co", co)
        co = co.reshape(-1, 3) @ W[:3, :3].T + W[:3, 3]
        nrm = np.empty(len(me.loops) * 3); me.corner_normals.foreach_get("vector", nrm)
        nrm = nrm.reshape(-1, 3) @ np.linalg.inv(W[:3, :3])
        uvl = me.uv_layers.active
        uv = np.zeros(len(me.loops) * 2)
        if uvl is not None:
            uvl.data.foreach_get("uv", uv)
        uv = uv.reshape(-1, 2)
        tri_v = np.empty(len(me.loop_triangles) * 3, np.int32); me.loop_triangles.foreach_get("vertices", tri_v)
        tri_l = np.empty(len(me.loop_triangles) * 3, np.int32); me.loop_triangles.foreach_get("loops", tri_l)
        tri_m = np.empty(len(me.loop_triangles), np.int32); me.loop_triangles.foreach_get("material_index", tri_m)
        tri_v, tri_l = tri_v.reshape(-1, 3), tri_l.reshape(-1, 3)
        for mi in np.unique(tri_m):
            mat = me.materials[mi] if mi < len(me.materials) else None
            key = mat.name if mat else "None"
            if key not in materials:
                materials[key] = _material_info(mat, tex_dir)
            sel = tri_m == mi
            i = len(chunks)
            arrays["p%d" % i] = co[tri_v[sel]].astype(np.float32)
            arrays["n%d" % i] = nrm[tri_l[sel]].astype(np.float32)
            arrays["t%d" % i] = uv[tri_l[sel]].astype(np.float32)
            chunks.append({"group": _top(ob), "material": key})
        ev.to_mesh_clear()
    os.makedirs(work, exist_ok=True)
    np.savez_compressed(os.path.join(work, "dump.npz"), **arrays)
    with open(os.path.join(work, "dump.json"), "w", encoding="utf-8") as fh:
        json.dump({"chunks": chunks, "materials": materials}, fh, indent=1)
    print("[blender_rig] dumped %d chunks from %s" % (len(chunks), model))


def main():
    args = sys.argv[sys.argv.index("--") + 1:]
    if args[0] == "rig":
        rig(args[1], args[2], float(args[3]))
    elif args[0] == "dump":
        dump(args[1], args[2], args[3])
    else:
        raise SystemExit("usage: blender_rig.py -- rig|dump ...")


if __name__ == "__main__":
    main()
