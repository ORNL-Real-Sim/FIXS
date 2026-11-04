"""Unreal editor script: build, register or unregister one library vehicle.

Run by import_vehicle.py as `UE4Editor <uproject> <map> -ExecutePythonScript=<this>` (the
-run=pythonscript commandlet has no viewport and crashes on these assets). Every input
comes from the job JSON named by FIXS_VEHICLE_JOB; the result JSON goes to job["result"].

Writes only under job["dest"] and job["shared"], plus the one VehicleFactory entry for
job["id"]. Stock assets named in job["stock"] are read or duplicated, never saved.
"""
import json
import os
import sys
import traceback

import unreal

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import factory  # noqa: E402

at = unreal.AssetToolsHelpers.get_asset_tools()
eal = unreal.EditorAssetLibrary
mel = unreal.MaterialEditingLibrary
res = {"ok": False, "log": [], "saved": {}}


def log(msg):
    unreal.log_warning("[fixs-vehicle] " + str(msg))
    res["log"].append(str(msg))


def cdo_of(path):
    return unreal.get_default_object(eal.load_blueprint_class(path))


def must_load(path):
    a = unreal.load_asset(path)
    if a is None:
        raise RuntimeError("stock asset not found: {}".format(path))
    return a


def save(path):
    ok = eal.save_asset(path, only_if_is_dirty=False)
    res["saved"][path] = bool(ok)
    if not ok:
        log("could not save {} (held open by another process?)".format(path))
    return ok


def srgb_to_lin(c):
    return c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4


def lin_color(rgb01):
    r, g, b = [srgb_to_lin(float(x)) for x in rgb01[:3]]
    return unreal.LinearColor(r, g, b, 1.0)


# ---------------------------------------------------------------- materials

def _connect(src, out, prop):
    if not mel.connect_material_property(src, out, prop):
        raise RuntimeError("connect_material_property failed: {}".format(prop))


def shared_master(job, name, textured):
    """A two-sided opaque master with BaseColor/Tex/Metallic/Roughness/Specular/Emissive params."""
    path = "{}/{}".format(job["shared"], name)
    if eal.does_asset_exist(path):
        return unreal.load_asset(path)
    m = at.create_asset(name, job["shared"], unreal.Material, unreal.MaterialFactoryNew())
    m.set_editor_property("two_sided", True)
    m.set_editor_property("used_with_skeletal_mesh", True)
    col = mel.create_material_expression(m, unreal.MaterialExpressionVectorParameter, -600, 0)
    col.set_editor_property("parameter_name", "BaseColor")
    col.set_editor_property("default_value", unreal.LinearColor(0.5, 0.5, 0.5, 1))
    rgb = mel.create_material_expression(m, unreal.MaterialExpressionComponentMask, -450, 0)
    for ch, on in (("r", True), ("g", True), ("b", True), ("a", False)):
        rgb.set_editor_property(ch, on)
    if not mel.connect_material_expressions(col, "", rgb, ""):
        raise RuntimeError("colour mask wiring failed")
    if textured:
        tex = mel.create_material_expression(m, unreal.MaterialExpressionTextureSampleParameter2D, -600, -250)
        tex.set_editor_property("parameter_name", "Tex")
        mul = mel.create_material_expression(m, unreal.MaterialExpressionMultiply, -300, -100)
        if not (mel.connect_material_expressions(tex, "RGB", mul, "A")
                and mel.connect_material_expressions(rgb, "", mul, "B")):
            raise RuntimeError("texture wiring failed")
        _connect(mul, "", unreal.MaterialProperty.MP_BASE_COLOR)
    else:
        _connect(rgb, "", unreal.MaterialProperty.MP_BASE_COLOR)
    for i, (pname, dv, prop) in enumerate([("Metallic", 0.0, unreal.MaterialProperty.MP_METALLIC),
                                          ("Roughness", 0.6, unreal.MaterialProperty.MP_ROUGHNESS),
                                          ("Specular", 0.5, unreal.MaterialProperty.MP_SPECULAR)]):
        s = mel.create_material_expression(m, unreal.MaterialExpressionScalarParameter, -600, 200 + 120 * i)
        s.set_editor_property("parameter_name", pname)
        s.set_editor_property("default_value", dv)
        _connect(s, "", prop)
    em = mel.create_material_expression(m, unreal.MaterialExpressionScalarParameter, -900, 600)
    em.set_editor_property("parameter_name", "Emissive")
    em.set_editor_property("default_value", 0.0)
    emul = mel.create_material_expression(m, unreal.MaterialExpressionMultiply, -300, 600)
    if not (mel.connect_material_expressions(rgb, "", emul, "A")
            and mel.connect_material_expressions(em, "", emul, "B")):
        raise RuntimeError("emissive wiring failed")
    _connect(emul, "", unreal.MaterialProperty.MP_EMISSIVE_COLOR)
    mel.recompile_material(m)
    save(path)
    return m


def material_instance(job, name, parent):
    path = "{}/{}".format(job["dest"], name)
    mi = unreal.load_asset(path) if eal.does_asset_exist(path) else at.create_asset(
        name, job["dest"], unreal.MaterialInstanceConstant, unreal.MaterialInstanceConstantFactoryNew())
    mel.set_material_instance_parent(mi, parent)
    return mi


def import_textures(job, names):
    out = {}
    tasks = []
    for n in sorted(set(names)):
        f = os.path.join(job["textures_dir"], n)
        if not os.path.isfile(f):
            log("texture {} not in the bundle; slot left untextured".format(n))
            continue
        t = unreal.AssetImportTask()
        for k, v in dict(filename=f, destination_path=job["dest"] + "/Textures", automated=True,
                         replace_existing=True, save=False).items():
            t.set_editor_property(k, v)
        tasks.append((n, t))
    at.import_asset_tasks([t for _, t in tasks])
    for n, t in tasks:
        paths = list(t.get_editor_property("imported_object_paths"))
        if paths:
            out[n] = unreal.load_asset(paths[0])
    return out


def build_materials(job):
    """{slot name: material interface} for every slot in the bundle's materials.json."""
    stock = job["stock"]
    opaque = shared_master(job, "M_FIXS_VehicleOpaque", textured=False)
    textured = shared_master(job, "M_FIXS_VehicleTextured", textured=True)
    texmap = import_textures(job, [s["texture"] for s in job["slots"] if s.get("texture")])
    out = {}
    for s in job["slots"]:
        role, look = s["role"], s.get("look") or {}
        if role == "paint":
            mi = material_instance(job, "MI_{}_Bodywork".format(job["asset_name"]), must_load(stock["paint_parent"]))
            mel.set_material_instance_vector_parameter_value(mi, "Base Color", lin_color(
                [c / 255.0 for c in job["paint_color"]]))
        elif role == "glass":
            mi = must_load(stock["glass"])
        elif role == "light_glass":
            mi = must_load(stock["light_glass"])
        else:
            tex = texmap.get(s.get("texture"))
            mi = material_instance(job, "MI_{}_{}".format(job["asset_name"], s["slot"][2:]),
                                   textured if tex is not None else opaque)
            color = [c / 255.0 for c in look["color"]] if look.get("color") else s["color"]
            if tex is not None:
                mel.set_material_instance_texture_parameter_value(mi, "Tex", tex)
                if not look.get("color"):
                    color = [1.0, 1.0, 1.0]
            mel.set_material_instance_vector_parameter_value(mi, "BaseColor", lin_color(color))
            for p, k in (("Metallic", "metallic"), ("Roughness", "roughness"), ("Specular", "specular"),
                         ("Emissive", "emissive")):
                if look.get(k) is not None:
                    mel.set_material_instance_scalar_parameter_value(mi, p, float(look[k]))
        out[s["slot"]] = mi
    return out


# ---------------------------------------------------------------- mesh + physics

def import_skeletal(job):
    stock = job["stock"]
    ui = unreal.FbxImportUI()
    for k, v in dict(import_mesh=True, import_as_skeletal=True, import_materials=False,
                     import_textures=False, import_animations=False, create_physics_asset=False,
                     mesh_type_to_import=unreal.FBXImportType.FBXIT_SKELETAL_MESH,
                     skeleton=must_load(stock["skeleton"])).items():
        ui.set_editor_property(k, v)
    sd = ui.get_editor_property("skeletal_mesh_import_data")
    for k, v in dict(import_morph_targets=False, convert_scene=True, convert_scene_unit=True,
                     normal_import_method=unreal.FBXNormalImportMethod.FBXNIM_IMPORT_NORMALS).items():
        sd.set_editor_property(k, v)
    t = unreal.AssetImportTask()
    for k, v in dict(filename=job["fbx"], destination_path=job["dest"],
                     destination_name="SK_" + job["asset_name"], automated=True,
                     replace_existing=True, save=False, options=ui).items():
        t.set_editor_property(k, v)
    at.import_asset_tasks([t])
    paths = list(t.get_editor_property("imported_object_paths"))
    log("import: {}".format(paths))
    sks = [x for x in (unreal.load_asset(p) for p in paths) if isinstance(x, unreal.SkeletalMesh)]
    if not sks:
        raise RuntimeError("skeletal mesh import produced no SkeletalMesh: {}".format(paths))
    return sks[0]


def physics_asset(job, sk):
    dst = "{}/PA_{}".format(job["dest"], job["asset_name"])
    pa = unreal.load_asset(dst) if eal.does_asset_exist(dst) else eal.duplicate_asset(job["stock"]["physics_asset"], dst)
    box, info = job["collision_box_cm"], []
    try:
        setups = pa.get_editor_property("skeletal_body_setups")
    except Exception as e:
        # UE 4.26 Python does not expose the bodies; the duplicated stock shapes stay
        setups = []
        info.append("body shapes not editable from Python ({}); kept the template's".format(type(e).__name__))
    for bs in setups:
        bone = str(bs.get_editor_property("bone_name"))
        ag = bs.get_editor_property("agg_geom")
        boxes = list(ag.get_editor_property("box_elems"))
        sph = list(ag.get_editor_property("sphere_elems"))
        if bone == "Vehicle_Base" and boxes:
            b = boxes[0]
            b.set_editor_property("center", unreal.Vector(*box["center"]))
            for k in ("x", "y", "z"):
                b.set_editor_property(k, float(box[k]))
            ag.set_editor_property("box_elems", [b] + boxes[1:])
            bs.set_editor_property("agg_geom", ag)
        if bone.startswith("Wheel") and sph:
            s = sph[0]
            s.set_editor_property("radius", float(job["wheels"]["radius_cm"]))
            ag.set_editor_property("sphere_elems", [s] + sph[1:])
            bs.set_editor_property("agg_geom", ag)
        info.append({"bone": bone, "boxes": len(boxes), "spheres": len(sph)})
    res["physics_asset"] = info
    sk.set_editor_property("physics_asset", pa)
    return pa


def wheel_bp(job, src, name, steer_deg):
    dst = "{}/{}".format(job["dest"], name)
    if not eal.does_asset_exist(dst):  # reused once created: the vehicle BP references it
        eal.duplicate_asset(src, dst)
    w = cdo_of(dst)
    w.set_editor_property("shape_radius", float(job["wheels"]["radius_cm"]))
    w.set_editor_property("shape_width", float(job["wheels"]["width_cm"]))
    w.set_editor_property("steer_angle", float(steer_deg))
    res.setdefault("wheels", {})[name] = {k: w.get_editor_property(k) for k in
                                          ("shape_radius", "shape_width", "steer_angle", "affected_by_handbrake")}
    return dst


def vehicle_bp(job, sk, front_path, rear_path):
    stock = job["stock"]
    name = "BP_" + job["asset_name"]
    dst = "{}/{}".format(job["dest"], name)
    if eal.does_asset_exist(dst):  # reused: the VehicleFactory entry references this class
        bp = unreal.load_asset(dst)
    else:
        f = unreal.BlueprintFactory()
        f.set_editor_property("parent_class", eal.load_blueprint_class(stock["base_pawn"]))
        bp = at.create_asset(name, job["dest"], None, f)
    lib = getattr(unreal, "BlueprintEditorLibrary", None)
    if lib is not None and hasattr(lib, "compile_blueprint"):
        lib.compile_blueprint(bp)
    cdo, tmpl = cdo_of(dst), cdo_of(stock["template_vehicle"])
    mesh = cdo.get_editor_property("mesh")
    mesh.set_editor_property("skeletal_mesh", sk)
    mesh.set_editor_property("anim_class", eal.load_blueprint_class(stock["anim_class"]))
    vm, tvm = cdo.get_editor_property("vehicle_movement"), tmpl.get_editor_property("vehicle_movement")
    for p in ("engine_setup", "transmission_setup", "differential_setup", "steering_curve",
              "inertia_tensor_scale", "min_normalized_tire_load", "max_normalized_tire_load",
              "min_normalized_tire_load_filtered", "max_normalized_tire_load_filtered",
              "threshold_longitudinal_speed", "low_forward_speed_sub_step_count",
              "high_forward_speed_sub_step_count", "ackermann_accuracy", "idle_brake_input",
              "stop_threshold", "wrong_direction_threshold"):
        try:
            vm.set_editor_property(p, tvm.get_editor_property(p))
        except Exception as e:
            log("copy {}: {}".format(p, type(e).__name__))
    body = job["body"]
    vm.set_editor_property("mass", float(body["mass_kg"]))
    vm.set_editor_property("drag_coefficient", float(body["drag_coefficient"]))
    vm.set_editor_property("chassis_width", float(body["chassis_width_cm"]))
    vm.set_editor_property("chassis_height", float(body["chassis_height_cm"]))
    dt = job.get("drivetrain")
    if dt:
        ds = vm.get_editor_property("differential_setup")
        ds.set_editor_property("differential_type",
                               getattr(unreal.VehicleDifferential4W, dt["differential"].upper()))
        ds.set_editor_property("front_rear_split", float(dt["front_rear_split"]))
        vm.set_editor_property("differential_setup", ds)
    front_cls, rear_cls = eal.load_blueprint_class(front_path), eal.load_blueprint_class(rear_path)
    ws = list(tvm.get_editor_property("wheel_setups"))
    for w in ws:
        w.set_editor_property("wheel_class", front_cls if "Front" in str(w.get_editor_property("bone_name")) else rear_cls)
    vm.set_editor_property("wheel_setups", ws)
    res["vehicle_cdo"] = {
        "mesh": mesh.get_editor_property("skeletal_mesh").get_path_name(),
        "mass": vm.get_editor_property("mass"), "drag": vm.get_editor_property("drag_coefficient"),
        "wheels": [[str(w.get_editor_property("bone_name")), w.get_editor_property("wheel_class").get_path_name()]
                   for w in vm.get_editor_property("wheel_setups")]}
    save(dst)
    return dst


# ---------------------------------------------------------------- VehicleFactory

def _to_dict(vp):
    d = {}
    for k in factory.FIELDS:
        v = vp.get_editor_property(k)
        if k == "class_":
            v = v.get_path_name() if v is not None else ""
        elif k == "recommended_colors":
            v = [[c.r, c.g, c.b, c.a] for c in v]
        elif k == "supported_drivers":
            v = list(v)
        elif isinstance(v, unreal.Name) or k in ("make", "model", "object_type", "base_type", "special_type"):
            v = str(v)
        d[k] = v
    return d


def _to_struct(d, original=None):
    if original is not None:
        return original
    vp = unreal.VehicleParameters()
    for k in factory.FIELDS:
        v = d[k]
        if k == "class_":
            v = eal.load_blueprint_class(v.rsplit(".", 1)[0]) if v else None
        elif k == "recommended_colors":
            cols = []
            for rgba in v:
                c = unreal.Color()
                for ch, x in zip("rgba", list(rgba) + [255] * (4 - len(rgba))):
                    c.set_editor_property(ch, int(x))
                cols.append(c)
            v = cols
        vp.set_editor_property(k, v)
    return vp


def edit_factory(job, new_entry):
    """Add/update (new_entry) or remove (None) job["id"] in the factory; save it."""
    fpath = job["stock"]["vehicle_factory"]
    fcdo = cdo_of(fpath)
    structs = list(fcdo.get_editor_property("Vehicles"))
    dicts = [_to_dict(v) for v in structs]
    owned = job.get("owned_ids", [])
    if new_entry is None:
        out, n = factory.remove(dicts, job["id"], owned)
        action = "removed" if n else "absent"
    else:
        out, action = factory.merge(dicts, new_entry, owned)
    res["factory"] = {"before": len(dicts), "after": len(out), "action": action}
    if action in ("unchanged", "absent"):
        return True
    # keep untouched stock structs as they are; only the edited entry is rebuilt
    keep = {id(d): s for d, s in zip(dicts, structs)}
    fcdo.set_editor_property("Vehicles", [_to_struct(d, keep.get(id(d))) for d in out])
    ok = save(fpath)
    res["factory"]["saved"] = ok
    res["factory"]["after_reload"] = len(cdo_of(fpath).get_editor_property("Vehicles"))
    return ok


# ---------------------------------------------------------------- main

def run(job):
    action = job["action"]
    if action == "unregister":
        res["ok"] = edit_factory(job, None)
        return
    if action == "register":
        res["ok"] = edit_factory(job, job["factory_entry"])
        return
    mats = build_materials(job)
    sk = import_skeletal(job)
    slots = list(sk.get_editor_property("materials"))
    missing = []
    for s in slots:
        name = str(s.get_editor_property("material_slot_name"))
        if name in mats:
            s.set_editor_property("material_interface", mats[name])
        else:
            missing.append(name)
    sk.set_editor_property("materials", slots)
    if missing:
        log("slots with no material in materials.json: {}".format(missing))
    comp = unreal.SkeletalMeshComponent()
    comp.set_skeletal_mesh(sk)
    res["bones"] = [[str(comp.get_bone_name(i)),
                     [round(x, 2) for x in (lambda v: (v.x, v.y, v.z))(comp.get_ref_pose_position(i))]]
                    for i in range(comp.get_num_bones())]
    bb = sk.get_bounds()
    res["bounds_cm"] = {"origin": [bb.origin.x, bb.origin.y, bb.origin.z],
                        "extent": [bb.box_extent.x, bb.box_extent.y, bb.box_extent.z]}
    physics_asset(job, sk)
    stock = job["stock"]
    front = wheel_bp(job, stock["front_wheel"], "BP_{}_FrontWheel".format(job["asset_name"]),
                     job["wheels"]["max_steer_deg"])
    rear = wheel_bp(job, stock["rear_wheel"], "BP_{}_RearWheel".format(job["asset_name"]),
                    job["wheels"]["rear_max_steer_deg"])
    cls_path = vehicle_bp(job, sk, front, rear)
    res["dir_saved"] = bool(eal.save_directory(job["dest"], only_if_is_dirty=False, recursive=True))
    res["class_path"] = cls_path
    ok = res["dir_saved"]
    if job.get("register"):
        ok = edit_factory(job, job["factory_entry"]) and ok
    res["ok"] = ok


def main():
    job = None
    try:
        with open(os.environ["FIXS_VEHICLE_JOB"], "r", encoding="utf-8") as fh:
            job = json.load(fh)
        run(job)
    except Exception:
        res["error"] = traceback.format_exc()
        unreal.log_error(res["error"])
    finally:
        out = (job or {}).get("result") or os.environ.get("FIXS_VEHICLE_RESULT")
        if out:
            with open(out, "w", encoding="utf-8") as fh:
                json.dump(res, fh, indent=1, default=str)
        unreal.SystemLibrary.quit_editor()


main()
