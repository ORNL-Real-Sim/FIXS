"""Vehicle library tooling (FIXS#470): manifest schema, catalog, factory edit, prep geometry,
the importer's editor job, and the CarlaSetup keys that use a library vehicle.

    python -m pytest tests/Python/unit/test_vehicle_library.py
"""
from __future__ import annotations

import copy
import os
import sys
import types
import zipfile

import numpy as np
import pytest
import yaml

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "..", "Carla"))
from vehicles import catalog, factory, install_guard, mesh  # noqa: E402
from vehicles import manifest as vm  # noqa: E402

MANIFEST = {
    "schema": 1, "id": "vehicle.acme.roadster", "make": "Acme", "model": "Roadster",
    "engine": {"unreal": "4.26", "carla": "0.9.15"},
    "credit": {"author": "someone", "license": "CC-BY-4.0", "source_url": "https://example.org/m"},
    "source": {"file": "source/model.dae", "scale": 1.0, "forward": "-y", "up": "+z",
               "dedupe_twins": True,
               "wheels": {"detect": {"diameter_m": [0.5, 0.9], "width_m": [0.1, 0.4]}}},
    "materials": {"paint": ["Body"], "glass": ["Glass"], "light_glass": ["Lamp"],
                  "look": [{"match": "Chrome", "metallic": 1.0, "roughness": 0.1}]},
    "dimensions": {"tolerance_m": 0.05, "length_m": 4.0, "width_m": 1.8, "height_m": 1.5,
                   "wheelbase_m": 2.6, "track_front_m": 1.5,
                   "track_rear_m": {"value": 1.55, "tolerance_m": 0.02},
                   "ground_clearance_m": 0.15},
    "wheels": {"radius_m": 0.35, "width_m": 0.22, "max_steer_deg": 38.0},
    "physics": {"mass_kg": 1500.0, "drag_coefficient": 0.3,
                "torque_curve": [[0, 400], [6000, 300]],
                "forward_gears": [{"ratio": 1.0, "down_ratio": 0.0, "up_ratio": 0.0}],
                "center_of_mass_m": [0.1, 0.0, -0.2],
                "tires": {"all": {"tire_friction": 3.0}, "rear": {"max_handbrake_torque": 1500.0}}},
    "drivetrain": {"differential": "limited_slip_4w", "front_torque_fraction": 0.4},
    "factory": {"base_type": "car", "special_type": "electric", "generation": 2,
                "recommended_colors": [[10, 20, 30], [200, 200, 200]]},
}


def validated(**changes):
    raw = copy.deepcopy(MANIFEST)
    for k, v in changes.items():
        if v is None:
            raw.pop(k)
        else:
            raw[k] = v
    return vm.validate(raw, "vehicle.yaml")


# ---------------------------------------------------------------- manifest

def test_a_complete_manifest_validates():
    m = validated()
    assert m["id"] == "vehicle.acme.roadster"
    assert m["asset_name"] == "AcmeRoadster"
    assert m["dimensions"]["track_rear_m"] == {"value": 1.55, "tolerance_m": 0.02}
    assert m["dimensions"]["length_m"]["tolerance_m"] == 0.05
    assert m["wheels"]["rear_max_steer_deg"] == 0.0
    assert m["drivetrain"] == {"differential": "limited_slip_4w", "front_rear_split": 0.4}
    assert m["source"]["format"] == "collada"


def test_every_missing_value_is_reported_at_once():
    raw = copy.deepcopy(MANIFEST)
    del raw["physics"]["mass_kg"]
    del raw["wheels"]["radius_m"]
    del raw["dimensions"]["wheelbase_m"]
    with pytest.raises(vm.ManifestError) as exc:
        vm.validate(raw, "v.yaml")
    text = str(exc.value)
    for key in ("physics.mass_kg", "wheels.radius_m", "dimensions.wheelbase_m"):
        assert key in text


def test_the_id_must_be_what_carla_derives_from_make_and_model():
    with pytest.raises(vm.ManifestError, match="vehicle.acme.roadster"):
        validated(id="vehicle.acme.other")


def test_colours_are_required_so_the_color_attribute_exists():
    raw = copy.deepcopy(MANIFEST)
    raw["factory"]["recommended_colors"] = []
    with pytest.raises(vm.ManifestError, match="color"):
        vm.validate(raw, "v.yaml")


def test_axes_must_be_perpendicular_and_named():
    src = dict(MANIFEST["source"], forward="+z", up="-z")
    with pytest.raises(vm.ManifestError, match="perpendicular"):
        validated(source=src)
    with pytest.raises(vm.ManifestError, match="source.forward"):
        validated(source=dict(MANIFEST["source"], forward="north"))


def test_a_light_rig_is_refused_rather_than_promised():
    raw = copy.deepcopy(MANIFEST)
    raw["factory"]["has_lights"] = True
    with pytest.raises(vm.ManifestError, match="has_lights"):
        vm.validate(raw, "v.yaml")


def test_curves_must_increase():
    raw = copy.deepcopy(MANIFEST)
    raw["physics"]["torque_curve"] = [[1000, 1], [500, 2]]
    with pytest.raises(vm.ManifestError, match="torque_curve"):
        vm.validate(raw, "v.yaml")


def test_source_is_optional_for_an_already_prepared_bundle():
    assert validated(source=None)["source"] is None


def test_the_ev6_library_manifest_validates():
    """The draft in the vehicle library, when that checkout sits beside this one."""
    path = os.environ.get("FIXS_EV6_MANIFEST", r"C:\src_git\RS_VehDynLib\3_ev6_model\vehicles"
                                                r"\vehicle.kia.ev6\vehicle.yaml")
    if not os.path.isfile(path):
        pytest.skip("vehicle library checkout not present")
    m = vm.load(path)
    assert m["id"] == "vehicle.kia.ev6" and m["wheels"]["max_steer_deg"] == 40.0


def test_check_dimensions_uses_per_key_tolerance():
    m = validated()
    rows = {r[0]: r for r in vm.check_dimensions(
        {"length_m": 4.03, "track_rear_m": 1.58, "wheel_radius_m": 0.36}, m)}
    assert rows["length_m"][4] is True
    assert rows["track_rear_m"][4] is False          # 0.03 off, tolerance 0.02
    assert rows["wheel_radius_m"][4] is True
    assert "height_m" not in rows                    # not measured, not checked


# ---------------------------------------------------------------- physics

def test_physics_plan_is_in_carla_units_and_wheel_order():
    plan = vm.physics_plan(validated())
    assert plan["vehicle"]["mass"] == 1500.0
    assert plan["vehicle"]["center_of_mass"] == (0.1, 0.0, -0.2)
    assert "max_rpm" not in plan["vehicle"]          # undeclared keeps the blueprint's
    fl, fr, rl, rr = plan["wheels"]
    assert fl["radius"] == pytest.approx(35.0)       # cm
    assert fl["max_steer_angle"] == 38.0 and rl["max_steer_angle"] == 0.0
    assert fl["tire_friction"] == rr["tire_friction"] == 3.0
    assert "max_handbrake_torque" in rl and "max_handbrake_torque" not in fl


class _V2:
    def __init__(self, x, y):
        self.x, self.y = x, y


class _Gear:
    def __init__(self, ratio, down_ratio, up_ratio):
        self.ratio, self.down_ratio, self.up_ratio = ratio, down_ratio, up_ratio


FAKE_CARLA = types.SimpleNamespace(Vector2D=_V2, Location=lambda *a: tuple(a), GearPhysicsControl=_Gear)


def test_apply_physics_writes_declared_values_and_assigns_wheels_back():
    stock = [types.SimpleNamespace(radius=30.0, max_steer_angle=70.0, tire_friction=3.5)
             for _ in range(4)]

    class PC:
        mass = 1.0
        _wheels = stock

        @property
        def wheels(self):
            return [copy.copy(w) for w in self._wheels]  # CARLA hands out copies

        @wheels.setter
        def wheels(self, v):
            self._wheels = v
    pc = PC()
    done = vm.apply_physics(pc, vm.physics_plan(validated()), FAKE_CARLA)
    assert pc.mass == 1500.0 and "wheels" in done
    assert [w.max_steer_angle for w in pc._wheels] == [38.0, 38.0, 0.0, 0.0]
    assert pc.torque_curve[1].x == 6000.0
    assert pc.forward_gears[0].ratio == 1.0


def test_for_blueprint_leaves_stock_cars_alone(tmp_path, monkeypatch):
    monkeypatch.setenv("FIXS_VEHICLE_CACHE", str(tmp_path))
    assert vm.for_blueprint("vehicle.tesla.model3") == (None, None)


def test_for_blueprint_refuses_a_manifest_for_another_car(tmp_path):
    p = tmp_path / "vehicle.yaml"
    p.write_text(yaml.safe_dump(MANIFEST), encoding="utf-8")
    with pytest.raises(vm.ManifestError, match="EgoBlueprint"):
        vm.for_blueprint("vehicle.kia.ev6", str(p))
    m, path = vm.for_blueprint("vehicle.acme.roadster", str(p))
    assert m["make"] == "Acme"


def test_for_blueprint_finds_the_installed_manifest(tmp_path, monkeypatch):
    monkeypatch.setenv("FIXS_VEHICLE_CACHE", str(tmp_path))
    d = tmp_path / "vehicle.acme.roadster" / "bundle"
    d.mkdir(parents=True)
    (d / "vehicle.yaml").write_text(yaml.safe_dump(MANIFEST), encoding="utf-8")
    m, path = vm.for_blueprint("vehicle.acme.roadster")
    assert path == str(d / "vehicle.yaml")


# ---------------------------------------------------------------- VehicleFactory

STOCK = [{"make": "Tesla", "model": "Model3", "class_": "/Game/Carla/BP_TeslaM3.BP_TeslaM3_C",
          "recommended_colors": [[1, 2, 3, 255]]}]


def new_entry(colors=((10, 20, 30, 255),)):
    return {"make": "Acme", "model": "Roadster", "class_": "/Game/FIXS/Vehicles/AcmeRoadster/BP_AcmeRoadster",
            "number_of_wheels": 4, "generation": 2, "object_type": "", "base_type": "car",
            "special_type": "", "has_dynamic_doors": False, "has_lights": False,
            "recommended_colors": [list(c) for c in colors], "supported_drivers": []}


def test_merge_appends_a_new_vehicle_and_keeps_stock_entries():
    out, action = factory.merge(STOCK, new_entry())
    assert action == "added" and out[0] is STOCK[0] and len(out) == 2


def test_merge_updates_only_an_entry_fixs_owns():
    first, _ = factory.merge(STOCK, new_entry())
    out, action = factory.merge(first, new_entry(colors=((1, 1, 1, 255),)), owned_ids=["vehicle.acme.roadster"])
    assert action == "updated" and len(out) == 2 and out[-1]["recommended_colors"] == [[1, 1, 1, 255]]
    with pytest.raises(factory.FactoryError):
        factory.merge(first, new_entry(colors=((1, 1, 1, 255),)))


def test_merge_never_replaces_a_stock_car():
    tesla = dict(new_entry(), make="Tesla", model="Model3")
    with pytest.raises(factory.FactoryError, match="vehicle.tesla.model3"):
        factory.merge(STOCK, tesla)


def test_merge_is_idempotent_whatever_the_class_path_spelling():
    first, _ = factory.merge(STOCK, new_entry())
    first[-1]["class_"] += ".BP_AcmeRoadster_C"
    out, action = factory.merge(first, new_entry(), owned_ids=["vehicle.acme.roadster"])
    assert action == "unchanged" and out == first


def test_remove_only_what_fixs_owns():
    first, _ = factory.merge(STOCK, new_entry())
    out, n = factory.remove(first, "vehicle.acme.roadster", owned_ids=["vehicle.acme.roadster"])
    assert n == 1 and out == STOCK
    with pytest.raises(factory.FactoryError):
        factory.remove(first, "vehicle.tesla.model3")
    assert factory.remove(STOCK, "vehicle.acme.roadster")[1] == 0


def test_factory_entry_from_manifest():
    e = vm.factory_entry(validated(), "/Game/X/BP_Y.BP_Y_C")
    assert factory.entry_id(e) == "vehicle.acme.roadster"
    assert e["recommended_colors"][0] == [10, 20, 30, 255]
    assert set(e) == set(factory.FIELDS)


# ---------------------------------------------------------------- catalog

def test_catalog_parse_and_lookup():
    doc = {"schema": 1, "maps": [], "vehicles": [
        {"id": "vehicle.kia.ev6", "release": "vehicle-kia-ev6", "asset": "vehicle.kia.ev6.zip"}]}
    vehicles = catalog.parse_catalog(doc)
    assert catalog.entry(vehicles, "ev6")["id"] == "vehicle.kia.ev6"
    assert catalog.entry(vehicles, "VEHICLE.KIA.EV6") is not None
    assert catalog.entry(vehicles, "model3") is None
    assert catalog.parse_catalog({"maps": []}) == []


@pytest.mark.parametrize("bad", [
    {"vehicles": [{"id": "vehicle.kia.ev6", "release": "r"}]},
    {"vehicles": [{"id": "kia.ev6", "release": "r", "asset": "a"}]},
    {"vehicles": [{"id": "vehicle.a.b", "release": "r", "asset": "a"}] * 2},
])
def test_catalog_rejects_bad_entries(bad):
    with pytest.raises(catalog.CatalogError):
        catalog.parse_catalog(bad)


def test_extract_unwraps_one_folder_and_refuses_escapes(tmp_path):
    z = tmp_path / "b.zip"
    with zipfile.ZipFile(z, "w") as f:
        f.writestr("roadster/vehicle.yaml", "schema: 1\n")
        f.writestr("roadster/rig/vehicle.fbx", "x")
    dest = catalog.extract(str(z), str(tmp_path / "out"))
    assert os.path.isfile(os.path.join(dest, "vehicle.yaml"))
    assert os.path.isfile(os.path.join(dest, "rig", "vehicle.fbx"))
    evil = tmp_path / "evil.zip"
    with zipfile.ZipFile(evil, "w") as f:
        f.writestr("../escape.txt", "x")
    with pytest.raises(catalog.CatalogError, match="escapes"):
        catalog.extract(str(evil), str(tmp_path / "out2"))


def test_resolve_repo_order(tmp_path):
    assert catalog.resolve_repo("a/b", app_root=str(tmp_path)) == "a/b"
    assert catalog.resolve_repo(None, app_root=str(tmp_path)) == catalog.DEFAULT_REPO
    (tmp_path / "fixs_sources.txt").write_text("map_repo=x/y\nvehicle_repo = org/vehicles\n")
    assert catalog.resolve_repo(None, app_root=str(tmp_path)) == "org/vehicles"


def test_owned_and_installed_ids_follow_receipts(tmp_path, monkeypatch):
    monkeypatch.setenv("FIXS_VEHICLE_CACHE", str(tmp_path / "cache"))
    root = str(tmp_path / "carla")
    catalog.write_receipt("vehicle.a.b", {"carla_root": root, "registered": True})
    catalog.write_receipt("vehicle.c.d", {"carla_root": root, "registered": False})
    catalog.write_receipt("vehicle.e.f", {"carla_root": str(tmp_path / "other"), "registered": True})
    assert catalog.installed_ids(root) == ["vehicle.a.b"]
    assert catalog.owned_ids(root) == ["vehicle.a.b", "vehicle.c.d"]
    assert catalog.resolve_id("d") == "vehicle.c.d"


# ---------------------------------------------------------------- shared installs

def test_instances_on_install_matches_only_this_install():
    procs = [
        {"pid": 1, "name": "UE4Editor.exe", "exe": r"C:\UE\Engine\Binaries\Win64\UE4Editor.exe",
         "cmd": r'"C:\UE\UE4Editor.exe" "C:\src_ext\Carla\Unreal\CarlaUE4\CarlaUE4.uproject" -game'},
        {"pid": 2, "name": "UE4Editor.exe", "exe": "", "cmd": r"UE4Editor.exe C:\src_ext\Carla2\Unreal\x.uproject"},
        {"pid": 3, "name": "CarlaUE4-Win64-Shipping.exe", "exe": r"C:\src_ext\Carla\Dist\CarlaUE4.exe", "cmd": ""},
        {"pid": 4, "name": "python.exe", "exe": "", "cmd": r"python C:\src_ext\Carla\PythonAPI\x.py"},
    ]
    hits = install_guard.instances_on_install(procs, r"C:/src_ext/Carla")
    assert [p["pid"] for p in hits] == [1, 3]
    assert install_guard.instances_on_install(procs, r"C:/src_ext/Carla", exclude_pids=(1, 3)) == []


def test_file_in_use_is_false_for_a_free_file(tmp_path):
    p = tmp_path / "f.uasset"
    p.write_bytes(b"x")
    assert install_guard.file_in_use(str(p)) is False
    assert install_guard.file_in_use(str(tmp_path / "missing")) is False


@pytest.mark.skipif(os.name != "nt", reason="Windows sharing semantics")
def test_file_in_use_sees_another_open_handle(tmp_path):
    import subprocess
    p = tmp_path / "held.uasset"
    p.write_bytes(b"x")
    holder = subprocess.Popen([sys.executable, "-c",
                               "import sys,time;f=open(sys.argv[1],'rb');print('ok',flush=True);time.sleep(30)",
                               str(p)], stdout=subprocess.PIPE, text=True)
    try:
        assert holder.stdout.readline().strip() == "ok"
        assert install_guard.file_in_use(str(p)) is True
    finally:
        holder.kill()


# ---------------------------------------------------------------- prep geometry

def _box(lo, hi):
    """12 outward triangles of an axis box, as (P, N, UV)."""
    x0, y0, z0 = lo
    x1, y1, z1 = hi
    v = np.array([[x0, y0, z0], [x1, y0, z0], [x1, y1, z0], [x0, y1, z0],
                  [x0, y0, z1], [x1, y0, z1], [x1, y1, z1], [x0, y1, z1]], float)
    f = [(0, 2, 1), (0, 3, 2), (4, 5, 6), (4, 6, 7), (0, 1, 5), (0, 5, 4),
         (1, 2, 6), (1, 6, 5), (2, 3, 7), (2, 7, 6), (3, 0, 4), (3, 4, 7)]
    P = v[np.array(f)]
    return P, np.zeros_like(P), np.zeros(P.shape[:2] + (2,))


def toy_car(twins=True):
    """A box body and four box wheels, in a SketchUp-like frame (x left, -y forward, z up)."""
    chunks = []

    def add(group, mat, lo, hi):
        P, N, UV = _box(lo, hi)
        if twins:
            P, N, UV = np.concatenate([P, P[:, ::-1]]), np.concatenate([N, N]), np.concatenate([UV, UV])
        chunks.append({"group": group, "material": mat, "P": P, "N": N, "UV": UV})
    add("body", "Body", (-0.9, -2.0, 0.2), (0.9, 2.0, 1.5))
    add("body", "Glass", (-0.7, -1.0, 1.2), (0.7, 0.5, 1.45))
    for name, x, y in (("w1", 0.75, -1.3), ("w2", -0.75, -1.3), ("w3", 0.75, 1.3), ("w4", -0.75, 1.3)):
        add(name, "Tyre", (x - 0.11, y - 0.35, 0.0), (x + 0.11, y + 0.35, 0.7))
    mats = {k: {"name": k, "color": [0.1, 0.2, 0.3, 1.0], "texture": None} for k in ("Body", "Glass", "Tyre")}
    return chunks, mats


def test_orientation_maps_source_axes_to_the_vehicle_frame():
    R = mesh.orientation("-y", "+z")
    assert np.allclose(R @ [0, -1, 0], [1, 0, 0])    # source forward -> +X
    assert np.allclose(R @ [1, 0, 0], [0, 1, 0])     # source +x is left of a -y forward
    assert np.allclose(R @ [0, 0, 1], [0, 0, 1])
    with pytest.raises(mesh.MeshError):
        mesh.orientation("+x", "-x")


def test_dedupe_keeps_one_of_each_twin():
    P, N, UV = _box((0, 0, 0), (1, 1, 1))
    P2 = np.concatenate([P, P[:, ::-1]])
    N2, UV2 = np.concatenate([N, N]), np.concatenate([UV, UV])
    assert len(mesh.dedupe(P2, N2, UV2, "first", (0.5, 0.5, 0.5))[0]) == 12
    assert len(mesh.dedupe(P2, N2, UV2, "both", (0.5, 0.5, 0.5))[0]) == 24
    out = mesh.dedupe(P2, N2, UV2, "outward", (0.5, 0.5, 0.5))[0]
    normals = np.cross(out[:, 1] - out[:, 0], out[:, 2] - out[:, 0])
    assert len(out) == 12 and ((normals * (out.mean(1) - 0.5)).sum(1) > 0).all()


def test_build_parts_and_measure_a_toy_car():
    m = validated()
    chunks, mats = toy_car()
    chunks = mesh.to_vehicle_frame(chunks, 1.0, "-y", "+z")
    arrays, meta = mesh.build_parts(chunks, mats, m)
    bones = {w["bone"]: w["centre"] for w in meta["wheels"]}
    assert bones["Wheel_Front_Left"][0] > 0 and bones["Wheel_Front_Left"][1] > 0
    assert bones["Wheel_Rear_Right"][0] < 0 and bones["Wheel_Rear_Right"][1] < 0
    roles = {s["slot"]: s["role"] for s in meta["slot_info"]}
    assert roles == {"Bodywork_Mat": "paint", "M_Glass": "glass", "M_Tyre": "opaque"}
    tris = {(p["bone"], p["slot"]): p["tris"] for p in meta["parts"]}
    assert tris[("Vehicle_Base", "Bodywork_Mat")] == 24      # paint keeps both twins
    assert tris[("Wheel_Front_Left", "M_Tyre")] == 12
    d = mesh.measure(meta)
    assert d["length_m"] == pytest.approx(4.0)
    assert d["wheelbase_m"] == pytest.approx(2.6)
    assert d["track_front_m"] == pytest.approx(1.5)
    assert d["wheel_radius_m"] == pytest.approx(0.35)
    assert all(k in arrays for k in ("v0", "n0", "t0", "f0"))


def test_wheels_must_be_found_exactly():
    m = validated()
    chunks, mats = toy_car()
    chunks = [c for c in chunks if c["group"] != "w4"]
    chunks = mesh.to_vehicle_frame(chunks, 1.0, "-y", "+z")
    with pytest.raises(mesh.MeshError, match="need exactly 4"):
        mesh.build_parts(chunks, mats, m)


def test_wheels_by_node_name():
    boxes = {"a": (np.array([1.0, 0.5, 0]), np.array([1.6, 0.8, 0.7])),
             "b": (np.array([1.0, -0.8, 0]), np.array([1.6, -0.5, 0.7]))}
    spec = {"nodes": {"front_left": "a", "front_right": "b", "rear_left": "zz", "rear_right": "b"}}
    with pytest.raises(mesh.MeshError, match="zz"):
        mesh.find_wheels(boxes, spec)


# ---------------------------------------------------------------- import job

def import_vehicle_module():
    import import_vehicle
    return import_vehicle


def test_unreal_job_is_resolved_from_the_manifest():
    iv = import_vehicle_module()
    m = validated()
    slots = [{"slot": "Bodywork_Mat", "role": "paint"}]
    job = iv.unreal_job(m, slots, "install", ["vehicle.x.y"], True, "r.json", "tex", "v.fbx")
    assert job["dest"] == "/Game/FIXS/Vehicles/AcmeRoadster"
    assert job["wheels"] == {"radius_cm": pytest.approx(35.0), "width_cm": pytest.approx(22.0),
                             "max_steer_deg": 38.0, "rear_max_steer_deg": 0.0}
    assert job["paint_color"] == [10, 20, 30]
    assert job["factory_entry"]["class_"] == "/Game/FIXS/Vehicles/AcmeRoadster/BP_AcmeRoadster.BP_AcmeRoadster_C"
    assert job["owned_ids"] == ["vehicle.x.y"]
    box = job["collision_box_cm"]
    assert box["z"] == pytest.approx(135.0) and box["center"][2] == pytest.approx(82.5)
    assert box["x"] == pytest.approx(395.0) and box["y"] == pytest.approx(180.0)
    assert job["drivetrain"]["front_rear_split"] == 0.4
    assert not job["dest"].startswith("/Game/Carla")


def test_refusal_names_the_processes_and_the_way_out():
    iv = import_vehicle_module()
    text = iv.refusal([{"pid": 42, "name": "UE4Editor.exe", "cmd": "x"}], True, "VF.uasset")
    assert "42" in text and "--wait" in text and "--no-register" in text and "VF.uasset" in text


def test_pick_by_number_id_or_model():
    iv = import_vehicle_module()
    vs = [{"id": "vehicle.kia.ev6"}, {"id": "vehicle.acme.roadster"}]
    assert iv.pick(vs, "2")["id"] == "vehicle.acme.roadster"
    assert iv.pick(vs, "ev6")["id"] == "vehicle.kia.ev6"
    assert iv.pick(vs, "9") is None


def test_unreal_script_imports_factory_from_its_own_folder():
    """unreal_build.py runs inside the editor, which only has its own directory to import from."""
    here = os.path.join(os.path.dirname(__file__), "..", "..", "..", "Carla", "vehicles")
    src = open(os.path.join(here, "unreal_build.py"), encoding="utf-8").read()
    assert "import factory" in src and "import yaml" not in src
    fsrc = open(os.path.join(here, "factory.py"), encoding="utf-8").read()
    assert "import " not in fsrc.replace('"""', "").split("FIELDS")[0]


# ---------------------------------------------------------------- CarlaSetup keys

BASE = {
    "SimulationSetup": {"EnableRealSim": True, "SimulationEndTime": 100,
                        "SelectedTrafficSimulator": "SUMO",
                        "VehicleMessageField": ["id", "speed", "positionX", "positionY"]},
    "CarlaSetup": {"EnableCosimulation": True, "InterestedIds": ["ego"]},
}


def parse(tmp_path, **carla):
    from CommonLib.ConfigHelper import ConfigHelper
    doc = copy.deepcopy(BASE)
    doc["CarlaSetup"].update(carla)
    p = tmp_path / "c.yaml"
    p.write_text(yaml.safe_dump(doc), encoding="utf-8")
    ch = ConfigHelper()
    ch.getConfig(str(p))
    return ch.Carla_setup


def test_existing_configs_parse_unchanged(tmp_path):
    cs = parse(tmp_path)
    assert cs["EgoBlueprint"] == "vehicle.tesla.model3"
    assert cs["EgoColor"] == "" and cs["EgoVehicleManifest"] == "" and cs["EgoBlueprintSet"] is False


@pytest.mark.parametrize("value, want", [("150,20,30", "150,20,30"), ("150, 20, 30", "150,20,30"),
                                         ([150, 20, 30], "150,20,30")])
def test_ego_color_forms(tmp_path, value, want):
    cs = parse(tmp_path, EgoColor=value, EgoBlueprint="vehicle.kia.ev6")
    assert cs["EgoColor"] == want and cs["EgoBlueprintSet"] is True


@pytest.mark.parametrize("value", ["red", "1,2", [1, 2, 300], "1,2,3,4", [True, 0, 0]])
def test_bad_ego_color_is_refused(tmp_path, value):
    with pytest.raises(SystemExit, match="EgoColor"):
        parse(tmp_path, EgoColor=value)


# ---------------------------------------------------------------- bridge

def test_a_mirrored_ego_takes_the_named_blueprint_and_colour():
    carla = pytest.importorskip("carla")
    from Carla.VirEnv.CarlaBackend import CarlaBackend
    from CommonLib.VirEnv.IVirEnvBackend import Pose

    class BP:
        def __init__(self, bp_id):
            self.id, self.attrs = bp_id, {}

        def has_attribute(self, k):
            return k == "color"

        def set_attribute(self, k, v):
            self.attrs[k] = v

    asked = []

    class Lib:
        def find(self, bp_id):
            asked.append(BP(bp_id))
            return asked[-1]

    class World:
        def try_spawn_actor(self, bp, tf):
            return types.SimpleNamespace(id=len(asked), set_simulate_physics=lambda on: None)

    be = CarlaBackend(World(), None, False, False)
    be._bpLib = Lib()
    be.setMirroredEgo("ego", "vehicle.kia.ev6", "150,20,30")
    be.spawnVehicle("car", "passenger", Pose(x=0, y=0, z=0, headingDeg=0, gradeRad=0), "ego")
    be.spawnVehicle("car", "passenger", Pose(x=9, y=0, z=0, headingDeg=0, gradeRad=0), "veh1")
    assert asked[0].id == "vehicle.kia.ev6" and asked[0].attrs == {"color": "150,20,30"}
    assert asked[1].id != "vehicle.kia.ev6" and asked[1].attrs == {}
    del carla
