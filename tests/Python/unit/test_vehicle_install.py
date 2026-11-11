"""Where library vehicles are installed (FIXS#470): one receipt per CARLA install, the cooked
archive a packaged CARLA installs, and run_cosim's check that the configured ego exists.

    python -m pytest tests/Python/unit/test_vehicle_install.py
"""
import json
import os
import sys
import tarfile

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "..", "Carla"))
from vehicles import catalog, cooked  # noqa: E402

EV6 = "vehicle.kia.ev6"


@pytest.fixture(autouse=True)
def cache(tmp_path, monkeypatch):
    monkeypatch.setenv("FIXS_VEHICLE_CACHE", str(tmp_path / "cache"))
    return tmp_path / "cache"


# ---------------------------------------------------------------- receipts

def test_receipts_are_kept_per_carla_install(tmp_path):
    src, pkg = str(tmp_path / "src"), str(tmp_path / "pkg")
    catalog.write_receipt("vehicle.a.b", {"carla_root": src, "registered": True, "mode": "source"})
    catalog.write_receipt("vehicle.a.b", {"carla_root": pkg, "registered": True, "mode": "packaged"})
    assert catalog.read_receipt("vehicle.a.b", src)["mode"] == "source"
    assert catalog.read_receipt("vehicle.a.b", pkg)["mode"] == "packaged"
    catalog.remove_receipt("vehicle.a.b", pkg)
    assert catalog.read_receipt("vehicle.a.b", pkg) is None
    assert catalog.read_receipt("vehicle.a.b", src) is not None


def test_legacy_single_receipt_is_read_then_replaced(tmp_path, cache):
    root = str(tmp_path / "src")
    d = cache / "vehicle.a.b"
    d.mkdir(parents=True)
    (d / "install.json").write_text(json.dumps({"carla_root": root, "registered": True}))
    assert catalog.installed_ids(root) == ["vehicle.a.b"]
    catalog.write_receipt("vehicle.a.b", {"carla_root": root, "registered": False})
    assert not (d / "install.json").exists()
    assert catalog.installed_ids(root) == []


# ---------------------------------------------------------------- cooked archive

def test_cooked_asset_name_follows_the_os():
    assert cooked.asset_name(EV6, system="Windows") == EV6 + "_cooked_windows.tar.gz"
    assert cooked.asset_name(EV6, system="Linux") == EV6 + "_cooked_linux.tar.gz"
    ent = {"id": EV6, "cooked_asset_windows": ""}
    assert cooked.catalog_cooked_asset(ent, system="Windows") is None
    assert cooked.catalog_cooked_asset(ent, system="Linux") == EV6 + "_cooked_linux.tar.gz"


@pytest.mark.parametrize("name,ok", [
    ("fixs_vehicle_cook.json", True),
    ("manifests/vehicle.kia.ev6/vehicle.yaml", True),
    ("CarlaUE4/Content/FIXS/Vehicles/KiaEV6/BP_KiaEV6.uasset", True),
    ("CarlaUE4/Content/Carla/Blueprints/Vehicles/VehicleFactory.uexp", True),
    ("CarlaUE4/Content/Carla/Blueprints/Vehicles/Tesla/BP_TeslaM3.uasset", False),
    ("CarlaUE4/Content/FIXS/Vehicles/../../Carla/x.uasset", False),
    ("/etc/passwd", False),
    ("CarlaUE4/Binaries/Win64/CarlaUE4.exe", False),
])
def test_a_cook_may_only_write_vehicles_and_the_factory(name, ok):
    assert cooked.allowed_member(name) is ok


def test_package_list_covers_assets_and_factory(tmp_path):
    d = tmp_path / "FIXS" / "Vehicles" / "KiaEV6" / "Textures"
    d.mkdir(parents=True)
    (d / "T_Plate.uasset").write_bytes(b"x")
    (d.parent / "BP_KiaEV6.uasset").write_bytes(b"x")
    (d.parent / "BP_KiaEV6.uexp").write_bytes(b"x")
    assert cooked.package_packages(["/Game/FIXS/Vehicles/KiaEV6"], str(tmp_path)) == [
        "/Game/FIXS/Vehicles/KiaEV6/BP_KiaEV6", "/Game/FIXS/Vehicles/KiaEV6/Textures/T_Plate",
        cooked.FACTORY_GAME_PATH]


def _packaged(tmp_path):
    root = tmp_path / "WindowsNoEditor"
    fac = root / "CarlaUE4" / "Content" / "Carla" / "Blueprints" / "Vehicles"
    fac.mkdir(parents=True)
    (fac / "VehicleFactory.uasset").write_bytes(b"stock-uasset")
    (fac / "VehicleFactory.uexp").write_bytes(b"stock-uexp")
    return root, fac


def _cook(tmp_path, ids=(EV6,)):
    out = tmp_path / "cook_out"
    out.mkdir(exist_ok=True)
    files = {}
    for ext in cooked.FACTORY_EXTS:
        p = out / ("factory" + ext)
        p.write_bytes(b"cooked" + ext.encode())
        files[cooked.FACTORY_PREFIX[:-1] + ext] = str(p)
    asset_dirs = {}
    for i in ids:
        name = i.split(".")[-1].upper()
        p = out / (name + ".uasset")
        p.write_bytes(b"bp")
        files["CarlaUE4/Content/FIXS/Vehicles/{0}/BP_{0}.uasset".format(name)] = str(p)
        asset_dirs[i] = "/Game/FIXS/Vehicles/" + name
    yml = tmp_path / "vehicle.yaml"
    yml.write_text("id: x\n")
    info = {"schema": cooked.COOK_SCHEMA, "carla": "0.9.15", "platform": "windows",
            "ids": list(ids), "asset_dirs": asset_dirs}
    return cooked.write_archive(str(tmp_path / "cook.tar.gz"), files, info, {i: str(yml) for i in ids})


def quiet(_):
    pass


def test_cook_installs_then_uninstalls_back_to_stock(tmp_path):
    root, fac = _packaged(tmp_path)
    assert cooked.install(_cook(tmp_path), str(root), str(root), log=quiet) == [EV6]
    assert (fac / "VehicleFactory.uasset").read_bytes() == b"cooked.uasset"
    assert (root / "CarlaUE4/Content/FIXS/Vehicles/EV6/BP_EV6.uasset").is_file()
    rec = catalog.read_receipt(EV6, str(root))
    assert rec["registered"] and rec["mode"] == "packaged" and os.path.isfile(rec["manifest"])

    assert cooked.uninstall(str(root), str(root), log=quiet) == [EV6]
    assert (fac / "VehicleFactory.uasset").read_bytes() == b"stock-uasset"
    assert (fac / "VehicleFactory.uexp").read_bytes() == b"stock-uexp"
    assert not (root / "CarlaUE4/Content/FIXS/Vehicles").exists()
    assert catalog.read_receipt(EV6, str(root)) is None


def test_reinstall_keeps_the_first_factory_as_the_original(tmp_path):
    root, fac = _packaged(tmp_path)
    cooked.install(_cook(tmp_path), str(root), str(root), log=quiet)
    cooked.install(_cook(tmp_path), str(root), str(root), log=quiet)
    cooked.uninstall(str(root), str(root), log=quiet)
    assert (fac / "VehicleFactory.uasset").read_bytes() == b"stock-uasset"


def test_a_new_cook_unregisters_vehicles_it_does_not_carry(tmp_path):
    root, _ = _packaged(tmp_path)
    catalog.write_receipt("vehicle.acme.old", {"carla_root": str(root), "registered": True,
                                               "asset_dir": "/Game/FIXS/Vehicles/Old"})
    cooked.install(_cook(tmp_path), str(root), str(root), log=quiet)
    assert catalog.installed_ids(str(root)) == [EV6]


def test_a_cook_with_foreign_files_is_refused_untouched(tmp_path):
    root, fac = _packaged(tmp_path)
    good = _cook(tmp_path)
    evil = tmp_path / "evil.exe"
    evil.write_bytes(b"x")
    bad = str(tmp_path / "bad.tar.gz")
    with tarfile.open(good) as src, tarfile.open(bad, "w:gz") as dst:
        for m in src.getmembers():
            dst.addfile(m, src.extractfile(m))
        dst.add(str(evil), arcname="CarlaUE4/Binaries/Win64/evil.exe")
    with pytest.raises(cooked.CookError):
        cooked.install(bad, str(root), str(root), log=quiet)
    assert (fac / "VehicleFactory.uasset").read_bytes() == b"stock-uasset"


# ---------------------------------------------------------------- run_cosim preflight

@pytest.fixture
def iv():
    import import_vehicle
    return import_vehicle


def test_stock_blueprints_need_no_install(tmp_path, iv):
    assert len(catalog.STOCK_IDS) == 41
    assert iv.ensure_for_run("vehicle.tesla.model3", str(tmp_path), "source") == "stock"


def test_installed_library_vehicle_passes(tmp_path, iv):
    root = tmp_path / "carla"
    (root / "Unreal" / "CarlaUE4" / "Content" / "FIXS" / "Vehicles" / "KiaEV6").mkdir(parents=True)
    catalog.write_receipt(EV6, {"carla_root": str(root), "registered": True,
                                "asset_dir": "/Game/FIXS/Vehicles/KiaEV6"})
    assert iv.ensure_for_run(EV6, str(root), "source") == "installed"


def test_receipt_without_its_assets_counts_as_missing(tmp_path, iv):
    catalog.write_receipt(EV6, {"carla_root": str(tmp_path), "registered": True,
                                "asset_dir": "/Game/FIXS/Vehicles/KiaEV6"})
    assert iv.install_state(EV6, str(tmp_path), "source") == "missing"


def test_missing_library_vehicle_is_offered_or_stops_the_run(tmp_path, iv, monkeypatch):
    monkeypatch.setattr(catalog, "fetch_catalog", lambda repo: (
        [{"id": EV6, "release": "ev6", "asset": EV6 + ".zip", "title": "EV6"}], repo))
    with pytest.raises(SystemExit) as e:
        iv.ensure_for_run(EV6, str(tmp_path), "source", interactive=False)
    assert "import_vehicle.py --vehicle " + EV6 in str(e.value)
    asked = []
    with pytest.raises(SystemExit) as e:
        iv.ensure_for_run(EV6, str(tmp_path), "source", interactive=True,
                          ask=lambda prompt: asked.append(prompt) or "n")
    assert asked and "not imported" in str(e.value)


def test_unknown_blueprint_is_named_as_such(tmp_path, iv, monkeypatch):
    monkeypatch.setattr(catalog, "fetch_catalog", lambda repo: (
        [{"id": EV6, "release": "ev6", "asset": EV6 + ".zip", "title": "EV6"}], repo))
    with pytest.raises(SystemExit) as e:
        iv.ensure_for_run("vehicle.tesla.modle3", str(tmp_path), "source", interactive=True)
    assert "neither a stock" in str(e.value)


def test_unreachable_library_says_so(tmp_path, iv, monkeypatch):
    monkeypatch.setattr(catalog, "fetch_catalog", lambda repo: ([], None))
    with pytest.raises(SystemExit) as e:
        iv.ensure_for_run(EV6, str(tmp_path), "source", interactive=True)
    assert "could not be read" in str(e.value) and "gh auth login" in str(e.value)


def test_failed_download_stops_with_a_message(iv, monkeypatch):
    monkeypatch.setattr(catalog, "fetch_catalog", lambda repo: (
        [{"id": EV6, "release": "ev6", "asset": EV6 + ".zip", "title": "EV6"}], repo))

    def fail(*a, **k):
        raise catalog.CatalogError("`gh release download` failed (rc=1)")
    monkeypatch.setattr(catalog, "download", fail)
    with pytest.raises(SystemExit) as e:
        iv.bundle_from_library("ORNL-Real-Sim/VehicleDynamicsLibrary", EV6)
    assert "failed (rc=1)" in str(e.value)
