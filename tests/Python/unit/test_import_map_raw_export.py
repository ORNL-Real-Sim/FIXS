"""Import a raw RoadRunner export: descriptor generation and map naming.

A raw export ships .fbx/.xodr/.geojson/.rrdata.xml and no CARLA `<name>.json`,
so import_map synthesises the descriptor. The exports also carry the author's
working name (`MLK_no_signal_0805`), never the name the map should keep, so the
geometry is renamed to the target map name on the way in - see
Carla/import_map.py:_rename_export for why the navmesh forces that.

Fixtures are synthetic: the shapes matter, not the bytes. CARLA is never invoked;
these cover staging + descriptor generation only.
"""
import json
import os
import sys
import zipfile

import pytest

sys.path.insert(0, os.path.join(
    os.path.dirname(__file__), '..', '..', '..', 'Carla'))
import import_map  # noqa: E402


def make_package(tmp_path, name, entries):
    """A .zip holding `entries` (name -> content), returned as a path string."""
    path = tmp_path / (name + ".zip")
    with zipfile.ZipFile(path, "w") as z:
        for entry, content in entries.items():
            z.writestr(entry, content)
    return str(path)


def raw_export(tmp_path, stem, tiles=None, extra=None):
    """A raw RoadRunner export named `stem`: its .xodr plus either `<stem>.fbx`
    or the given tile suffixes, and the inert files a real export ships."""
    entries = {f"{stem}.xodr": "x",
               f"{stem}.geojson": "x",
               f"{stem}.rrdata.xml": "x"}
    for suffix in (tiles if tiles is not None else [""]):
        entries[f"{stem}{suffix}.fbx"] = "x"
    entries.update(extra or {})
    return make_package(tmp_path, stem, entries)


def stage(tmp_path, package, map_name):
    """stage_package into a throwaway carla_root; returns the descriptor dict."""
    root = tmp_path / ("carla_" + map_name)
    root.mkdir()
    import_map.stage_package(str(root), map_name, package_dir=package)
    with open(root / "Import" / (map_name + ".json"), encoding="utf-8") as f:
        return json.load(f)


# --------------------------------------------------------------- happy paths

def test_single_source_export_is_described(tmp_path):
    """An export already named after the map yields the classic descriptor."""
    pkg = raw_export(tmp_path, "uga_v4")
    entry = stage(tmp_path, pkg, "uga_v4")["maps"][0]

    assert entry["name"] == "uga_v4"
    assert entry["source"] == "uga_v4/uga_v4.fbx"
    assert entry["xodr"] == "uga_v4/uga_v4.xodr"
    assert entry["use_carla_materials"] is False   # RoadRunner ships its own
    assert "tiles" not in entry and "tile_size" not in entry
    assert "exported_as" not in entry              # nothing was renamed


def test_export_geometry_is_renamed_to_the_map_name(tmp_path):
    """The export's working name does not become the map's name.

    The .fbx moves because the cook writes the navmesh as <fbx_stem>.bin while
    the runtime loads <MapName>.bin; the .xodr stays put because CARLA's own
    importer copies it to <name>.xodr.
    """
    pkg = raw_export(tmp_path, "MLK_no_signal_0805")
    entry = stage(tmp_path, pkg, "mlk_no_signal")["maps"][0]

    assert entry["name"] == "mlk_no_signal"
    assert entry["source"] == "mlk_no_signal/mlk_no_signal.fbx"
    assert entry["xodr"] == "mlk_no_signal/MLK_no_signal_0805.xodr"
    assert entry["exported_as"] == "MLK_no_signal_0805"


def test_rename_leaves_the_exports_own_files_alone(tmp_path):
    """Only geometry moves, so Import/<map>/ still shows where it came from."""
    pkg = raw_export(tmp_path, "Roosevelt_R2024b")
    root = tmp_path / "carla"
    root.mkdir()
    import_map.stage_package(str(root), "roosevelt_full", package_dir=pkg)

    staged = sorted(os.listdir(root / "Import" / "roosevelt_full"))
    assert staged == ["Roosevelt_R2024b.geojson", "Roosevelt_R2024b.rrdata.xml",
                      "Roosevelt_R2024b.xodr", "roosevelt_full.fbx"]


def test_tiled_export_keeps_its_tile_indices_through_the_rename(tmp_path):
    """Tile (x, y) is the streaming grid; only the prefix may change."""
    pkg = raw_export(tmp_path, "Atl_R2024b_final",
                     tiles=["_Tile_0_0", "_Tile_0_1", "_Tile_1_0"])
    entry = stage(tmp_path, pkg, "atlanta_full")["maps"][0]

    assert entry["tiles"] == ["atlanta_full/atlanta_full_Tile_0_0.fbx",
                              "atlanta_full/atlanta_full_Tile_0_1.fbx",
                              "atlanta_full/atlanta_full_Tile_1_0.fbx"]
    assert entry["tile_size"] == 2000        # cook default == Unreal's clamp
    assert "source" not in entry             # `tiles` is what selects large-map


def test_tile_size_comes_from_tilesinfo_when_the_export_ships_one(tmp_path):
    pkg = raw_export(tmp_path, "grid", tiles=["_Tile_0_0"],
                     extra={"TilesInfo.txt": "-1000,-1000,1024\n"})
    assert stage(tmp_path, pkg, "gridmap")["maps"][0]["tile_size"] == 1024


def test_a_packaged_map_is_staged_verbatim(tmp_path):
    """A hand-authored descriptor is the package's identity; never regenerate it."""
    pkg = make_package(tmp_path, "roosevelt_full", {
        "roosevelt_full.json": json.dumps({"maps": [{
            "name": "roosevelt_full",
            "xodr": "roosevelt_full/roosevelt_full.xodr",
            "source": "roosevelt_full/roosevelt_full.fbx",
            "use_carla_materials": False}], "props": []}),
        "roosevelt_full/roosevelt_full.xodr": "x",
        "roosevelt_full/roosevelt_full.fbx": "x"})
    entry = stage(tmp_path, pkg, "roosevelt_full")["maps"][0]

    assert entry["xodr"] == "roosevelt_full/roosevelt_full.xodr"
    assert "exported_as" not in entry


# ------------------------------------------------------------ failure modes

def test_export_without_geometry_is_refused(tmp_path):
    pkg = make_package(tmp_path, "lonely", {"lonely.xodr": "x"})
    root = tmp_path / "carla"
    root.mkdir()
    with pytest.raises(SystemExit, match="no .fbx"):
        import_map.stage_package(str(root), "lonely_map", package_dir=pkg)


def test_export_without_a_road_network_is_refused(tmp_path):
    pkg = make_package(tmp_path, "thing", {"thing.fbx": "x"})
    root = tmp_path / "carla"
    root.mkdir()
    with pytest.raises(SystemExit, match="no .xodr"):
        import_map.stage_package(str(root), "thing", package_dir=pkg)


def test_a_failed_stage_does_not_leave_the_export_behind(tmp_path):
    """Else the next attempt reads the leftovers as the same map staged twice."""
    pkg = make_package(tmp_path, "lonely", {"lonely.xodr": "x"})
    root = tmp_path / "carla"
    root.mkdir()
    with pytest.raises(SystemExit):
        import_map.stage_package(str(root), "lonely_map", package_dir=pkg)

    assert not (root / "Import" / "lonely_map").exists()


def test_the_same_map_staged_twice_is_refused(tmp_path):
    """Two copies would fight over one /Game/<pkg>/Maps/<name> destination."""
    pkg = make_package(tmp_path, "dupe", {"a/dup.xodr": "x", "a/dup.fbx": "x",
                                          "b/dup.xodr": "x", "b/dup.fbx": "x"})
    root = tmp_path / "carla"
    root.mkdir()
    with pytest.raises(SystemExit, match="staged in 2 places"):
        import_map.stage_package(str(root), "dup_map", package_dir=pkg)


def test_single_source_and_tiles_together_is_a_contradiction(tmp_path):
    pkg = raw_export(tmp_path, "m", tiles=["", "_Tile_0_0"])
    root = tmp_path / "carla"
    root.mkdir()
    with pytest.raises(SystemExit, match="single-source or tiled"):
        import_map.stage_package(str(root), "m", package_dir=pkg)


def test_two_maps_in_one_package_are_refused(tmp_path):
    """One package holds one map - a tiled map is still one map. A second .xodr
    is a packaging mistake, not a choice to offer: both would land on the same
    /Game/<pkg>/Maps/<name> and overwrite each other."""
    pkg = make_package(tmp_path, "twomaps", {"one.xodr": "x", "one.fbx": "x",
                                             "two.xodr": "x", "two.fbx": "x"})
    root = tmp_path / "carla"
    root.mkdir()
    with pytest.raises(SystemExit, match="one package holds one map"):
        import_map.stage_package(str(root), "one_of_them", package_dir=pkg)


def test_geometry_named_apart_from_the_road_network_is_warned_about(tmp_path):
    """One map per package identifies it - a lone .fbx can only be this map's
    geometry - but the mismatch is worth seeing: it usually means the package was
    assembled by hand, and that .fbx might be scenery rather than the road."""
    pkg = make_package(tmp_path, "apart", {"MLK.xodr": "x", "MLK_final.fbx": "x"})
    entry = stage(tmp_path, pkg, "mlk_no_signal")["maps"][0]

    assert entry["source"] == "mlk_no_signal/mlk_no_signal.fbx"
    assert entry["xodr"] == "mlk_no_signal/MLK.xodr"


def test_the_mismatch_warning_names_the_file_it_adopted(tmp_path, capsys):
    pkg = make_package(tmp_path, "apart", {"MLK.xodr": "x", "MLK_final.fbx": "x"})
    stage(tmp_path, pkg, "mlk_no_signal")

    out = capsys.readouterr().out
    assert "warning: MLK.xodr has no MLK.fbx" in out
    assert "MLK_final.fbx" in out


def test_a_lone_tile_set_named_apart_is_warned_about(tmp_path):
    pkg = make_package(tmp_path, "apart_tiles", {
        "Atl.xodr": "x", "Atl_R2024b_Tile_0_0.fbx": "x", "Atl_R2024b_Tile_0_1.fbx": "x"})
    entry = stage(tmp_path, pkg, "atlanta_full")["maps"][0]

    assert entry["tiles"] == ["atlanta_full/atlanta_full_Tile_0_0.fbx",
                              "atlanta_full/atlanta_full_Tile_0_1.fbx"]


def test_a_layer_split_export_is_refused(tmp_path):
    """Nothing ties any of these to the .xodr, and cooking the wrong layer as the
    map would only surface after the cook."""
    pkg = make_package(tmp_path, "layers", {"Roosevelt.xodr": "x",
                                            "Roosevelt_roads.fbx": "x",
                                            "Roosevelt_buildings.fbx": "x"})
    root = tmp_path / "carla"
    root.mkdir()
    with pytest.raises(SystemExit, match="several geometries could be it"):
        import_map.stage_package(str(root), "roosevelt_full", package_dir=pkg)


# ---------------------------------------------------- name inference helpers

def test_package_name_falls_back_to_the_xodr_stem(tmp_path):
    """A raw export has no descriptor to read a name off, but it has one .xodr."""
    assert import_map._infer_package_name(
        raw_export(tmp_path, "Deep_Export")) == "Deep_Export"


def test_package_name_prefers_the_descriptor(tmp_path):
    pkg = make_package(tmp_path, "pkg", {"roosevelt_full.json": "{}",
                                         "roosevelt_full/roosevelt_full.xodr": "x"})
    assert import_map._infer_package_name(pkg) == "roosevelt_full"


def test_package_name_ignores_the_road_painter_decals(tmp_path):
    """It ships beside a real descriptor and is not a map."""
    pkg = make_package(tmp_path, "pkg", {"roadpainter_decals.json": "{}",
                                         "uga_v4.json": "{}"})
    assert import_map._infer_package_name(pkg) == "uga_v4"


def test_package_name_is_none_when_ambiguous(tmp_path):
    pkg = make_package(tmp_path, "amb", {"a.xodr": "x", "b.xodr": "x"})
    assert import_map._infer_package_name(pkg) is None


def test_package_name_reads_a_nested_folder(tmp_path):
    inner = tmp_path / "export" / "inner"
    inner.mkdir(parents=True)
    (inner / "Deep_Export.xodr").write_text("x")
    assert import_map._infer_package_name(str(tmp_path / "export")) == "Deep_Export"


# ------------------------------------------------------------- tile parsing

@pytest.mark.parametrize("name, expected", [
    ("Map_Tile_3_7.fbx", ("Map", 3, 7)),
    ("A_B_C_Tile_0_0.fbx", ("A_B_C", 0, 0)),
    ("Map_Tile_0_0_final.fbx", None),   # would collide with tile (0, 0)
    ("Map_Tile_x_y.fbx", None),
    ("Map.fbx", None),
    ("Map_Tile_0_0.xodr", None),
])
def test_tile_names_are_parsed_strictly(name, expected):
    """CARLA reads the grid index off the last two underscore tokens, so a loose
    name would cook as the wrong tile rather than be rejected."""
    assert import_map._tile_split(name) == expected
