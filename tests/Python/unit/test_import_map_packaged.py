"""Precooked-map install into a PACKAGED CARLA (FIXS #258).

No packaged CARLA is needed: the install is a plain extract into the package root,
so a directory with a stub launcher is a faithful stand-in for everything these
functions look at. What is NOT faked is the archive - the tests run against a real
Digital-Twin-Library `*_cooked.tar.gz`, because the point is that the layout CARLA
actually publishes resolves through our path.

Point FIXS_COOKED_TAR at one to enable them, e.g.

    gh release download mlk -R ORNL-Real-Sim/Digital-Twin-Library \
        -p 'mlk_no_signal_cooked.tar.gz' -D /tmp
    FIXS_COOKED_TAR=/tmp/mlk_no_signal_cooked.tar.gz pytest tests/Python/unit
"""
import os
import platform
import sys
import tarfile

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__),
                                "..", "..", "..", "scripts", "cosim"))
import import_map  # noqa: E402

COOKED_TAR = os.environ.get("FIXS_COOKED_TAR")

pytestmark = pytest.mark.skipif(
    not COOKED_TAR or not os.path.isfile(COOKED_TAR),
    reason="set FIXS_COOKED_TAR to a Digital-Twin-Library *_cooked.tar.gz")


def _map_name():
    """The map the archive under test provides, read from the archive itself."""
    with tarfile.open(COOKED_TAR, "r:*") as tar:
        return import_map._cooked_name_in(tar.getmembers())


@pytest.fixture
def packaged_root(tmp_path):
    """A stub packaged-CARLA root: only the launcher env.packaged_exe looks for."""
    root = tmp_path / "CarlaPackaged"
    root.mkdir()
    exe = "CarlaUE4.exe" if platform.system() == "Windows" else "CarlaUE4.sh"
    (root / exe).write_text("stub")
    return str(root)


def test_content_root_differs_by_flavour(packaged_root):
    """The ONLY path difference between the two flavours is this prefix."""
    src = import_map.content_root("/some/carla", mode="source")
    pkg = import_map.content_root(packaged_root, mode="packaged")
    assert src.replace("\\", "/").endswith("Unreal/CarlaUE4/Content")
    assert pkg.replace("\\", "/").endswith("CarlaPackaged/CarlaUE4/Content")


def test_content_root_accepts_the_wrapper_spelling(tmp_path):
    """carla_root may name the folder holding {Windows,Linux}NoEditor/ instead of
    the package root itself; both must resolve to the same Content dir."""
    outer = tmp_path / "CARLA_0.9.15"
    sub = "WindowsNoEditor" if platform.system() == "Windows" else "LinuxNoEditor"
    inner = outer / sub
    inner.mkdir(parents=True)
    exe = "CarlaUE4.exe" if platform.system() == "Windows" else "CarlaUE4.sh"
    (inner / exe).write_text("stub")

    assert import_map.content_root(str(outer), mode="packaged") == \
        import_map.content_root(str(inner), mode="packaged")


def test_install_resolves_the_level_the_same_way_a_cook_would(packaged_root):
    name = _map_name()
    assert name, "archive does not declare exactly one package descriptor"

    installed = import_map.install_cooked(packaged_root, COOKED_TAR)
    assert installed == name

    assert import_map.map_is_imported(packaged_root, name, mode="packaged")
    assert import_map.resolve_cooked_map(packaged_root, name, mode="packaged") == \
        (name, f"/Game/{name}/Maps/{name}/{name}")

    # The shipped descriptor must agree - resolve_cooked_map falls back to it when
    # the conventional layout is absent, so a disagreement here is a latent bug.
    assert import_map.declared_maps(packaged_root, name, mode="packaged") == \
        [(name, f"/Game/{name}/Maps/{name}/{name}")]

    assert import_map.list_imported_maps(packaged_root, mode="packaged") == [name]


def test_reinstall_is_a_noop_and_force_reinstalls(packaged_root):
    name = import_map.install_cooked(packaged_root, COOKED_TAR)
    umap = import_map.cooked_map_path(packaged_root, name, mode="packaged")
    stamp = os.path.getmtime(umap)

    import_map.install_cooked(packaged_root, COOKED_TAR)
    assert os.path.getmtime(umap) == stamp, "plain re-install rewrote the content"

    # A stray file under Content/<name> proves force wiped the tree rather than
    # merging onto it - the failure mode ensure_map's clean-reimport exists for.
    stray = os.path.join(import_map.cooked_content_dir(packaged_root, name,
                                                       mode="packaged"), "stale.uasset")
    open(stray, "w").close()
    import_map.install_cooked(packaged_root, COOKED_TAR, force=True)
    assert not os.path.exists(stray)
    assert import_map.map_is_imported(packaged_root, name, mode="packaged")


def test_traversal_outside_the_package_root_is_refused(tmp_path, packaged_root):
    evil = tmp_path / "evil.tar.gz"
    with tarfile.open(evil, "w:gz") as tar:
        payload = tmp_path / "payload"
        payload.write_text("x")
        tar.add(str(payload), arcname="../../escaped.uasset")

    with pytest.raises(SystemExit):
        import_map.install_cooked(packaged_root, str(evil), name="whatever")
    assert not (tmp_path.parent / "escaped.uasset").exists()


def test_installed_package_reports_its_shader_platform(packaged_root):
    """The published assets are Linux cooks (FIXS_Applications#29): SPIR-V only,
    no D3D. A packaged CARLA cannot compile shaders, so on Windows this is the
    difference between a correct map and a grey one - and nothing downstream can
    tell, which is why it is detected at install time."""
    name = import_map.install_cooked(packaged_root, COOKED_TAR)
    content = import_map.cooked_content_dir(packaged_root, name, mode="packaged")

    found = import_map.shader_platforms_in(content)
    assert found, "no recognised shader bytecode in the package at all"
    assert found <= {"d3d", "vulkan"}
    # Not asserting 'vulkan' specifically: a future Windows or dual cook must not
    # fail this test. What must hold is that we can NAME what is in there.
    assert import_map.host_shader_platform() in ("d3d", "vulkan")


def test_shader_scan_ignores_large_blobs(packaged_root):
    """Shader maps live in materials (~80KB); the .umap is ~19MB and the meshes
    larger still. Scanning them would make every install pay for nothing."""
    name = import_map.install_cooked(packaged_root, COOKED_TAR)
    content = import_map.cooked_content_dir(packaged_root, name, mode="packaged")
    assert import_map.shader_platforms_in(content, size_cap=1024) == set()


def test_cooked_asset_name_convention_and_catalog_override():
    assert import_map.cooked_asset_name("mlk_no_signal.zip") == \
        "mlk_no_signal_cooked.tar.gz"
    # catalog wins over the convention ...
    assert import_map.catalog_cooked_asset(
        {"asset": "a.zip", "cooked_asset": "b.tar.gz"}) == "b.tar.gz"
    # ... and an explicit empty value means "this map has none", which must NOT
    # fall back to a derived name that is not published.
    assert import_map.catalog_cooked_asset({"asset": "a.zip", "cooked_asset": ""}) is None
    assert import_map.catalog_cooked_asset({"asset": "a.zip"}) == "a_cooked.tar.gz"
