"""Client mode extracts only the SUMO half of a map bundle (FIXS #309).

A machine with no local CARLA never imports, so the bundle's carla/ half is written
once and read never - 368 MB of roosevelt_full's 370. No CARLA or network needed
here: a bundle is just a zip with carla/ and sumo/ at the top.
"""
import os
import sys
import zipfile

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__),
                                "..", "..", "..", "scripts", "cosim"))
import import_map  # noqa: E402


@pytest.fixture
def bundle(tmp_path):
    """A minimal DT-Library-shaped bundle: carla/ + sumo/ + props/."""
    path = tmp_path / "roosevelt_full.zip"
    with zipfile.ZipFile(path, "w") as z:
        z.writestr("carla/roosevelt_full.fbx", "x" * 4096)
        z.writestr("carla/roosevelt_full.xodr", "<OpenDRIVE/>")
        z.writestr("sumo/roosevelt.sumocfg", "<configuration/>")
        z.writestr("sumo/roosevelt.net.xml", "<net/>")
        z.writestr("props/placement.yaml", "props: []")
    return str(path)


@pytest.fixture
def cache(tmp_path, monkeypatch):
    """Redirect the map cache so nothing touches the real ~/.fixs."""
    d = tmp_path / "cache"
    d.mkdir()
    monkeypatch.setattr(import_map, "_map_cache_dir", lambda name=None: str(d))
    return d


def _mode(monkeypatch, mode):
    monkeypatch.setattr(import_map, "_mode", lambda m=None: mode)


def test_client_mode_skips_the_carla_half(bundle, cache, monkeypatch):
    _mode(monkeypatch, "client")
    carla_src, sumo_dir = import_map.open_bundle(bundle, cache_name="roosevelt_full")

    assert not os.path.isdir(cache / "carla")
    assert os.path.isfile(cache / "sumo" / "roosevelt.sumocfg")
    # None, not the cache dir: handing back a directory holding no CARLA package
    # would send an importer off to fail on it.
    assert carla_src is None
    assert sumo_dir == str(cache / "sumo")


def test_source_mode_still_gets_everything(bundle, cache, monkeypatch):
    _mode(monkeypatch, "source")
    carla_src, sumo_dir = import_map.open_bundle(bundle, cache_name="roosevelt_full")

    assert os.path.isfile(cache / "carla" / "roosevelt_full.fbx")
    assert carla_src == str(cache / "carla")
    assert sumo_dir == str(cache / "sumo")


def test_client_mode_does_not_re_extract_every_run(bundle, cache, monkeypatch, capsys):
    """The freshness check has to key on a directory that is actually extracted.
    Keyed on carla/ - permanently absent here - every run would re-unpack."""
    _mode(monkeypatch, "client")
    import_map.open_bundle(bundle, cache_name="roosevelt_full")
    capsys.readouterr()

    import_map.open_bundle(bundle, cache_name="roosevelt_full")
    assert "unpacked" not in capsys.readouterr().out


def test_a_newer_bundle_still_re_extracts(bundle, cache, monkeypatch, capsys):
    """...but the freshness check must still fire when the zip genuinely moves on."""
    _mode(monkeypatch, "client")
    import_map.open_bundle(bundle, cache_name="roosevelt_full")
    capsys.readouterr()

    future = os.path.getmtime(str(cache / "sumo")) + 60
    os.utime(bundle, (future, future))
    import_map.open_bundle(bundle, cache_name="roosevelt_full")
    assert "unpacked" in capsys.readouterr().out


def test_switching_to_a_local_carla_recovers(bundle, cache, monkeypatch):
    """No bookkeeping records that the CARLA half was skipped, so the recovery path
    has to work on its own: extracting again in source mode fills it in."""
    _mode(monkeypatch, "client")
    import_map.open_bundle(bundle, cache_name="roosevelt_full")
    assert not os.path.isdir(cache / "carla")

    # No mtime bump: the zip is untouched, and recovery still has to happen. It does
    # because the freshness sentinel moves with the mode - in source mode it is
    # carla/, which this cache does not have, so the bundle is unpacked in full.
    _mode(monkeypatch, "source")
    carla_src, _ = import_map.open_bundle(bundle, cache_name="roosevelt_full")
    assert os.path.isfile(cache / "carla" / "roosevelt_full.fbx")
    assert carla_src == str(cache / "carla")
