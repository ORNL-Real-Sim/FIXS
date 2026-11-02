"""The generated traffic-light table is cached by the SUMO net's contents (#243).

Keyed by name, a net whose signals changed under the same name reused the old table,
and since heads are matched to CARLA by proximity, nothing reported it.
"""
import os
import sys
import types

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "..", "Carla"))
import run_cosim  # noqa: E402


@pytest.fixture
def cache(tmp_path, monkeypatch):
    """Point ~/.fixs at tmp and replace the generator with one that copies the net."""
    home = tmp_path / "fixs"
    home.mkdir()
    monkeypatch.setattr(run_cosim.env, "CONFIG_PATH", str(home / "carla.json"))
    calls = []

    def generate_table(net, out):
        calls.append(net)
        os.makedirs(os.path.dirname(out), exist_ok=True)
        with open(net, encoding="utf-8") as src, open(out, "w", encoding="utf-8") as dst:
            dst.write(src.read())
        return out

    monkeypatch.setitem(sys.modules, "extract_sumo_tls_as_table",
                        types.SimpleNamespace(generate_table=generate_table))
    return home, calls


def _scenario(folder, net_text):
    folder.mkdir(parents=True, exist_ok=True)
    (folder / "map.net.xml").write_text(net_text, encoding="utf-8")
    cfg = folder / "map.sumocfg"
    cfg.write_text('<configuration><input><net-file value="map.net.xml"/></input>'
                   "</configuration>", encoding="utf-8")
    return str(cfg)


def _read(path):
    with open(path, encoding="utf-8") as f:
        return f.read()


def test_edited_net_under_same_name_gets_a_new_table(tmp_path, cache):
    cfg = _scenario(tmp_path / "sumo", "<net>signals v1</net>")
    first = run_cosim.resolve_tl_table(cfg)
    _scenario(tmp_path / "sumo", "<net>signals v2</net>")
    second = run_cosim.resolve_tl_table(cfg)
    assert second != first
    assert _read(second) == "<net>signals v2</net>"


def test_different_nets_with_the_same_file_name_do_not_share(tmp_path, cache):
    a = run_cosim.resolve_tl_table(_scenario(tmp_path / "a", "<net>A</net>"))
    b = run_cosim.resolve_tl_table(_scenario(tmp_path / "b", "<net>B</net>"))
    assert a != b
    assert (_read(a), _read(b)) == ("<net>A</net>", "<net>B</net>")


def test_same_net_contents_hit_the_cache(tmp_path, cache):
    home, calls = cache
    a = run_cosim.resolve_tl_table(_scenario(tmp_path / "a", "<net>same</net>"))
    b = run_cosim.resolve_tl_table(_scenario(tmp_path / "b", "<net>same</net>"))
    assert a == b
    assert len(calls) == 1
    assert os.path.dirname(a) == str(home / "tables")
    assert not (home / "maps").exists()


def test_force_regenerates(tmp_path, cache):
    _home, calls = cache
    cfg = _scenario(tmp_path / "sumo", "<net>x</net>")
    run_cosim.resolve_tl_table(cfg)
    run_cosim.resolve_tl_table(cfg, force=True)
    assert len(calls) == 2
