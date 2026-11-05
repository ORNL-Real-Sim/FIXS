"""The importer reads a badly packed bundle instead of asking the user to repair it.

roosevelt_ring.zip shipped carla/ + sumo/ inside a roosevelt_ring/ folder, with a
.sumocfg still naming the files of the bundle it was derived from. Neither needs a
CARLA, SUMO or the network to exercise: a bundle is just a zip.
"""
import os
import sys
import zipfile

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "..", "Carla"))
import import_map  # noqa: E402

CFG = """<?xml version="1.0" ?>
<configuration xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
    xsi:noNamespaceSchemaLocation="http://sumo.dlr.de/xsd/sumoConfiguration.xsd">
    <input>
        <net-file value="{net}"/>
        <route-files value="{rou}"/>
        <additional-files value=""/>
    </input>
    <output><fcd-output value="fcd.xml"/></output>
    <time><begin value="28800"/></time>
</configuration>
"""


def _zip(path, entries):
    with zipfile.ZipFile(path, "w") as z:
        for name, data in entries.items():
            z.writestr(name, data)
    return str(path)


def _bundle_entries(root="", carla="carla", sumo="sumo", net="m.net.xml", rou="m.rou.xml"):
    return {
        f"{root}{carla}/m.xodr": "<OpenDRIVE/>",
        f"{root}{carla}/m.fbx": "x",
        f"{root}{sumo}/m.sumocfg": CFG.format(net=net, rou=rou),
        f"{root}{sumo}/m.net.xml": "<net/>",
        f"{root}{sumo}/m.rou.xml": "<routes/>",
    }


@pytest.fixture
def cache(tmp_path, monkeypatch):
    """Redirect the map cache so nothing touches the real ~/.fixs."""
    d = tmp_path / "cache"
    monkeypatch.setenv("FIXS_MAP_CACHE", str(d))
    monkeypatch.setattr(import_map, "_mode", lambda m=None: "source")
    monkeypatch.setattr(import_map, "_SUMOCFG_NOTED", set())
    monkeypatch.setattr(import_map, "_ASSETS_SETTLED", {})
    return d


# ------------------------------------------------------------- recognising a bundle

@pytest.mark.parametrize("names, expected", [
    (["carla/m.xodr", "sumo/m.sumocfg"], ""),
    (["carla/m.xodr"], ""),                           # always enough at the top
    (["Carla/m.xodr", "SUMO/m.sumocfg"], ""),
    (["m/", "m/carla/", "m/carla/m.xodr", "m/sumo/m.sumocfg"], "m/"),
    (["a/b/carla/m.xodr", "a/b/sumo/m.sumocfg"], "a/b/"),
    (["README.txt", "m/carla/m.xodr", "m/sumo/m.sumocfg"], "m/"),
    (["m\\carla\\m.xodr", "m\\sumo\\m.sumocfg"], "m/"),
    (["__MACOSX/m/carla/._m.xodr", "m/carla/m.xodr", "m/sumo/m.sumocfg"], "m/"),
    # The shallowest wins, so a carla/ further inside the package is not a rival.
    (["m/carla/x/carla/a", "m/carla/x/sumo/a", "m/sumo/a"], "m/"),
])
def test_bundle_prefix_finds_the_root(names, expected):
    assert import_map._bundle_prefix(names) == expected


@pytest.mark.parametrize("names", [
    ["m.json", "m/m.xodr", "m/m.fbx"],                # legacy CARLA-only package
    ["m.json", "m/carla/textures.png"],               # a folder merely named carla
    ["__MACOSX/carla/._m.xodr", "m.json"],            # only the mac mirror has it
    ["a/carla/m", "a/sumo/m", "b/carla/m", "b/sumo/m"],   # two maps: cannot pick
    ["a/b/c/d/carla/m", "a/b/c/d/sumo/m"],            # beyond any plausible wrapper
    ["sumo/m.sumocfg"],                               # a SUMO-only scenario
])
def test_bundle_prefix_leaves_other_packages_alone(names):
    assert import_map._bundle_prefix(names) is None


# -------------------------------------------------------------- opening a bundle

def test_wrapped_zip_extracts_to_the_normal_layout(tmp_path, cache, capsys):
    src = _zip(tmp_path / "m.zip", _bundle_entries(root="m/"))
    carla, sumo = import_map.open_bundle(src, cache_name="m")

    assert carla == str(cache / "m" / "carla")
    assert sumo == str(cache / "m" / "sumo")
    assert os.path.isfile(os.path.join(carla, "m.xodr"))
    assert os.path.isfile(os.path.join(sumo, "m.sumocfg"))
    assert not os.path.exists(cache / "m" / "m")      # the wrapper is not recreated
    assert "wraps carla/ + sumo/" in capsys.readouterr().out


def test_published_layout_reports_nothing_unusual(tmp_path, cache, capsys):
    src = _zip(tmp_path / "m.zip", _bundle_entries())
    carla, sumo = import_map.open_bundle(src, cache_name="m")
    assert carla == str(cache / "m" / "carla") and sumo == str(cache / "m" / "sumo")
    assert "wraps" not in capsys.readouterr().out


def test_wrapped_zip_in_client_mode_takes_only_the_sumo_half(tmp_path, cache, monkeypatch):
    monkeypatch.setattr(import_map, "_mode", lambda m=None: "client")
    src = _zip(tmp_path / "m.zip", _bundle_entries(root="m/"))
    carla, sumo = import_map.open_bundle(src, cache_name="m")
    assert carla is None
    assert os.path.isfile(os.path.join(sumo, "m.net.xml"))
    assert not os.path.exists(cache / "m" / "carla")


def test_half_names_are_normalised_for_a_case_sensitive_disk(tmp_path, cache):
    src = _zip(tmp_path / "m.zip", _bundle_entries(carla="Carla", sumo="SUMO"))
    carla, sumo = import_map.open_bundle(src, cache_name="m")
    assert os.path.basename(carla) == "carla" and os.path.basename(sumo) == "sumo"
    assert "carla" in os.listdir(cache / "m") and "sumo" in os.listdir(cache / "m")


def test_archiver_junk_is_not_extracted(tmp_path, cache):
    entries = _bundle_entries(root="m/")
    entries["m/sumo/.DS_Store"] = "x"
    entries["__MACOSX/m/sumo/._m.net.xml"] = "x"
    src = _zip(tmp_path / "m.zip", entries)
    _carla, sumo = import_map.open_bundle(src, cache_name="m")
    assert ".DS_Store" not in os.listdir(sumo)
    assert not os.path.exists(cache / "m" / "__MACOSX")


def test_a_path_escaping_the_cache_stays_inside_it(tmp_path, cache):
    entries = _bundle_entries(root="m/")
    entries["m/../../escaped.txt"] = "x"
    src = _zip(tmp_path / "m.zip", entries)
    import_map.open_bundle(src, cache_name="m")
    assert not (tmp_path / "escaped.txt").exists()
    assert not (cache / "escaped.txt").exists()


def test_a_zip_picked_at_the_sumo_prompt_is_accepted(tmp_path, cache):
    """The prompt that got a user stuck: classify_source refused this zip, since it
    carries geometry, so it had to be extracted by hand."""
    src = _zip(tmp_path / "m.zip", _bundle_entries(root="m/"))
    _carla, sumo = import_map.classify_source(src, "m")
    assert sumo == str(cache / "m" / "sumo")


def test_hand_extracted_wrapper_folder_is_read_through(tmp_path, cache):
    src = tmp_path / "picked"
    with zipfile.ZipFile(_zip(tmp_path / "m.zip", _bundle_entries(root="m/x/"))) as z:
        z.extractall(src)
    carla, sumo = import_map.open_bundle(str(src), cache_name="m")
    assert carla == str(src / "m" / "x" / "carla")
    assert sumo == str(src / "m" / "x" / "sumo")


def test_folder_with_several_things_in_it_is_not_guessed_at(tmp_path, cache):
    (tmp_path / "picked" / "a" / "carla").mkdir(parents=True)
    (tmp_path / "picked" / "a" / "sumo").mkdir()
    (tmp_path / "picked" / "b").mkdir()
    src = str(tmp_path / "picked")
    assert import_map.open_bundle(src, cache_name="m") == (src, None)


def test_wrapped_carla_without_sumo_folder_stays_a_plain_package(tmp_path, cache):
    (tmp_path / "picked" / "m" / "carla").mkdir(parents=True)
    src = str(tmp_path / "picked")
    assert import_map.open_bundle(src, cache_name="m") == (src, None)


# ------------------------------------------------------------------- parity

def test_parity_matches_through_a_wrapper(tmp_path, cache):
    """The cached zip is the reference, so a wrapper must not read as every file gone."""
    (cache / "m").mkdir(parents=True)
    src = _zip(cache / "m" / "m.zip", _bundle_entries(root="m/"))
    import_map.open_bundle(src, cache_name="m")
    assert import_map.bundle_parity("m", "sumo") == ("match", [], [], [])
    assert import_map.bundle_parity("m", "carla") == ("match", [], [], [])


def test_parity_still_sees_an_edit_through_a_wrapper(tmp_path, cache):
    (cache / "m").mkdir(parents=True)
    src = _zip(cache / "m" / "m.zip", _bundle_entries(root="m/"))
    _carla, sumo = import_map.open_bundle(src, cache_name="m")
    with open(os.path.join(sumo, "m.rou.xml"), "w", encoding="utf-8") as f:
        f.write("<routes>edited</routes>")
    assert import_map.bundle_parity("m", "sumo") == ("diverged", ["m.rou.xml"], [], [])


# --------------------------------------------------------- a .sumocfg's own names

def _scenario(tmp_path, net, rou, files=("m.net.xml", "m.rou.xml"), stem="m"):
    d = tmp_path / "sumo"
    d.mkdir()
    for f in files:
        (d / f).write_text("<x/>")
    cfg = d / f"{stem}.sumocfg"
    cfg.write_text(CFG.format(net=net, rou=rou))
    return str(cfg)


def _values(path):
    root = import_map.ET.parse(path).getroot()
    return root.find(".//net-file").get("value"), root.find(".//route-files").get("value")


def test_a_correct_cfg_is_returned_as_is(tmp_path, cache):
    cfg = _scenario(tmp_path, "m.net.xml", "m.rou.xml")
    assert import_map.sumocfg_missing_inputs(cfg) == []
    assert import_map.runnable_sumocfg(cfg, cache_name="m") == cfg
    assert not (cache / "m" / "sumo_corrected").exists()


def test_wrong_names_with_a_certain_answer_get_a_corrected_copy(tmp_path, cache, capsys):
    cfg = _scenario(tmp_path, "old.net.xml", "old.rou.xml")
    before = open(cfg, encoding="utf-8").read()

    got = import_map.runnable_sumocfg(cfg, cache_name="m")

    folder = os.path.dirname(cfg)
    assert got == str(cache / "m" / "sumo_corrected" / "m.sumocfg")
    assert _values(got) == (os.path.join(folder, "m.net.xml"),
                            os.path.join(folder, "m.rou.xml"))
    assert open(cfg, encoding="utf-8").read() == before      # shipped file untouched
    out = capsys.readouterr().out
    assert "old.net.xml -> m.net.xml" in out and "untouched" in out


def test_the_copy_keeps_everything_else_in_the_cfg(tmp_path, cache):
    cfg = _scenario(tmp_path, "old.net.xml", "old.rou.xml")
    root = import_map.ET.parse(import_map.runnable_sumocfg(cfg, cache_name="m")).getroot()
    assert root.find(".//begin").get("value") == "28800"
    assert root.find(".//fcd-output").get("value") == "fcd.xml"      # outputs not resolved
    assert root.find(".//additional-files").get("value") == ""


def test_the_notice_prints_once_a_run(tmp_path, cache, capsys):
    cfg = _scenario(tmp_path, "old.net.xml", "old.rou.xml")
    first = import_map.runnable_sumocfg(cfg, cache_name="m")
    capsys.readouterr()
    assert import_map.runnable_sumocfg(cfg, cache_name="m") == first
    assert capsys.readouterr().out == ""


def test_a_lone_net_is_certain_whatever_its_name(tmp_path, cache):
    cfg = _scenario(tmp_path, "old.net.xml", "m.rou.xml", files=("other.net.xml", "m.rou.xml"))
    assert import_map.sumocfg_missing_inputs(cfg) == [("net-file", "old.net.xml", "other.net.xml")]


def test_two_nets_are_not_guessed_between(tmp_path, cache):
    cfg = _scenario(tmp_path, "old.net.xml", "m.rou.xml",
                    files=("a.net.xml", "b.net.xml", "m.rou.xml"))
    with pytest.raises(SystemExit, match="no certain replacement"):
        import_map.runnable_sumocfg(cfg, cache_name="m")


def test_a_lone_route_file_of_another_name_is_not_assumed_to_be_the_demand(tmp_path, cache):
    """It could be vtypes.rou.xml. Running on it would be different traffic, silently."""
    cfg = _scenario(tmp_path, "m.net.xml", "old.rou.xml", files=("m.net.xml", "vtypes.rou.xml"))
    assert import_map.sumocfg_missing_inputs(cfg) == [("route-files", "old.rou.xml", None)]
    with pytest.raises(SystemExit, match="old.rou.xml"):
        import_map.runnable_sumocfg(cfg, cache_name="m")


def test_one_replacement_is_not_used_for_two_missing_files(tmp_path, cache):
    cfg = _scenario(tmp_path, "m.net.xml", "a.rou.xml,b.rou.xml")
    assert import_map.sumocfg_missing_inputs(cfg) == [("route-files", "a.rou.xml", "m.rou.xml"),
                                                      ("route-files", "b.rou.xml", None)]


def test_a_file_already_used_is_not_offered_again(tmp_path, cache):
    cfg = _scenario(tmp_path, "m.net.xml", "m.rou.xml,old.rou.xml")
    assert import_map.sumocfg_missing_inputs(cfg) == [("route-files", "old.rou.xml", None)]


def test_a_cfg_the_user_named_is_diagnosed_not_copied(tmp_path, cache):
    cfg = _scenario(tmp_path, "old.net.xml", "old.rou.xml")
    with pytest.raises(SystemExit, match="old.net.xml -> m.net.xml"):
        import_map.runnable_sumocfg(cfg, cache_name=None)
    assert not cache.exists() or not list(cache.rglob("sumo_corrected"))


def test_an_unreadable_cfg_is_left_for_sumo_to_report(tmp_path, cache):
    cfg = tmp_path / "m.sumocfg"
    cfg.write_text("<configuration")
    assert import_map.runnable_sumocfg(str(cfg), cache_name="m") == str(cfg)


def test_the_copy_does_not_disturb_parity_or_the_one_cfg_rule(tmp_path, cache):
    entries = _bundle_entries(root="m/", net="old.net.xml", rou="old.rou.xml")
    (cache / "m").mkdir(parents=True)
    src = _zip(cache / "m" / "m.zip", entries)
    _carla, sumo = import_map.open_bundle(src, cache_name="m")
    cfg = import_map.bundle_sumocfg(sumo)
    assert import_map.runnable_sumocfg(cfg, cache_name="m") != cfg
    assert import_map.bundle_sumocfg(sumo) == cfg
    assert import_map.map_sumo_dir("m") == sumo
    assert import_map.bundle_parity("m", "sumo") == ("match", [], [], [])


# ---------------------------------------------------------- asking only once

def _cached_zip(cache, monkeypatch):
    (cache / "m").mkdir(parents=True)
    (cache / "m" / "m.zip").write_text("x")
    monkeypatch.setattr(import_map, "_require_gh", lambda: "gh")
    monkeypatch.setattr(import_map.sys.stdin, "isatty", lambda: True, raising=False)
    asked = []

    def answer(prompt=""):
        asked.append(prompt)
        return "u"
    monkeypatch.setattr("builtins.input", answer)
    return asked


def test_the_cached_zip_question_is_asked_once_a_run(cache, monkeypatch):
    asked = _cached_zip(cache, monkeypatch)
    first = import_map.download_release_zip("o/r", "t", cache_name="m", asset="m.zip")
    second = import_map.download_release_zip("o/r", "t", cache_name="m", asset="m.zip")
    assert first == second and len(asked) == 1


def test_a_redownload_is_not_repeated_within_a_run(cache, monkeypatch):
    _cached_zip(cache, monkeypatch)
    downloads = []
    monkeypatch.setattr(import_map, "_release_asset_sha", lambda *a: None)
    monkeypatch.setattr(import_map.subprocess, "call", lambda cmd: downloads.append(cmd) or 0)
    for _ in range(2):
        import_map.download_release_zip("o/r", "t", force_redownload=True,
                                        cache_name="m", asset="m.zip")
    assert len(downloads) == 1


def test_a_new_run_in_the_same_process_decides_again(cache, monkeypatch):
    """--serve re-enters main() per run, where a later --reimport must still download."""
    _cached_zip(cache, monkeypatch)
    downloads = []
    monkeypatch.setattr(import_map, "_release_asset_sha", lambda *a: None)
    monkeypatch.setattr(import_map.subprocess, "call", lambda cmd: downloads.append(cmd) or 0)
    for _ in range(2):
        import_map.start_new_run()
        import_map.download_release_zip("o/r", "t", force_redownload=True,
                                        cache_name="m", asset="m.zip")
    assert len(downloads) == 2


def test_serve_starts_each_run_afresh(cache, monkeypatch):
    import run_cosim
    import_map._ASSETS_SETTLED[("d", "p")] = "stale"
    import_map._SUMOCFG_NOTED.add("stale")
    seen = []

    def one_run():
        seen.append((dict(import_map._ASSETS_SETTLED), set(import_map._SUMOCFG_NOTED)))
        raise KeyboardInterrupt
    monkeypatch.setattr(run_cosim, "main", one_run)
    assert run_cosim.serve_forever() == 0
    assert seen == [({}, set())]


def test_a_different_asset_is_still_its_own_question(cache, monkeypatch):
    asked = _cached_zip(cache, monkeypatch)
    (cache / "m" / "n.zip").write_text("x")
    import_map.download_release_zip("o/r", "t", cache_name="m", asset="m.zip")
    import_map.download_release_zip("o/r", "t", cache_name="m", asset="n.zip")
    assert len(asked) == 2
