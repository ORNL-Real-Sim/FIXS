"""PlotSetup: parsing, the rolling timeline, and fixs.plot.

    python -m pytest tests/Python/unit/test_plot_view.py
"""
from __future__ import annotations

import math

import pytest
import yaml

from CommonLib.ConfigHelper import ConfigHelper
from CommonLib.VirEnv.PlotView import PlotWindows, Timeline, labelOf, unitOf
from CommonLib.fixs._plot import Plot

BASE = {"SimulationSetup": {"SelectedTrafficSimulator": "SUMO",
                            "VehicleMessageField": ["id", "speed", "speedDesired", "steerAngleDesired"]}}


def parse(tmp_path, plots):
    doc = dict(BASE, PlotSetup={"Plots": plots})
    p = tmp_path / "c.yaml"
    p.write_text(yaml.safe_dump(doc), encoding="utf-8")
    cfg = ConfigHelper()
    cfg.getConfig(str(p))
    return cfg.Plot_setup["Plots"]


def test_no_plotsetup_means_no_plots(tmp_path):
    p = tmp_path / "c.yaml"
    p.write_text(yaml.safe_dump(BASE), encoding="utf-8")
    cfg = ConfigHelper()
    cfg.getConfig(str(p))
    assert cfg.Plot_setup["Plots"] == []


def test_plot_defaults_and_series(tmp_path):
    (plot,) = parse(tmp_path, [{"Name": "ego", "Panels": [["speedDesired", "speed"],
                                                           ["steerAngleDesired", "steerAngle"],
                                                           ["app.laneTarget", "app.lateralOffset"]]}])
    assert plot["Target"] == "ego" and plot["Size"] == [760, 480] and plot["Window"] == 60.0
    assert plot["Panels"][2] == {"Series": ["app.laneTarget", "app.lateralOffset"], "Title": "", "Unit": "",
                                 "Labels": []}


def test_a_panel_can_name_itself(tmp_path):
    (plot,) = parse(tmp_path, [{"Panels": [{"Series": ["speedDesired", "speed"], "Title": "Speed",
                                            "Labels": ["command", "actual"]}]}])
    assert plot["Panels"] == [{"Series": ["speedDesired", "speed"], "Title": "Speed", "Unit": "",
                               "Labels": ["command", "actual"]}]


@pytest.mark.parametrize("plots", [
    [{"Panels": [["accelerationDesired"]]}],         # not in VehicleMessageField
    [{"Panels": []}],
    [{"Panels": ["speed"]}],
    [{"Panels": [{"Title": "no series"}]}],
    [{"Panels": [{"Series": ["speed"], "Labels": ["one", "too many"]}]}],
    [{"Name": "a", "Panels": [["speed"]]}, {"Name": "a", "Panels": [["speed"]]}],
])
def test_bad_plots_are_refused(tmp_path, plots):
    with pytest.raises(SystemExit):
        parse(tmp_path, plots)


def test_disabled_plot_is_dropped(tmp_path):
    assert parse(tmp_path, [{"Enable": False, "Panels": [["speed"]]}]) == []


def test_timeline_keeps_the_window_and_converts_steer_to_degrees():
    tl = Timeline({"Name": "p", "Target": "ego", "Size": [320, 200], "Window": 10.0,
                   "Panels": [["speedDesired", "speed"], ["steerAngleDesired"]]})
    for k in range(300):
        t = k * 0.1
        tl.add(t, {"speed": 5.0, "speedDesired": None, "steerAngleDesired": 0.1}, ["LC"] if k == 50 else ())
    d = tl.since(0)
    assert tl.series == ["speedDesired", "speed", "steerAngleDesired"]
    assert d["rows"][0][0] >= d["now"] - 10.0 - 1e-9          # times are seconds since the first sample
    assert all(r[1] is None for r in d["rows"])
    assert d["rows"][-1][3] == pytest.approx(math.degrees(0.1))
    assert not d["marks"]                                        # the mark at t=5 s has scrolled out
    assert tl.since(d["seq"])["rows"] == []


def test_units_and_labels():
    assert unitOf("steerAngle")[0] == "deg" and unitOf("app.speedRef") == ("m/s", 1.0)
    assert labelOf("app.lateralOffset") == "lateral offset" and labelOf("steerAngleDesired") == "steer angle desired"


def test_panels_name_themselves_or_take_the_config():
    tl = Timeline({"Name": "p", "Target": "ego", "Size": [480, 300], "Window": 10.0,
                   "Panels": [["speedDesired", "speed"],
                              {"Series": ["app.laneTarget", "app.lateralOffset"], "Title": "Lateral offset",
                               "Unit": "m", "Labels": ["command", "actual"]}]})
    a, b = tl.spec()["panels"]
    assert (a["title"], a["unit"], a["labels"]) == ("Speed", "m/s", ["speed desired", "speed"])
    assert (b["title"], b["unit"], b["labels"]) == ("Lateral offset", "m", ["command", "actual"])


def test_the_server_answers_the_page():
    import json
    import urllib.request
    pw = PlotWindows([{"Name": "ego", "Target": "ego", "Size": [480, 300], "Window": 10.0,
                       "Panels": [["speedDesired", "speed"]]}], open_windows=False)
    try:
        for k in range(20):
            pw.add(29000 + k * 0.1, {"ego": {"speed": 5.0, "speedDesired": 6.0}}, ["LC"] if k == 10 else ())
        get = lambda path: urllib.request.urlopen(pw.urls["ego"].replace("/plot/ego", path), timeout=5).read()
        assert b"FIXS plot: ego" in get("/plot/ego")
        assert json.loads(get("/spec/ego"))["panels"][0]["series"] == ["speedDesired", "speed"]
        d = json.loads(get("/data/ego?since=0"))
        assert d["rows"][0][1:] == [6.0, 5.0] and d["marks"] == [[pytest.approx(1.0), "LC"]]
        assert json.loads(get("/clock"))["sim"] == pytest.approx(29001.9)
    finally:
        pw.close()


def test_plot_api_keeps_values_and_hands_marks_once():
    p = Plot()
    p.put("lateralOffset", 1.5)
    p.mark("LC right")
    assert p.take() == ({"lateralOffset": 1.5}, ["LC right"])
    assert p.take() == ({"lateralOffset": 1.5}, [])
