"""The FIXS window's form -> engine flags, with a canned backend (#78).

Skipped where PySide6 is not installed: the window is optional, and core.py (where
the flags and the process handling live) is covered by test_gui_core.py without
it. Runs headless (QT_QPA_PLATFORM=offscreen)."""
import os
import sys
import time

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__),
                                "..", "..", "..", "scripts", "cosim"))
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

# gui.app readies Qt's DLL search itself before it imports PySide6, so it is the
# module to probe for; importing PySide6 first would skip that step.
gui_app = pytest.importorskip("gui.app", exc_type=ImportError)
from PySide6.QtWidgets import QApplication  # noqa: E402


class FakeBackend:
    """What the engine's --list/--version/--doctor would answer."""

    def list_setups(self, **_):
        return {"last": "mlk_headless", "setups": [
            {"name": "mlk_headless", "app": "mlk", "map": "mlk_no_signal",
             "config": "C:/fixs/apps/mlk/scen_a.yaml", "sumo_gui": False,
             "summary": "mlk | mlk_no_signal | scen_a.yaml | headless"},
            {"name": "uga_default", "app": None, "map": "uga_untextured",
             "config": None, "sumo_gui": True, "summary": "no app | uga"}]}

    def list_apps(self, **_):
        return {"apps": [
            {"id": "mlk", "title": "MLK eco-driving", "maps": ["mlk", "mlk_uturn"],
             "launch": "eco.py --rate 0.0",
             "configs": [{"title": "L2 (default)", "source": "s/scen_a.yaml",
                          "staged": "C:/fixs/apps/mlk/scen_a.yaml", "engine": "py"},
                         {"title": "SUMO only", "source": "s/scen_b.yaml",
                          "staged": "C:/fixs/apps/mlk/scen_b.yaml", "engine": "py"}]},
            {"id": "roosevelt", "title": "Roosevelt", "maps": ["roosevelt"],
             "launch": None, "configs": []}]}

    def list_maps(self, **_):
        return {"maps": [{"name": "mlk_no_signal", "cooked": True, "cached": True,
                          "library": True, "title": "MLK"},
                         {"name": "uga_untextured", "cooked": True, "cached": False,
                          "library": True, "title": "UGA"}]}

    def version(self, **_):
        return {"fixs": "v0.10.0", "host": "box", "python": sys.executable,
                "modules": {"carla": True, "traci": False}, "carla_mode": "source",
                "carla_root": "C:/carla", "ue4_root": "C:/ue4",
                "carla_host": "localhost", "carla_port": 2000}

    def doctor(self, **_):
        return {"worst": "WARN", "host": "box", "role": "traffic", "rows": [
            {"section": "SUMO", "label": "binary", "status": "OK", "detail": "x"},
            {"section": "CARLA", "label": "reachable", "status": "WARN",
             "detail": "not up"}]}


@pytest.fixture(scope="module")
def qapp():
    return QApplication.instance() or QApplication([])


@pytest.fixture
def win(qapp):
    w = gui_app.MainWindow(engine="run_cosim.py", backend=FakeBackend())
    _pump(qapp, lambda: w.run_tab.setups and w.machine_tab.info.rowCount())
    yield w
    w.close()


def _pump(qapp, pred, timeout=10):
    end = time.time() + timeout
    while time.time() < end:
        qapp.processEvents()
        if pred():
            return
        time.sleep(0.02)
    raise AssertionError("the window never finished loading")


def _args(w):
    return w.run_tab.request().args()


def test_opens_on_the_last_setup_and_runs_it_as_is(win):
    rt = win.run_tab
    assert rt.setup_list.currentItem().data(0x0100) == "mlk_headless"
    assert rt.map_box.currentText() == "mlk_no_signal"
    assert rt.config_box.currentData() == "C:/fixs/apps/mlk/scen_a.yaml"
    assert not rt.sumo_gui.isChecked()
    assert _args(win) == ["--profile", "mlk_headless"]
    assert rt.command.text().endswith("--profile mlk_headless")


def test_only_what_changed_is_passed(win):
    rt = win.run_tab
    rt.map_box.setEditText("mlk_uturn")
    rt.sumo_gui.setChecked(True)
    rt.app_args.setText("--rate 0.3")
    assert _args(win) == ["--profile", "mlk_headless", "--map", "mlk_uturn",
                          "--sumo-gui", "--app-args", "--rate 0.3"]


def test_new_run_starts_on_the_apps_first_scenario(win):
    rt = win.run_tab
    rt.setup_list.setCurrentRow(0)                       # + New run
    assert rt.app_box.currentData() == "mlk"
    assert rt.map_box.currentText() == "mlk"
    assert _args(win) == ["--fresh", "--app", "mlk", "--map", "mlk",
                          "--config", "C:/fixs/apps/mlk/scen_a.yaml"]


def test_app_without_a_program_takes_no_arguments(win):
    rt = win.run_tab
    rt.setup_list.setCurrentRow(0)
    rt.app_box.setCurrentIndex(rt.app_box.findData("roosevelt"))
    assert not rt.app_args.isEnabled()
    assert rt.config_box.currentData() is None          # automatic
    assert _args(win) == ["--fresh", "--app", "roosevelt", "--map", "roosevelt"]


def test_scenario_tab_follows_the_run_form(win):
    assert win.scenario_tab.path == "C:/fixs/apps/mlk/scen_a.yaml"


def test_machine_tab_prefills_setup_and_shows_doctor(win, qapp):
    mt = win.machine_tab
    assert mt._mode() == "source"
    assert mt.ue4_root.text() == os.path.normpath("C:/ue4")
    mt.run_doctor()
    _pump(qapp, lambda: mt.tree.topLevelItemCount() == 2)
    assert "warnings" in mt.verdict.text()


def test_log_filter(qapp):
    log = gui_app.LogView()
    for line in ("[SUMO] up", "[TL] warm-up", "[cosim]   DEAD SUMO exited",
                 "[APP] ok"):
        log.add(line)
    log._flush()
    assert log.view.toPlainText().count("\n") == 3
    log.filter.setCurrentText("Problems only")
    assert log.view.toPlainText() == "[cosim]   DEAD SUMO exited"
    log.filter.setCurrentText("SUMO")
    assert log.view.toPlainText() == "[SUMO] up"
