"""`fixs.launch` -- this run's launch facts, so an application reads no FIXS_ env.

run_cosim sets five variables for every app it starts. Two were already hidden
(connect() reads FIXS_CONFIG_YAML, fixs.sumo writes FIXS_HANDOFF) and the rest
leaked into application code, so an app mixed both styles and re-spelled FIXS's
own variable names. These cover the replacement.

The load-bearing one is test_answers_before_connect: `sumocfg` is wanted BEFORE
connect(), while the app is building the scenario it will report, and that is
exactly where `fixs.sim` cannot be used -- test_sim_still_refuses_before_a_tick is
the control showing why this is a separate namespace rather than a field on that
one.
"""
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "..", "CommonLib"))
import fixs  # noqa: E402


VARS = ("FIXS_HANDOFF", "FIXS_CONFIG_YAML", "FIXS_SUMOCFG", "FIXS_SUMO_ONLY")


@pytest.fixture(autouse=True)
def clean_env(monkeypatch):
    for name in VARS:
        monkeypatch.delenv(name, raising=False)


def test_unsupervised_is_the_honest_default():
    assert fixs.launch.supervised is False
    assert fixs.launch.configPath is None
    assert fixs.launch.sumocfg is None
    # None, not False: nothing started us, so there is no run to have CARLA in.
    assert fixs.launch.carla is None


def test_reads_what_run_cosim_sets(monkeypatch):
    monkeypatch.setenv("FIXS_HANDOFF", r"C:\RealSim_tmp\handoff_demo_1.json")
    monkeypatch.setenv("FIXS_CONFIG_YAML", r"C:\cfg\config_Sumo_ecoDriving.yaml")
    monkeypatch.setenv("FIXS_SUMOCFG", r"C:\maps\mlk\sumo\mlk.sumocfg")
    assert fixs.launch.supervised is True
    assert fixs.launch.configPath.endswith("config_Sumo_ecoDriving.yaml")
    assert fixs.launch.sumocfg.endswith("mlk.sumocfg")
    assert fixs.launch.carla is True


def test_sumo_only_says_there_is_no_carla(monkeypatch):
    monkeypatch.setenv("FIXS_HANDOFF", "h.json")
    monkeypatch.setenv("FIXS_SUMO_ONLY", "1")
    assert fixs.launch.carla is False


def test_not_latched_at_import(monkeypatch):
    """Each access re-reads, so a test (or a re-exec) is believed."""
    assert fixs.launch.sumocfg is None
    monkeypatch.setenv("FIXS_SUMOCFG", "a.sumocfg")
    assert fixs.launch.sumocfg == "a.sumocfg"
    monkeypatch.setenv("FIXS_SUMOCFG", "b.sumocfg")
    assert fixs.launch.sumocfg == "b.sumocfg"


def test_empty_string_is_unset(monkeypatch):
    """run_cosim omits a variable rather than passing it empty; treat both alike
    so `fixs.launch.sumocfg or MY_DEFAULT` cannot pick up ''."""
    monkeypatch.setenv("FIXS_SUMOCFG", "")
    monkeypatch.setenv("FIXS_CONFIG_YAML", "")
    assert fixs.launch.sumocfg is None
    assert fixs.launch.configPath is None


def test_answers_before_connect(monkeypatch):
    """The point of the namespace: usable while no stack exists.

    An application builds its scenario and reports it BEFORE connecting, so
    anything it needs then cannot live behind a connection guard.
    """
    monkeypatch.setenv("FIXS_HANDOFF", "h.json")
    monkeypatch.setenv("FIXS_SUMOCFG", r"C:\maps\mlk\sumo\mlk.sumocfg")
    assert fixs.launch.sumocfg.endswith("mlk.sumocfg")   # no connect() anywhere


def test_sim_still_refuses_before_a_tick():
    """Control for the above: this is why launch is not a field on fixs.sim."""
    with pytest.raises((fixs.ProtocolError, fixs.NotConnected)):
        fixs.sim.time


def test_repr_says_which_state_it_is_in(monkeypatch):
    assert "not started by run_cosim" in repr(fixs.launch)
    monkeypatch.setenv("FIXS_HANDOFF", "h.json")
    monkeypatch.setenv("FIXS_SUMOCFG", "x.sumocfg")
    assert "x.sumocfg" in repr(fixs.launch)


def test_exported():
    assert "launch" in fixs.__all__
