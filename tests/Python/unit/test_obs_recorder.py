"""Which OBS a recording run uses: the open one when it is free, else one of its own.

No OBS is needed: the WebSocket and the second OBS are stand-ins.
"""
import os
import subprocess
import sys

import pytest

pytestmark = pytest.mark.skipif(os.name != "nt", reason="the recorder is Windows only")

CARLA_DIR = os.path.normpath(os.path.join(os.path.dirname(__file__), "..", "..", "..", "Carla"))
sys.path.insert(0, CARLA_DIR)
import obs_recorder as rec  # noqa: E402


class FakeSpare:
    started = []

    def __init__(self, port):
        self.port = port
        FakeSpare.started.append(port)

    def connect(self):
        return f"spare-ws:{self.port}"

    def close(self):
        pass


@pytest.fixture
def obs(monkeypatch):
    """The open OBS as a stand-in: `state["recording"]`, `state["up"]` say how it answers."""
    state = {"recording": False, "up": True}
    FakeSpare.started = []
    monkeypatch.delenv("OBS_WS_URL", raising=False)
    monkeypatch.setattr(rec, "OBS_PORT", 49880)  # clear of real runs' locks
    monkeypatch.setattr(rec, "SPARE_PORTS", range(49881, 49884))
    monkeypatch.setattr(rec, "SpareObs", FakeSpare)
    monkeypatch.setattr(rec, "clean_up_killed_run", lambda ws: None)

    class WS:
        def close(self):
            pass

    def connect(url, password):
        if not state["up"]:
            raise ConnectionRefusedError
        return WS()

    monkeypatch.setattr(rec, "obs_connect", connect)
    monkeypatch.setattr(rec, "call", lambda ws, rtype, data=None: {"outputActive": state["recording"]})
    return state


def test_free_open_obs_is_used(obs):
    ws, spare, lock = rec.open_obs()
    assert spare is None and FakeSpare.started == []
    lock.close()


def test_recording_open_obs_gets_a_second_one(obs):
    obs["recording"] = True
    ws, spare, lock = rec.open_obs()
    assert ws == "spare-ws:49881" and FakeSpare.started == [49881]
    lock.close()


def test_no_open_obs_gets_one_of_its_own(obs):
    obs["up"] = False
    ws, spare, lock = rec.open_obs()
    assert ws == "spare-ws:49881"
    lock.close()


def test_runs_side_by_side_each_get_their_own(obs):
    first = rec.open_obs()
    second = rec.open_obs()
    third = rec.open_obs()
    assert first[1] is None
    assert [second[0], third[0]] == ["spare-ws:49881", "spare-ws:49882"]
    for _, _, lock in (first, second, third):
        lock.close()


def test_port_lock_holds_against_another_process(tmp_path):
    lock = rec._claim(49889)
    assert lock is not None
    probe = ("import sys; sys.path.insert(0, sys.argv[1]); import obs_recorder as r; "
             "print(r._claim(49889) is None)")
    try:
        held = subprocess.run([sys.executable, "-c", probe, CARLA_DIR], capture_output=True, text=True, timeout=60)
        assert held.stdout.strip() == "True", held.stderr
    finally:
        lock.close()
    freed = subprocess.run([sys.executable, "-c", probe, CARLA_DIR], capture_output=True, text=True, timeout=60)
    assert freed.stdout.strip() == "False", freed.stderr


def test_ini_set_changes_only_the_named_keys(tmp_path):
    ini = tmp_path / "basic.ini"
    ini.write_text("[General]\nName=Untitled\n\n[Video]\nBaseCX=1920\nName=keep\n", encoding="utf-8")
    rec._ini_set(str(ini), "General", {"Name": "FIXS rec 4456"})
    assert rec._ini_get(str(ini), "General", "Name") == "FIXS rec 4456"
    assert rec._ini_get(str(ini), "Video", "Name") == "keep"
    assert rec._ini_get(str(ini), "Video", "BaseCX") == "1920"
