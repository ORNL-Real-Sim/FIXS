"""Port slots (#434 Phase 2): the remapped scenario yaml, claiming a slot, the
per-map lock and the per-run environment.

Slots are claimed against a real registry dir and real OS locks; the socket table
is stubbed so nothing listens on a real port.
"""
import os
import subprocess
import sys
import textwrap
import time

import pytest
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "..", "Carla"))
sys.path.insert(0, os.path.join(HERE, "..", "..", ".."))
import port_slots as ps  # noqa: E402
import run_cosim  # noqa: E402
import run_registry as rr  # noqa: E402
from CommonLib.VirEnv.DataLogger import perRunPath  # noqa: E402

RUNG2 = textwrap.dedent("""\
    # a comment that must survive
    SimulationSetup:
      SimulationEndTime: 29160
      WarmUpServePorts: [4430]
      TrafficSimulatorIP: "127.0.0.1"
      TrafficSimulatorPort: 1337    # SUMO
    SumoSetup:
      SpeedMode: 31
      SumoConfigFile: sumo/x.sumocfg
    ApplicationSetup:
      EnableApplicationLayer: true
      VehicleSubscription:
      - type: "ego"
        attribute: {all: ["true"], radius: [0]}
        ip: ["127.0.0.1"]
        port: [4430]
      - type: "ego"
        attribute: {all: ["true"]}
        ip: ["127.0.0.1"]
        port: [4440]
      SignalSubscription:
      - type: "intersection"
        attribute: {all: ["true"]}
        ip: ["127.0.0.1"]
        port: [4430, 4431]
    XilSetup:
      EnableXil: false
      VehicleSubscription:
      - type: "ego"
        attribute: {id: ["ego"]}
        ip: ["127.0.0.1"]
        port: [4420]
    CarlaSetup:
        # a comment indented deeper than the keys
      EnablePythonBackend: true
      CarlaServerIP: 127.0.0.1
      CarlaServerPort: 2000
      CarlaClientIP: 127.0.0.1
      CarlaClientPort: 4440
    ViewSetup:
      Views:
      - Name: chase
        Simulator: carla
        Record: 'videos/chase.mp4'
    DataLogSetup:
      EnableDataLog: true
      DataLogPath: '_datalog/l2_ego.csv'
    """)


@pytest.fixture
def rung2(tmp_path):
    p = tmp_path / "src" / "config_rung2.yaml"
    p.parent.mkdir()
    p.write_text(RUNG2, encoding="utf-8")
    return str(p)


@pytest.fixture
def registry(tmp_path, monkeypatch):
    monkeypatch.setenv(rr.RUNS_DIR_ENV, str(tmp_path / "runs"))
    monkeypatch.setattr(run_cosim, "_RUN", None)
    monkeypatch.setattr(run_cosim, "_SLOT", {"k": 0, "held": None, "keep_carla": False})
    for k in ("FIXS_RUN_DIR", "FIXS_RUN_ID", "FIXS_SLOT"):
        monkeypatch.delenv(k, raising=False)
    root = str(tmp_path / "runs")
    os.makedirs(root)
    return root


def _args(**kw):
    import argparse
    base = dict(serve=False, carla_only=False, peer=None, standalone_bridge=False,
                slot=None, no_launch=False, carla_port=None)
    base.update(kw)
    return argparse.Namespace(**base)


# --------------------------------------------------------------------------- #
# the yaml
# --------------------------------------------------------------------------- #
def test_discover_finds_every_role(rung2):
    f = ps.discover(rung2)
    assert (f["traci"], f["bridge"], f["carla"], f["tm"]) == (1337, 4440, 2000, 8000)
    assert f["app"] == [4430, 4431] and f["xil"] == [4420] and not f["xil_udp"]


def test_slot_zero_is_the_yaml_as_written(rung2):
    assert ps.port_map(ps.discover(rung2), 0) == {}
    ports = dict((l, p) for l, p in ps.bound_ports(ps.discover(rung2), 0)[:1])
    assert ports == {"SUMO TraCI": 1337}


def test_slot_k_moves_every_port_into_its_block(rung2, tmp_path):
    dst = str(tmp_path / "run" / "config_rung2.yaml")
    m = ps.write_slot_yaml(rung2, dst, 3, "eco-1")
    assert m == {1337: 20300, 4440: 20340, 8000: 20360, 2000: 20350,
                 4430: 20330, 4431: 20331, 4420: 20320}
    doc = yaml.safe_load(open(dst, encoding="utf-8"))
    assert doc["SimulationSetup"]["TrafficSimulatorPort"] == 20300
    assert doc["SimulationSetup"]["WarmUpServePorts"] == [20330]
    subs = doc["ApplicationSetup"]["VehicleSubscription"]
    assert [s["port"] for s in subs] == [[20330], [20340]]
    assert doc["ApplicationSetup"]["SignalSubscription"][0]["port"] == [20330, 20331]
    assert doc["XilSetup"]["VehicleSubscription"][0]["port"] == [20320]
    cs = doc["CarlaSetup"]
    assert (cs["CarlaServerPort"], cs["CarlaClientPort"], cs["TrafficManagerPort"]) == \
        (20350, 20340, 20360)
    assert doc["DataLogSetup"]["DataLogPath"] == "_datalog/eco-1/l2_ego.csv"
    assert doc["ViewSetup"]["Views"][0]["Record"] == "videos/eco-1/chase.mp4"
    sumocfg = doc["SumoSetup"]["SumoConfigFile"]
    assert os.path.isabs(sumocfg) and sumocfg.endswith("src/sumo/x.sumocfg")
    text = open(dst, encoding="utf-8").read()
    assert "# a comment that must survive" in text and "# SUMO" in text
    assert open(rung2, encoding="utf-8").read() == RUNG2      # the source is untouched


def test_keep_carla_leaves_the_rpc_port(rung2, tmp_path):
    dst = str(tmp_path / "out.yaml")
    ps.write_slot_yaml(rung2, dst, 1, "r", keep_carla=True)
    doc = yaml.safe_load(open(dst, encoding="utf-8"))
    assert doc["CarlaSetup"]["CarlaServerPort"] == 2000
    assert doc["CarlaSetup"]["CarlaClientPort"] == 20140


def test_a_udp_dyno_keeps_its_port(tmp_path):
    src = tmp_path / "udp.yaml"
    src.write_text(RUNG2.replace("  EnableXil: false\n",
                                 "  EnableXil: true\n  Transport: udp\n")
                   .replace("port: [4420]", "port: [5010]"), encoding="utf-8")
    dst = str(tmp_path / "out.yaml")
    m = ps.write_slot_yaml(str(src), dst, 1, "r")
    assert 5010 not in m
    doc = yaml.safe_load(open(dst, encoding="utf-8"))
    assert doc["XilSetup"]["VehicleSubscription"][0]["port"] == [5010]


def test_a_traffic_only_yaml_has_no_carla_ports(tmp_path):
    src = tmp_path / "sumo.yaml"
    src.write_text(RUNG2.split("CarlaSetup:")[0].replace("port: [4440]", "port: [4430]"),
                   encoding="utf-8")
    f = ps.discover(str(src))
    assert f["bridge"] is None and f["carla"] is None and f["tm"] is None
    m = ps.write_slot_yaml(str(src), str(tmp_path / "o.yaml"), 2, "r")
    assert m[1337] == 20200 and m[4430] == 20230
    assert "TrafficManagerPort" not in open(tmp_path / "o.yaml").read()


def test_a_port_list_the_rewrite_cannot_read_is_refused(tmp_path):
    src = tmp_path / "multi.yaml"
    src.write_text(RUNG2.replace("port: [4430, 4431]", "port:\n    - 4430\n    - 4431"),
                   encoding="utf-8")
    with pytest.raises(ps.SlotError):
        ps.write_slot_yaml(str(src), str(tmp_path / "o.yaml"), 1, "r")
    assert not (tmp_path / "o.yaml").exists()


def test_per_run_path():
    assert ps.per_run_path("_datalog/a.csv", "id") == "_datalog/id/a.csv"
    assert ps.per_run_path("a.mp4", "id") == "id/a.mp4"
    assert ps.per_run_path(os.path.abspath("a.csv"), "id") == os.path.abspath("a.csv")


def test_python_writers_follow_the_slot(monkeypatch):
    monkeypatch.setenv("FIXS_RUN_ID", "eco-1")
    monkeypatch.setenv("FIXS_SLOT", "0")
    assert perRunPath("_datalog/agent_embedded.csv") == "_datalog/agent_embedded.csv"
    monkeypatch.setenv("FIXS_SLOT", "2")
    assert perRunPath("_datalog/agent_embedded.csv") == os.path.join("_datalog", "eco-1",
                                                                     "agent_embedded.csv")


# --------------------------------------------------------------------------- #
# claiming a slot
# --------------------------------------------------------------------------- #
def test_claims_are_exclusive_while_held(registry):
    k1, h1, _ = rr.claim_slot(registry, lambda k: None, range(0, 5))
    k2, h2, skipped = rr.claim_slot(registry, lambda k: None, range(0, 5))
    assert (k1, k2) == (0, 1) and skipped == [(0, "claimed by another run")]
    h1.release()
    k3, h3, _ = rr.claim_slot(registry, lambda k: None, range(0, 5))
    assert k3 == 0
    h2.release()
    h3.release()


def test_a_busy_slot_is_skipped(registry):
    k, held, skipped = rr.claim_slot(registry, lambda k: "port held" if k < 2 else None,
                                     range(0, 5))
    assert k == 2 and [s for s, _w in skipped] == [0, 1]
    held.release()
    k, held, _ = rr.claim_slot(registry, lambda k: None, [0])
    assert k == 0                                  # a skipped slot was not left locked
    held.release()


_CLAIMER = textwrap.dedent("""\
    import sys, time
    sys.path.insert(0, sys.argv[1])
    import run_registry as rr
    time.sleep(max(0.0, float(sys.argv[3]) - time.time()))
    k, held, _ = rr.claim_slot(sys.argv[2], lambda k: None, range(0, 10))
    print(k, flush=True)
    time.sleep(3)
    """)


def test_simultaneous_claims_never_share_a_slot(registry, tmp_path):
    script = tmp_path / "claim.py"
    script.write_text(_CLAIMER, encoding="utf-8")
    go = str(time.time() + 1.5)
    carla_dir = os.path.join(HERE, "..", "..", "..", "Carla")
    procs = [subprocess.Popen([sys.executable, str(script), carla_dir, registry, go],
                              stdout=subprocess.PIPE, text=True) for _ in range(4)]
    got = [int(p.communicate(timeout=60)[0].split()[0]) for p in procs]
    assert sorted(got) == [0, 1, 2, 3]


def test_a_dead_claimers_slot_is_free_again(registry, tmp_path):
    script = tmp_path / "claim.py"
    script.write_text(_CLAIMER, encoding="utf-8")
    carla_dir = os.path.join(HERE, "..", "..", "..", "Carla")
    p = subprocess.Popen([sys.executable, str(script), carla_dir, registry, "0"],
                         stdout=subprocess.PIPE, text=True)
    assert p.stdout.readline().strip() == "0"
    p.kill()
    p.wait(timeout=10)
    k, held, _ = rr.claim_slot(registry, lambda k: None, range(0, 3))
    assert k == 0
    held.release()


def test_claim_slot_takes_the_next_slot_past_a_foreign_holder(registry, rung2,
                                                              monkeypatch, capsys):
    foreign = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(60)"])
    kills = []
    monkeypatch.setattr(run_cosim, "_kill_pid_tree", kills.append)
    monkeypatch.setattr(run_cosim, "_listeners", lambda: {1337: foreign.pid})
    try:
        run_cosim._begin_run({"id": "eco"}, rung2, None)
        run_cosim._claim_slot(_args(), rung2)
        assert run_cosim._SLOT["k"] == 1
        out = capsys.readouterr().out
        assert "port slot 0 is busy" in out and str(foreign.pid) in out
        assert "port slot 1:" in out and "SUMO TraCI 20100" in out
        lease = run_cosim._RUN.lease
        assert lease["slot"] == 1 and lease["ports"]["traci"] == 20100
        assert os.environ["FIXS_SLOT"] == "1"
        assert os.environ["FIXS_RUN_DIR"] == run_cosim._RUN.dir
        assert foreign.poll() is None and kills == []
        dst = run_cosim._slot_yaml(rung2)
        assert os.path.dirname(dst) == run_cosim._RUN.dir
        assert os.path.basename(dst) == os.path.basename(rung2)
        assert ps.discover(dst)["traci"] == 20100
    finally:
        if run_cosim._SLOT["held"]:
            run_cosim._SLOT["held"].release()
        run_cosim._RUN.close()
        foreign.kill()
        foreign.wait(timeout=10)


def test_an_explicit_busy_slot_is_refused_not_killed(registry, rung2, monkeypatch):
    foreign = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(60)"])
    kills = []
    monkeypatch.setattr(run_cosim, "_kill_pid_tree", kills.append)
    monkeypatch.setattr(run_cosim, "_listeners", lambda: {1337: foreign.pid})
    try:
        run_cosim._begin_run({"id": "eco"}, rung2, None)
        with pytest.raises(SystemExit) as exc:
            run_cosim._claim_slot(_args(slot=0), rung2)
        msg = str(exc.value.code)
        assert "slot 0 is not free" in msg and str(foreign.pid) in msg
        assert "will not stop it" in msg
        assert foreign.poll() is None and kills == []
    finally:
        run_cosim._RUN.close()
        foreign.kill()
        foreign.wait(timeout=10)


def test_a_single_run_stays_on_slot_zero_and_reads_the_users_yaml(registry, rung2,
                                                                  monkeypatch, capsys):
    monkeypatch.setattr(run_cosim, "_listeners", lambda: {})
    try:
        run_cosim._begin_run({"id": "eco"}, rung2, None)
        run_cosim._claim_slot(_args(), rung2)
        assert run_cosim._SLOT["k"] == 0
        assert run_cosim._slot_yaml(rung2) == rung2
        assert "slot" not in capsys.readouterr().out
    finally:
        run_cosim._SLOT["held"].release()
        run_cosim._RUN.close()


def test_a_run_of_an_older_fixs_blocks_the_slot_its_yaml_uses(registry, rung2, monkeypatch):
    monkeypatch.setattr(run_cosim, "_listeners", lambda: {})
    old = rr.Run.start(app_id="eco", config=rung2)    # no "slot" in its lease
    try:
        run_cosim._begin_run({"id": "eco"}, rung2, None)
        run_cosim._claim_slot(_args(), rung2)
        assert run_cosim._SLOT["k"] == 1
    finally:
        run_cosim._SLOT["held"].release()
        run_cosim._RUN.close()
        old.close()


def test_slot_holder_names_the_run(registry):
    run = rr.Run.start(app_id="eco")
    try:
        run.update(slot=4)
        assert rr.slot_holder(4).id == run.id and rr.slot_holder(3) is None
    finally:
        run.close()


# --------------------------------------------------------------------------- #
# the per-map lock
# --------------------------------------------------------------------------- #
_MAP_HOLDER = textwrap.dedent("""\
    import sys, time
    sys.path.insert(0, sys.argv[1])
    import run_registry as rr
    held = rr.map_lock(sys.argv[2], "mlk_notexture_uturn", log=None)
    print("held", flush=True)
    time.sleep(float(sys.argv[3]))
    """)


def test_a_second_run_waits_for_the_map(registry, tmp_path):
    script = tmp_path / "hold.py"
    script.write_text(_MAP_HOLDER, encoding="utf-8")
    carla_dir = os.path.join(HERE, "..", "..", "..", "Carla")
    p = subprocess.Popen([sys.executable, str(script), carla_dir, registry, "2.5"],
                         stdout=subprocess.PIPE, text=True)
    assert p.stdout.readline().strip() == "held"
    said = []
    t0 = time.time()
    held = rr.map_lock(registry, "mlk_notexture_uturn", log=said.append, poll=0.1)
    waited = time.time() - t0
    held.release()
    p.wait(timeout=10)
    assert waited > 1.0
    assert "another run is preparing map 'mlk_notexture_uturn'" in said[0]


def test_different_maps_do_not_wait(registry):
    a = rr.map_lock(registry, "a", log=None)
    t0 = time.time()
    b = rr.map_lock(registry, "b", log=None)
    assert time.time() - t0 < 1.0
    a.release()
    b.release()


# --------------------------------------------------------------------------- #
# run_profiles.json
# --------------------------------------------------------------------------- #
def test_profile_writes_are_whole_and_leave_no_temp(tmp_path, monkeypatch):
    import run_profile
    path = tmp_path / "run_profiles.json"
    monkeypatch.setattr(run_profile, "profiles_path", lambda: str(path))
    run_profile.save("a", {"app": "eco"})
    run_profile.save_partial("b", {"app": "eco"}, "map import")
    doc = run_profile.load_doc()
    assert set(doc["setups"]) == {"a", "b"} and doc["last"] == "b"
    assert sorted(os.listdir(tmp_path)) == ["run_profiles.json", "run_profiles.json.lock"]
