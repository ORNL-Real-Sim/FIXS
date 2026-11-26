"""A source-build CARLA that crashes during LoadMap is reported by its exception, not
only by the map it was loading (#350).

The editor's log just stops after `LogLoad: LoadMap: <map>`; the exception is only in
Saved/Crashes/<id>/CrashContext.runtime-xml. Nothing is launched here.
"""
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "..", "Carla"))
import run_cosim  # noqa: E402

MAP = "/Game/mlk_notexture_uturn/Maps/mlk_notexture_uturn/mlk_notexture_uturn"
ERROR = "Unhandled Exception: EXCEPTION_ACCESS_VIOLATION reading address 0x000000050000011f"

CRASH_XML = """<?xml version="1.0" encoding="UTF-8"?>
<FGenericCrashContext>
<RuntimeProperties>
<CrashGUID>UE4CC-Windows-{guid}_0000</CrashGUID>
<CrashType>Crash</CrashType>
<ErrorMessage>{error}</ErrorMessage>
<ProcessId>{pid}</ProcessId>
<CommandLine> CarlaUE4.uproject {map} -game -carla-rpc-port=2000 -RenderOffScreen</CommandLine>
<CallStack></CallStack>
<PCallStack>
UE4Editor-Core         0x00007ff85ca40000 + 3b6527
UE4Editor-CoreUObject  0x00007ff88a540000 + 389fd5
UE4Editor-CoreUObject  0x00007ff88a540000 + 27d7d1
UE4Editor-Engine       0x00007ff88a540000 + 27c37c
</PCallStack>
</RuntimeProperties>
</FGenericCrashContext>
"""


def _source_build(tmp_path):
    saved = tmp_path / "Unreal" / "CarlaUE4" / "Saved"
    (saved / "Logs").mkdir(parents=True)
    (saved / "Logs" / "CarlaUE4.log").write_text(
        f"[2026.09.08-02.28.30:113][  0]LogLoad: LoadMap: {MAP}?Name=Player\n"
        "[2026.09.08-02.28.30:121][  0]LogUObjectHash: Compacting FUObjectHashTables\n",
        encoding="utf-8")
    return {"mode": "source", "carla_root": str(tmp_path)}, saved / "Crashes"


def _crash(crashes, guid, pid, mtime=None):
    d = crashes / f"UE4CC-Windows-{guid}_0000"
    d.mkdir(parents=True)
    xml = d / "CrashContext.runtime-xml"
    xml.write_text(CRASH_XML.format(guid=guid, error=ERROR, pid=pid, map=MAP), encoding="utf-8")
    if mtime is not None:
        os.utime(xml, (mtime, mtime))
    return xml


def test_the_exception_is_reported_not_just_the_map(tmp_path, capsys):
    cfg, crashes = _source_build(tmp_path)
    xml = _crash(crashes, "A1", pid=4242)
    run_cosim.report_unreal_crash(cfg, pid=4242, since=time.time() - 60)
    out = capsys.readouterr().out
    assert f"LoadMap: {MAP}" in out
    assert ERROR in out
    assert "UE4Editor-Core, UE4Editor-CoreUObject, UE4Editor-Engine" in out
    assert str(xml) in out


def test_another_process_crash_is_not_blamed(tmp_path, capsys):
    cfg, crashes = _source_build(tmp_path)
    _crash(crashes, "B1", pid=1111)
    run_cosim.report_unreal_crash(cfg, pid=4242, since=time.time() - 60)
    assert "Unreal crashed" not in capsys.readouterr().out


def test_an_older_crash_of_a_reused_pid_is_not_blamed(tmp_path, capsys):
    cfg, crashes = _source_build(tmp_path)
    _crash(crashes, "C1", pid=4242, mtime=time.time() - 86400)
    run_cosim.report_unreal_crash(cfg, pid=4242, since=time.time() - 60)
    assert "Unreal crashed" not in capsys.readouterr().out


def test_wait_for_port_reports_the_exited_servers_crash(tmp_path, capsys, monkeypatch):
    cfg, crashes = _source_build(tmp_path)
    _crash(crashes, "D1", pid=4242)

    class Exited:
        pid, returncode = 4242, 3221226505

        def poll(self):
            return self.returncode

    monkeypatch.setattr(run_cosim.socket.socket, "connect_ex", lambda self, addr: 1)
    assert run_cosim.wait_for_port("127.0.0.1", 1, timeout=5, proc=Exited(), cfg=cfg) is False
    out = capsys.readouterr().out
    assert "exited (3221226505)" in out and ERROR in out
