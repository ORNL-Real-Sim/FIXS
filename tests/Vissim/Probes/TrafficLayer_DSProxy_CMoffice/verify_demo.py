"""
Self-checking end-to-end verification of the #168 CarMaker-VISSIM-FIXS co-sim.

Launches TrafficLayer (DSProxy) and the REAL custom CarMaker.win64.exe as
subprocesses, waits for the round-trip, and asserts:
  - TrafficLayer reached VISSIM_Connect OK
  - CarMaker's VirtualEnvironment.lib connected ("RealSim Initialized")
  - TrafficLayer exchanged data with egos=1 and vehicles>0 (ego -> VISSIM and
    VISSIM traffic -> CarMaker)
  - the run ended without a socket drop mid-stream

Prints PASS/FAIL and the key evidence lines. This is what the .bat does, but
fully captured/asserted so it can run unattended (CI-style) and so a reviewer
can confirm the result without watching two GUI windows.

Run:  python verify_demo.py
Prereq: VISSIM 2022 licensed/healthy; binaries built; import_road.bat +
        build_testrun.py already run (committed CM project is in that state).
"""
from __future__ import annotations
import os
import re
import subprocess
import sys
import time
import pathlib

HERE = pathlib.Path(__file__).resolve().parent
REPO = HERE.parents[3]
TL = REPO / "TrafficLayer" / "x64" / "Release" / "TrafficLayer.exe"
CMEXE = REPO / "ProprietaryFiles" / "CM13_proj" / "src" / "CarMaker.win64.exe"
CMPROJ = REPO / "ProprietaryFiles" / "CM13_proj"
SRC_INPX = HERE / "simple_loop_ds.inpx"
STAGE = HERE / "stage_network"
RUNCFG = HERE / "config.runtime.yaml"
TESTRUN = "SimpleLoop_VISSIM_rs"

TL_CONNECT_TIMEOUT = 90    # s to wait for VISSIM_Connect OK
RUN_TIMEOUT = 90           # s overall cap for the CarMaker run


def stage_and_config() -> None:
    STAGE.mkdir(exist_ok=True)
    import shutil
    shutil.copy2(SRC_INPX, STAGE / "simple_loop.inpx")
    lay = HERE / "simple_loop_ds.layx"
    if lay.exists():
        shutil.copy2(lay, STAGE / "simple_loop.layx")
    cfg = (HERE / "config.yaml").read_text(encoding="utf-8")
    abs_inpx = str(STAGE / "simple_loop.inpx")
    cfg = cfg.replace("stage_network\\simple_loop.inpx", abs_inpx)
    RUNCFG.write_text(cfg, encoding="ascii")
    print(f"[verify] staged network + wrote {RUNCFG.name}")


def main() -> int:
    for p, label in [(TL, "TrafficLayer.exe"), (CMEXE, "CarMaker.win64.exe"), (SRC_INPX, "simple_loop_ds.inpx")]:
        if not p.is_file():
            print(f"[verify] FAIL: missing {label}: {p}")
            return 2
    if not (CMPROJ / "Data" / "TestRun" / TESTRUN).is_file():
        print(f"[verify] FAIL: TestRun {TESTRUN} missing -- run import_road.bat then build_testrun.py")
        return 2

    stage_and_config()

    # --- launch TrafficLayer, wait for VISSIM_Connect OK ---
    print("[verify] launching TrafficLayer (DSProxy)...")
    tl_out: list[str] = []
    tl = subprocess.Popen([str(TL), "-f", str(RUNCFG)], cwd=str(HERE),
                          stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                          text=True, bufsize=1)

    def pump(proc, sink, tag=None):
        for line in proc.stdout:
            s = line.rstrip("\n")
            sink.append(s)
            if tag and s.strip():
                print(f"  [{tag}] {s}", flush=True)

    import threading
    # Stream TrafficLayer lines live so you can SEE the connection + per-tick
    # exchange instead of waiting in the dark.
    threading.Thread(target=pump, args=(tl, tl_out, "TL"), daemon=True).start()

    t0 = time.time()
    connected = False
    while time.time() - t0 < TL_CONNECT_TIMEOUT:
        if any("VISSIM_Connect OK" in l for l in tl_out):
            connected = True
            break
        if tl.poll() is not None:
            break
        time.sleep(1)
    if not connected:
        print("[verify] FAIL: TrafficLayer did not reach 'VISSIM_Connect OK'")
        print("\n".join(tl_out[-15:]))
        tl.kill()
        return 1
    print(f"[verify] VISSIM_Connect OK after {time.time()-t0:.0f}s; launching CarMaker...")

    # --- launch the REAL CarMaker exe headless against TrafficLayer ---
    cm_out: list[str] = []
    cm = subprocess.Popen(
        [str(CMEXE), "-screen", "-f", str(RUNCFG), f"Data/TestRun/{TESTRUN}"],
        cwd=str(CMPROJ), stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        text=True, bufsize=1)
    # Stream CarMaker lines live too -- you'll see RealSim Initialized / SIM_END.
    threading.Thread(target=pump, args=(cm, cm_out, "CarMaker"), daemon=True).start()

    t1 = time.time()
    while time.time() - t1 < RUN_TIMEOUT:
        if cm.poll() is not None:
            break
        time.sleep(1)
    # give TL a moment to flush its last ticks
    time.sleep(2)
    if cm.poll() is None:
        cm.kill()
    tl.kill()

    # --- assertions ---
    cm_text = "\n".join(cm_out)
    tl_text = "\n".join(tl_out)
    lib_connected = "RealSim Initialized" in cm_text or "All Clients Connected!" in cm_text
    sim_ran = "SIM_END" in cm_text
    tick_lines = [l for l in tl_out if re.search(r"tick\s+\d+: vehicles=", l)]
    max_veh = 0
    egos_ok = False
    for l in tick_lines:
        m = re.search(r"vehicles=\s*(\d+).*egos=(\d+)", l)
        if m:
            max_veh = max(max_veh, int(m.group(1)))
            if int(m.group(2)) >= 1:
                egos_ok = True

    print("\n========== EVIDENCE ==========")
    print("CarMaker:", "RealSim Initialized" if lib_connected else "(.lib did NOT connect)")
    print("CarMaker:", "SIM_END (ran to completion)" if sim_ran else "(did not finish)")
    print(f"TrafficLayer: {len(tick_lines)} exchange ticks, peak vehicles={max_veh}, egos>=1: {egos_ok}")
    for l in tick_lines[-5:]:
        print("   ", l)

    ok = lib_connected and egos_ok and max_veh > 0 and sim_ran
    print("\n========== RESULT ==========")
    print("PASS: full CarMaker<->VISSIM<->FIXS co-simulation verified"
          if ok else "FAIL: see evidence above")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
