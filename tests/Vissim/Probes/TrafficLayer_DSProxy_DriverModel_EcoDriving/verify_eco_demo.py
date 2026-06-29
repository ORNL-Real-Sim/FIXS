"""
Self-checking end-to-end verification of the #173 eco-driving co-sim: CarMaker
<-> VISSIM <-> FIXS with DSProxy AND the FIXS DriverModel co-loaded (Stage C).

The headless analogue of Demo 2's verify_signal_demo.py, extended with the
DriverModel-relay half:
  - stages the signalized DS network AND hooks the FIXS DriverModel on Car
    type 100 (setup_eco.stage_network + patch_drivermodel_hook + write config),
  - launches TrafficLayer (DSProxy + DriverModel relay) -> spawns VISSIM 2022,
  - launches cav_controller.py (connects on the CAV port 1338),
  - launches the headless CarMaker exe (-f config -s signal table),
  - asserts the full round-trip AND the co-load:
      * TrafficLayer 'VISSIM_Connect OK', 'DriverModel connected',
        'CAV-controller listener bound on port 1338', 'CAV controller connected'
      * CarMaker '.lib connected' + 'SIM_END'
      * TrafficLayer per-tick vehicles>0 AND signals>0 AND egos>=1
      * the relay DID NOT deadlock (ticks advanced past tick 1 -- the historical
        SubEgoOnly hang) and the controller relayed CAV cmds.

Run:  python verify_eco_demo.py   (run_eco_demo_headless.bat does the asset build first)
Prereq: VISSIM 2022 licensed/healthy; TrafficLayer.exe + DriverModel_RealSim.dll +
        the headless CM exe built; Demo 2 assets built (rd5 + signal table + TestRun).
"""
from __future__ import annotations
import os, re, subprocess, sys, time, pathlib, threading

import setup_eco  # local: stage_network + patch_drivermodel_hook + write_runtime_config

HERE = pathlib.Path(__file__).resolve().parent
REPO = HERE.parents[3]
TL = REPO / "TrafficLayer" / "x64" / "Release" / "TrafficLayer.exe"
DM = REPO / "ProprietaryFiles" / "VISSIMserver" / "x64" / "Release" / "DriverModel_RealSim.dll"
CMEXE = REPO / "ProprietaryFiles" / "CM13_proj" / "src" / "CarMaker_headless.win64.exe"
CMPROJ = REPO / "ProprietaryFiles" / "CM13_proj"
SRC_INPX = REPO / "tests" / "Vissim" / "networks" / "simple_traffic_light" / "simple_traffic_light.inpx"
RUNCFG = HERE / "config.runtime.yaml"
TESTRUN = "SimpleTrafficLight_Cosim"
SIGNAL_TABLE = CMPROJ / "Data" / "Road" / "simple_traffic_light_RSsignalTable.csv"
PYEXE = sys.executable

TL_CONNECT_TIMEOUT = 120
RUN_TIMEOUT = 600


def ensure_headless_exe() -> None:
    """Reuse the #168 headless harness build (same User.c/.lib). Build if missing."""
    builder = HERE.parent / "TrafficLayer_DSProxy_CMoffice" / "build_headless_exe.bat"
    if CMEXE.is_file():
        return
    if not builder.is_file():
        raise SystemExit(f"[verify] FAIL: headless CM exe missing and no builder at {builder}")
    print(f"[verify] headless CM exe missing; building via {builder.name} ...")
    r = subprocess.run(["cmd", "/c", str(builder)], cwd=str(builder.parent))
    if r.returncode != 0 or not CMEXE.is_file():
        raise SystemExit("[verify] FAIL: headless CM exe build failed (see output above)")


def pump(proc, sink, tag):
    for line in proc.stdout:
        s = line.rstrip("\n")
        sink.append(s)
        if s.strip():
            print(f"  [{tag}] {s}", flush=True)


def main() -> int:
    ensure_headless_exe()
    for p, label in [(TL, "TrafficLayer.exe"), (DM, "DriverModel_RealSim.dll"),
                     (CMEXE, "CarMaker_headless.win64.exe"),
                     (SRC_INPX, "simple_traffic_light.inpx"), (SIGNAL_TABLE, "RSsignalTable.csv")]:
        if not p.is_file():
            print(f"[verify] FAIL: missing {label}: {p}")
            return 2
    if not (CMPROJ / "Data" / "TestRun" / TESTRUN).is_file():
        print(f"[verify] FAIL: TestRun {TESTRUN} missing -- run build_cosim_testrun.py")
        return 2

    # stage network + hook DriverModel + write config.runtime.yaml (NetworkFile abs)
    setup_eco.stage_network()
    setup_eco.patch_drivermodel_hook()
    setup_eco.write_runtime_config()

    print("[verify] launching TrafficLayer (DSProxy + DriverModel relay)...")
    tl_out: list[str] = []
    tl = subprocess.Popen([str(TL), "-f", str(RUNCFG)], cwd=str(HERE),
                          stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
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
    print(f"[verify] VISSIM_Connect OK after {time.time()-t0:.0f}s")

    # launch the eco CAV controller (connects on 1338; TL accepts it after CM)
    print("[verify] launching cav_controller.py (port 1338)...")
    ctl_out: list[str] = []
    ctl = subprocess.Popen(
        [PYEXE, str(HERE / "cav_controller.py"), "--config", str(HERE / "config.yaml"),
         "--signals", str(HERE / "signals.json"), "--penetration", "1.0"],
        cwd=str(HERE), stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
    threading.Thread(target=pump, args=(ctl, ctl_out, "CTL"), daemon=True).start()

    print("[verify] launching headless CarMaker (+ signal table)...")
    cm_out: list[str] = []
    cm = subprocess.Popen(
        [str(CMEXE), "-screen", "-dstore", "-f", str(RUNCFG),
         "-s", str(SIGNAL_TABLE.with_suffix("")), f"Data/TestRun/{TESTRUN}"],
        cwd=str(CMPROJ), stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
    threading.Thread(target=pump, args=(cm, cm_out, "CarMaker"), daemon=True).start()

    t1 = time.time()
    while time.time() - t1 < RUN_TIMEOUT:
        if cm.poll() is not None:
            break
        time.sleep(1)
    time.sleep(3)  # let the controller flush its summary
    for proc in (cm, ctl, tl):
        if proc.poll() is None:
            proc.kill()

    # --- assertions ---
    cm_text = "\n".join(cm_out)
    tl_text = "\n".join(tl_out)
    ctl_text = "\n".join(ctl_out)

    lib_connected = "RealSim Initialized" in cm_text or "All Clients Connected!" in cm_text
    sim_ran = "SIM_END" in cm_text
    signal_sock = "signal client connected" in tl_text
    cav_listener = "CAV-controller listener bound on port" in tl_text
    cav_connected = "CAV controller connected" in tl_text
    dm_connected = "DriverModel connected" in tl_text

    tick_lines = [l for l in tl_out if re.search(r"tick\s+\d+: vehicles=", l)]
    max_veh = max_sig = max_tick = 0
    egos_ok = False
    for l in tick_lines:
        m = re.search(r"tick\s+(\d+): vehicles=\s*(\d+)\s+signals=\s*(\d+)\s+egos=(\d+)", l)
        if m:
            max_tick = max(max_tick, int(m.group(1)))
            max_veh = max(max_veh, int(m.group(2)))
            max_sig = max(max_sig, int(m.group(3)))
            if int(m.group(4)) >= 1:
                egos_ok = True
    # relay liveness: ticks advanced well past tick 1 (the historical SubEgoOnly hang)
    relay_live = max_tick >= 5
    # controller relayed CAV cmds
    m = re.search(r"done: (\d+) ticks, (\d+) CAV cmds sent, (\d+) eco-active", ctl_text)
    ctl_cmds = int(m.group(2)) if m else 0
    ctl_eco = int(m.group(3)) if m else 0

    print("\n========== EVIDENCE ==========")
    print("CarMaker:", "RealSim Initialized" if lib_connected else "(.lib did NOT connect)")
    print("CarMaker:", "SIM_END (ran to completion)" if sim_ran else "(did not finish)")
    print("TrafficLayer:", "DriverModel connected (port 1337)" if dm_connected else "(NO DriverModel)")
    print("TrafficLayer:", "CAV-controller listener bound (1338)" if cav_listener else "(NO CAV listener)")
    print("TrafficLayer:", "CAV controller connected" if cav_connected else "(CAV controller did NOT connect)")
    print("TrafficLayer:", "signal client connected (2445)" if signal_sock else "(NO signal socket)")
    print(f"TrafficLayer: {len(tick_lines)} ticks (max tick={max_tick}), peak vehicles={max_veh}, "
          f"peak signals={max_sig}, egos>=1: {egos_ok}")
    print(f"controller: {ctl_cmds} CAV cmds sent, {ctl_eco} eco-active CAV-ticks")
    print(f"relay liveness (ticks advanced past 1): {relay_live}")
    for l in tick_lines[-5:]:
        print("   ", l)

    ok = (lib_connected and sim_ran and dm_connected and cav_connected and signal_sock
          and egos_ok and max_veh > 0 and max_sig > 0 and relay_live and ctl_cmds > 0)
    print("\n========== RESULT ==========")
    if ok:
        print("PASS: DSProxy + DriverModel co-load verified -- ego in, signals synced, CAVs "
              "relayed via DriverModel, relay did not deadlock.")
    else:
        print("FAIL: see evidence above.")
        if dm_connected and not relay_live:
            print("  NOTE: DriverModel connected but ticks stalled -> the SubEgoOnly/per-Car "
                  "relay path is the issue (see README 'Status').")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
