"""
Self-checking end-to-end verification of the #172 CarMaker<->VISSIM<->FIXS co-sim
WITH traffic-signal sync (the complete demo: vehicles transmitted AND signals synced).

Extends the #168 verify_demo.py with the signal half:
  - stages the signalized DS network (+ its .sig signal programs / .layx),
  - launches TrafficLayer (DSProxy) -> spawns VISSIM 2022,
  - launches the REAL custom CarMaker exe headless with BOTH -f <config> and
    -s <RSsignalTable.csv> so the .lib opens the signal socket (port 2445, served
    by DSProxyMode Plan A) and maps VISSIM signal-group state -> CM traffic lights,
  - asserts the full round-trip:
      * TrafficLayer 'VISSIM_Connect OK' + 'signal client connected'
      * CarMaker '.lib connected' + 'SIM_END'
      * TrafficLayer per-tick 'vehicles=N signals=M egos=1' with N>0 AND M>0
        (vehicles transmitted AND signal states relayed every tick).

Run:  python verify_signal_demo.py
Prereq: VISSIM 2022 licensed/healthy; binaries built; add_signal_stops.py +
        build_signal_table.py + build_cosim_testrun.py already run (run_signal_demo_headless.bat
        does this for you).
"""
from __future__ import annotations
import os, re, subprocess, sys, time, pathlib, shutil, threading

HERE = pathlib.Path(__file__).resolve().parent
REPO = HERE.parents[3]
TL = REPO / "TrafficLayer" / "x64" / "Release" / "TrafficLayer.exe"
CMEXE = REPO / "ProprietaryFiles" / "CM13_proj" / "src" / "CarMaker_headless.win64.exe"
CMPROJ = REPO / "ProprietaryFiles" / "CM13_proj"
VIS_NET = REPO / "tests" / "Vissim" / "networks" / "simple_traffic_light"   # the one VISSIM scenario
SRC_INPX = VIS_NET / "simple_traffic_light.inpx"                            # DS-enabled
STAGE = HERE / "stage_network"
RUNCFG = HERE / "config.runtime.yaml"
TESTRUN = "SimpleTrafficLight_Cosim"
SIGNAL_TABLE = CMPROJ / "Data" / "Road" / "simple_traffic_light_RSsignalTable.csv"

TL_CONNECT_TIMEOUT = 120
RUN_TIMEOUT = 600


def stage_and_config() -> None:
    STAGE.mkdir(exist_ok=True)
    staged_inpx = STAGE / "simple_traffic_light.inpx"
    shutil.copy2(SRC_INPX, staged_inpx)
    # carry the layout + the external .sig signal programs alongside the network
    for extra in list(VIS_NET.glob("*.layx")) + list(VIS_NET.glob("int_*.sig")):
        shutil.copy2(extra, STAGE / extra.name)
    cfg = (HERE / "config.yaml").read_text(encoding="utf-8")
    cfg = cfg.replace("stage_network\\simple_traffic_light.inpx", str(staged_inpx))
    RUNCFG.write_text(cfg, encoding="ascii")
    print(f"[verify] staged signalized network (+ .sig/.layx) + wrote {RUNCFG.name}")


def ensure_headless_exe() -> None:
    """Reuse the #168 headless harness build (same User.c/.lib). Build if missing."""
    builder = HERE.parent / "TrafficLayer_DSProxy_CMoffice" / "build_headless_exe.bat"
    srcs = [REPO / "CommonLib" / "VirEnv_Wrapper.cpp",
            REPO / "CommonLib" / "SocketHelper.cpp",
            REPO / "CommonLib" / "VirEnvHelper.cpp",
            CMPROJ / "src" / "User.c"]
    stale = CMEXE.is_file() and any(
        s.is_file() and s.stat().st_mtime > CMEXE.stat().st_mtime for s in srcs)
    if CMEXE.is_file() and not stale:
        return
    if not builder.is_file():
        raise SystemExit(f"[verify] FAIL: headless CM exe missing and no builder at {builder}")
    why = "missing" if not CMEXE.is_file() else "stale (co-sim source changed)"
    print(f"[verify] headless CM exe {why}; building via {builder.name} ...")
    r = subprocess.run(["cmd", "/c", str(builder)], cwd=str(builder.parent))
    if r.returncode != 0 or not CMEXE.is_file():
        raise SystemExit("[verify] FAIL: headless CM exe build failed (see output above)")


def main() -> int:
    ensure_headless_exe()
    for p, label in [(TL, "TrafficLayer.exe"), (CMEXE, "CarMaker_headless.win64.exe"),
                     (SRC_INPX, "simple_traffic_light.inpx"), (SIGNAL_TABLE, "RSsignalTable.csv")]:
        if not p.is_file():
            print(f"[verify] FAIL: missing {label}: {p}")
            return 2
    if not (CMPROJ / "Data" / "TestRun" / TESTRUN).is_file():
        print(f"[verify] FAIL: TestRun {TESTRUN} missing -- run build_cosim_testrun.py")
        return 2

    stage_and_config()

    print("[verify] launching TrafficLayer (DSProxy)...")
    tl_out: list[str] = []
    tl = subprocess.Popen([str(TL), "-f", str(RUNCFG)], cwd=str(HERE),
                          stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)

    def pump(proc, sink, tag):
        for line in proc.stdout:
            s = line.rstrip("\n")
            sink.append(s)
            if s.strip():
                print(f"  [{tag}] {s}", flush=True)

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
    print(f"[verify] VISSIM_Connect OK after {time.time()-t0:.0f}s; launching CarMaker (+ signal table)...")

    cm_out: list[str] = []
    cm = subprocess.Popen(
        # Pass the signal table WITHOUT the .csv extension -- CarMaker's GUI/HIL
        # fatals trying to auto-parse a ".csv" argument as an InfoFile; readSignalTable
        # appends ".csv" itself. (Headless tolerates either, but keep it consistent.)
        [str(CMEXE), "-screen", "-dstore", "-f", str(RUNCFG),
         "-s", str(SIGNAL_TABLE.with_suffix("")), f"Data/TestRun/{TESTRUN}"],
        cwd=str(CMPROJ), stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
    threading.Thread(target=pump, args=(cm, cm_out, "CarMaker"), daemon=True).start()

    t1 = time.time()
    while time.time() - t1 < RUN_TIMEOUT:
        if cm.poll() is not None:
            break
        time.sleep(1)
    time.sleep(2)
    if cm.poll() is None:
        cm.kill()
    tl.kill()

    # --- assertions ---
    cm_text = "\n".join(cm_out)
    tl_text = "\n".join(tl_out)
    lib_connected = "RealSim Initialized" in cm_text or "All Clients Connected!" in cm_text
    sim_ran = "SIM_END" in cm_text
    signal_sock = "signal client connected" in tl_text
    tick_lines = [l for l in tl_out if re.search(r"tick\s+\d+: vehicles=", l)]
    max_veh = max_sig = 0
    egos_ok = False
    for l in tick_lines:
        m = re.search(r"vehicles=\s*(\d+)\s+signals=\s*(\d+)\s+egos=(\d+)", l)
        if m:
            max_veh = max(max_veh, int(m.group(1)))
            max_sig = max(max_sig, int(m.group(2)))
            if int(m.group(3)) >= 1:
                egos_ok = True

    print("\n========== EVIDENCE ==========")
    print("CarMaker:", "RealSim Initialized" if lib_connected else "(.lib did NOT connect)")
    print("CarMaker:", "SIM_END (ran to completion)" if sim_ran else "(did not finish)")
    print("TrafficLayer:", "signal client connected (port 2445)" if signal_sock else "(NO signal socket)")
    print(f"TrafficLayer: {len(tick_lines)} ticks, peak vehicles={max_veh}, peak signals={max_sig}, egos>=1: {egos_ok}")
    for l in tick_lines[-5:]:
        print("   ", l)

    # --- supplementary: did the ego brake at a VISSIM-driven red? (from the ERG) ---
    # Non-gating -- the transport assertions above already prove vehicles + signal
    # sync. This confirms the visual payoff: with the signal state now coming from
    # VISSIM, the ego still stops at the in-road DrvStop when its light is red.
    try:
        import glob
        # CarMaker derives the ERG basename from the positional testrun arg with the
        # path separators flattened, i.e. "Data_TestRun_SimpleTrafficLight_Cosim_<stamp>.erg".
        ergs = sorted(glob.glob(str(CMPROJ / "SimOutput" / "*" / "*" / f"*{TESTRUN}_*.erg")),
                      key=os.path.getmtime)
        if ergs:
            rd5 = CMPROJ / "Data" / "Road" / "simple_traffic_light.rd5"
            r = subprocess.run([sys.executable, str(HERE / "verify_signalstop.py"), ergs[-1], str(rd5)],
                               capture_output=True, text=True)
            print("\n--- ego-stops-at-red (co-sim ERG) ---")
            print(r.stdout.strip() or r.stderr.strip())
        else:
            print("\n--- ego-stops-at-red: no ERG saved (SaveMode); skipped ---")
    except Exception as e:
        print(f"\n--- ego-stops-at-red check skipped: {e} ---")

    ok = lib_connected and sim_ran and signal_sock and egos_ok and max_veh > 0 and max_sig > 0
    print("\n========== RESULT ==========")
    print("PASS: CarMaker<->VISSIM<->FIXS co-sim with vehicles AND signal sync verified"
          if ok else "FAIL: see evidence above")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
