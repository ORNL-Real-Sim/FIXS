"""
Self-checking SUMO <-> CarMaker(office) co-sim verify (FIXS #174 SimpleLoop).

Launches the whole stack as subprocesses, records every line to log files, and
reasons about whether the output makes sense -- the SUMO sibling of the #168
verify_demo.py. There is NO cross-backend diff: we just run it, capture it, and
sanity-check the round-trip.

Pipeline:
  1. SUMO (headless) on the shared SimpleLoop net, TraCI server on port 1337
  2. TrafficLayer (SUMO path) -> connects to SUMO, serves CarMaker on 2444
  3. CarMaker_headless.win64.exe -screen -f config.yaml <TestRun>  (real .lib)

Asserts (lenient, evidence-first):
  - TrafficLayer reached "Traffic Simulator: SUMO" and did NOT print
    "Connect to SUMO failed"
  - CarMaker's VirtualEnvironment.lib connected ("RealSim Initialized" /
    "All Clients Connected!")
  - the sim advanced (TrafficLayer stepped) and ended cleanly
    ("Simulation end time reached." and/or CarMaker "SIM_END")
  - no ERROR lines mid-stream

Prints the key evidence + PASS/FAIL. Logs: _logs/tl.log, _logs/cm.log.

Run:  python verify_sumo_cm.py
Prereq: SUMO on PATH; TrafficLayer.exe built; the CM13 project + TestRun
        SimpleLoop_VISSIM_rs present (shared with the VISSIM probe). The headless
        CarMaker harness exe is (re)built automatically if missing/stale.
"""
from __future__ import annotations
import os
import re
import shutil
import subprocess
import sys
import threading
import time
import pathlib

HERE = pathlib.Path(__file__).resolve().parent
REPO = HERE.parents[3]
TL = REPO / "TrafficLayer" / "x64" / "Release" / "TrafficLayer.exe"
CMEXE = REPO / "ProprietaryFiles" / "CM13_proj" / "src" / "CarMaker_headless.win64.exe"
CMPROJ = REPO / "ProprietaryFiles" / "CM13_proj"
CONFIG = HERE / "config.yaml"
SUMOCFG = REPO / "tests" / "Sumo" / "network" / "simple_loop" / "simple_loop.sumocfg"
TESTRUN = "SimpleLoop_VISSIM_rs"
# The headless build helper lives in the VISSIM probe (config-agnostic: it builds
# VirtualEnvironment.lib + the RS_HEADLESS CarMaker exe). Reuse it, don't fork it.
BUILD_HEADLESS = REPO / "tests" / "Vissim" / "Probes" / "TrafficLayer_DSProxy_CMoffice" / "build_headless_exe.bat"
LOGS = HERE / "_logs"

SUMO_PORT = 1337
TL_READY_TIMEOUT = 60     # s for TrafficLayer to connect to SUMO
RUN_TIMEOUT = 600         # s overall cap for the CarMaker run


def sumo_exe() -> str:
    exe = shutil.which("sumo")
    if not exe:
        sh = os.environ.get("SUMO_HOME")
        if sh:
            cand = pathlib.Path(sh) / "bin" / "sumo.exe"
            if cand.is_file():
                return str(cand)
    return exe or "sumo"


def ensure_headless_exe() -> None:
    srcs = [
        REPO / "CommonLib" / "VirEnv_Wrapper.cpp",
        REPO / "CommonLib" / "SocketHelper.cpp",
        REPO / "CommonLib" / "VirEnvHelper.cpp",
        REPO / "ProprietaryFiles" / "CM13_proj" / "src" / "User.c",
    ]
    stale = CMEXE.is_file() and any(
        s.is_file() and s.stat().st_mtime > CMEXE.stat().st_mtime for s in srcs)
    if CMEXE.is_file() and not stale:
        return
    why = "missing" if not CMEXE.is_file() else "stale (co-sim source changed)"
    print(f"[verify] headless CarMaker harness {why}; building via build_headless_exe.bat ...")
    if not BUILD_HEADLESS.is_file():
        raise SystemExit(f"[verify] FAIL: build helper not found: {BUILD_HEADLESS}")
    r = subprocess.run(["cmd", "/c", str(BUILD_HEADLESS)], cwd=str(BUILD_HEADLESS.parent))
    if r.returncode != 0 or not CMEXE.is_file():
        raise SystemExit("[verify] FAIL: headless exe build failed (see output above)")


def pump(proc, sink, logfile, tag):
    with open(logfile, "w", encoding="utf-8") as fh:
        for line in proc.stdout:
            s = line.rstrip("\n")
            sink.append(s)
            fh.write(s + "\n")
            fh.flush()
            if s.strip():
                print(f"  [{tag}] {s}", flush=True)


def main() -> int:
    LOGS.mkdir(exist_ok=True)
    ensure_headless_exe()
    for p, label in [(TL, "TrafficLayer.exe"), (CMEXE, "CarMaker_headless.win64.exe"),
                     (CONFIG, "config.yaml"), (SUMOCFG, "simple_loop.sumocfg")]:
        if not p.is_file():
            print(f"[verify] FAIL: missing {label}: {p}")
            return 2
    if not (CMPROJ / "Data" / "TestRun" / TESTRUN).is_file():
        print(f"[verify] FAIL: TestRun {TESTRUN} missing in {CMPROJ}")
        return 2

    procs = []
    try:
        # --- 1. SUMO headless: TraCI server, waits for TrafficLayer to connect ---
        print(f"[verify] launching SUMO (headless) on port {SUMO_PORT} ...")
        sumo = subprocess.Popen(
            [sumo_exe(), "-c", str(SUMOCFG), "--remote-port", str(SUMO_PORT),
             "--step-length", "0.1", "--start", "--quit-on-end"],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
        procs.append(sumo)
        sumo_out: list[str] = []
        threading.Thread(target=pump, args=(sumo, sumo_out, LOGS / "sumo.log", "SUMO"),
                         daemon=True).start()
        time.sleep(2)  # let SUMO bind the port before the client connects

        # --- 2. TrafficLayer (SUMO path) ---
        print("[verify] launching TrafficLayer (SUMO path) ...")
        tl_out: list[str] = []
        tl = subprocess.Popen([str(TL), "-f", str(CONFIG)], cwd=str(HERE),
                              stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                              text=True, bufsize=1)
        procs.append(tl)
        threading.Thread(target=pump, args=(tl, tl_out, LOGS / "tl.log", "TL"),
                         daemon=True).start()

        t0 = time.time()
        tl_ready = False
        while time.time() - t0 < TL_READY_TIMEOUT:
            if any("Traffic Simulator: SUMO" in l for l in tl_out):
                tl_ready = True
                break
            if any("Connect to SUMO failed" in l for l in tl_out) or tl.poll() is not None:
                break
            time.sleep(1)
        if not tl_ready:
            print("[verify] FAIL: TrafficLayer did not start on the SUMO path")
            print("\n".join(tl_out[-15:]))
            return 1
        print("[verify] TrafficLayer on SUMO path; launching headless CarMaker ...")

        # --- 3. real CarMaker headless against TrafficLayer ---
        cm_out: list[str] = []
        cm = subprocess.Popen(
            [str(CMEXE), "-screen", "-f", str(CONFIG), f"Data/TestRun/{TESTRUN}"],
            cwd=str(CMPROJ), stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            text=True, bufsize=1)
        procs.append(cm)
        threading.Thread(target=pump, args=(cm, cm_out, LOGS / "cm.log", "CarMaker"),
                         daemon=True).start()

        t1 = time.time()
        while time.time() - t1 < RUN_TIMEOUT:
            if cm.poll() is not None:
                break
            time.sleep(1)
        time.sleep(2)  # let TrafficLayer flush its last steps
    finally:
        for p in reversed(procs):
            if p.poll() is None:
                p.kill()

    # --- reason about the captured output ---
    cm_text = "\n".join(cm_out)
    tl_text = "\n".join(tl_out)
    lib_connected = "RealSim Initialized" in cm_text or "All Clients Connected!" in cm_text
    cm_simend = "SIM_END" in cm_text
    tl_started = "Traffic Simulator: SUMO" in tl_text
    tl_sumo_fail = "Connect to SUMO failed" in tl_text
    tl_clean_end = "Simulation end time reached." in tl_text
    # TrafficLayer prints one "===========SimTime <t>==============" banner per
    # step it advances; count those (not "New time step", which it never emits).
    steps = len(re.findall(r"SimTime\s+[\d.]+", tl_text))
    # Mid-stream errors (ignore the benign SUMO-not-up message before connect).
    err_lines = [l for l in tl_out + cm_out if "ERROR" in l]

    print("\n========== EVIDENCE ==========")
    print("TrafficLayer:", "started on SUMO path" if tl_started else "(did NOT start on SUMO)")
    print("TrafficLayer:", "connected to SUMO" if not tl_sumo_fail else "(Connect to SUMO FAILED)")
    print(f"TrafficLayer: {steps} sim steps,",
          "clean end" if tl_clean_end else "(no clean-end marker)")
    print("CarMaker:", "RealSim Initialized (.lib connected)" if lib_connected
          else "(.lib did NOT connect)")
    print("CarMaker:", "SIM_END (ran to completion)" if cm_simend else "(did not finish)")
    if err_lines:
        print(f"ERROR lines ({len(err_lines)}):")
        for l in err_lines[:8]:
            print("   ", l)

    ok = (tl_started and not tl_sumo_fail and lib_connected and steps > 0
          and (cm_simend or tl_clean_end) and not err_lines)
    print("\n========== RESULT ==========")
    print("PASS: SUMO<->CarMaker<->FIXS co-simulation looks sane"
          if ok else "FAIL/REVIEW: see evidence + _logs/*.log above")
    print(f"(logs: {LOGS})")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
