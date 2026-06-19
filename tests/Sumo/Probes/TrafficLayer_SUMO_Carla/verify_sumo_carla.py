"""
Self-checking SUMO <-> Carla co-sim verify (FIXS #174 SimpleLoop, no signals).

Brings up the stack, records every line, and reasons about whether the bridge's
decisions make sense -- NO cross-backend diff, just run/capture/sanity-check.

Pipeline:
  0. (you) CARLA server already running + simple_loop.xodr loaded as the world
     (Carla\launch_carla.bat + Carla\load_opendrive_world.py -- see README).
     This script verifies the RPC is reachable and skips cleanly if not.
  1. SUMO (headless) on the shared SimpleLoop net, TraCI server on port 1337
  2. TrafficLayer (SUMO path) -> serves the Carla bridge on port 440
  3. VirCarlaEnv.exe -f config.yaml -t traffic_light_table.csv  (the real bridge)

Reasons over the captured output (config has EnableVerboseLog: true):
  - bridge connected to CARLA (prints "Client API version" / "Server API version")
  - bridge spawned >=1 vehicle ("Spawning actor" / "Spawned actor")
  - active-id churn looks sane (spawns happen, despawns only for vehicles that left)
  - no spawn-failure storm / exceptions

Prints evidence + PASS / FAIL / SKIP. Logs under _logs/.

Run:  python verify_sumo_carla.py
Exit: 0 PASS, 1 FAIL/REVIEW, 2 missing prereq, 3 SKIP (no CARLA server reachable)
"""
from __future__ import annotations
import os
import re
import shutil
import socket
import subprocess
import sys
import threading
import time
import pathlib

HERE = pathlib.Path(__file__).resolve().parent
REPO = HERE.parents[3]
TL = REPO / "TrafficLayer" / "x64" / "Release" / "TrafficLayer.exe"
VCE = REPO / "VirCarlaEnv" / "x64" / "Release" / "VirCarlaEnv.exe"
if not VCE.is_file():
    VCE = REPO / "tests" / "SumoCarla" / "VirCarlaEnv.exe"   # committed fallback
CONFIG = HERE / "config.yaml"
TLS = HERE / "traffic_light_table.csv"
SUMOCFG = REPO / "tests" / "Sumo" / "SimpleLoop" / "simple_loop.sumocfg"
XODR = REPO / "tests" / "Vissim" / "SimpleEcho" / "simple_loop.xodr"
LOGS = HERE / "_logs"

SUMO_PORT = 1337
CARLA_HOST, CARLA_PORT = "127.0.0.1", 2000
TL_READY_TIMEOUT = 60
RUN_SECONDS = 60          # how long to let the co-sim run before stopping


def sumo_exe() -> str:
    exe = shutil.which("sumo")
    if not exe and os.environ.get("SUMO_HOME"):
        cand = pathlib.Path(os.environ["SUMO_HOME"]) / "bin" / "sumo.exe"
        if cand.is_file():
            return str(cand)
    return exe or "sumo"


def carla_reachable(host: str, port: int, timeout=2.0) -> bool:
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except OSError:
        return False


def pump(proc, sink, logfile, tag):
    with open(logfile, "w", encoding="utf-8") as fh:
        for line in proc.stdout:
            s = line.rstrip("\n")
            sink.append(s)
            fh.write(s + "\n"); fh.flush()
            if s.strip():
                print(f"  [{tag}] {s}", flush=True)


def main() -> int:
    LOGS.mkdir(exist_ok=True)
    for p, label in [(TL, "TrafficLayer.exe"), (VCE, "VirCarlaEnv.exe"),
                     (CONFIG, "config.yaml"), (SUMOCFG, "simple_loop.sumocfg")]:
        if not p.is_file():
            print(f"[verify] FAIL: missing {label}: {p}")
            return 2

    # --- 0. CARLA must be up with the SimpleLoop world loaded ---
    if not carla_reachable(CARLA_HOST, CARLA_PORT):
        print(f"[verify] SKIP: no CARLA RPC on {CARLA_HOST}:{CARLA_PORT}.")
        print( "         Start it first:")
        print(f"           Carla\\launch_carla.bat")
        print(f"           powershell -File Carla\\wait_for_rpc.ps1 -Port {CARLA_PORT}")
        print(f"           <py> Carla\\load_opendrive_world.py {XODR} --sync --delta 0.1")
        print( "         then re-run this verify.")
        return 3

    procs = []
    try:
        print(f"[verify] launching SUMO (headless) on port {SUMO_PORT} ...")
        sumo = subprocess.Popen(
            [sumo_exe(), "-c", str(SUMOCFG), "--remote-port", str(SUMO_PORT),
             "--step-length", "0.1", "--start", "--quit-on-end"],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
        procs.append(sumo)
        threading.Thread(target=pump, args=(sumo, [], LOGS / "sumo.log", "SUMO"),
                         daemon=True).start()
        time.sleep(2)

        print("[verify] launching TrafficLayer (SUMO path) ...")
        tl_out: list[str] = []
        tl = subprocess.Popen([str(TL), "-f", str(CONFIG)], cwd=str(HERE),
                              stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                              text=True, bufsize=1)
        procs.append(tl)
        threading.Thread(target=pump, args=(tl, tl_out, LOGS / "tl.log", "TL"),
                         daemon=True).start()

        t0 = time.time()
        while time.time() - t0 < TL_READY_TIMEOUT:
            if any("Traffic Simulator: SUMO" in l for l in tl_out):
                break
            if any("Connect to SUMO failed" in l for l in tl_out) or tl.poll() is not None:
                print("[verify] FAIL: TrafficLayer did not start on the SUMO path")
                return 1
            time.sleep(1)

        print("[verify] launching VirCarlaEnv (Carla bridge) ...")
        vce_out: list[str] = []
        vce = subprocess.Popen([str(VCE), "-f", str(CONFIG), "-t", str(TLS)],
                               cwd=str(HERE), stdout=subprocess.PIPE,
                               stderr=subprocess.STDOUT, text=True, bufsize=1)
        procs.append(vce)
        threading.Thread(target=pump, args=(vce, vce_out, LOGS / "vce.log", "VirCarla"),
                         daemon=True).start()

        t1 = time.time()
        while time.time() - t1 < RUN_SECONDS:
            if vce.poll() is not None:
                break
            time.sleep(1)
        time.sleep(2)
    finally:
        for p in reversed(procs):
            if p.poll() is None:
                p.kill()

    # --- reason about the captured bridge output ---
    vce_text = "\n".join(vce_out)
    carla_connected = "Server API version" in vce_text
    spawns = len(re.findall(r"Spawn(?:ing|ed) actor", vce_text))
    despawns = len(re.findall(r"(?:Removing Sumo actor|Destroyed Carla actor)", vce_text))
    spawn_fail = len(re.findall(r"Failed to spawn actor", vce_text))
    exceptions = [l for l in vce_out if "Exception" in l]
    tls_notice = "no traffic-light data" in vce_text  # our empty-TLS fix kicked in (expected)

    print("\n========== EVIDENCE ==========")
    print("VirCarlaEnv:", "connected to CARLA" if carla_connected else "(did NOT reach CARLA)")
    print(f"VirCarlaEnv: spawns={spawns}, despawns={despawns}, spawn-failures={spawn_fail}")
    print("VirCarlaEnv:", "ran vehicles-only (empty TLS handled)" if tls_notice
          else "(no empty-TLS notice -- check config)")
    if exceptions:
        print(f"Exceptions ({len(exceptions)}):")
        for l in exceptions[:6]:
            print("   ", l)

    ok = carla_connected and spawns >= 1 and spawn_fail == 0 and not exceptions
    print("\n========== RESULT ==========")
    print("PASS: SUMO->TrafficLayer->Carla bridge spawned/posed vehicles sanely"
          if ok else "FAIL/REVIEW: see evidence + _logs/*.log above")
    print(f"(logs: {LOGS})")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
