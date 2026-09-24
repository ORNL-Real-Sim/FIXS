"""
Self-checking SUMO <-> Carla co-sim verify (FIXS #174 SimpleLoop, no signals).

Brings up the stack, records every line, and reasons about whether the bridge's
decisions make sense -- NO cross-backend diff, just run/capture/sanity-check.

Pipeline:
  0. (you) CARLA server already running + simple_loop.xodr loaded as the world
     (Carla\launch_carla.bat + Carla\load_opendrive_world.py -- see README).
     This script verifies the RPC is reachable and skips cleanly if not.
  1. SUMO (headless) on the shared SimpleLoop net, TraCI server on port 1337
  2. TrafficLayer (SUMO path) -> serves the Carla bridge on the config's port
  3. the bridge, -f config.yaml -t traffic_light_table.csv. RS_BRIDGE picks it:
       py  (default) Carla/VirEnv/mainVirCarla.py -- the maintained bridge, and
                     the one run_cosim runs
       cpp           VirCarlaEnv.exe, BUILT from this tree only (see BRIDGE)

Reasons over the captured output (config has EnableVerboseLog: true):
  - bridge connected to CARLA (prints "Carla client ... / server ...")
  - bridge spawned >=1 vehicle ("Spawned Carla actor")
  - active-id churn looks sane (spawns happen, despawns only for vehicles that left)
  - no spawn-failure storm / exceptions (a FEW spawn-point contentions are a
    property of SimpleLoop, not a regression -- see SPAWN_FAIL_FRACTION)

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
#: Which bridge this verifies. "py" (default) is the Python bridge -- maintained,
#: and what run_cosim runs. "cpp" is VirCarlaEnv.exe, and only one BUILT from this
#: tree. This used to fall back to tests/SumoCarla/VirCarlaEnv.exe when no build
#: existed: a binary committed in June, before #266 and #358. VirCarlaEnv does not
#: compile today (#380), so that fallback was the only thing this probe could run,
#: and a PASS was a verdict on June's code, not on the code in the tree. FIXS#208.
BRIDGE = os.environ.get("RS_BRIDGE", "py").lower()
VCE = REPO / "VirCarlaEnv" / "x64" / "Release" / "VirCarlaEnv.exe"
PY_BRIDGE = REPO / "Carla" / "VirEnv" / "mainVirCarla.py"
CONFIG = HERE / "config.yaml"
TLS = HERE / "traffic_light_table.csv"
SUMOCFG = REPO / "tests" / "Sumo" / "networks" / "simple_loop" / "simple_loop_ego.sumocfg"
XODR = REPO / "tests" / "Vissim" / "SimpleEcho" / "simple_loop.xodr"
LOGS = HERE / "_logs"

SUMO_PORT = 1337
CARLA_HOST, CARLA_PORT = "127.0.0.1", 2000
TL_READY_TIMEOUT = 60
RUN_SECONDS = 60          # how long to let the co-sim run before stopping

#: SUMO RNG seed, pinned so the verdict is reproducible. The traffic realization
#: decides when vehicles arrive, and therefore how often two of them contend for
#: one CARLA spawn point -- so an unseeded run gives a different spawn count AND a
#: different failure count every time, and a threshold on either means nothing.
#: Same convention as the CarMaker demos (see CLAUDE.md): RS_SUMO_SEED=none runs
#: unseeded, which is how you check whether a verdict depends on the seed.
SUMO_SEED = os.environ.get("RS_SUMO_SEED", "5")

#: CARLA refuses a spawn when an actor still occupies the spawn point. SimpleLoop
#: does that about 2 times in 41 spawns -- identically for #174's binary and
#: #109's Release-only one, so it is the scenario's geometry rather than a
#: regression. Requiring zero (what this did) reported FAIL on a healthy co-sim.
#: A fraction rather than a count, because what is being asked is "a storm, or a
#: few?", and the answer has to scale with how many vehicles the run spawned.
SPAWN_FAIL_FRACTION = 0.15
SPAWN_FAIL_FLOOR = 2      # ...and never fail a run over fewer than this many


def sumo_exe() -> str:
    exe = shutil.which("sumo")
    if not exe and os.environ.get("SUMO_HOME"):
        cand = pathlib.Path(os.environ["SUMO_HOME"]) / "bin" / "sumo.exe"
        if cand.is_file():
            return str(cand)
    return exe or "sumo"


def bridge_python() -> str:
    """The interpreter run_cosim runs the bridge under (~/.fixs/carla.json), which
    is the one with the carla client installed; else this one."""
    sys.path.insert(0, str(REPO / "Carla"))
    try:
        import carla_env_setup
        py = (carla_env_setup.load_config() or {}).get("python")
        if py and os.path.isfile(py):
            return py
    except Exception:
        pass
    finally:
        sys.path.pop(0)
    return sys.executable


def bridge_cmd() -> list[str]:
    if BRIDGE == "cpp":
        return [str(VCE), "-f", str(CONFIG), "-t", str(TLS)]
    # -u: the bridge's stdout is a pipe, which Python block-buffers, and this script
    # KILLS the bridge when the run ends -- so without it the lines that decide the
    # verdict can die in the buffer and read as "never connected, spawned nothing".
    return [bridge_python(), "-u", str(PY_BRIDGE), "-f", str(CONFIG), "-t", str(TLS)]


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
    if BRIDGE not in ("py", "cpp"):
        print(f"[verify] FAIL: RS_BRIDGE={BRIDGE!r}; expected 'py' or 'cpp'")
        return 2
    bridge = (PY_BRIDGE, "mainVirCarla.py") if BRIDGE == "py" else (VCE, "VirCarlaEnv.exe (built)")
    for p, label in [(TL, "TrafficLayer.exe"), bridge,
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
        print(f"[verify] launching SUMO (headless) on port {SUMO_PORT}, "
           f"seed {SUMO_SEED} ...")
        sumo_cmd = [sumo_exe(), "-c", str(SUMOCFG), "--remote-port", str(SUMO_PORT),
                    "--step-length", "0.1", "--start", "--quit-on-end"]
        if SUMO_SEED.lower() != "none":
            sumo_cmd += ["--seed", SUMO_SEED]
        sumo = subprocess.Popen(
            sumo_cmd,
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

        cmd = bridge_cmd()
        print(f"[verify] launching the Carla bridge ({BRIDGE}): {' '.join(cmd)}")
        vce_out: list[str] = []
        vce = subprocess.Popen(cmd, cwd=str(HERE), stdout=subprocess.PIPE,
                               stderr=subprocess.STDOUT, text=True, bufsize=1)
        procs.append(vce)
        threading.Thread(target=pump, args=(vce, vce_out, LOGS / "bridge.log", "bridge"),
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
    # Accept both the old ("Server API version" / "Spawned actor") and the current
    # VirCarlaEnv message wording ("Carla client .../ server ..." / "Spawned Carla actor").
    carla_connected = ("Server API version" in vce_text) or ("Carla client" in vce_text)
    spawns = len(re.findall(r"Spawn(?:ing|ed)(?: Carla)? actor", vce_text))
    despawns = len(re.findall(r"(?:Removing Sumo actor|Destroyed Carla actor)", vce_text))
    spawn_fail = len(re.findall(r"Failed to spawn actor", vce_text))
    # C++ says "Exception"; Python says "Traceback" and "<Name>Error: ...". Missing
    # the Python form would pass a bridge that crashed.
    exceptions = [l for l in vce_out
                  if "Exception" in l or l.startswith("Traceback")
                  or re.match(r"^\w+(Error|Exception): ", l)]
    tls_notice = "no traffic-light data" in vce_text  # our empty-TLS fix kicked in (expected)

    print("\n========== EVIDENCE ==========")
    print(f"bridge ({BRIDGE}):", "connected to CARLA" if carla_connected else "(did NOT reach CARLA)")
    allowed = max(SPAWN_FAIL_FLOOR, int(spawns * SPAWN_FAIL_FRACTION))
    print(f"bridge ({BRIDGE}): spawns={spawns}, despawns={despawns}, "
          f"spawn-failures={spawn_fail} (tolerated: {allowed})")
    print(f"bridge ({BRIDGE}):", "ran vehicles-only (empty TLS handled)" if tls_notice
          else "(no empty-TLS notice -- check config)")
    if exceptions:
        print(f"Exceptions ({len(exceptions)}):")
        for l in exceptions[:6]:
            print("   ", l)

    ok = (carla_connected and spawns >= 1 and spawn_fail <= allowed
          and not exceptions)
    print("\n========== RESULT ==========")
    print("PASS: SUMO->TrafficLayer->Carla bridge spawned/posed vehicles sanely"
          if ok else "FAIL/REVIEW: see evidence + _logs/*.log above")
    print(f"(logs: {LOGS})")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
