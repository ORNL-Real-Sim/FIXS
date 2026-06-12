"""Perf RE-VALIDATION (#168). The first UpdRate sweep was INVALID: lowering UpdRate FROZE
the teleported FreeMotion traffic, so the ego stalls and the measured "speedup" was the cost
of a STATIC (broken) co-sim, not a moving one. This version checks CORRECTNESS first -- the
ego's distance travelled, parsed from the CarMaker SIM_END line -- at each UpdRate, and only
trusts the cost where the traffic actually moved (a moving co-sim makes the ego follow + travel;
a frozen one leaves it stalled near 0 m). Fixed N_TRAFFIC=50 (the demo value).

  python sweep_updrate.py   (needs the RS_DEBUG builds + a short SimulationEndTime)
"""
import os
import subprocess
import sys
import time
import pathlib
import shutil
import csv
import statistics
import re

HERE = pathlib.Path(__file__).resolve().parent
REPO = HERE.parents[3]
CMPROJ = REPO / "ProprietaryFiles" / "CM13_proj"
PY = sys.executable
RESULTS = HERE / "RS_tmp"
RESULTS.mkdir(exist_ok=True)
N_FIXED = 50
UPD_SWEEP = [1000, 500, 250, 200]
WARMUP_S = 60.0
DIST = {}


def kill_stale():
    for img in ("VISSIM220.exe", "TrafficLayer.exe", "CarMaker_headless.win64.exe"):
        subprocess.run(["taskkill", "/F", "/IM", img], capture_output=True)


def run_one(upd):
    print(f"\n===== UpdRate={upd} Hz (N_TRAFFIC={N_FIXED}) =====", flush=True)
    kill_stale()
    time.sleep(2)
    env = dict(os.environ, RS_N_TRAFFIC=str(N_FIXED), RS_UPD_RATE=str(upd))
    subprocess.run([PY, str(HERE / "build_testrun.py")], env=env, check=True)
    try:
        (CMPROJ / "rs_runstep_cm.csv").unlink()
    except FileNotFoundError:
        pass
    r = subprocess.run([PY, str(HERE / "verify_demo.py")], env=env,
                       capture_output=True, text=True)
    # ego distance from the CarMaker SIM_END line: "SIM_END ... 120.001s 266.671m"
    d = -1.0
    for line in r.stdout.splitlines():
        if "SIM_END" in line:
            mm = re.search(r"([\d.]+)\s*m\b", line)
            if mm:
                d = float(mm.group(1))
    DIST[upd] = d
    print(f"  ego distance = {d:.1f} m  (large => traffic moved + ego followed; ~0 => FROZEN)")
    src = CMPROJ / "rs_runstep_cm.csv"
    if src.exists():
        shutil.copy2(src, RESULTS / f"rs_runstep_upd{upd}.csv")


def cost(path):
    lib, full = [], []
    with open(path) as f:
        for row in csv.DictReader(f):
            try:
                if float(row["simTime"]) < WARMUP_S:
                    continue
                lib.append(float(row["lib_runstep_us"]))
                full.append(float(row["full_step_us"]))
            except (ValueError, KeyError):
                continue
    if not lib:
        return None
    return statistics.mean(full) - statistics.mean(lib)


if __name__ == "__main__":
    for upd in UPD_SWEEP:
        run_one(upd)
    kill_stale()
    print(f"\n===== UpdRate RE-VALIDATION (N_TRAFFIC={N_FIXED}, simTime > {WARMUP_S:.0f}s) =====")
    print(f"{'UpdRate':>8} {'ego_dist_m':>11} {'co-sim OK?':>11} {'CMcore us/obj':>14}")
    for upd in UPD_SWEEP:
        d = DIST.get(upd, -1.0)
        ok = "YES" if d > 50 else "NO (frozen)"
        p = RESULTS / f"rs_runstep_upd{upd}.csv"
        c = cost(p) if p.exists() else None
        cstr = f"{c / N_FIXED:.2f}" if c is not None else "-"
        print(f"{upd:>8} {d:>11.1f} {ok:>11} {cstr:>14}")
    print("\nThe cost column is ONLY valid where co-sim OK? = YES. If only 1000 keeps the")
    print("traffic moving, UpdRate is NOT a usable speed lever here (the .lib refreshes")
    print("traffic positions at 1 kHz; CarMaker drops them below its FreeMotion default).")
