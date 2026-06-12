"""Perf: fix N_TRAFFIC=100, sweep the CarMaker traffic UpdRate (1000/200/50/20 Hz). Does
lowering UpdRate cut CarMaker's per-object geometry cost, and does it touch the FIXS .lib?
Reuses verify_demo.py; reads rs_runstep_cm.csv (.lib vs CarMaker-core, per 1 ms step).

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

HERE = pathlib.Path(__file__).resolve().parent
REPO = HERE.parents[3]
CMPROJ = REPO / "ProprietaryFiles" / "CM13_proj"
PY = sys.executable
RESULTS = HERE / "RS_tmp"
RESULTS.mkdir(exist_ok=True)
N_FIXED = 100
UPD_SWEEP = [1000, 200, 50, 20]
WARMUP_S = 60.0


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
    subprocess.run([PY, str(HERE / "verify_demo.py")], env=env)
    src = CMPROJ / "rs_runstep_cm.csv"
    dst = RESULTS / f"rs_runstep_upd{upd}.csv"
    if src.exists():
        shutil.copy2(src, dst)
        print(f"  collected {dst.name}")
    else:
        print(f"  WARNING: rs_runstep_cm.csv not produced for UpdRate={upd}")


def summ(path):
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
    return dict(lib=statistics.mean(lib), full=statistics.mean(full))


if __name__ == "__main__":
    for upd in UPD_SWEEP:
        run_one(upd)
    kill_stale()
    print(f"\n===== UpdRate sweep (N_TRAFFIC={N_FIXED}, simTime > {WARMUP_S:.0f}s) =====")
    print(f"{'UpdRate':>8} {'lib_us':>8} {'full_us':>9} {'CMcore_us':>10} {'us/object':>10}")
    for upd in UPD_SWEEP:
        p = RESULTS / f"rs_runstep_upd{upd}.csv"
        s = summ(p) if p.exists() else None
        if s:
            cmcore = s['full'] - s['lib']
            print(f"{upd:>8} {s['lib']:>8.1f} {s['full']:>9.1f} {cmcore:>10.1f} {cmcore / N_FIXED:>10.2f}")
        else:
            print(f"{upd:>8} {'(no data)':>8}")
    print("\nlib_us flat across UpdRate -> the .lib (incl. position interpolation) is untouched;")
    print("it runs every refresh regardless. If CMcore_us/object drops as UpdRate falls,")
    print("UpdRate gates CarMaker's per-object road-network/envelope cost (= the lever).")
