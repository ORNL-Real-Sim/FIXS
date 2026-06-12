"""Perf sweep (#168): hold VISSIM fixed, vary CarMaker N_TRAFFIC (20/50/150) and
measure the real-time factor (RTF). Isolates CarMaker's per-traffic-object cost --
including the idle/parked slots beyond the active VISSIM vehicles -- from the active
vehicle count itself (which VISSIM controls and we keep unchanged).

Reuses verify_demo.py to orchestrate TrafficLayer + VISSIM + the headless CarMaker exe,
then collects the RS_DEBUG timing CSVs per N and prints an RTF summary. Requires the
RS_DEBUG builds (VE.lib + TrafficLayer.exe + CarMaker_headless.win64.exe) to be in place,
and a short config.yaml SimulationEndTime so each run finishes quickly.

  python sweep_traffic.py
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
RESULTS = HERE / "RS_tmp"          # gitignored (**/RS_tmp/*)
RESULTS.mkdir(exist_ok=True)
N_SWEEP = [20, 50, 100, 150]
WARMUP_S = 60.0                    # ignore the first WARMUP_S sim-seconds (loop filling)


def kill_stale():
    for img in ("VISSIM220.exe", "TrafficLayer.exe", "CarMaker_headless.win64.exe"):
        subprocess.run(["taskkill", "/F", "/IM", img], capture_output=True)


def run_one(n):
    print(f"\n===== SWEEP N_TRAFFIC={n} =====", flush=True)
    kill_stale()
    time.sleep(2)
    env = dict(os.environ, RS_N_TRAFFIC=str(n))
    subprocess.run([PY, str(HERE / "build_testrun.py")], env=env, check=True)
    for f in (CMPROJ / "rs_timing_cm.csv", CMPROJ / "rs_runstep_cm.csv", HERE / "rs_timing_tl.csv"):
        try:
            f.unlink()
        except FileNotFoundError:
            pass
    subprocess.run([PY, str(HERE / "verify_demo.py")], env=env)
    for src, dst in [(CMPROJ / "rs_timing_cm.csv", RESULTS / f"rs_timing_cm_N{n}.csv"),
                     (CMPROJ / "rs_runstep_cm.csv", RESULTS / f"rs_runstep_cm_N{n}.csv"),
                     (HERE / "rs_timing_tl.csv", RESULTS / f"rs_timing_tl_N{n}.csv")]:
        if src.exists():
            shutil.copy2(src, dst)
            print(f"  collected {dst.name}")
        else:
            print(f"  WARNING: {src.name} not produced for N={n}")


def summarize_cm(path):
    rts, walls, nveh = [], [], []
    with open(path) as f:
        for row in csv.DictReader(f):
            try:
                if float(row["simTime"]) < WARMUP_S:
                    continue
                rts.append(float(row["rtf"]))
                walls.append(float(row["wall_us_per_step"]))
                nveh.append(int(float(row["nVeh"])))
            except (ValueError, KeyError):
                continue
    if not rts:
        return None
    return dict(rtf=statistics.mean(rts), wall_us=statistics.mean(walls),
                nveh=int(statistics.median(nveh)), samples=len(rts))


def summarize_tl(path):
    ticks, gtv = [], []
    with open(path) as f:
        for row in csv.DictReader(f):
            try:
                ticks.append(float(row["avg_tick_us"]))
                gtv.append(float(row["avg_getTraffic_us"]))
            except (ValueError, KeyError):
                continue
    if not ticks:
        return None
    # drop the first few buckets (startup) -> tail mean
    tail = max(1, len(ticks) // 3)
    return dict(tick_us=statistics.mean(ticks[-tail:]), gtv_us=statistics.mean(gtv[-tail:]))


def summarize_runstep(path):
    lib, full, frac, nveh = [], [], [], []
    with open(path) as f:
        for row in csv.DictReader(f):
            try:
                if float(row["simTime"]) < WARMUP_S:
                    continue
                lib.append(float(row["lib_runstep_us"]))
                full.append(float(row["full_step_us"]))
                frac.append(float(row["lib_frac"]))
                nveh.append(int(float(row["nVeh"])))
            except (ValueError, KeyError):
                continue
    if not lib:
        return None
    return dict(lib_us=statistics.mean(lib), full_us=statistics.mean(full),
                frac=statistics.mean(frac), nveh=int(statistics.median(nveh)), samples=len(lib))


if __name__ == "__main__":
    for n in N_SWEEP:
        run_one(n)
    kill_stale()
    print("\n================= SWEEP SUMMARY (steady-state, simTime > %.0fs) =================" % WARMUP_S)
    print("CarMaker side (full per-step wall clock; RTF = sim/wall, 1.0 = real-time):")
    print(f"{'N_TRAFFIC':>10} {'active nVeh':>12} {'avg RTF':>10} {'wall_us/step':>14} {'samples':>9}")
    for n in N_SWEEP:
        p = RESULTS / f"rs_timing_cm_N{n}.csv"
        s = summarize_cm(p) if p.exists() else None
        if s:
            print(f"{n:>10} {s['nveh']:>12} {s['rtf']:>10.3f} {s['wall_us']:>14.1f} {s['samples']:>9}")
        else:
            print(f"{n:>10} {'(no data)':>12}")
    print("\nTrafficLayer side (DSProxy/VISSIM per-tick; unchanged VISSIM -> should be ~flat):")
    print(f"{'N_TRAFFIC':>10} {'tick_us':>10} {'getTraffic_us':>14}")
    for n in N_SWEEP:
        p = RESULTS / f"rs_timing_tl_N{n}.csv"
        s = summarize_tl(p) if p.exists() else None
        if s:
            print(f"{n:>10} {s['tick_us']:>10.1f} {s['gtv_us']:>14.1f}")
        else:
            print(f"{n:>10} {'(no data)':>10}")
    print("\n========== .lib vs CarMaker-core breakdown (the root-cause proof) ==========")
    print(f"{'N_TRAFFIC':>10} {'active':>7} {'lib_us':>9} {'full_us':>9} {'CMcore_us':>10} {'lib_%':>7}")
    for n in N_SWEEP:
        p = RESULTS / f"rs_runstep_cm_N{n}.csv"
        s = summarize_runstep(p) if p.exists() else None
        if s:
            cmcore = s['full_us'] - s['lib_us']
            print(f"{n:>10} {s['nveh']:>7} {s['lib_us']:>9.1f} {s['full_us']:>9.1f} {cmcore:>10.1f} {100*s['frac']:>6.1f}%")
        else:
            print(f"{n:>10} {'(no data)':>7}")
    print("\nReading: if RTF falls as N_TRAFFIC rises while the TL side stays flat, the cost is")
    print("CarMaker's per-traffic-object processing (the slots), not VISSIM/DSProxy.")
    print("And if lib_us stays ~flat while full_us / CMcore_us scale with N_TRAFFIC, that")
    print("cost is CarMaker CORE -- the FIXS .lib only ever touches the active vehicles.")
