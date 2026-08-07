"""#168 cost proof: does UpdRate=200 (with the re-park fix + TrafficRefreshRate=0.001) actually
cost LESS than UpdRate=1000, or does the 1 kHz t_0 write re-trigger the geometry every step?

Runs the full co-sim at UpdRate=1000 and 200 (both refresh=0.001, same N_TRAFFIC), RS_DEBUG, and
compares rs_runstep_cm.csv steady-state (simTime>60):
  full_step_us  = total CarMaker step time
  lib_runstep_us = the .lib's own time (incl. the new re-park loop)
  CMcore = full - lib = CarMaker's per-object geometry -- the EXPENSIVE part UpdRate gates

If CMcore at 200 ~= CMcore at 1000 / 5 -> the geometry IS gated by UpdRate -> real ~5x saving.
If CMcore at 200 ~= CMcore at 1000     -> the 1 kHz writes re-trigger geometry -> NO saving.

  python diag_cost.py   (needs the RS_DEBUG exe with the re-park; config refresh=0.001, SimEnd short)
"""
import os
import csv
import shutil
import subprocess
import sys
import pathlib
import statistics

HERE = pathlib.Path(__file__).resolve().parent
REPO = HERE.parents[3]
CMPROJ = REPO / "ProprietaryFiles" / "CM13_proj"
PY = sys.executable
RESULTS = HERE / "RS_tmp"
RESULTS.mkdir(exist_ok=True)
UPDS = [1000, 200]
WARMUP = 60.0


def run(upd: int) -> None:
    env = dict(os.environ, RS_N_TRAFFIC="50", RS_UPD_RATE=str(upd))
    subprocess.run([PY, str(HERE / "build_testrun.py")], env=env, check=True, capture_output=True)
    try:
        (CMPROJ / "rs_runstep_cm.csv").unlink()
    except FileNotFoundError:
        pass
    subprocess.run([PY, str(HERE / "verify_demo.py")], env=env, capture_output=True, text=True)
    src = CMPROJ / "rs_runstep_cm.csv"
    if src.exists():
        shutil.copy2(src, RESULTS / f"rs_runstep_upd{upd}.csv")


def summarize(upd: int):
    p = RESULTS / f"rs_runstep_upd{upd}.csv"
    if not p.exists():
        return None
    lib, full = [], []
    for r in csv.DictReader(open(p)):
        try:
            if float(r["simTime"]) < WARMUP:
                continue
            lib.append(float(r["lib_runstep_us"]))
            full.append(float(r["full_step_us"]))
        except (ValueError, KeyError):
            continue
    if not full:
        return None
    return statistics.mean(full), statistics.mean(lib)


if __name__ == "__main__":
    for upd in UPDS:
        print(f"=== running UpdRate={upd} (refresh=0.001) ===", flush=True)
        run(upd)

    print("\n===== COST: UpdRate=1000 vs 200 (steady-state per CarMaker step, us) =====")
    print(f"{'UpdRate':>8} {'full_us':>9} {'lib_us':>9} {'CMcore_us':>10}")
    base = None
    for upd in UPDS:
        s = summarize(upd)
        if s is None:
            print(f"{upd:>8} {'(no data)':>9}")
            continue
        full, lib = s
        cmcore = full - lib
        if upd == 1000:
            base = cmcore
        print(f"{upd:>8} {full:>9.1f} {lib:>9.1f} {cmcore:>10.1f}")
    s200 = summarize(200)
    if base and s200:
        cm200 = s200[0] - s200[1]
        print(f"\n  CMcore 1000 -> 200: {base:.1f} -> {cm200:.1f} us  ({base / cm200:.1f}x cheaper)"
              if cm200 > 0 else "")
        print("  ~5x => geometry IS gated by UpdRate (real saving). ~1x => re-triggered every write (no saving).")
