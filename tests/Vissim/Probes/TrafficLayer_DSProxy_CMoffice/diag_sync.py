"""#168 sync/accuracy check: does UpdRate=200 (refresh=1000) render the traffic LESS accurately
than UpdRate=1000? Runs both, collects rs_cm_pos.csv (drawnX = CarMaker-rendered, targetX =
.lib-intended), and reports robust gap stats (median, p95, mean) -- median/p95 are insensitive to
the rare entry transients and loop-seam wraps that inflate the raw max.

If 200 and 1000 have ~the same median/p95 gap, the rate difference causes no real sync issue.

  python diag_sync.py   (RS_DEBUG exe; config refresh=0.001, SimEnd short)
"""
import os
import csv
import shutil
import subprocess
import sys
import pathlib
import statistics
import collections

HERE = pathlib.Path(__file__).resolve().parent
REPO = HERE.parents[3]
CMPROJ = REPO / "ProprietaryFiles" / "CM13_proj"
PY = sys.executable
RESULTS = HERE / "RS_tmp"
RESULTS.mkdir(exist_ok=True)
UPDS = [1000, 200]
ENTRY_SETTLE = 2.0   # s -- drop each vehicle's first samples (entry/teleport transient)
WRAP = 387.0         # m -- gaps bigger than half the ~774 m loop are seam wraps, not error


def run(upd: int) -> None:
    env = dict(os.environ, RS_N_TRAFFIC="50", RS_UPD_RATE=str(upd))
    subprocess.run([PY, str(HERE / "build_testrun.py")], env=env, check=True, capture_output=True)
    try:
        (CMPROJ / "rs_cm_pos.csv").unlink()
    except FileNotFoundError:
        pass
    subprocess.run([PY, str(HERE / "verify_demo.py")], env=env, capture_output=True, text=True)
    src = CMPROJ / "rs_cm_pos.csv"
    if src.exists():
        shutil.copy2(src, RESULTS / f"rs_cm_pos_upd{upd}.csv")


def gaps(upd: int):
    p = RESULTS / f"rs_cm_pos_upd{upd}.csv"
    if not p.exists():
        return None
    byv = collections.defaultdict(list)
    for r in csv.DictReader(open(p)):
        byv[r["vissimId"]].append((float(r["simTime"]), float(r["drawnX"]), float(r["targetX"])))
    out = []
    for v, rows in byv.items():
        rows.sort()
        t0 = rows[0][0]
        for (t, d, x) in rows:
            if t - t0 < ENTRY_SETTLE:      # skip entry transient
                continue
            g = abs(d - x)
            if g > WRAP:                   # skip loop-seam wrap
                continue
            out.append(g)
    return out


if __name__ == "__main__":
    for upd in UPDS:
        print(f"=== running UpdRate={upd} ===", flush=True)
        run(upd)
    print("\n===== SYNC/ACCURACY: rendered-vs-intended gap (entry transient + seam wrap excluded) =====")
    print(f"{'UpdRate':>8} {'samples':>9} {'median_m':>9} {'p95_m':>8} {'mean_m':>8}")
    for upd in UPDS:
        g = gaps(upd)
        if not g:
            print(f"{upd:>8} {'(no data)':>9}")
            continue
        g.sort()
        p95 = g[int(0.95 * len(g))]
        print(f"{upd:>8} {len(g):>9} {statistics.median(g):>9.3f} {p95:>8.3f} {statistics.mean(g):>8.3f}")
    print("\n  ~equal median/p95 between 1000 and 200 => refresh!=UpdRate causes no real sync error.")
