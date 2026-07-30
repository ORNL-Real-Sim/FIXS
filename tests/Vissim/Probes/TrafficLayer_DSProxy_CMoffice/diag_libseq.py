"""#168 .lib SEQUENCE proof (triple-check): run the real co-sim at refresh=200 (=UpdRate, freezes)
and refresh=1000 (the fix), UpdRate=200, and dump rs_libseq.csv -- an unmapped slot's z (t_0[2])
every runStep over simTime [1.0,1.2]. The .lib re-parks z=-5000 each refresh; if at refresh=200
the held z drifts off -5000 (write reset by CarMaker) while at 1000 it holds, the aliasing is
proven on the .lib's OWN write path, identical to the User.c mover.

  python diag_libseq.py   (RS_DEBUG exe; needs VISSIM)
"""
import os
import re
import csv
import shutil
import subprocess
import sys
import pathlib

HERE = pathlib.Path(__file__).resolve().parent
REPO = HERE.parents[3]
CMPROJ = REPO / "ProprietaryFiles" / "CM13_proj"
CFG = HERE / "config.yaml"
PY = sys.executable
RESULTS = HERE / "RS_tmp"
RESULTS.mkdir(exist_ok=True)
REFRESH = [("200", "0.005"), ("1000", "0.001")]   # (label Hz, TrafficRefreshRate seconds)


def set_refresh(val: str):
    txt = CFG.read_text(encoding="utf-8")
    txt = re.sub(r"TrafficRefreshRate: [\d.]+", f"TrafficRefreshRate: {val}", txt)
    txt = re.sub(r"SimulationEndTime: \d+", "SimulationEndTime: 120", txt)
    CFG.write_text(txt, encoding="ascii")


def run(hz: str, val: str):
    set_refresh(val)
    env = dict(os.environ, RS_N_TRAFFIC="50", RS_UPD_RATE="200")
    subprocess.run([PY, str(HERE / "build_testrun.py")], env=env, check=True, capture_output=True)
    try:
        (CMPROJ / "rs_libseq.csv").unlink()
    except FileNotFoundError:
        pass
    subprocess.run([PY, str(HERE / "verify_demo.py")], env=env, capture_output=True, text=True)
    src = CMPROJ / "rs_libseq.csv"
    if src.exists():
        shutil.copy2(src, RESULTS / f"rs_libseq_{hz}.csv")


if __name__ == "__main__":
    for hz, val in REFRESH:
        print(f"=== co-sim refresh={hz} Hz (TrafficRefreshRate={val}), UpdRate=200 ===", flush=True)
        run(hz, val)

    for hz, _ in REFRESH:
        p = RESULTS / f"rs_libseq_{hz}.csv"
        print(f"\n===== .lib re-park z trace, refresh={hz} Hz =====")
        if not p.exists():
            print("  no data (slot 0 may have mapped, or no run)")
            continue
        rows = list(csv.DictReader(open(p)))
        print(f"  {'simTime':>8} {'z_readback':>11} {'slot':>7} {'didRef':>6}")
        for r in rows[:20]:
            print(f"  {float(r['simTime']):>8.4f} {float(r['z_readback']):>11.2f} {r['slot']:>7} {r['didRefresh']:>6}")
        z = [float(r['z_readback']) for r in rows]
        zmean = sum(z) / len(z)
        held = abs(zmean - (-5000.0)) < 50.0
        print(f"  -> z over window: {min(z):.1f}..{max(z):.1f}, mean {zmean:.1f}  "
              f"({'HOLDS -5000 (re-park applied)' if held else 'NOT at -5000 -> re-park DISCARDED (write not applied)'})")
