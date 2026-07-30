"""#168 FREEZE MECHANISM diagnostic. Runs the matched co-sim at a FROZEN rate (500) and a
MOVING rate (600), both with RS_DEBUG instrumentation, to find WHERE/WHY low-UpdRate traffic
freezes. The probe (rs_freeze.csv, written in VirEnvHelper.cpp's refresh loop) logs, every
refresh for simTime<3s: CarMaker's t_0 BEFORE the .lib overwrites it (t0_pre = what CarMaker
left after its own traffic update) vs the position the .lib is about to teleport to (t0_set).

Reading the output (RS_tmp/rs_freeze_R{500,600}.csv):
  - if, per vehicle, t0_pre at refresh N != t0_set at refresh N-1 (t0_pre static while t0_set
    advances) -> CarMaker CLOBBERS the teleport each step = the freeze.
  - if t0_pre tracks t0_set -> teleport survives; freeze (if any) is elsewhere.

  RS_DEBUG_BUILD=1 python diag_freeze.py
(needs RS_DEBUG_BUILD so the headless exe is instrumented; edits config.yaml per run + restores)
"""
import os
import re
import shutil
import subprocess
import sys
import pathlib

HERE = pathlib.Path(__file__).resolve().parent
REPO = HERE.parents[3]
CMPROJ = REPO / "ProprietaryFiles" / "CM13_proj"
PY = sys.executable
CFG = HERE / "config.yaml"
RESULTS = HERE / "RS_tmp"
RESULTS.mkdir(exist_ok=True)
RATES = [500, 600]  # frozen, moving (cliff is between 500 and 550)


def set_config(refresh: float) -> None:
    txt = CFG.read_text()
    txt = re.sub(r"SimulationEndTime: \d+", "SimulationEndTime: 120", txt)
    if "TrafficRefreshRate:" in txt:
        txt = re.sub(r"TrafficRefreshRate: [\d.]+", f"TrafficRefreshRate: {refresh:.6f}", txt)
    else:
        txt = txt.replace(
            "EnableCosimulation: true",
            f"EnableCosimulation: true\n    TrafficRefreshRate: {refresh:.6f}", 1)
    CFG.write_text(txt)


if __name__ == "__main__":
    if not os.environ.get("RS_DEBUG_BUILD"):
        print("WARNING: RS_DEBUG_BUILD not set -- rs_freeze.csv will NOT be produced.")
    orig = CFG.read_text()
    try:
        for R in RATES:
            print(f"\n===== freeze diag: matched {R}/{R} ({'FROZEN' if R < 550 else 'MOVING'}) =====", flush=True)
            set_config(1.0 / R)
            env = dict(os.environ, RS_N_TRAFFIC="50", RS_UPD_RATE=str(R))
            subprocess.run([PY, str(HERE / "build_testrun.py")], env=env, check=True,
                           capture_output=True)
            for f in ("rs_freeze.csv", "rs_cm_pos.csv", "rs_slots.csv"):
                try:
                    (CMPROJ / f).unlink()
                except FileNotFoundError:
                    pass
            r = subprocess.run([PY, str(HERE / "verify_demo.py")], env=env,
                               capture_output=True, text=True)
            dist = -1.0
            for line in (r.stdout + r.stderr).splitlines():
                if "SIM_END" in line:
                    m = re.search(r"([\d.]+)\s*m\b", line)
                    if m:
                        dist = float(m.group(1))
            print(f"  ego={dist:.1f}m ({'MOVES' if dist > 50 else 'FROZEN'})")
            for f in ("rs_freeze.csv", "rs_cm_pos.csv", "rs_slots.csv"):
                src = CMPROJ / f
                if src.exists():
                    dst = RESULTS / f"{f[:-4]}_R{R}.csv"
                    shutil.copy2(src, dst)
                    print(f"  collected {dst.name} ({src.stat().st_size} bytes)")
                else:
                    print(f"  WARNING: {f} not produced (RS_DEBUG off, or no traffic)")
    finally:
        CFG.write_text(orig)
        print("\nconfig.yaml restored.")
