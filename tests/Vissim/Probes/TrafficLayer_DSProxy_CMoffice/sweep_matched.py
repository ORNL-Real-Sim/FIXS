"""Perf sweep (#168): MATCHED refresh/UpdRate pairs. Earlier work showed 1000/1000 moves and
200/200 freezes; this finds the threshold in between, since the user's experience is that some
settings co-sim fine and some freeze. For each rate R it sets CarMakerSetup.TrafficRefreshRate
= 1/R (yaml-matched to Traffic.<i>.UpdRate=R) and runs the real headless co-sim, recording the
ego's travelled distance (the freeze test: a moving co-sim lets the ego cruise ~1600 m in 120 s;
a frozen one leaves it stalled ~3 m) AND the peak VISSIM vehicle count. A CarMaker-license
failure (peak=0 + "no valid license") is flagged separately so it is NOT misread as a freeze.

  RS_RATES="500,400,300" python sweep_matched.py   (needs short SimulationEndTime; config.yaml
  is edited in place per run and RESTORED at the end)
"""
import os
import re
import subprocess
import sys
import pathlib

HERE = pathlib.Path(__file__).resolve().parent
PY = sys.executable
CFG = HERE / "config.yaml"
RATES = [int(x) for x in os.environ.get("RS_RATES", "500,400,300").split(",")]
N_FIXED = 50
results = {}


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


def run_one(R: int):
    set_config(1.0 / R)
    env = dict(os.environ, RS_N_TRAFFIC=str(N_FIXED), RS_UPD_RATE=str(R))
    subprocess.run([PY, str(HERE / "build_testrun.py")], env=env, check=True,
                   capture_output=True)
    r = subprocess.run([PY, str(HERE / "verify_demo.py")], env=env,
                       capture_output=True, text=True)
    out = r.stdout + r.stderr
    dist, peak = -1.0, -1
    for line in out.splitlines():
        if "SIM_END" in line:
            m = re.search(r"([\d.]+)\s*m\b", line)
            if m:
                dist = float(m.group(1))
        m = re.search(r"peak vehicles=(\d+)", line)
        if m:
            peak = int(m.group(1))
    lic = "no valid license" in out.lower()
    return dist, peak, lic


def verdict(dist, peak, lic):
    if lic or (peak <= 0 and dist < 0):
        return "LICENSE (rerun)"
    return "MOVES" if dist > 50 else "FROZEN"


if __name__ == "__main__":
    orig = CFG.read_text()
    try:
        for R in RATES:
            d, p, lic = run_one(R)
            results[R] = (d, p, lic)
            print(f"  matched {R}/{R}: ego={d:.1f}m peak={p} -> {verdict(d, p, lic)}", flush=True)
    finally:
        CFG.write_text(orig)
        print("config.yaml restored.")

    print(f"\n===== MATCHED refresh/UpdRate SWEEP (N_TRAFFIC={N_FIXED}, SimEnd=120) =====")
    print(f"{'rate':>7} {'ego_m':>9} {'peak':>5} {'verdict':>16}")
    print(f"{'1000':>7} {'1606.5':>9} {'17':>5} {'MOVES (known)':>16}")
    for R in RATES:
        d, p, lic = results.get(R, (-1.0, -1, False))
        print(f"{R:>7} {d:>9.1f} {p:>5} {verdict(d, p, lic):>16}")
    print(f"{'200':>7} {'3.0':>9} {'1':>5} {'FROZEN (known)':>16}")
    print("\nThreshold = lowest matched rate that still MOVES. LICENSE rows are CarMaker")
    print("CodeMeter leaks (rerun after they linger-expire), not freezes.")
