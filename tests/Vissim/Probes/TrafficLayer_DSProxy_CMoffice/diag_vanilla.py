"""#168 VANILLA CONTROL: run CarMaker on the SAME TestRun with the co-sim OFF, NO TrafficLayer.

EnableCosimulation=false makes the .lib inert (skips connect/recv/send -- VirEnvHelper.cpp
L241/446/826), so CarMaker runs the 50 RS_C FreeMotion+AutoDriver slots natively. We run at
UpdRate=200 (freezes WITH co-sim) and 600 (moves WITH co-sim) and read the ego distance.

Decisive control for "is the freeze our co-sim runtime, or CarMaker's own slot handling?":
  - standalone 200 MOVES (while co-sim 200 freezes) -> the co-sim RUNTIME is implicated.
  - standalone 200 FREEZES too                       -> CarMaker's native handling of the
    RS_C slots at low UpdRate (the TestRun/slot config), independent of the co-sim runtime.

  python diag_vanilla.py
"""
import os
import re
import subprocess
import sys
import pathlib
import threading
import time

HERE = pathlib.Path(__file__).resolve().parent
REPO = HERE.parents[3]
CMEXE = REPO / "ProprietaryFiles" / "CM13_proj" / "src" / "CarMaker_headless.win64.exe"
CMPROJ = REPO / "ProprietaryFiles" / "CM13_proj"
RUNCFG = HERE / "config.vanilla.yaml"
TESTRUN = "SimpleLoop_VISSIM_rs"
PY = sys.executable
RATES = [200, 600]


def make_vanilla_config() -> None:
    cfg = (HERE / "config.yaml").read_text(encoding="utf-8")
    cfg = re.sub(r"EnableCosimulation: \w+", "EnableCosimulation: false", cfg)
    cfg = re.sub(r"SimulationEndTime: \d+", "SimulationEndTime: 120", cfg)
    # network path is irrelevant with the co-sim off; leave it as-is (CarMaker ignores it)
    RUNCFG.write_text(cfg, encoding="ascii")


def run_one(R: int):
    env = dict(os.environ, RS_N_TRAFFIC="50", RS_UPD_RATE=str(R), RS_RUN_SECONDS="120")
    subprocess.run([PY, str(HERE / "build_testrun.py")], env=env, check=True, capture_output=True)
    out: list[str] = []
    cm = subprocess.Popen(
        [str(CMEXE), "-screen", "-f", str(RUNCFG), f"Data/TestRun/{TESTRUN}"],
        cwd=str(CMPROJ), stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)

    def pump():
        for line in cm.stdout:
            out.append(line.rstrip())
    threading.Thread(target=pump, daemon=True).start()
    t0 = time.time()
    while time.time() - t0 < 300:
        if cm.poll() is not None:
            break
        time.sleep(1)
    if cm.poll() is None:
        cm.kill()
    text = "\n".join(out)
    dist = -1.0
    for l in out:
        if "SIM_END" in l:
            m = re.search(r"([\d.]+)\s*m\b", l)
            if m:
                dist = float(m.group(1))
    ran = "SIM_END" in text
    return ran, dist, out


if __name__ == "__main__":
    make_vanilla_config()
    print(f"vanilla config: EnableCosimulation=false, SimEnd=120 ({RUNCFG.name})")
    res = {}
    for R in RATES:
        print(f"\n===== VANILLA (co-sim OFF) UpdRate={R} =====", flush=True)
        ran, dist, out = run_one(R)
        res[R] = (ran, dist)
        verdict = "MOVES" if dist > 50 else ("FROZEN" if ran else "DID NOT FINISH")
        print(f"  ran={ran}, ego={dist:.1f} m -> {verdict}")
        for l in out:
            if any(k in l for k in ("SIM_END", "ERROR", "RealSim", "license", "Abort", "Connect")):
                print(f"    {l}")
    print("\n===== VANILLA SUMMARY (co-sim OFF, standalone CarMaker) =====")
    for R in RATES:
        ran, dist = res[R]
        print(f"  UpdRate {R}: ego {dist:7.1f} m  ({'MOVES' if dist > 50 else 'FROZEN/incomplete'})")
    print("  COMPARE co-sim ON: 200 -> 3 m (frozen), 600 -> 1586 m (moves).")
    print("  vanilla 200 MOVES => co-sim runtime implicated; vanilla 200 FROZEN => CarMaker slot handling.")
