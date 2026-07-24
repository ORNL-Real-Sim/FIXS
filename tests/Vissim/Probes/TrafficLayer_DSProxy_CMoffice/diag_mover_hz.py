"""#168 follow-up to diag_mover: hold UpdRate=200 and sweep the mover's WRITE rate (RS_MOVER_HZ)
to test whether the .lib's freeze is caused by throttling t_0 writes to TrafficRefreshRate.

  RS_MOVER_HZ unset/0 = write t_0 every step (~1 kHz, the plain mover that TRACKS at 200)
  RS_MOVER_HZ=1000/200/100 = throttle writes to that rate

If 1000/every-step track but 200/100 go flat -> CarMaker at UpdRate=200 needs t_0 refreshed
faster than it consumes; the .lib's matched-refresh throttling is the bug, fix = write every step.

  python diag_mover_hz.py   (needs the exe rebuilt with the RS_MOVER_HZ mover)
"""
import os
import re
import csv
import shutil
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
TESTRUN = "SimpleLoop_rs"
PY = sys.executable
RESULTS = HERE / "RS_tmp"
RESULTS.mkdir(exist_ok=True)
HZS = os.environ.get("RS_HZS", "0,1000,200,100").split(",")   # write rates to sweep; 0 = every step


def make_cfg() -> None:
    cfg = (HERE / "config.yaml").read_text(encoding="utf-8")
    cfg = re.sub(r"EnableCosimulation: \w+", "EnableCosimulation: false", cfg)
    cfg = re.sub(r"SimulationEndTime: \d+", "SimulationEndTime: 120", cfg)
    RUNCFG.write_text(cfg, encoding="ascii")


def run(hz: str) -> bool:
    env = dict(os.environ, RS_N_TRAFFIC="1", RS_UPD_RATE=os.environ.get("RS_UPD", "200"),
               RS_RUN_SECONDS="30", RS_TEST_MOVER="1", RS_MOVER_HZ=hz)
    subprocess.run([PY, str(HERE / "build_testrun.py")], env=env, check=True, capture_output=True)
    try:
        (CMPROJ / "rs_mover.csv").unlink()
    except FileNotFoundError:
        pass
    out: list[str] = []
    cm = subprocess.Popen(
        [str(CMEXE), "-screen", "-f", str(RUNCFG), f"Data/TestRun/{TESTRUN}"],
        cwd=str(CMPROJ), stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1, env=env)
    threading.Thread(target=lambda: [out.append(l.rstrip()) for l in cm.stdout], daemon=True).start()
    t0 = time.time()
    while time.time() - t0 < 120:
        if cm.poll() is not None:
            break
        time.sleep(1)
    if cm.poll() is None:
        cm.kill()
    src = CMPROJ / "rs_mover.csv"
    if src.exists():
        shutil.copy2(src, RESULTS / f"rs_mover_hz{hz}.csv")
        return True
    return False


if __name__ == "__main__":
    make_cfg()
    print("UpdRate=200 fixed; sweeping the mover WRITE rate (RS_MOVER_HZ)\n")
    for hz in HZS:
        ok = run(hz)
        print(f"RS_MOVER_HZ={hz:>5} ({'every step' if hz == '0' else hz + ' Hz'}): "
              f"rs_mover.csv {'collected' if ok else 'NOT produced'}", flush=True)

    print("\n===== WRITE-RATE SWEEP at UpdRate=200 (readback = CarMaker applied pos) =====")
    for hz in HZS:
        p = RESULTS / f"rs_mover_hz{hz}.csv"
        if not p.exists():
            print(f"  hz={hz}: no data")
            continue
        rows = list(csv.DictReader(open(p)))
        if not rows:
            print(f"  hz={hz}: empty")
            continue
        rb = [float(r["readback"]) for r in rows]
        moved = max(rb) - min(rb)
        label = "every step" if hz == "0" else f"{hz} Hz"
        print(f"  write {label:>10}: readback moved {moved:7.1f} m -> {'TRACKS' if moved > 50 else 'FLAT (frozen)'}")
    print("\n  If throttled (200/100) goes flat while every-step/1000 tracks -> the .lib's")
    print("  refresh-throttling is the freeze cause; fix = write t_0 every step at low UpdRate.")
