"""#168 SEQUENCE proof: dump the per-step producer/consumer handshake (rs_moverseq.csv) at a
FROZEN write rate vs a TRACKING one, both UpdRate=200. Shows directly whether CarMaker's held
value (readback) advances between the mover's writes, or stalls -- proving the aliasing in code.

  readback = t_0[0] CarMaker holds at User_Calc entry (= result of its last UpdRate update)
  target   = the position the mover is about to write
  did_write= 1 if the throttle wrote t_0 this step

  python diag_seq.py
"""
import os
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
import re
HZS = ["200", "210", "400", "500"]   # 200=1x freeze, 210 track, 400=2x freeze, 500=2.5x track


def make_cfg():
    cfg = (HERE / "config.yaml").read_text(encoding="utf-8")
    cfg = re.sub(r"EnableCosimulation: \w+", "EnableCosimulation: false", cfg)
    cfg = re.sub(r"SimulationEndTime: \d+", "SimulationEndTime: 120", cfg)
    RUNCFG.write_text(cfg, encoding="ascii")


def run(hz):
    env = dict(os.environ, RS_N_TRAFFIC="1", RS_UPD_RATE="200",
               RS_RUN_SECONDS="5", RS_TEST_MOVER="1", RS_MOVER_HZ=hz)
    subprocess.run([PY, str(HERE / "build_testrun.py")], env=env, check=True, capture_output=True)
    try:
        (CMPROJ / "rs_moverseq.csv").unlink()
    except FileNotFoundError:
        pass
    cm = subprocess.Popen([str(CMEXE), "-screen", "-f", str(RUNCFG), f"Data/TestRun/{TESTRUN}"],
                          cwd=str(CMPROJ), stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, env=env)
    t0 = time.time()
    while time.time() - t0 < 90:
        if cm.poll() is not None:
            break
        time.sleep(1)
    if cm.poll() is None:
        cm.kill()
    src = CMPROJ / "rs_moverseq.csv"
    if src.exists():
        shutil.copy2(src, RESULTS / f"rs_moverseq_hz{hz}.csv")
        return True
    return False


if __name__ == "__main__":
    make_cfg()
    for hz in HZS:
        run(hz)
    for hz in HZS:
        p = RESULTS / f"rs_moverseq_hz{hz}.csv"
        print(f"\n===== write {hz} Hz @ UpdRate=200  (first steps) =====")
        if not p.exists():
            print("  no data")
            continue
        rows = list(csv.DictReader(open(p)))
        print(f"  {'t':>8} {'dt':>7} {'readback':>9} {'target':>8} {'wrote':>5}")
        for r in rows[:28]:
            print(f"  {float(r['t']):>8.4f} {float(r['dt']):>7.4f} {float(r['readback']):>9.4f} {float(r['target']):>8.4f} {r['did_write']:>5}")
        rb = [float(r['readback']) for r in rows]
        print(f"  -> readback over window: {min(rb):.3f}..{max(rb):.3f} m ({'STALLS' if max(rb)-min(rb) < 1 else 'advances'})")
