"""#168 DEFINITIVE isolation: a MINIMAL external mover (User.c, gated on RS_TEST_MOVER, NO .lib,
NO VISSIM) moves Traffic 0 (a FreeMotion=1 object) at 13.9 m/s. Run at UpdRate=1000 vs 200.
rs_mover.csv logs CarMaker's read-back (applied position) vs the target each step.

  readback tracks target at BOTH 1000 and 200  -> externally-driven FreeMotion is fine at 200,
      so the .lib's 200-freeze is a FIXS implementation bug (and 200 is reachable).
  readback FLAT at 200 while it tracks at 1000  -> CarMaker simply does not apply an external
      position at UpdRate=200; the freeze is inherent to FreeMotion below ~600.

  python diag_mover.py   (needs the RS_DEBUG headless exe built with the User.c mover)
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


def make_cfg() -> None:
    cfg = (HERE / "config.yaml").read_text(encoding="utf-8")
    cfg = re.sub(r"EnableCosimulation: \w+", "EnableCosimulation: false", cfg)
    cfg = re.sub(r"SimulationEndTime: \d+", "SimulationEndTime: 120", cfg)
    RUNCFG.write_text(cfg, encoding="ascii")


def run(upd: int) -> bool:
    env = dict(os.environ, RS_N_TRAFFIC="1", RS_UPD_RATE=str(upd),
               RS_RUN_SECONDS="30", RS_TEST_MOVER="1")
    subprocess.run([PY, str(HERE / "build_testrun.py")], env=env, check=True, capture_output=True)
    try:
        (CMPROJ / "rs_mover.csv").unlink()
    except FileNotFoundError:
        pass
    out: list[str] = []
    cm = subprocess.Popen(
        [str(CMEXE), "-screen", "-f", str(RUNCFG), f"Data/TestRun/{TESTRUN}"],
        cwd=str(CMPROJ), stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1, env=env)

    def pump():
        for line in cm.stdout:
            out.append(line.rstrip())
    threading.Thread(target=pump, daemon=True).start()
    t0 = time.time()
    while time.time() - t0 < 120:
        if cm.poll() is not None:
            break
        time.sleep(1)
    if cm.poll() is None:
        cm.kill()
    src = CMPROJ / "rs_mover.csv"
    if src.exists():
        shutil.copy2(src, RESULTS / f"rs_mover_U{upd}.csv")
        return True
    return False


if __name__ == "__main__":
    make_cfg()
    print("mover: move Traffic 0 (FreeMotion=1) at 13.9 m/s, NO .lib, NO VISSIM\n")
    for upd in (1000, 200):
        ok = run(upd)
        print(f"UpdRate={upd:>4}: rs_mover.csv {'collected' if ok else 'NOT produced'}", flush=True)

    print("\n===== MOVER ISOLATION (readback = CarMaker's applied pos vs target) =====")
    for upd in (1000, 200):
        p = RESULTS / f"rs_mover_U{upd}.csv"
        if not p.exists():
            print(f"  UpdRate={upd}: no data")
            continue
        rows = list(csv.DictReader(open(p)))
        if not rows:
            print(f"  UpdRate={upd}: empty")
            continue
        rb = [float(r["readback"]) for r in rows]
        tg = [float(r["target"]) for r in rows]
        moved = max(rb) - min(rb)
        print(f"  UpdRate={upd:>4}: {len(rows)} samples | readback moved {moved:7.1f} m, "
              f"target moved {max(tg)-min(tg):7.1f} m -> {'TRACKS (moves)' if moved > 50 else 'FLAT (frozen)'}")
    print("\n  Both track => driven FreeMotion is fine at 200 => .lib 200-freeze is a FIXS bug.")
    print("  200 flat while 1000 tracks => CarMaker won't apply an external pos at 200 => inherent.")
