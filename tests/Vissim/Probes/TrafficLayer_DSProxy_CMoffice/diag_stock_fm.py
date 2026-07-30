"""#168 ISOLATION control: take the STOCK CarMaker Overtaking example and flip ONLY FreeMotion
(0->1) and/or UpdRate -- holding the road, AutoDriver, route, maneuvers, and ego identical -- and
run each with the STOCK install exe (zero FIXS). This isolates FreeMotion as the single variable.

  FM0 @ 200  = known-good baseline (moves ~5940 m)
  FM1 @ 200  = the question: does flipping ONLY FreeMotion freeze it?
  FM1 @ 1000 = control: does FreeMotion=1 work at high UpdRate?

If FM0@200 moves AND FM1@200 freezes AND FM1@1000 moves -> FreeMotion=1 is the isolated cause of
the low-UpdRate freeze, in pure vanilla CarMaker, no FIXS involved.

  python diag_stock_fm.py
"""
import subprocess
import re
import threading
import time
import pathlib

CM = pathlib.Path(r"C:/IPG/carmaker/win64-13.1.3")
EXE = CM / "bin" / "CarMaker.win64.exe"
SRC = CM / "Data" / "TestRun" / "Examples" / "BasicFunctions" / "Driver" / "Overtaking"
TRDIR = CM / "Data" / "TestRun"


def make_variant(name: str, fm: int, upd: int) -> None:
    txt = SRC.read_text(encoding="utf-8", errors="replace")
    txt = re.sub(r"(Traffic\.\d+\.FreeMotion = )\d", lambda m: m.group(1) + str(fm), txt)
    txt = re.sub(r"(Traffic\.\d+\.UpdRate = )\d+", lambda m: m.group(1) + str(upd), txt)
    (TRDIR / name).write_text(txt, encoding="ascii", errors="replace")


def run(name: str):
    out: list[str] = []
    cm = subprocess.Popen(
        [str(EXE), "-screen", f"Data/TestRun/{name}"],
        cwd=str(CM), stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)

    def pump():
        for line in cm.stdout:
            out.append(line.rstrip())
    threading.Thread(target=pump, daemon=True).start()
    t0 = time.time()
    while time.time() - t0 < 200:
        if cm.poll() is not None:
            break
        time.sleep(1)
    if cm.poll() is None:
        cm.kill()
    dist = -1.0
    for l in out:
        if "SIM_END" in l:
            m = re.search(r"([\d.]+)\s*m\b", l)
            if m:
                dist = float(m.group(1))
    return dist, "SIM_END" in "\n".join(out)


if __name__ == "__main__":
    print(f"stock exe: {EXE}\nbase TestRun: {SRC.name} (only FreeMotion + UpdRate are changed)\n")
    cases = [(0, 200), (1, 200), (1, 1000)]
    res = {}
    for fm, upd in cases:
        name = f"zz_FM{fm}_U{upd}"
        try:
            make_variant(name, fm, upd)
        except PermissionError:
            print(f"PermissionError writing {name} to install Data/TestRun -- need a writable TestRun dir.")
            break
        dist, ran = run(name)
        res[(fm, upd)] = (dist, ran)
        verdict = "MOVES" if dist > 50 else ("FROZEN" if ran else "no SIM_END")
        print(f"FreeMotion={fm}, UpdRate={upd:>4}: ran={ran} ego={dist:8.1f} m -> {verdict}", flush=True)
        try:
            (TRDIR / name).unlink()
        except OSError:
            pass

    print("\n===== ISOLATION (stock Overtaking, ONLY FreeMotion/UpdRate changed) =====")
    for fm, upd in cases:
        if (fm, upd) in res:
            d, r = res[(fm, upd)]
            print(f"  FreeMotion={fm}  UpdRate={upd:>4}:  {d:8.1f} m")
    print("  FM0@200 moves + FM1@200 freezes + FM1@1000 moves => FreeMotion=1 is the isolated")
    print("  low-UpdRate cause, in pure vanilla CarMaker. Otherwise my claim is wrong.")
