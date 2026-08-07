"""#168 TRULY-VANILLA control: run a STOCK CarMaker example TestRun (Overtaking -- 8 normal
FreeMotion=0 AutoDriver traffic cars, Traffic.UpdRate=200 which is CarMaker's OWN default) with
the STOCK CarMaker.win64.exe from the install (zero FIXS code). If the ego overtakes -- i.e. the
traffic moves at UpdRate=200 -- then 200 is fine for NORMAL CarMaker traffic and the FIXS freeze
is specific to FreeMotion=1 teleport placeholders, NOT a CarMaker-core UpdRate=200 limit.

  python diag_stock.py
"""
import subprocess
import re
import threading
import time
import pathlib

CM = pathlib.Path(r"C:/IPG/carmaker/win64-13.1.3")
EXE = CM / "bin" / "CarMaker.win64.exe"
TR = "Examples/BasicFunctions/Driver/Overtaking"

out: list[str] = []
print(f"stock exe:     {EXE}")
print(f"stock TestRun: {TR}  (Traffic.UpdRate=200, FreeMotion=0, 8 cars)")
cm = subprocess.Popen(
    [str(EXE), "-screen", f"Data/TestRun/{TR}"],
    cwd=str(CM), stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)


def pump():
    for line in cm.stdout:
        out.append(line.rstrip())


threading.Thread(target=pump, daemon=True).start()
t0 = time.time()
while time.time() - t0 < 180:
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
print(f"\nran={ran}, ego={dist:.1f} m -> {'MOVES (200 OK for normal traffic)' if dist > 50 else ('FROZEN' if ran else 'no SIM_END -- see lines below')}")
for l in out:
    if any(k in l for k in ("SIM_END", "ERROR", "Abort", "Failed", "tart", "dle", "icense")):
        print(f"  {l}")
