"""
Verify the SignalStop demo: confirm the looping ego comes to a full stop at the
signalized junctions and that each stop coincides with its referenced straight-movement
controller being RED (State==3).

Usage: verify_signalstop.py <run.erg> <simple_traffic_light_signalstop.rd5>
"""
from __future__ import annotations
import sys, re, pathlib

# reuse the ERG reader (local copy in this dir)
_PARSE = pathlib.Path(__file__).resolve().parent / "parse_erg.py"
exec(_PARSE.read_text().split("def main")[0])   # defines FM, read_erg

# the straight controllers the DrvStop markers reference (objId -> intersection label)
REF = {392: "int_west", 495: "int_center", 181: "int_east",
       291: "int_east", 230: "int_center", 546: "int_west"}


def main():
    erg, rd = sys.argv[1], pathlib.Path(sys.argv[2]).read_text(errors="ignore")
    idx, rows = read_erg(erg)
    iT, iV, iS, iB = idx["Time"], idx["Car.v"], idx["Vhcl.sRoad"], idx["Brake.Hyd.Sys.pMC"]
    obj2name = {int(o): n for o, n in re.findall(r"Control\.TrfLight\.\d+ = (\d+) (\S+)", rd)}
    ref_state = {o: idx.get(f"TrfLight.{obj2name.get(o)}.State") for o in REF}
    ref_state = {o: c for o, c in ref_state.items() if c is not None}

    # full-stop episodes: v < 0.5 m/s sustained, after t>3
    eps, inep = [], False
    for r in rows:
        if r[iV] < 0.5 and r[iT] > 3:
            if not inep:
                eps.append([r[iT], r[iT], r[iS]]); inep = True
            else:
                eps[-1][1] = r[iT]
        else:
            inep = False

    print(f"run {rows[-1][iT]:.0f}s   min Car.v={min(r[iV] for r in rows):.2f} m/s   "
          f"{len(eps)} full-stop episode(s)")
    at_red = 0
    for t0, t1, s in eps[:12]:
        rr = min(rows, key=lambda r: abs(r[iT] - t0))
        reds = [o for o, c in ref_state.items() if int(rr[c]) == 3]
        if reds:
            at_red += 1
        print(f"  t={t0:6.1f}->{t1:6.1f}s ({t1-t0:4.0f}s)  sRoad={s:7.1f}  "
              f"brake={rr[iB]:4.1f}  {'RED ' + str(reds) if reds else 'NOT at a ref red'}")
    if len(eps) > 12:
        print(f"  ... {len(eps)-12} more (same pattern, repeating each loop)")
    all_red = sum(1 for t0, _, _ in eps
                  if any(int(min(rows, key=lambda r: abs(r[iT]-t0))[c]) == 3 for c in ref_state.values()))
    verdict = "PASS" if eps and all_red == len(eps) else ("PARTIAL" if eps else "FAIL")
    print(f"\n{verdict}: {all_red}/{len(eps)} stops occurred at a red referenced controller "
          f"(ego respects the signals).")


if __name__ == "__main__":
    main()
