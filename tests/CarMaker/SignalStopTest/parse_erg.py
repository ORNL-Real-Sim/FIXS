"""
Minimal CarMaker ERG reader for the signal-stop test.

ERG layout (confirmed empirically on this install): a 16-byte header, then fixed-size
records. The companion <erg>.info lists File.At.<n>.Name / .Type in stream order
(Double=8, Float=4, Int / "4 Bytes"=4, little-endian).

Usage:  parse_erg.py <run.erg> [stop_s] [head_s]
Prints the speed / road-position / brake / traffic-light-state profile and a PASS/FAIL
verdict on where the ego came to rest relative to the stop line.
"""
from __future__ import annotations
import sys, struct, re, pathlib

FM = {"Double": "d", "Float": "f", "Int": "i", "UInt": "I", "4 Bytes": "I", "8 Bytes": "q"}


def read_erg(path):
    erg = pathlib.Path(path)
    info = (erg.parent / (erg.name + ".info")).read_text(errors="ignore")
    names = {int(m.group(1)): m.group(2).strip()
             for m in re.finditer(r"File\.At\.(\d+)\.Name = (.+)", info)}
    types = {int(m.group(1)): m.group(2).strip()
             for m in re.finditer(r"File\.At\.(\d+)\.Type = (.+)", info)}
    order = [i for i in sorted(names) if i in types]
    fmt = "<" + "".join(FM[types[i]] for i in order)
    rec = struct.calcsize(fmt)
    data = erg.read_bytes()
    hdr = len(data) % rec
    idx = {names[i]: k for k, i in enumerate(order)}
    rows = [struct.unpack_from(fmt, data, off)
            for off in range(hdr, len(data) - rec + 1, rec)]
    return idx, rows


def main():
    path = sys.argv[1]
    stop_s = float(sys.argv[2]) if len(sys.argv) > 2 else None
    head_s = float(sys.argv[3]) if len(sys.argv) > 3 else None
    idx, rows = read_erg(path)
    iT = idx.get("Time"); iV = idx.get("Car.v"); iS = idx.get("Vhcl.sRoad")
    iB = idx.get("Brake.Hyd.Sys.pMC")
    tl = sorted(k for k in idx if k.startswith("TrfLight.") and k.endswith(".State"))
    print(f"records={len(rows)} duration={rows[-1][iT]:.2f}s")
    print(f"traffic-light state channels: {tl if tl else 'NONE LOGGED'}")
    hdr = " t      sRoad    v(m/s)  brakeMC" + ("  " + " ".join(c.split('.')[1] for c in tl) if tl else "")
    print(hdr)
    prev = None
    for r in rows:
        t = r[iT]
        if prev is None or t - prev >= 1.0 or r is rows[-1]:
            prev = t
            line = f"{t:6.2f} {r[iS]:8.2f} {r[iV]:7.2f} {r[iB]:8.2f}"
            if tl:
                line += "  " + " ".join(f"{int(r[idx[c]])}" for c in tl)
            print(line)
    vmin = min(r[iV] for r in rows)
    final_s = rows[-1][iS]
    # rest position = first sample where v<0.3 after the car has started moving
    rest = next(((r[iT], r[iS]) for r in rows if r[iT] > 2 and r[iV] < 0.3), None)
    print(f"\nmin v = {vmin:.3f} m/s   final sRoad = {final_s:.2f}")
    if rest:
        print(f"came to rest at t={rest[0]:.2f}s, sRoad={rest[1]:.2f}")
    if stop_s is not None:
        if rest and abs(rest[1] - stop_s) < 8:
            print(f"PASS: stopped at the stop line (~{stop_s} m), head is downstream at {head_s} m")
        elif rest and head_s is not None and abs(rest[1] - head_s) < 8:
            print(f"PARTIAL: stopped AT THE HEAD ({head_s} m), not the stop line -> DrvStop ignored, driver stops at head")
        elif not rest:
            print(f"FAIL: never stopped (drove through); light not red or not considered")
        else:
            print(f"NOTE: stopped at {rest[1]:.2f} m (stop line {stop_s}, head {head_s})")


if __name__ == "__main__":
    main()
