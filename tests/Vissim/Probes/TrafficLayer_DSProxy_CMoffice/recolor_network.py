"""
Recolor the DS network so VISSIM matches the CarMaker view:
  - background vehicles -> BLUE  (match CM's IPG_CompanyCar_2018_Blue)
  - ego                 -> RED   (match the CM Ferrari)

How: in VISSIM a vehicle's color comes from its vehicle type's colorDistribution.
All road vehicle types (Car 100, HGV 200, Bus 300, Tram 400) share distribution
#1 "Default" (a random mix). We:
  1. collapse #1 to a single blue  -> all background vehicles render blue;
  2. add a red colorDistribution #2;
  3. add a DEDICATED ego vehicle type 110 "DS_Ego" (a clone of Car 100 but
     colorDistr1=2) so the ego is visually distinct (red) from the background.
config.yaml then sets CarMakerSetup.EgoType: '110' so the DSProxy creates the
ego with this type.

Idempotent: re-running detects type 110 / distribution #2 and skips. Edits the
committed asset simple_loop_ds.inpx in place.

Run:  python recolor_network.py
"""
from __future__ import annotations
import re
import pathlib
import sys

HERE = pathlib.Path(__file__).resolve().parent
INPX = HERE / "simple_loop_ds.inpx"

BLUE = "ff1543b6"   # ARGB, the blue already present in the Default distribution
RED  = "ffd62828"   # ARGB, the red already present in the Default distribution


def main() -> int:
    if not INPX.is_file():
        print(f"ERROR: {INPX} not found")
        return 1
    s = INPX.read_text(encoding="utf-8", errors="ignore")
    changed = False

    # 1. colorDistribution #1 "Default" -> single blue element.
    def collapse_blue(m: re.Match) -> str:
        return (m.group(1)
                + '\n\t\t\t\t<colorDistributionElement color="' + BLUE + '" share="1"/>\n\t\t\t'
                + m.group(2))
    s2 = re.sub(
        r'(<colorDistribution name="Default" no="1">\s*<colorDistrEl>).*?(</colorDistrEl>\s*</colorDistribution>)',
        collapse_blue, s, flags=re.S)
    if s2 != s:
        changed = True
        s = s2
        print("[recolor] collapsed Default distribution #1 -> single blue")
    else:
        print("[recolor] distribution #1 already blue (or pattern unchanged)")

    # 2. add red colorDistribution #2 right after #1 (if absent).
    if 'no="2"' not in re.search(r'<colorDistributions>.*?</colorDistributions>', s, re.S).group(0):
        red_cd = ('</colorDistribution>\n'
                  '\t\t<colorDistribution name="EgoRed" no="2">\n'
                  '\t\t\t<colorDistrEl>\n'
                  '\t\t\t\t<colorDistributionElement color="' + RED + '" share="1"/>\n'
                  '\t\t\t</colorDistrEl>\n'
                  '\t\t</colorDistribution>')
        s = s.replace('</colorDistribution>', red_cd, 1)  # after #1 (first one)
        changed = True
        print("[recolor] added red colorDistribution #2 EgoRed")
    else:
        print("[recolor] red distribution #2 already present")

    # 3. add dedicated ego vehicle type 110 (clone of Car 100, colorDistr1=2).
    if 'no="110"' not in s:
        m = re.search(r'<vehicleType\b[^>]*\bno="100".*?</vehicleType>', s, re.S)
        if not m:
            print("ERROR: could not find vehicleType 100 to clone")
            return 1
        t100 = m.group(0)
        t110 = (t100.replace('no="100"', 'no="110"')
                    .replace('name="Car"', 'name="DS_Ego"')
                    .replace('colorDistr1="1"', 'colorDistr1="2"'))
        s = s.replace(t100, t100 + "\n\t\t" + t110, 1)
        changed = True
        print("[recolor] added dedicated ego vehicleType 110 DS_Ego (red)")
    else:
        print("[recolor] ego vehicleType 110 already present")

    if changed:
        INPX.write_text(s, encoding="utf-8")
        print(f"[recolor] wrote {INPX.name}")
    else:
        print("[recolor] no changes needed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
