"""
Regenerate the SimpleEcho VISSIM network (.inpx/.layx) from the OpenDRIVE loop.

This re-creates the scratch importer that originally produced simple_loop.inpx
but was never committed (only its output was, in 156 commit 256be392). It is
restored here so the xodr -> inpx step is reproducible -- needed now that the
SUMO loop corners were rounded (gen_rounded_loop.py) and the whole chain must
be regenerated from the new geometry.

Pipeline:  simple_loop.net.xml --netconvert--> simple_loop.xodr --THIS--> .inpx

Steps (VISSIM 2022 via COM):
  1. Dispatch VISSIM.Vissim.2200, New() -> blank network
  2. ImportOpenDrive(simple_loop.xodr) -> links from the rounded SUMO loop
  3. Add a 600 veh/h vehicle input (composition 1 = "Default") on the first
     non-connector link, so background traffic flows for the co-sim demo
  4. SaveNetAs(simple_loop.inpx) + SaveLayoutAs(simple_loop.layx)

The DSProxy probe then derives simple_loop_ds.inpx from this via
patch_ds_inpx.py (drivSimActive) + recolor_network.py (blue background, red ego
type 110).

Run (conda env realsim_dev with pywin32; VISSIM 2022 installed):
  python setup_network.py
"""
from __future__ import annotations
import pathlib
import sys

import pythoncom
import win32com.client

HERE = pathlib.Path(__file__).resolve().parent
XODR = HERE / "simple_loop.xodr"
INPX = HERE / "simple_loop.inpx"
LAYX = HERE / "simple_loop.layx"
PROGID = "VISSIM.Vissim.2200"
VOLUME = 600  # veh/h background demand


def main() -> int:
    if not XODR.is_file():
        print(f"ERROR: {XODR} not found")
        return 1

    pythoncom.CoInitialize()
    print(f"[setup] Dispatching {PROGID} ...")
    vissim = win32com.client.Dispatch(PROGID)
    try:
        vissim.New()
        print(f"[setup] ImportOpenDrive({XODR.name}) ...")
        vissim.ImportOpenDrive(str(XODR))

        links = list(vissim.Net.Links.GetAll())
        # First non-connector link carries the demand (connectors have IsConn=1).
        target = next((lk for lk in links
                       if not bool(lk.AttValue("IsConn"))), None)
        if target is None:
            print("ERROR: no non-connector link found after import")
            return 2

        vi = vissim.Net.VehicleInputs.AddVehicleInput(1, target)
        vi.SetAttValue("Volume(1)", VOLUME)
        vi.SetAttValue("VehComp(1)", 1)
        print(f"[setup] {len(links)} links; {VOLUME} veh/h input on link "
              f"{int(target.AttValue('No'))}")

        for f in (INPX, LAYX):          # delete so SaveNetAs/SaveLayoutAs won't prompt
            if f.exists():
                f.unlink()
        vissim.SaveNetAs(str(INPX))
        try:
            vissim.SaveLayout(str(LAYX))
            print(f"[setup] wrote {INPX.name} + {LAYX.name}")
        except Exception as e:   # layout is cosmetic; the .inpx is what matters
            print(f"[setup] wrote {INPX.name} (layout save skipped: {e})")
    finally:
        try:
            vissim.Exit()
        except Exception:
            pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
