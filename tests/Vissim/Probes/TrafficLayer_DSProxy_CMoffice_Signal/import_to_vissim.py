"""
Import an OpenDRIVE (.xodr) corridor into VISSIM 2022 and save it as a .inpx,
via COM. The repo had no committed OpenDRIVE->.inpx importer (the #168 network
was imported by hand in the GUI); this scripts that step for #172.

VISSIM 2022 COM (verified against the live type library):
    ImportOpenDrive(openDriveFile, leftHandTraffic)
    SaveNetAs(NetPath, nonDefaultOnly)

Win11 26100+ dispatch fragility (FIXS#152): force STA + CLSCTX_LOCAL_SERVER, and
the License Manager must have been Started + VISSIM launched once this session
(see CLAUDE.md / vissim-dispatch-warmup memory) or Dispatch fails 0x80080005.

Usage:  python import_to_vissim.py <in.xodr> <out.inpx>
Paths are resolved to absolute (VISSIM COM needs absolute paths).
"""
from __future__ import annotations
import sys
import pathlib
import pythoncom
import win32com.client

PROGID = "VISSIM.Vissim.2200"


def main() -> int:
    if len(sys.argv) < 3:
        raise SystemExit(__doc__)
    xodr = pathlib.Path(sys.argv[1]).resolve()
    inpx = pathlib.Path(sys.argv[2]).resolve()
    if not xodr.is_file():
        raise SystemExit(f"ERROR: xodr not found: {xodr}")
    inpx.parent.mkdir(parents=True, exist_ok=True)

    # FIXS#152 dispatch workaround
    pythoncom.CoInitializeEx(pythoncom.COINIT_APARTMENTTHREADED)
    print(f"[import] dispatching {PROGID} ...", file=sys.stderr)
    v = win32com.client.Dispatch(PROGID, clsctx=pythoncom.CLSCTX_LOCAL_SERVER)
    print("[import] dispatch OK", file=sys.stderr)

    try:
        print(f"[import] ImportOpenDrive({xodr.name}, leftHandTraffic=False)", file=sys.stderr)
        v.ImportOpenDrive(str(xodr), False)   # False = right-hand (US) traffic

        # report what came in
        try:
            n_links = v.Net.Links.Count
            print(f"[import] imported network: Links.Count = {n_links}", file=sys.stderr)
        except Exception as e:
            print(f"[import] (link count read failed: {e!r})", file=sys.stderr)

        print(f"[import] SaveNetAs({inpx})", file=sys.stderr)
        v.SaveNetAs(str(inpx), False)
        print(f"[import] saved -> {inpx}")
    finally:
        # release; let VISSIM shut down on process exit (don't taskkill -- zombies)
        v = None

    return 0


if __name__ == "__main__":
    sys.exit(main())
