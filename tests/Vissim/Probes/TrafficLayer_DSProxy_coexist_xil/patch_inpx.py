"""
Stage of the .inpx patching step for the integrated B+C regression test.

Mirrors PTV's shipped DS example .inpx into a writable directory and
hooks the FIXS DriverModel onto vehicle type 100 (Car), pointing at
coexist_par.yaml (which has EnableRealSim: false).

Run by run_coexist_xil.bat before TrafficLayer starts.
"""

from __future__ import annotations

import shutil
import sys
from pathlib import Path
from xml.etree import ElementTree as ET

HERE = Path(__file__).resolve().parent
REPO_ROOT = HERE.parents[3]   # tests/Vissim/Probes/<probe>/ -> repo root

PTV_EXAMPLE = Path(
    r"C:\Program Files\PTV Vision\PTV Vissim 2022\API\DrivingSimulator_DLL\example\DrivingSimulatorTextClient\data"
)
FIXS_DRIVERMODEL = REPO_ROOT / "ProprietaryFiles" / "VISSIMserver" / "x64" / "Release" / "DriverModel_RealSim.dll"
PAR_FILE = HERE / "coexist_par.yaml"
STAGE = HERE / "stage_network"

DM_FLAGGED_TYPE_NO = 100   # Car


def stage_files() -> Path:
    STAGE.mkdir(parents=True, exist_ok=True)
    for name in ("driving_simulator_test.inpx", "driving_simulator_test.layx",
                 "driving_simulator_test.fzp",  "driving_simulator_test.pp",
                 "CARRE4E_RO_500_1.sig"):
        src = PTV_EXAMPLE / name
        if src.is_file():
            shutil.copy2(src, STAGE / name)
    inpx = STAGE / "driving_simulator_test.inpx"
    if not inpx.is_file():
        sys.exit(f"ERROR: staging failed; no .inpx at {inpx}")
    return inpx


def patch_inpx_attach_drivermodel(inpx_path: Path) -> int:
    if not FIXS_DRIVERMODEL.is_file():
        sys.exit(f"ERROR: FIXS DriverModel not built at {FIXS_DRIVERMODEL}\n"
                 f"Run: scripts/dispatch/3_vissim_components.bat first")
    if not PAR_FILE.is_file():
        sys.exit(f"ERROR: par-file missing at {PAR_FILE}")

    tree = ET.parse(inpx_path)
    root = tree.getroot()

    patched = 0
    for vt in root.iter("vehicleType"):
        if vt.get("no") == str(DM_FLAGGED_TYPE_NO):
            vt.set("extDriver", "true")
            vt.set("extDriverDLLFile", str(FIXS_DRIVERMODEL))
            vt.set("extDriverParFile", str(PAR_FILE))
            patched += 1

    tree.write(inpx_path, encoding="utf-8", xml_declaration=True)
    return patched


def main() -> int:
    inpx = stage_files()
    n = patch_inpx_attach_drivermodel(inpx)
    if n != 1:
        sys.exit(f"ERROR: expected exactly one vehicleType with no={DM_FLAGGED_TYPE_NO}, patched {n}")
    print(f"[patch_inpx] staged {inpx}")
    print(f"[patch_inpx] hooked FIXS DriverModel ({FIXS_DRIVERMODEL.name}) on type {DM_FLAGGED_TYPE_NO} (Car)")
    print(f"[patch_inpx] par-file: {PAR_FILE} (EnableRealSim: false)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
