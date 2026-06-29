"""
Prepare the #173 eco-driving GUI demo. This is Demo 2's setup_gui.py PLUS one
step: hook the FIXS DriverModel onto Car type 100 in the staged network so the
DSProxy + DriverModel co-load actually happens.

Steps:
  1. stage the signalized DS network (simple_traffic_light.inpx + .sig + .layx)
  2. PATCH the staged .inpx: vehicleType no=100 -> extDriver=true,
     extDriverDLLFile=DriverModel_RealSim.dll, extDriverParFile=eco_par.yaml
  3. write config.runtime.yaml (NetworkFile -> staged absolute path)
  4. register the custom CarMaker exe + "-f <config> -s <RSsignalTable.csv>"
     into the CM project's GUI config (shared with Demo 2)

The CM scene + signal table themselves are Demo 2 assets (the eco demo depends
on Demo 2 per the issue); run Demo 2's build scripts first, or the one-click
run_eco_demo_gui.bat does it for you.
"""
from __future__ import annotations
import pathlib, shutil, sys
from xml.etree import ElementTree as ET

HERE = pathlib.Path(__file__).resolve().parent
REPO = HERE.parents[3]
CMPROJ = REPO / "ProprietaryFiles" / "CM13_proj"
GUICFG = CMPROJ / "Data" / "Config" / "GUI"
VIS_NET = REPO / "tests" / "Vissim" / "networks" / "simple_traffic_light"
SRC_INPX = VIS_NET / "simple_traffic_light.inpx"
STAGE = HERE / "stage_network"
RUNCFG = HERE / "config.runtime.yaml"
SIGNAL_TABLE = CMPROJ / "Data" / "Road" / "simple_traffic_light_RSsignalTable.csv"

FIXS_DRIVERMODEL = REPO / "ProprietaryFiles" / "VISSIMserver" / "x64" / "Release" / "DriverModel_RealSim.dll"
PAR_FILE = HERE / "eco_par.yaml"
DM_FLAGGED_TYPE_NO = 100  # Car -> eco-driving CAVs


def stage_network() -> None:
    STAGE.mkdir(exist_ok=True)
    if not SRC_INPX.is_file():
        raise SystemExit(f"ERROR: VISSIM network missing: {SRC_INPX}")
    shutil.copy2(SRC_INPX, STAGE / SRC_INPX.name)
    for extra in list(VIS_NET.glob("*.layx")) + list(VIS_NET.glob("int_*.sig")):
        shutil.copy2(extra, STAGE / extra.name)
    print(f"[setup_eco] staged signalized DS network (+ .sig/.layx) -> {STAGE}")


def patch_drivermodel_hook() -> None:
    if not FIXS_DRIVERMODEL.is_file():
        raise SystemExit(f"ERROR: FIXS DriverModel not built: {FIXS_DRIVERMODEL}\n"
                         f"  build it: scripts\\dispatch\\3_vissim_components.bat")
    if not PAR_FILE.is_file():
        raise SystemExit(f"ERROR: par-file missing: {PAR_FILE}")
    inpx = STAGE / SRC_INPX.name
    tree = ET.parse(inpx)
    root = tree.getroot()
    patched = 0
    for vt in root.iter("vehicleType"):
        if vt.get("no") == str(DM_FLAGGED_TYPE_NO):
            vt.set("extDriver", "true")
            vt.set("extDriverDLLFile", str(FIXS_DRIVERMODEL))
            vt.set("extDriverParFile", str(PAR_FILE))
            patched += 1
    tree.write(inpx, encoding="utf-8", xml_declaration=True)
    if patched != 1:
        raise SystemExit(f"ERROR: expected 1 vehicleType no={DM_FLAGGED_TYPE_NO}, patched {patched}")
    print(f"[setup_eco] hooked FIXS DriverModel on Car type {DM_FLAGGED_TYPE_NO} "
          f"(EnableRealSim: true via eco_par.yaml)")


def write_runtime_config() -> None:
    cfg = (HERE / "config.yaml").read_text(encoding="utf-8")
    abs_inpx = str(STAGE / SRC_INPX.name)
    cfg = cfg.replace("stage_network\\simple_traffic_light.inpx", abs_inpx)
    RUNCFG.write_text(cfg, encoding="ascii")
    print(f"[setup_eco] wrote {RUNCFG.name} (NetworkFile -> {abs_inpx})")


def patch_gui_config() -> None:
    if not GUICFG.is_file():
        raise SystemExit(f"ERROR: CM GUI config not found: {GUICFG}")
    if not SIGNAL_TABLE.is_file():
        raise SystemExit(f"ERROR: signal table missing: {SIGNAL_TABLE}\n"
                         f"  build it first (Demo 2 build_signal_table.py)")
    runcfg_fwd = RUNCFG.as_posix()
    sig_fwd = SIGNAL_TABLE.with_suffix("").as_posix()   # CarMaker appends .csv itself
    lines = GUICFG.read_text(encoding="utf-8").splitlines()
    out, found_exe, found_args = [], False, False
    for l in lines:
        if l.startswith("CM.Exe =") or l.rstrip() == "CM.Exe =":
            out.append("CM.Exe = src/CarMaker.win64.exe"); found_exe = True
        elif l.startswith("CM.Args =") or l.rstrip() == "CM.Args =":
            out.append(f"CM.Args = -f {runcfg_fwd} -s {sig_fwd}"); found_args = True
        else:
            out.append(l)
    if not (found_exe and found_args):
        raise SystemExit(f"ERROR: could not find CM.Exe/CM.Args in {GUICFG} "
                         f"(exe={found_exe}, args={found_args})")
    GUICFG.write_text("\n".join(out) + "\n", encoding="ascii")
    print(f"[setup_eco] registered custom exe + '-f {RUNCFG.name} -s {SIGNAL_TABLE.name}' into CM GUI config")


def main() -> int:
    stage_network()
    patch_drivermodel_hook()
    write_runtime_config()
    patch_gui_config()
    print("[setup_eco] done")
    return 0


if __name__ == "__main__":
    sys.exit(main())
