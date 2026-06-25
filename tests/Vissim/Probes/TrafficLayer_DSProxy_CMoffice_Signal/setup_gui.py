"""
Prepare the GUI co-sim demo (#172): stage the signalized network, write
config.runtime.yaml, and register the custom CarMaker exe + its args into the CM
project's GUI config -- the signal-sync analogue of the #168 setup_gui.py.

The ONE difference from #168 that makes signals work in the GUI: CM.Args carries
BOTH "-f <config>" AND "-s <RSsignalTable.csv>", so when CarMaker Office launches
the app it passes the signal table to User.c (-> the .lib opens the signal socket
on 2445 and maps VISSIM signal-group state to CM traffic lights).

After this runs:
  - stage_network/simple_traffic_light_ds.inpx exists (DS-enabled, writable),
  - config.runtime.yaml points NetworkFile at that absolute path,
  - Data/Config/GUI has  CM.Exe = src/CarMaker.win64.exe  and
                         CM.Args = -f <abs config.runtime.yaml> -s <abs signal table>
"""
from __future__ import annotations
import pathlib, shutil, sys

HERE = pathlib.Path(__file__).resolve().parent
REPO = HERE.parents[3]
CMPROJ = REPO / "ProprietaryFiles" / "CM13_proj"
GUICFG = CMPROJ / "Data" / "Config" / "GUI"
SRC_INPX = HERE / "simple_traffic_light_ds.inpx"
STAGE = HERE / "stage_network"
RUNCFG = HERE / "config.runtime.yaml"
SIGNAL_TABLE = CMPROJ / "Data" / "Road" / "simple_traffic_light_signalstop_RSsignalTable.csv"


def stage_network() -> None:
    STAGE.mkdir(exist_ok=True)
    if not SRC_INPX.is_file():
        raise SystemExit(f"ERROR: DS network asset missing: {SRC_INPX}")
    shutil.copy2(SRC_INPX, STAGE / SRC_INPX.name)
    # carry the layout + any external .sig signal programs next to the network
    for extra in list(HERE.glob("*.layx")) + list(HERE.glob("int_*.sig")):
        shutil.copy2(extra, STAGE / extra.name)
    print(f"[setup_gui] staged signalized DS network (+ .sig/.layx) -> {STAGE}")


def write_runtime_config() -> None:
    cfg = (HERE / "config.yaml").read_text(encoding="utf-8")
    abs_inpx = str(STAGE / SRC_INPX.name)
    cfg = cfg.replace("stage_network\\simple_traffic_light_ds.inpx", abs_inpx)
    RUNCFG.write_text(cfg, encoding="ascii")
    print(f"[setup_gui] wrote {RUNCFG.name} (NetworkFile -> {abs_inpx})")


def patch_gui_config() -> None:
    if not GUICFG.is_file():
        raise SystemExit(f"ERROR: CM GUI config not found: {GUICFG}")
    if not SIGNAL_TABLE.is_file():
        raise SystemExit(f"ERROR: signal table missing: {SIGNAL_TABLE}\n"
                         f"  run build_signal_table.py first")
    # CarMaker's GUI config is an INFOFILE that treats '\' as an escape char, so
    # use forward slashes -- CarMaker accepts them everywhere.
    runcfg_fwd = RUNCFG.as_posix()
    sig_fwd = SIGNAL_TABLE.as_posix()
    lines = GUICFG.read_text(encoding="utf-8").splitlines()
    out = []
    found_exe = found_args = False
    for l in lines:
        if l.startswith("CM.Exe =") or l.rstrip() == "CM.Exe =":
            out.append("CM.Exe = src/CarMaker.win64.exe")
            found_exe = True
        elif l.startswith("CM.Args =") or l.rstrip() == "CM.Args =":
            out.append(f"CM.Args = -f {runcfg_fwd} -s {sig_fwd}")
            found_args = True
        else:
            out.append(l)
    if not (found_exe and found_args):
        raise SystemExit(f"ERROR: could not find CM.Exe/CM.Args lines in {GUICFG} "
                         f"(exe={found_exe}, args={found_args})")
    GUICFG.write_text("\n".join(out) + "\n", encoding="ascii")
    print(f"[setup_gui] registered custom exe + '-f {RUNCFG.name} -s {SIGNAL_TABLE.name}' into CM GUI config")


def main() -> int:
    stage_network()
    write_runtime_config()
    patch_gui_config()
    print("[setup_gui] done")
    return 0


if __name__ == "__main__":
    sys.exit(main())
