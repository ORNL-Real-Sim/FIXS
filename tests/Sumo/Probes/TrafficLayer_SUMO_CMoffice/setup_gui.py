"""
Register the custom CarMaker exe + this probe's FIXS SUMO config into the CM
project's GUI config, so opening CarMaker Office and pressing Start runs the
office exe (linking VirtualEnvironment.lib) with `-f <config.yaml>` -> User.c
connects to TrafficLayer on the SUMO path.

SUMO sibling of the DSProxy probe's setup_gui.py, MINUS the VISSIM network
staging: SUMO is launched separately from its own .sumocfg, and the SUMO config
has no NetworkFile to absolute-ify, so we only patch the GUI config. Pure stdlib
(any Python 3), called by run_sumo_cm_demo.bat.

After this runs, Data/Config/GUI has:
    CM.Exe  = src/CarMaker.win64.exe
    CM.Args = -f <abs config.yaml>
"""
from __future__ import annotations
import pathlib
import sys

HERE = pathlib.Path(__file__).resolve().parent
REPO = HERE.parents[3]
CMPROJ = REPO / "ProprietaryFiles" / "CM13_proj"
GUICFG = CMPROJ / "Data" / "Config" / "GUI"
CONFIG = HERE / "config.yaml"


def patch_gui_config() -> int:
    if not GUICFG.is_file():
        print(f"ERROR: CM GUI config not found: {GUICFG}")
        return 2
    # CarMaker's GUI config is an INFOFILE that treats '\' as an escape char, so
    # a Windows path with single backslashes gets mangled. Use forward slashes --
    # CarMaker accepts them everywhere (cf. CM.Exe = src/CarMaker.win64.exe).
    cfg_fwd = CONFIG.as_posix()
    lines = GUICFG.read_text(encoding="utf-8").splitlines()
    out = []
    found_exe = found_args = False
    for l in lines:
        # Match the key lines exactly; value may be empty ("CM.Args ="). Must NOT
        # match CM.Exe.History: / CM.Args.History: which start with "CM.Exe."/"CM.Args.".
        if l.startswith("CM.Exe =") or l.rstrip() == "CM.Exe =":
            out.append("CM.Exe = src/CarMaker.win64.exe")
            found_exe = True
        elif l.startswith("CM.Args =") or l.rstrip() == "CM.Args =":
            out.append(f"CM.Args = -f {cfg_fwd}")
            found_args = True
        else:
            out.append(l)
    if not (found_exe and found_args):
        print(f"ERROR: could not find CM.Exe/CM.Args lines in {GUICFG} "
              f"(exe={found_exe}, args={found_args})")
        return 2
    GUICFG.write_text("\n".join(out) + "\n", encoding="ascii")
    print(f"[setup_gui] registered custom exe + '-f {CONFIG.name}' into CM GUI config")
    return 0


if __name__ == "__main__":
    sys.exit(patch_gui_config())
