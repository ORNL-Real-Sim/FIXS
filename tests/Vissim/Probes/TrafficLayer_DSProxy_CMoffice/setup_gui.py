"""
Prepare the GUI demo (#168): stage the network, write config.runtime.yaml, and
register the custom CarMaker exe + "-f config.runtime.yaml" into the CM project's
GUI config. Called by run_cm_office_demo.bat -- kept in Python because the batch
`echo ... -> file` / PowerShell-regex approach was fragile (a stray `>` in an
echo redirected into and corrupted config.runtime.yaml).

After this runs:
  - stage_network/simple_loop.inpx exists (DS-enabled, writable),
  - config.runtime.yaml points NetworkFile at that absolute path,
  - Data/Config/GUI has  CM.Exe = src/CarMaker.win64.exe  and
                         CM.Args = -f <abs config.runtime.yaml>
so when CarMaker Office launches the app it passes the FIXS config to User.c.
"""
from __future__ import annotations
import os
import pathlib
import re
import shutil
import sys

HERE = pathlib.Path(__file__).resolve().parent
REPO = HERE.parents[3]
CMPROJ = REPO / "ProprietaryFiles" / "CM13_proj"
GUICFG = CMPROJ / "Data" / "Config" / "GUI"
SRC_INPX = HERE / "simple_loop_ds.inpx"
SRC_LAYX = HERE / "simple_loop_ds.layx"
STAGE = HERE / "stage_network"
RUNCFG = HERE / "config.runtime.yaml"


def stage_network() -> None:
    STAGE.mkdir(exist_ok=True)
    if not SRC_INPX.is_file():
        raise SystemExit(f"ERROR: DS network asset missing: {SRC_INPX}\n"
                         f"  regenerate: python patch_ds_inpx.py "
                         f"..\\..\\SimpleEcho\\simple_loop.inpx simple_loop_ds.inpx 200")
    shutil.copy2(SRC_INPX, STAGE / "simple_loop.inpx")
    # RS_VISSIM_SEED varies the random realization WITHOUT changing the network
    # (rewrites the SimRun randSeed in the STAGED inpx only; the committed source
    # is untouched). Lets the GUI demo reproduce a specific seed -- e.g. a seed
    # known to drive the ego into the junction off-road for a live demo.
    _vseed = os.environ.get("RS_VISSIM_SEED")
    if _vseed:
        _p = STAGE / "simple_loop.inpx"
        _txt = _p.read_text(encoding="utf-8", errors="ignore")
        _txt = re.sub(r'randSeed="\d+"', f'randSeed="{_vseed}"', _txt, count=1)
        _p.write_text(_txt, encoding="utf-8")
        print(f"[setup_gui] VISSIM randSeed -> {_vseed}")
    if SRC_LAYX.is_file():
        shutil.copy2(SRC_LAYX, STAGE / "simple_loop.layx")
    print(f"[setup_gui] staged DS network -> {STAGE}")


def write_runtime_config() -> None:
    cfg = (HERE / "config.yaml").read_text(encoding="utf-8")
    abs_inpx = str(STAGE / "simple_loop.inpx")
    cfg = cfg.replace("stage_network\\simple_loop.inpx", abs_inpx)
    RUNCFG.write_text(cfg, encoding="ascii")
    print(f"[setup_gui] wrote {RUNCFG.name} (NetworkFile -> {abs_inpx})")


def patch_gui_config() -> None:
    if not GUICFG.is_file():
        raise SystemExit(f"ERROR: CM GUI config not found: {GUICFG}")
    # CarMaker's GUI config is an INFOFILE that treats '\' as an escape char,
    # so a Windows path with single backslashes gets mangled. Use forward
    # slashes -- CarMaker accepts them everywhere (cf. CM.Exe = src/CarMaker...).
    runcfg_fwd = RUNCFG.as_posix()   # C:/src_git/.../config.runtime.yaml
    lines = GUICFG.read_text(encoding="utf-8").splitlines()
    out = []
    found_exe = found_args = False
    for l in lines:
        # Match the key lines exactly; note the value may be empty so there is
        # no trailing space (e.g. "CM.Args ="). Must NOT match CM.Exe.History: /
        # CM.Args.History: which start with "CM.Exe." / "CM.Args.".
        if l.startswith("CM.Exe =") or l.rstrip() == "CM.Exe =":
            out.append("CM.Exe = src/CarMaker.win64.exe")
            found_exe = True
        elif l.startswith("CM.Args =") or l.rstrip() == "CM.Args =":
            out.append(f"CM.Args = -f {runcfg_fwd}")
            found_args = True
        else:
            out.append(l)
    if not (found_exe and found_args):
        raise SystemExit(f"ERROR: could not find CM.Exe/CM.Args lines in {GUICFG} "
                         f"(exe={found_exe}, args={found_args})")
    GUICFG.write_text("\n".join(out) + "\n", encoding="ascii")
    print(f"[setup_gui] registered custom exe + '-f {RUNCFG.name}' into CM GUI config")


def main() -> int:
    stage_network()
    write_runtime_config()
    patch_gui_config()
    print("[setup_gui] done")
    return 0


if __name__ == "__main__":
    sys.exit(main())
