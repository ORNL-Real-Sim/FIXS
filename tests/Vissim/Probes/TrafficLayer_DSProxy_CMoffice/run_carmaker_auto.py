"""
Headless auto-run of the REAL custom CarMaker.win64.exe for the #168 demo.

This drives the actual office executable -- the one compiled from
ProprietaryFiles/CM13_proj/src/User.c linking VirtualEnvironment.lib -- via the
CarMaker Python API (cmapi). It is the GUI-equivalent of: open CarMaker Office,
select this exe + "-f config.runtime.yaml" as the application, load the
SimpleLoop_rs TestRun, and press Start. The simulation output is identical
to the GUI path (the GUI is just a front-end to this same exe).

Use it to verify the real VirtualEnvironment.lib round-trip against TrafficLayer
(DSProxy mode) without manual clicking, e.g. in run_demo_auto.bat which starts
TrafficLayer first.

Requires: realsim_dev python (3.10) + CarMaker 13.1.3 Python API on sys.path.
"""
from __future__ import annotations
import argparse
import pathlib
import sys

CM_PYAPI = r"C:\IPG\carmaker\win64-13.1.3\Python\python3.10"
sys.path.insert(0, CM_PYAPI)
import cmapi  # noqa: E402

HERE = pathlib.Path(__file__).resolve().parent
REPO = HERE.parents[3]
CM_PROJECT = REPO / "ProprietaryFiles" / "CM13_proj"
CUSTOM_EXE = CM_PROJECT / "src" / "CarMaker.win64.exe"
TESTRUN = "SimpleLoop_rs"


async def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default=str(HERE / "config.runtime.yaml"),
                    help="FIXS config passed to the exe via -f (the .lib reads it)")
    ap.add_argument("--seconds", type=float, default=30.0,
                    help="how long to run the sim before stopping")
    args = ap.parse_args()

    cfg = pathlib.Path(args.config).resolve()
    if not CUSTOM_EXE.is_file():
        cmapi.logger.error(f"custom CarMaker exe not found: {CUSTOM_EXE}")
        return 2
    if not cfg.is_file():
        cmapi.logger.error(f"config not found: {cfg} (run the launcher staging first)")
        return 2

    cmapi.logger.info(f"Project:  {CM_PROJECT}")
    cmapi.logger.info(f"Exe:      {CUSTOM_EXE}")
    cmapi.logger.info(f"TestRun:  {TESTRUN}")
    cmapi.logger.info(f"FIXS cfg: {cfg}")

    cmapi.Project.load(pathlib.Path(CM_PROJECT))
    testrun = cmapi.Project.instance().load_testrun_parametrization(pathlib.Path(TESTRUN))
    variation = cmapi.Variation.create_from_testrun(testrun)
    variation.set_name("DSProxy_CMoffice_demo")

    # Master = OUR custom exe (the one linking VirtualEnvironment.lib), with the
    # FIXS config passed via -f so User.c -> VirEnv_initialization connects to
    # TrafficLayer. -screen sends CarMaker messages to stdout.
    master = cmapi.CarMaker.create(cmapi.AppType.CarMaker)
    master.set_executable_path(cmapi.Path(str(CUSTOM_EXE)))
    # Only -f (the FIXS config for User.c). Do NOT pass -screen or a positional
    # testrun: those make the exe auto-start a sim on launch (with an empty
    # testrun) and abort before cmapi's apo connection establishes. cmapi needs
    # the exe to come up idle and wait for apo; it pushes the testrun via the
    # variation on start_sim().
    master.set_args([("-f", [str(cfg)])])

    simcontrol = cmapi.SimControlInteractive()
    simcontrol.set_variation(variation)
    await simcontrol.set_master(master)

    cmapi.logger.info("Starting CarMaker exe + connecting (this also triggers the "
                      ".lib socket connect to TrafficLayer)...")
    await simcontrol.start_and_connect()
    await simcontrol.start_sim()
    cmapi.logger.info("Simulation started. Ego is now driving; .lib is exchanging "
                      "VehFullData_t with TrafficLayer every tick.")

    # Sample the real ego speed to prove the exe is simulating (not just idling).
    import asyncio
    t_end = args.seconds
    last = -5.0
    while True:
        try:
            t, v, x, y = await simcontrol.get_simio().dva_read_async("Time", "Car.v", "Car.Fr1.x", "Car.Fr1.y")
        except Exception:
            break
        if t - last >= 5.0:
            cmapi.logger.info(f"  t={t:6.2f}s  Car.v={v:5.2f} m/s  pos=({x:7.2f},{y:7.2f})")
            last = t
        if t >= t_end:
            break
        await asyncio.sleep(0.2)

    cmapi.logger.info("Stopping simulation + disconnecting...")
    await simcontrol.stop_sim()
    await simcontrol.stop_and_disconnect()
    cmapi.logger.info("=== Real-CarMaker auto-run complete ===")
    return 0


if __name__ == "__main__":
    sys.exit(cmapi.Task.run_main_task(main()))
