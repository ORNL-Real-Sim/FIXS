"""
VISSIM bootstrap for SpeedLimitLite — Python port of ../SpeedLimit/startVissim.m.
Loads the SpeedLimit network, runs to egoEnterTime, injects an externally-
controlled ego vehicle (type 1000), then runs to completion.

Reuses ../SpeedLimit/speedLimit.{inpx,layx} directly. SpeedLimitLite is
the same scenario as SpeedLimit — just a different client stack.

Requires: pywin32  (pip install pywin32)
"""
import argparse
import os
import sys
import math
import pathlib

try:
    import win32com.client
except ImportError:
    print("ERROR: pywin32 not installed. Run: pip install pywin32", file=sys.stderr)
    sys.exit(1)

HERE = pathlib.Path(__file__).parent.resolve()
REPO_ROOT = HERE.parents[2]
NET_DIR = HERE.parent / 'networks' / 'speedLimit'
NET = NET_DIR / 'speedLimit.inpx'
LAYOUT = NET_DIR / 'speedLimit.layx'
CONFIG = HERE / 'config.yaml'

# FIXS driver model DLL (built by scripts/dispatch/3_vissim_components.bat).
# Two variants ship today:
#   - DriverModel_RealSim.dll       — long-API source (DriverModel_RealSim.cpp),
#                                     targets VISSIM <= 2020. Works on 2021+
#                                     too because x64 LLP64 keeps long and
#                                     int both 32-bit; doesn't exercise the
#                                     int-API-only code blocks.
#   - DriverModel_RealSim_v2021.dll — int-API source, the default ABI for
#                                     VISSIM 2021+ and the one PTV's headers
#                                     ship today.
# After #147 the names will flip (the v2021/int build becomes
# DriverModel_RealSim.dll; long-API becomes DriverModel_RealSim_legacy.dll).
# speedLimit.inpx has a stale baked-in DLL path that no longer resolves
# after the network was elevated to tests/Vissim/networks/speedLimit/, so
# we override at runtime regardless.
DRIVER_DLL_DIR = REPO_ROOT / 'ProprietaryFiles' / 'VISSIMserver' / 'x64' / 'Release'
DEFAULT_DRIVER_DLL = 'int'   # int-API; switch to 'legacy' for the long-API path

# VISSIM ProgID — 2200 (VISSIM 2022) is the documented target. Use 2600
# (VISSIM 2026) only when your install only has 2026. Override at runtime
# via --progid / env var VISSIM_PROGID, or hardcode default here.
DEFAULT_VISSIM_PROGID = 'VISSIM.Vissim.2200'

STOP_TIME_S = 120
STEP_HZ = 10
RAND_SEED = 42

# Ego injection parameters — match startVissim.m exactly so traces are
# comparable against speedLimitTest{1,2,3}_orig.{mat,csv}.
EGO_ENTER_TIME_S = 11.5
EGO_VEHICLE_TYPE = 1000
EGO_LINK = 1
EGO_LANE = 1
EGO_XCOORD = 1
EGO_INITIAL_SPEED_MS = 18.0
EGO_DESIRED_SPEED_MS = 20.0


def speed_unit_factor(vissim):
    """Returns the m/s → VISSIM-display-unit conversion factor."""
    unit = vissim.Net.NetPara.AttValue('UnitSpeed')
    if str(unit).upper() == 'MILESPERHOUR':
        return 2.23694
    return 3.6  # kilometers-per-hour (also the fallback)


def resolve_driver_dll(value: str) -> pathlib.Path:
    """`int` / `legacy` map to the two PF-shipped DLLs. Anything else is
    treated as an explicit path."""
    if value == 'int':
        return DRIVER_DLL_DIR / 'DriverModel_RealSim_v2021.dll'
    if value == 'legacy':
        return DRIVER_DLL_DIR / 'DriverModel_RealSim.dll'
    return pathlib.Path(value).expanduser().resolve()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--progid',
        default=os.environ.get('VISSIM_PROGID', DEFAULT_VISSIM_PROGID),
        help=f'VISSIM COM ProgID. Default: env VISSIM_PROGID or '
             f'{DEFAULT_VISSIM_PROGID}. Use VISSIM.Vissim.2600 for 2026.',
    )
    parser.add_argument(
        '--driver-dll',
        default=os.environ.get('FIXS_DRIVER_DLL', DEFAULT_DRIVER_DLL),
        help=f'Which FIXS driver DLL to load. "int" (default, VISSIM 2021+ '
             f'native ABI), "legacy" (VISSIM <=2020 long-API source), or an '
             f'explicit .dll path.',
    )
    args = parser.parse_args()
    progid = args.progid
    DRIVER_DLL = resolve_driver_dll(args.driver_dll)

    if not NET.is_file():
        sys.exit(f"ERROR: VISSIM network not found at {NET}")
    if not DRIVER_DLL.is_file():
        sys.exit(f"ERROR: driver DLL not found at {DRIVER_DLL}")

    print(f"[start_vissim] Dispatching {progid} (driver_dll={args.driver_dll})...",
          file=sys.stderr)
    # On Windows 11 build 26100+ (ucrtbase 10.0.26100.8328) VISSIM 2022 and
    # 2026 crash in their COM -Embedding init with STATUS_STACK_BUFFER_OVERRUN
    # when launched in pywin32's default multi-threaded apartment. Force STA
    # + LOCAL_SERVER so the dispatch survives. See FIXS#152.
    import pythoncom
    pythoncom.CoInitializeEx(pythoncom.COINIT_APARTMENTTHREADED)
    vissim = win32com.client.Dispatch(progid, clsctx=pythoncom.CLSCTX_LOCAL_SERVER)

    print(f"[start_vissim] Loading {NET}", file=sys.stderr)
    vissim.LoadNet(str(NET))
    vissim.LoadLayout(str(LAYOUT))

    # FIXS hook on ego (type 1000): override stale DLL path baked into .inpx
    ego_type = vissim.Net.VehicleTypes.ItemByKey(EGO_VEHICLE_TYPE)
    ego_type.SetAttValue('ExtDriver', True)
    ego_type.SetAttValue('ExtDriverDLLFile', str(DRIVER_DLL))
    ego_type.SetAttValue('ExtDriverParFile', str(CONFIG))
    # Disable FIXS on background Car traffic (type 100) — speedLimit.inpx
    # had ExtDriver=True with the same stale path which would just error.
    car_type = vissim.Net.VehicleTypes.ItemByKey(100)
    car_type.SetAttValue('ExtDriver', False)
    print(f"[start_vissim] FIXS hook on type {EGO_VEHICLE_TYPE} (ego); "
          f"disabled on type 100 (Car background)", file=sys.stderr)

    sim = vissim.Simulation
    sim.SetAttValue('SimPeriod', STOP_TIME_S)
    sim.SetAttValue('SimRes', STEP_HZ)
    sim.SetAttValue('RandSeed', RAND_SEED)
    sim.SetAttValue('RandSeedIncr', 1)
    sim.SetAttValue('NumRuns', 1)
    sim.SetAttValue('UseMaxSimSpeed', True)
    # FIXS DLL is not multi-thread safe; UseAllCores must be False.
    sim.SetAttValue('UseAllCores', False)
    sim.SetAttValue('NumCores', 1)

    unit_conv = speed_unit_factor(vissim)

    # Step manually up to ego entry time so we can observe progress and
    # avoid the appearance of a hang during RunContinuous.
    n_pre_steps = round(STEP_HZ * EGO_ENTER_TIME_S)
    print(f"[start_vissim] Stepping {n_pre_steps} sub-steps to t={EGO_ENTER_TIME_S}s ...", file=sys.stderr)
    for i in range(n_pre_steps):
        sim.RunSingleStep()
        if (i + 1) % STEP_HZ == 0:
            print(f"  ...t={(i+1)/STEP_HZ:.1f}s", file=sys.stderr, flush=True)

    print(f"[start_vissim] Injecting ego (type={EGO_VEHICLE_TYPE}) ...", file=sys.stderr)
    ego = vissim.Net.Vehicles.AddVehicleAtLinkPosition(
        EGO_VEHICLE_TYPE, EGO_LINK, EGO_LANE, EGO_XCOORD,
        EGO_DESIRED_SPEED_MS * unit_conv, True,
    )
    ego.SetAttValue('Speed', EGO_INITIAL_SPEED_MS * unit_conv)
    ego.SetAttValue('ExtContr', 1)
    print(f"[start_vissim] Ego id={ego.AttValue('No')}", file=sys.stderr)

    print(f"[start_vissim] Running to t={STOP_TIME_S}s ...", file=sys.stderr)
    sim.RunContinuous()
    print("[start_vissim] Done.", file=sys.stderr)


if __name__ == '__main__':
    main()
