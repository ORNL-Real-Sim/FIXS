"""
Minimal VISSIM bootstrap via COM (pywin32). Loads simple_loop.inpx and
wires up the FIXS driver model DLL on the Car vehicle type so that
TrafficLayer receives vehicle state.

No ego vehicle is injected — the vehicle input on link 1 (added by
archive/setup_network.py) produces traffic organically.

Requires: pywin32  (pip install pywin32)
"""
import sys
import pathlib

try:
    import win32com.client
except ImportError:
    print("ERROR: pywin32 not installed. Run: pip install pywin32", file=sys.stderr)
    sys.exit(1)

HERE = pathlib.Path(__file__).parent.resolve()
REPO_ROOT = HERE.parents[2]   # tests/Vissim/SimpleEcho -> repo root

NET = HERE / 'simple_loop.inpx'
LAYOUT = HERE / 'simple_loop.layx'
CONFIG = HERE / 'config.yaml'

# FIXS driver model DLL (built by scripts/dispatch/3_vissim_components.bat)
DRIVER_DLL = REPO_ROOT / 'ProprietaryFiles' / 'VISSIMserver' / 'x64' / 'Release' / 'DriverModel_RealSim.dll'

# Vehicle type to hook FIXS into — Car (no=100, the only type that
# actually appears in traffic given the Default composition).
EGO_VEHICLE_TYPE = 100

# VISSIM 2022 ProgID — repo's target version, matches MATLAB scripts.
# Switch to 'VISSIM.Vissim.2600' if your dev machine runs VISSIM 2026.
VISSIM_PROGID = 'VISSIM.Vissim.2200'

STOP_TIME_S = 60
STEP_HZ = 10
RAND_SEED = 42


def main():
    for p, label in [(NET, 'network'), (LAYOUT, 'layout'),
                     (CONFIG, 'config'), (DRIVER_DLL, 'driver DLL')]:
        if not p.is_file():
            sys.exit(f"ERROR: {label} not found at {p}")

    print(f"[start_vissim] Dispatching {VISSIM_PROGID} ...", file=sys.stderr)
    vissim = win32com.client.Dispatch(VISSIM_PROGID)

    print(f"[start_vissim] Loading network {NET.name}", file=sys.stderr)
    vissim.LoadNet(str(NET))
    vissim.LoadLayout(str(LAYOUT))

    # Hook the FIXS driver model DLL onto the Car vehicle type.
    # ExtDriverDLLFile + ExtDriverParFile must both be absolute paths.
    car = vissim.Net.VehicleTypes.ItemByKey(EGO_VEHICLE_TYPE)
    car.SetAttValue('ExtDriver', True)
    car.SetAttValue('ExtDriverDLLFile', str(DRIVER_DLL))
    car.SetAttValue('ExtDriverParFile', str(CONFIG))
    print(f"[start_vissim] FIXS hook on type {EGO_VEHICLE_TYPE}: "
          f"DLL={DRIVER_DLL.name}, Par={CONFIG.name}", file=sys.stderr)

    sim = vissim.Simulation
    sim.SetAttValue('SimPeriod', STOP_TIME_S)
    sim.SetAttValue('SimRes', STEP_HZ)
    sim.SetAttValue('RandSeed', RAND_SEED)
    sim.SetAttValue('UseMaxSimSpeed', True)

    print(f"[start_vissim] Running for {STOP_TIME_S}s ...", file=sys.stderr)
    sim.RunContinuous()
    print("[start_vissim] Done.", file=sys.stderr)


if __name__ == '__main__':
    main()
