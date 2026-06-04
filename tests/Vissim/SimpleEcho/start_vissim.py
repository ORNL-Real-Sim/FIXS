"""
Minimal VISSIM bootstrap via COM (pywin32). Mirrors the role of
startVissim.m but adds no ego vehicle — relies on the network's own
vehicle demand to produce traffic for the echo client to subscribe to.

Reuses ../SpeedLimit/speedLimit.{inpx,layx} as a placeholder network.
Replace with a smaller dummy network if/when one is added to the repo.

Requires: pywin32  (pip install pywin32)
"""
import os
import sys
import pathlib

try:
    import win32com.client
except ImportError:
    print("ERROR: pywin32 not installed. Run: pip install pywin32", file=sys.stderr)
    sys.exit(1)

HERE = pathlib.Path(__file__).parent.resolve()
# SimpleEcho owns its network: simple_loop.inpx is generated from
# simple_loop.xodr (netconvert output) via a one-time GUI import +
# archive/setup_demand.py to bake in vehicle demand.
# See README.md for the full setup workflow.
NET = HERE / 'simple_loop.inpx'
LAYOUT = HERE / 'simple_loop.layx'
CONFIG = HERE / 'config.yaml'

# VISSIM 2022 ProgID — repo's target version, matches MATLAB scripts.
# Use 2200 if your test machine runs VISSIM 2022; switch to 2600 if you
# run 2026 (the 2022-saved .inpx is loadable in both).
VISSIM_PROGID = 'VISSIM.Vissim.2200'

STOP_TIME_S = 60
STEP_HZ = 10
RAND_SEED = 42


def main():
    if not NET.is_file():
        sys.exit(f"ERROR: VISSIM network not found at {NET}")

    print(f"[start_vissim] Dispatching {VISSIM_PROGID} ...", file=sys.stderr)
    vissim = win32com.client.Dispatch(VISSIM_PROGID)

    print(f"[start_vissim] Loading network {NET}", file=sys.stderr)
    vissim.LoadNet(str(NET))
    vissim.LoadLayout(str(LAYOUT))

    # Point every vehicle type at this config so the FIXS driver-model
    # DLL inside VISSIM finds the right TrafficLayer endpoint.
    vissim.Net.VehicleTypes.SetAllAttValues('ExtDriverParFile', str(CONFIG))

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
