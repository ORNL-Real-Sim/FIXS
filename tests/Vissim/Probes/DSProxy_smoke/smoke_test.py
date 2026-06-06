"""
Stage 1 smoke test for issue #156.

Loads PTV's DrivingSimulatorProxy.dll from a chosen VISSIM install, connects
to the shipped `driving_simulator_test.inpx`, replays the shipped ego
trajectory (`driving_simulator_test.fzp`) for a bounded number of frames,
and records both vehicle and signal-state samples to CSV.

The objective is to characterize the DLL contract empirically:
  - does VISSIM_Connect succeed (license + COM dispatch + DS module check)
  - does the DS-controlled ego round-trip cleanly (Create -> VehicleID)
  - do VISSIM-internal vehicles appear with ControlledByVissim=True
  - does VISSIM_GetSignalStates produce non-empty data
  - does the same Python wrapper work against VISSIM 2022 and 2026

Run:
    python smoke_test.py --version 2022 [--frames 200] [--out-dir out_2022]
    python smoke_test.py --version 2026 [--frames 200] [--out-dir out_2026]
"""

from __future__ import annotations

import argparse
import csv
import shutil
import sys
import time
from dataclasses import dataclass
from pathlib import Path

# Make the wrapper importable when run from this directory
sys.path.insert(0, str(Path(__file__).resolve().parent))
from dsproxy_wrapper import (  # noqa: E402
    DSProxy,
    Simulator_Veh_Data,
    SignalState,
)


VISSIM_INSTALLS = {
    "2022": {
        "version_no": 2200,
        "dll": r"C:\Program Files\PTV Vision\PTV Vissim 2022\API\DrivingSimulator_DLL\bin\x64\DrivingSimulatorProxy.dll",
        "example_dir": r"C:\Program Files\PTV Vision\PTV Vissim 2022\API\DrivingSimulator_DLL\example\DrivingSimulatorTextClient\data",
    },
    "2026": {
        "version_no": 2600,
        "dll": r"C:\Program Files\PTV Vision\PTV Vissim 2026\API\DrivingSimulator_DLL\bin\x64\DrivingSimulatorProxy.dll",
        "example_dir": r"C:\Program Files\PTV Vision\PTV Vissim 2026\API\DrivingSimulator_DLL\example\DrivingSimulatorTextClient\data",
    },
}


@dataclass
class FzpRow:
    t: float
    front_x: float
    front_y: float
    front_z: float
    rear_x: float
    rear_y: float
    rear_z: float
    speed: float


def read_fzp(path: Path) -> list[FzpRow]:
    """Parse PTV's shipped vehicle-trajectory file (semicolon-delimited)."""
    rows: list[FzpRow] = []
    with open(path, "r", encoding="utf-8", errors="replace") as f:
        for line in f:
            line = line.strip()
            if not line or not line[0].isdigit():
                continue
            parts = [p.strip().rstrip(";") for p in line.split(";") if p.strip()]
            if len(parts) < 8:
                continue
            try:
                rows.append(FzpRow(
                    t=float(parts[0]),
                    front_x=float(parts[1]), front_y=float(parts[2]), front_z=float(parts[3]),
                    rear_x=float(parts[4]),  rear_y=float(parts[5]),  rear_z=float(parts[6]),
                    speed=float(parts[7]),
                ))
            except ValueError:
                continue
    return rows


def heading_from_row(row: FzpRow) -> float:
    import math
    dx = row.front_x - row.rear_x
    dy = row.front_y - row.rear_y
    return math.atan2(dy, dx)


def stage_data_files(example_dir: Path, dest_dir: Path) -> Path:
    """
    Copy PTV's shipped DS example data files into a writable working directory.

    VISSIM writes lockfiles etc. into the network directory, and the install
    tree is read-only without admin. Returns the path to the staged .inpx.
    """
    dest_dir.mkdir(parents=True, exist_ok=True)
    for name in ("driving_simulator_test.inpx", "driving_simulator_test.layx",
                "driving_simulator_test.fzp",  "driving_simulator_test.pp",
                "CARRE4E_RO_500_1.sig"):
        src = example_dir / name
        if src.is_file():
            shutil.copy2(src, dest_dir / name)
    inpx = dest_dir / "driving_simulator_test.inpx"
    if not inpx.is_file():
        raise FileNotFoundError(f"Staging failed; no inpx at {inpx}")
    return inpx


def run(args: argparse.Namespace) -> int:
    cfg = VISSIM_INSTALLS[args.version]
    out_dir = Path(args.out_dir).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    log_path = out_dir / "run.log"
    veh_csv  = out_dir / "vehicles.csv"
    sig_csv  = out_dir / "signals.csv"

    log = open(log_path, "w", encoding="utf-8")

    def emit(msg: str) -> None:
        print(msg)
        log.write(msg + "\n")
        log.flush()

    emit(f"=== Stage 1 DSProxy smoke test — VISSIM {args.version} ===")
    emit(f"DLL:         {cfg['dll']}")
    emit(f"versionNo:   {cfg['version_no']}")
    emit(f"output dir:  {out_dir}")

    example_dir = Path(cfg["example_dir"])
    staged_inpx = stage_data_files(example_dir, out_dir / "network")
    emit(f"staged inpx: {staged_inpx}")

    trajectory = read_fzp(out_dir / "network" / "driving_simulator_test.fzp")
    if not trajectory:
        emit("ERROR: trajectory file is empty")
        return 2
    if args.frames > 0:
        trajectory = trajectory[:args.frames]
    emit(f"trajectory:  {len(trajectory)} frames")

    proxy = DSProxy(cfg["dll"])

    emit("calling VISSIM_Connect...")
    t0 = time.perf_counter()
    ok = proxy.connect(
        version_no=cfg["version_no"],
        network_file=staged_inpx,
        simulator_frequency=10,
        visibility_radius=-1.0,
        max_simulator_veh=10,
        max_simulator_ped=0,
        max_simulator_det=0,
        max_total_veh=50000,
        max_vissim_ped=0,
        max_vissim_sig_grp=1000,
    )
    dt_connect = time.perf_counter() - t0
    emit(f"VISSIM_Connect returned {ok} in {dt_connect:.2f}s")
    if not ok:
        emit(f"last error: {proxy.last_error()!r}")
        log.close()
        return 3

    create_id = 4711
    ego_vissim_id = 0
    ego_vehicle = Simulator_Veh_Data()
    ego_vehicle.VehicleType = 0          # use default DS vehicle type from .inpx
    ego_vehicle.Create = True
    ego_vehicle.CreateID = create_id
    ego_vehicle.Delete = False
    ego_vehicle.ControlledByVissim = False
    ego_vehicle.RoutingDecisionNo = 0
    ego_vehicle.RouteNo = 0

    veh_writer = csv.writer(open(veh_csv, "w", newline="", encoding="utf-8"))
    veh_writer.writerow([
        "frame", "vehicle_id", "create_id", "controlled_by_vissim",
        "vtype", "x", "y", "z", "heading", "pitch", "speed",
        "link_id", "lane", "leading_id", "trailing_id",
    ])

    sig_writer = csv.writer(open(sig_csv, "w", newline="", encoding="utf-8"))
    sig_writer.writerow([
        "frame", "num_signals", "controller_id", "signal_group_id",
        "state_int", "state_name",
    ])

    try:
        for frame, row in enumerate(trajectory):
            ego_vehicle.Position_X = row.front_x
            ego_vehicle.Position_Y = row.front_y
            ego_vehicle.Position_Z = row.front_z
            ego_vehicle.Orient_Heading = heading_from_row(row)
            ego_vehicle.Orient_Pitch = 0.0
            ego_vehicle.Speed = row.speed

            ok = proxy.set_driver_vehicles([ego_vehicle])
            if not ok:
                emit(f"frame {frame}: VISSIM_SetDriverVehicles failed: {proxy.last_error()!r}")
                return 4

            veh_list = proxy.get_traffic_vehicles()
            sig_list = proxy.get_signal_states()

            ego_found = False
            for v in veh_list:
                if v.CreateID == create_id and not v.ControlledByVissim:
                    if ego_vissim_id == 0:
                        ego_vissim_id = v.VehicleID
                        emit(f"frame {frame}: ego registered, VISSIM VehicleID = {ego_vissim_id}")
                    ego_found = True
                veh_writer.writerow([
                    frame, v.VehicleID, v.CreateID, int(v.ControlledByVissim),
                    v.VehicleType,
                    f"{v.Position_X:.4f}", f"{v.Position_Y:.4f}", f"{v.Position_Z:.4f}",
                    f"{v.Orient_Heading:.4f}", f"{v.Orient_Pitch:.4f}", f"{v.Speed:.4f}",
                    v.LinkID, v.LaneIndex,
                    v.LeadingVehicleID, v.TrailingVehicleID,
                ])

            if sig_list:
                for s in sig_list:
                    sig_writer.writerow([
                        frame, len(sig_list),
                        s.ControllerID, s.SignalGroupID,
                        s.SignalState, SignalState.name(s.SignalState),
                    ])
            else:
                sig_writer.writerow([frame, 0, "", "", "", ""])

            # Drop Create flag once the ego has roundtripped
            if ego_vissim_id != 0:
                ego_vehicle.VehicleID = ego_vissim_id
                ego_vehicle.Create = False

            if frame % 25 == 0:
                emit(f"frame {frame:4d}: vehicles={len(veh_list):3d} "
                     f"signals={len(sig_list):3d} ego_in_list={ego_found}")

    finally:
        emit("calling VISSIM_Disconnect...")
        proxy.disconnect()

    emit("=== run complete ===")
    emit(f"final ego VISSIM VehicleID: {ego_vissim_id}")
    emit(f"vehicles.csv: {veh_csv}")
    emit(f"signals.csv:  {sig_csv}")
    log.close()
    return 0


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--version", choices=sorted(VISSIM_INSTALLS), required=True)
    ap.add_argument("--frames", type=int, default=200,
                    help="Number of frames to replay (0 = full trajectory)")
    ap.add_argument("--out-dir", default=None,
                    help="Output directory (default: out_<version>/)")
    args = ap.parse_args()
    if args.out_dir is None:
        args.out_dir = f"out_{args.version}"
    return args


if __name__ == "__main__":
    sys.exit(run(parse_args()))
