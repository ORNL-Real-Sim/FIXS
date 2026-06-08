"""
Stage 2 coexistence test for issue #156.

Empirically answers the load-bearing question: can VISSIM load both
DriverModel.dll (FIXS-built) AND DrivingSimulatorProxy.dll (PTV) for the
same .inpx in the same VISSIM process?

Approach:
  1. Stage PTV's shipped driving_simulator_test example into a writable
     working directory.
  2. Edit the .inpx XML to set `extDriver=true` on vehicle type 100 (Car),
     pointing at the FIXS-built `DriverModel_RealSim.dll`. The DLL is
     pointed at a par-file that disables FIXS's TrafficLayer socket
     (EnableRealSim=false), so the DLL initializes and gets per-vehicle
     callbacks but does not try to reach out to a network port.
  3. Run the Stage 1 DSProxy smoke loop against the patched network.
  4. After the run, look at the DriverModelLog.txt / DriverModelError.txt
     files VISSIM's working directory accumulated. Their presence and
     content tell us whether FIXS's DriverModel was actually loaded.

This script reuses the Stage 1 wrapper at
`tests/Vissim/Probes/DSProxy_smoke/dsproxy_wrapper.py`.
"""

from __future__ import annotations

import argparse
import csv
import shutil
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from xml.etree import ElementTree as ET

# Stage 1 wrapper
PROBES_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROBES_ROOT / "DSProxy_smoke"))
from dsproxy_wrapper import DSProxy, Simulator_Veh_Data, SignalState  # noqa: E402

# Repo roots
REPO_ROOT = PROBES_ROOT.parents[2]
FIXS_DRIVERMODEL_DLL = REPO_ROOT / "ProprietaryFiles" / "VISSIMserver" / "x64" / "Release" / "DriverModel_RealSim.dll"


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
    """Copy PTV's shipped DS example files into the writable working dir."""
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


def patch_inpx_attach_drivermodel(inpx_path: Path,
                                  dll_path: Path,
                                  par_path: Path,
                                  vehicle_type_no: int = 100,
                                  allow_missing_dll: bool = False) -> int:
    """
    Patch the .inpx so vehicle type `vehicle_type_no` is hooked to the
    given DriverModel DLL.

    Returns the count of vehicleType elements patched (should be 1).
    """
    if not allow_missing_dll and not dll_path.is_file():
        raise FileNotFoundError(f"DriverModel DLL not built: {dll_path}")
    if not par_path.is_file():
        raise FileNotFoundError(f"par-file missing: {par_path}")

    tree = ET.parse(inpx_path)
    root = tree.getroot()

    # vehicleType elements may live anywhere in the tree depending on .inpx version
    patched = 0
    for vt in root.iter("vehicleType"):
        if vt.get("no") == str(vehicle_type_no):
            vt.set("extDriver", "true")
            vt.set("extDriverDLLFile", str(dll_path))
            vt.set("extDriverParFile", str(par_path))
            patched += 1

    tree.write(inpx_path, encoding="utf-8", xml_declaration=True)
    return patched


def run(args: argparse.Namespace) -> int:
    cfg = VISSIM_INSTALLS[args.version]
    out_dir = Path(args.out_dir).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    log_path = out_dir / "run.log"
    veh_csv = out_dir / "vehicles.csv"
    sig_csv = out_dir / "signals.csv"

    log = open(log_path, "w", encoding="utf-8")

    def emit(msg: str) -> None:
        print(msg)
        log.write(msg + "\n")
        log.flush()

    emit(f"=== Stage 2 DSProxy + DriverModel coexistence — VISSIM {args.version} ===")
    emit(f"DSProxy DLL:    {cfg['dll']}")
    emit(f"DriverModel:    {FIXS_DRIVERMODEL_DLL}")
    emit(f"versionNo:      {cfg['version_no']}")
    emit(f"output dir:     {out_dir}")

    if not FIXS_DRIVERMODEL_DLL.is_file():
        emit(f"ERROR: FIXS DriverModel not built. Run: scripts/dispatch/3_vissim_components.bat")
        return 2

    example_dir = Path(cfg["example_dir"])
    network_dir = out_dir / "network"
    staged_inpx = stage_data_files(example_dir, network_dir)
    emit(f"staged inpx:    {staged_inpx}")

    par_src = Path(__file__).resolve().parent / "coexist_par.yaml"
    par_in_network = network_dir / "coexist_par.yaml"
    shutil.copy2(par_src, par_in_network)

    if args.mode == "bogus_dll":
        bogus_path = Path(__file__).resolve().parent / "BOGUS_DOES_NOT_EXIST.dll"
        emit(f"diag mode:      bogus_dll (path={bogus_path})")
        n_patched = patch_inpx_attach_drivermodel(
            staged_inpx, bogus_path, par_in_network,
            vehicle_type_no=100, allow_missing_dll=True,
        )
    elif args.mode == "real_drivermodel":
        emit(f"diag mode:      real_drivermodel (FIXS-built DriverModel_RealSim.dll)")
        n_patched = patch_inpx_attach_drivermodel(
            staged_inpx, FIXS_DRIVERMODEL_DLL, par_in_network, vehicle_type_no=100
        )
    elif args.mode == "ptv_stock":
        stock_dll = Path(__file__).resolve().parent / "PTV_stock_drivermodel.dll"
        emit(f"diag mode:      ptv_stock (PTV's shipped DriverModel.cpp sample, x64)")
        n_patched = patch_inpx_attach_drivermodel(
            staged_inpx, stock_dll, par_in_network, vehicle_type_no=100
        )
    elif args.mode == "no_drivermodel":
        n_patched = 0
        emit("diag mode:      no_drivermodel (no ExtDriver attribute set)")
    else:
        emit(f"ERROR: unknown mode {args.mode}")
        return 2
    emit(f"patched .inpx:  attached DriverModel to {n_patched} vehicle type(s) (Car=100)")

    trajectory = read_fzp(network_dir / "driving_simulator_test.fzp")
    if args.frames > 0:
        trajectory = trajectory[:args.frames]
    emit(f"trajectory:     {len(trajectory)} frames")

    # Ensure DriverModel log files land in the network directory
    import os
    cwd_saved = os.getcwd()
    os.chdir(network_dir)
    try:
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
        dt = time.perf_counter() - t0
        emit(f"VISSIM_Connect returned {ok} in {dt:.2f}s")
        if not ok:
            emit(f"last error: {proxy.last_error()!r}")
            return 3

        create_id = 4711
        ego_vissim_id = 0
        ego = Simulator_Veh_Data()
        ego.VehicleType = 0
        ego.Create = True
        ego.CreateID = create_id
        ego.Delete = False
        ego.ControlledByVissim = False

        veh_writer = csv.writer(open(veh_csv, "w", newline="", encoding="utf-8"))
        veh_writer.writerow([
            "frame", "vehicle_id", "create_id", "controlled_by_vissim",
            "vtype", "x", "y", "heading", "speed",
            "link_id", "lane", "leading_id",
        ])
        sig_writer = csv.writer(open(sig_csv, "w", newline="", encoding="utf-8"))
        sig_writer.writerow(["frame", "num_signals", "controller_id", "signal_group_id",
                             "state_int", "state_name"])

        type100_seen_frames: set[int] = set()

        for frame, row in enumerate(trajectory):
            ego.Position_X = row.front_x
            ego.Position_Y = row.front_y
            ego.Position_Z = row.front_z
            ego.Orient_Heading = heading_from_row(row)
            ego.Orient_Pitch = 0.0
            ego.Speed = row.speed

            if not proxy.set_driver_vehicles([ego]):
                emit(f"frame {frame}: SetDriverVehicles failed: {proxy.last_error()!r}")
                return 4

            veh_list = proxy.get_traffic_vehicles()
            sig_list = proxy.get_signal_states()

            for v in veh_list:
                if v.CreateID == create_id and not v.ControlledByVissim and ego_vissim_id == 0:
                    ego_vissim_id = v.VehicleID
                    emit(f"frame {frame}: ego registered, VISSIM VehicleID={ego_vissim_id}")
                if v.VehicleType == 100:
                    type100_seen_frames.add(frame)
                veh_writer.writerow([
                    frame, v.VehicleID, v.CreateID, int(v.ControlledByVissim),
                    v.VehicleType,
                    f"{v.Position_X:.4f}", f"{v.Position_Y:.4f}",
                    f"{v.Orient_Heading:.4f}", f"{v.Speed:.4f}",
                    v.LinkID, v.LaneIndex, v.LeadingVehicleID,
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

            if ego_vissim_id != 0:
                ego.VehicleID = ego_vissim_id
                ego.Create = False

            if frame % 25 == 0:
                emit(f"frame {frame:4d}: vehicles={len(veh_list):3d} "
                     f"signals={len(sig_list):3d} "
                     f"type100_seen={frame in type100_seen_frames}")

            # Inter-tick pacing — see --throttle. Default 0.0 = flat-out
            # for CI; 0.1 lets VISSIM repaint so vehicles visibly move.
            if args.throttle > 0:
                time.sleep(args.throttle)

        emit("calling VISSIM_Disconnect...")
        proxy.disconnect()

        # Look for DriverModel evidence
        dm_log = network_dir / "DriverModelLog.txt"
        dm_err = network_dir / "DriverModelError.txt"
        evidence = {
            "DriverModelLog.txt": (dm_log.is_file(), dm_log.stat().st_size if dm_log.is_file() else 0),
            "DriverModelError.txt": (dm_err.is_file(), dm_err.stat().st_size if dm_err.is_file() else 0),
        }
        emit("")
        emit("=== DriverModel evidence ===")
        for k, (present, size) in evidence.items():
            emit(f"  {k:30s} present={present}  size={size} bytes")
        if dm_log.is_file():
            emit("--- DriverModelLog.txt head ---")
            for line in dm_log.read_text(errors="replace").splitlines()[:10]:
                emit(f"  {line}")
        if dm_err.is_file():
            emit("--- DriverModelError.txt head ---")
            for line in dm_err.read_text(errors="replace").splitlines()[:10]:
                emit(f"  {line}")

        emit("")
        emit(f"=== run complete ===")
        emit(f"final ego VISSIM VehicleID: {ego_vissim_id}")
        emit(f"frames with type 100 (Car, DriverModel-flagged) present: {len(type100_seen_frames)}/{len(trajectory)}")
        emit(f"vehicles.csv: {veh_csv}")
        emit(f"signals.csv:  {sig_csv}")
    finally:
        os.chdir(cwd_saved)
        log.close()
    return 0


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--version", choices=sorted(VISSIM_INSTALLS), default="2022")
    ap.add_argument("--frames", type=int, default=100)
    ap.add_argument("--out-dir", default=None)
    ap.add_argument("--throttle", type=float, default=0.0,
                    help="Sleep N seconds between ticks (e.g. 0.1) so the "
                         "VISSIM window repaints and vehicles visibly move. "
                         "Default 0 = flat-out for CI. VSCode launch.json "
                         "passes 0.1.")
    ap.add_argument(
        "--mode",
        choices=["real_drivermodel", "ptv_stock", "bogus_dll", "no_drivermodel"],
        default="real_drivermodel",
        help=(
            "real_drivermodel: FIXS-built DriverModel_RealSim.dll attached to "
            "vehicle type 100. ptv_stock: PTV's shipped DriverModel.cpp sample "
            "(pass-through to internal Wiedemann; isolates whether ANY working "
            "DLL coexists with DSProxy). bogus_dll: ExtDriver=true with a "
            "non-existent DLL path (isolates whether the .inpx attribute alone "
            "is the issue). no_drivermodel: do not patch (control case)."
        ),
    )
    args = ap.parse_args()
    if args.out_dir is None:
        args.out_dir = f"out_{args.version}_{args.mode}"
    return args


if __name__ == "__main__":
    sys.exit(run(parse_args()))
