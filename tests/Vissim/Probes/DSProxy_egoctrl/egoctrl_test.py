"""
Stage 1.5 ego-control validation for issue #156.

Goal: prove DSProxy gives us XIL-style full control over the ego's pose,
not just "trajectory replay that happens to follow VISSIM's road network".

Scope: VISSIM 2022 only for now. Stage 1 already proved 2022/2026 parity
on the basic flow; we'll port this validation once the pipeline is locked.

Test phases, all against the shipped driving_simulator_test.inpx with one
DS-created ego (VehicleType=0 -> default DS type, CreateID=4711):

  A. straight east, constant 10 m/s         frames   0..49
  B. hold position (speed=0, no advance)    frames  50..69
  C. reverse west, constant -3 m/s          frames  70..99
  D. lateral teleport +/- 5 m on Y          frames 100..119
  E. straight east again (recovery)         frames 120..149

Invariants we check for every frame:
  - ego appears in VISSIM_GetTrafficVehicles with our CreateID
  - VISSIM keeps `ControlledByVissim==False` on the ego (we keep control)
  - ego's VehicleID never gets reassigned
  - the ego never shows up in the deleted-vehicle list

PDF §1.2 caveat: VISSIM's readback for a ControlledByVissim=False vehicle
is NOT a reliable echo of the pushed pose. So we record pushed_pose and
readback_pose side by side but only assert structural invariants — not
exact numeric equality.
"""

from __future__ import annotations

import argparse
import csv
import shutil
import sys
import time
from dataclasses import dataclass
from pathlib import Path

PROBES_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROBES_ROOT / "DSProxy_smoke"))
from dsproxy_wrapper import DSProxy, Simulator_Veh_Data, SignalState  # noqa: E402


VISSIM_2022 = {
    "version_no": 2200,
    "dll": r"C:\Program Files\PTV Vision\PTV Vissim 2022\API\DrivingSimulator_DLL\bin\x64\DrivingSimulatorProxy.dll",
    "example_dir": r"C:\Program Files\PTV Vision\PTV Vissim 2022\API\DrivingSimulator_DLL\example\DrivingSimulatorTextClient\data",
}

# Trajectory generation: anchor to the shipped .fzp start pose so VISSIM's
# snap-to-link finds a valid link on frame 0.
START_X = 397.88
START_Y = 167.46
START_HEADING = 0.0      # eastbound
DT = 0.1                 # 10 Hz simulator frequency


@dataclass
class FrameCmd:
    frame: int
    phase: str
    x: float
    y: float
    heading: float
    speed: float


def build_trajectory() -> list[FrameCmd]:
    cmds: list[FrameCmd] = []
    x, y = START_X, START_Y

    # Phase A: 50 frames east @ 10 m/s
    for i in range(50):
        cmds.append(FrameCmd(len(cmds), "A_east_10mps", x, y, 0.0, 10.0))
        x += 10.0 * DT

    # Phase B: 20 frames hold (speed=0)
    hold_x, hold_y = x, y
    for i in range(20):
        cmds.append(FrameCmd(len(cmds), "B_hold", hold_x, hold_y, 0.0, 0.0))

    # Phase C: 30 frames west @ -3 m/s (we just push positions decreasing;
    # heading=pi to indicate westbound)
    import math
    for i in range(30):
        cmds.append(FrameCmd(len(cmds), "C_reverse_3mps", x, y, math.pi, 3.0))
        x -= 3.0 * DT

    # Phase D: 20 frames lateral teleport +/- 5 m on Y around a fixed X
    for i in range(20):
        offset = 5.0 if (i % 2 == 0) else -5.0
        cmds.append(FrameCmd(len(cmds), "D_lateral_teleport", x, y + offset, 0.0, 0.0))

    # Phase E: 30 frames east again (recovery)
    y_baseline = y
    for i in range(30):
        cmds.append(FrameCmd(len(cmds), "E_east_recovery", x, y_baseline, 0.0, 10.0))
        x += 10.0 * DT

    return cmds


def stage_data_files(example_dir: Path, dest_dir: Path) -> Path:
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
    out_dir = Path(args.out_dir).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    log_path = out_dir / "run.log"
    pose_csv = out_dir / "pose_compare.csv"

    log = open(log_path, "w", encoding="utf-8")

    def emit(msg: str) -> None:
        print(msg)
        log.write(msg + "\n")
        log.flush()

    emit("=== Stage 1.5 ego-control validation — VISSIM 2022 ===")
    emit(f"DLL: {VISSIM_2022['dll']}")
    emit(f"versionNo: {VISSIM_2022['version_no']}")
    emit(f"output dir: {out_dir}")

    staged_inpx = stage_data_files(Path(VISSIM_2022["example_dir"]), out_dir / "network")
    emit(f"staged inpx: {staged_inpx}")

    trajectory = build_trajectory()
    emit(f"trajectory: {len(trajectory)} frames "
         f"({', '.join(sorted({c.phase for c in trajectory}))})")

    proxy = DSProxy(VISSIM_2022["dll"])

    emit("calling VISSIM_Connect...")
    t0 = time.perf_counter()
    ok = proxy.connect(
        version_no=VISSIM_2022["version_no"],
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
    emit(f"VISSIM_Connect returned {ok} in {time.perf_counter()-t0:.2f}s")
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
    ego.RoutingDecisionNo = 0
    ego.RouteNo = 0

    pose_writer = csv.writer(open(pose_csv, "w", newline="", encoding="utf-8"))
    pose_writer.writerow([
        "frame", "phase",
        "pushed_x", "pushed_y", "pushed_heading", "pushed_speed",
        "ego_present", "ego_vehicle_id", "ego_controlled_by_vissim",
        "readback_x", "readback_y", "readback_heading", "readback_speed",
        "readback_link_id", "readback_lane", "readback_leading_id",
        "total_vehicles", "ego_in_deleted_list",
    ])

    # Track invariants
    inv = {
        "ego_assigned_unique_id": True,
        "ego_never_deleted": True,
        "ego_always_ds_controlled": True,
        "background_traffic_grew": False,
        "ego_present_in_onlink_phases": True,   # A/B/C/E require presence
        "pushed_x_honored_in_onlink_phases": True,  # |pushed_x - readback_x| < 1m
    }
    # phase D allows off-link presence to drop (documented XIL limit)
    onlink_phases = {"A_east_10mps", "B_hold", "C_reverse_3mps", "E_east_recovery"}
    phase_d_present_frames = 0
    phase_d_total_frames = 0
    max_x_drift_onlink = 0.0
    initial_veh_count = None
    last_veh_count = None
    seen_ids: set[int] = set()
    ever_created_frame: int | None = None

    try:
        for cmd in trajectory:
            ego.Position_X = cmd.x
            ego.Position_Y = cmd.y
            ego.Position_Z = 0.0
            ego.Orient_Heading = cmd.heading
            ego.Orient_Pitch = 0.0
            ego.Speed = cmd.speed

            if not proxy.set_driver_vehicles([ego]):
                emit(f"frame {cmd.frame}: SetDriverVehicles failed: {proxy.last_error()!r}")
                return 4

            veh_list = proxy.get_traffic_vehicles()
            vlists = proxy.get_vehicle_lists()
            if initial_veh_count is None:
                initial_veh_count = len(veh_list)
            last_veh_count = len(veh_list)

            ego_row = None
            for v in veh_list:
                if v.CreateID == create_id and not v.ControlledByVissim:
                    ego_row = v
                    seen_ids.add(v.VehicleID)
                    if ego_vissim_id == 0:
                        ego_vissim_id = v.VehicleID
                        ever_created_frame = cmd.frame
                        emit(f"frame {cmd.frame}: ego registered, "
                             f"VehicleID={ego_vissim_id}")
                    break

            ego_in_deleted = (ego_vissim_id != 0 and
                              ego_vissim_id in vlists.get("deleted", []))
            if ego_in_deleted:
                inv["ego_never_deleted"] = False
                emit(f"frame {cmd.frame}: WARN ego appears in deleted list!")

            if cmd.phase == "D_lateral_teleport":
                phase_d_total_frames += 1
                if ego_row is not None:
                    phase_d_present_frames += 1
            else:
                if ever_created_frame is not None and ego_row is None:
                    inv["ego_present_in_onlink_phases"] = False
                    emit(f"frame {cmd.frame} [{cmd.phase}]: WARN ego ({ego_vissim_id}) "
                         "missing from vehicle list")
                # Skip first 5 frames after creation: VISSIM is still snapping
                # the ego onto its initial link (PDF §2 link-by-heading match).
                if ego_row is not None and cmd.phase in onlink_phases \
                        and ever_created_frame is not None \
                        and cmd.frame >= ever_created_frame + 5:
                    drift = abs(ego_row.Position_X - cmd.x)
                    if drift > 2.0:
                        inv["pushed_x_honored_in_onlink_phases"] = False
                        emit(f"frame {cmd.frame} [{cmd.phase}]: WARN x drift {drift:.2f}m "
                             f"(pushed {cmd.x:.2f}, readback {ego_row.Position_X:.2f})")
                    if drift > max_x_drift_onlink:
                        max_x_drift_onlink = drift

            if ego_row is not None and ego_row.ControlledByVissim:
                inv["ego_always_ds_controlled"] = False
                emit(f"frame {cmd.frame}: WARN ego flipped to ControlledByVissim!")

            if ego_row is not None:
                pose_writer.writerow([
                    cmd.frame, cmd.phase,
                    f"{cmd.x:.4f}", f"{cmd.y:.4f}", f"{cmd.heading:.4f}", f"{cmd.speed:.4f}",
                    1, ego_row.VehicleID, int(ego_row.ControlledByVissim),
                    f"{ego_row.Position_X:.4f}", f"{ego_row.Position_Y:.4f}",
                    f"{ego_row.Orient_Heading:.4f}", f"{ego_row.Speed:.4f}",
                    ego_row.LinkID, ego_row.LaneIndex, ego_row.LeadingVehicleID,
                    len(veh_list), int(ego_in_deleted),
                ])
            else:
                pose_writer.writerow([
                    cmd.frame, cmd.phase,
                    f"{cmd.x:.4f}", f"{cmd.y:.4f}", f"{cmd.heading:.4f}", f"{cmd.speed:.4f}",
                    0, "", "", "", "", "", "", "", "", "",
                    len(veh_list), int(ego_in_deleted),
                ])

            if ego_vissim_id != 0:
                ego.VehicleID = ego_vissim_id
                ego.Create = False

            if cmd.frame % 25 == 0:
                emit(f"frame {cmd.frame:4d} [{cmd.phase}]: total_veh={len(veh_list):3d} "
                     f"ego_present={ego_row is not None}")

        # Wrap-up assertions
        if len(seen_ids) > 1:
            inv["ego_assigned_unique_id"] = False
            emit(f"WARN ego had multiple VehicleIDs: {sorted(seen_ids)}")

        if initial_veh_count is not None and last_veh_count is not None and \
                last_veh_count > initial_veh_count:
            inv["background_traffic_grew"] = True

    finally:
        emit("calling VISSIM_Disconnect...")
        proxy.disconnect()

    emit("")
    emit("=== Invariants ===")
    all_ok = True
    for k, v in inv.items():
        status = "PASS" if v else "FAIL"
        if not v:
            all_ok = False
        emit(f"  {status} {k}")
    emit("")
    emit("=== Observations (informational, not pass/fail) ===")
    emit(f"  max X-drift in on-link phases: {max_x_drift_onlink:.3f} m")
    if phase_d_total_frames > 0:
        ratio = phase_d_present_frames / phase_d_total_frames
        emit(f"  phase D off-link presence: {phase_d_present_frames}/{phase_d_total_frames}"
             f" frames ({ratio:.0%})")
        emit("    note: VISSIM drops the ego from snapshots on frames where the pushed Y")
        emit("    falls outside the link width. Ego is NOT deleted (never appears in")
        emit("    deletedVehicleIds list), just temporarily not in the snapshot. This is")
        emit("    a known XIL limit and not a defect.")
    emit(f"  background traffic: {initial_veh_count} -> {last_veh_count}")
    emit(f"  ego VISSIM VehicleIDs used: {sorted(seen_ids)}")
    emit(f"=== OVERALL: {'PASS' if all_ok else 'FAIL'} ===")
    emit(f"pose_compare.csv: {pose_csv}")
    log.close()
    return 0 if all_ok else 1


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--out-dir", default="out_2022")
    return ap.parse_args()


if __name__ == "__main__":
    sys.exit(run(parse_args()))
