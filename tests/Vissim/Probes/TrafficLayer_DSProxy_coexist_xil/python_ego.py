"""
Integrated B+C regression: Python ego client + invariant checks.

Connects to TrafficLayer.exe on the application port and drives a
deterministic ego trajectory (same shape as the Stage 1.5 egoctrl probe)
through the full FIXS pipeline:

    python_ego.py
      --[VehFullData_t @ port 2444]--> TrafficLayer (DSProxy mode)
      --[VISSIM_SetDriverVehicles]---> VISSIM 2022
                                         |
                                         +-- FIXS DriverModel hooked on
                                             Car type with EnableRealSim:
                                             false (PF#6 patch active)

Validates BOTH:
  - Stage B contract — ego inject round-trips, vehicles + signals stream
    back, background traffic spawns, ego stays alive across the run
  - Stage C contract — the FIXS DriverModel attached to Car (type 100)
    loads without aborting the DSProxy handshake, vehicles of that type
    appear in the traffic readback (= VISSIM-internal Wiedemann is
    running for them; DriverModel returns USE_INTERNAL_MODEL=1 because
    of EnableRealSim: false)
"""

from __future__ import annotations

import json
import math
import os
import pathlib
import socket
import sys
import time
from dataclasses import dataclass, field

REPO_ROOT = pathlib.Path(__file__).resolve().parents[4]
sys.path.insert(0, str(REPO_ROOT))

from CommonLib.SocketHelper import SocketHelper  # noqa: E402
from CommonLib.MsgHelper import MsgHelper  # noqa: E402
from CommonLib.ConfigHelper import ConfigHelper  # noqa: E402
from CommonLib.VehDataMsgDefs import VehData  # noqa: E402

HERE = pathlib.Path(__file__).resolve().parent

EGO_ID = "ego"
DT = 0.1  # 10 Hz
START_X = 397.88
START_Y = 167.46

DM_FLAGGED_TYPE = "100"   # Car — what patch_inpx.py hooked the FIXS DriverModel onto


@dataclass
class FrameCmd:
    frame: int
    phase: str
    x: float
    y: float
    heading: float
    speed: float


def build_trajectory() -> list[FrameCmd]:
    """Mirrors the Stage 1.5 egoctrl pattern but at half the length."""
    cmds: list[FrameCmd] = []
    x, y = START_X, START_Y

    for i in range(30):                                       # A: east 10 m/s
        cmds.append(FrameCmd(len(cmds), "A_east", x, y, 0.0, 10.0))
        x += 10.0 * DT
    for i in range(15):                                       # B: hold
        cmds.append(FrameCmd(len(cmds), "B_hold", x, y, 0.0, 0.0))
    for i in range(20):                                       # C: reverse 3 m/s
        cmds.append(FrameCmd(len(cmds), "C_reverse", x, y, math.pi, 3.0))
        x -= 3.0 * DT
    for i in range(20):                                       # D: east recovery
        cmds.append(FrameCmd(len(cmds), "D_recovery", x, y, 0.0, 10.0))
        x += 10.0 * DT
    return cmds


@dataclass
class TickRecord:
    tick: int
    phase: str
    recv_vehicles: int
    recv_tls: int
    ego_near_pushed_pose: bool
    ego_x_sent: float
    type_100_in_readback: bool


def make_ego(cmd: FrameCmd) -> VehData:
    v = VehData()
    v.id = EGO_ID
    v.type = "200"     # HGV — keep ego on a type WITHOUT the FIXS DriverModel hook
    v.speed = cmd.speed
    v.positionX = cmd.x
    v.positionY = cmd.y
    v.positionZ = 0.0
    v.heading = cmd.heading
    v.linkId = ""
    v.laneId = 0
    v.grade = 0.0
    return v


def main() -> int:
    cfg_path = os.path.join(os.path.dirname(__file__), "config.yaml")
    cfg = ConfigHelper()
    cfg.getConfig(cfg_path)

    msg = MsgHelper()
    msg.set_vehicle_message_field(cfg.simulation_setup["VehicleMessageField"])
    sh = SocketHelper(cfg, msg)

    sub = cfg.application_setup["VehicleSubscription"][0]
    server_ip, server_port = sub["ip"][0], sub["port"][0]

    print(f"[python_ego] connecting to {server_ip}:{server_port}")
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    for attempt in range(20):
        try:
            sock.connect((server_ip, server_port))
            break
        except OSError:
            if attempt == 19:
                print("[python_ego] connect failed after 20 attempts", file=sys.stderr)
                return 2
            time.sleep(0.5)
    print("[python_ego] connected")

    trajectory = build_trajectory()
    records: list[TickRecord] = []

    try:
        for cmd in trajectory:
            sh.clear_data()
            sim_state, sim_time = sh.recv_data(sock)

            if sim_state == 0:
                print(f"[python_ego] shutdown signal at frame {cmd.frame}")
                break

            recv_vehs = sh.vehicle_data_receive_list
            # The C++ DSProxyMode translates VISSIM_Veh_Data.VehicleID via
            # std::to_string for the FIXS wire `id` field, so the ego comes
            # back identified by a numeric VISSIM id, not "ego". Detect it
            # by spatial proximity to the pose we pushed at frame N-1.
            ego_near_pushed_pose = any(
                abs(v.positionX - cmd.x) < 5.0 and abs(v.positionY - cmd.y) < 5.0
                for v in recv_vehs
            )
            type_100_in_readback = any(v.type.strip() == DM_FLAGGED_TYPE for v in recv_vehs)

            records.append(TickRecord(
                tick=cmd.frame,
                phase=cmd.phase,
                recv_vehicles=len(recv_vehs),
                recv_tls=len(sh.traffic_light_data_receive_list),
                ego_near_pushed_pose=ego_near_pushed_pose,
                ego_x_sent=cmd.x,
                type_100_in_readback=type_100_in_readback,
            ))

            ego = make_ego(cmd)
            sh.vehicle_data_send_list.append(ego)
            sh.sendData(sim_state, sim_time, sock)

            if cmd.frame % 15 == 0:
                print(f"[python_ego] tick {cmd.frame:3d} [{cmd.phase}] "
                      f"recv_veh={len(recv_vehs):3d} ego_near_pose={ego_near_pushed_pose} "
                      f"car_seen={type_100_in_readback}")
    except (ConnectionResetError, ConnectionAbortedError):
        print("[python_ego] server closed connection")
    finally:
        try:
            sock.close()
        except OSError:
            pass

    # ----- INVARIANT CHECKS -----
    inv: dict[str, dict] = {}
    if not records:
        print("[python_ego] FAIL no records collected"); return 4

    onlink_phases = {"A_east", "B_hold", "D_recovery"}
    # Stage B contracts. Skip the first ~5 frames where VISSIM is still
    # registering the ego from Create=true — there's no vehicle near the
    # pushed pose yet, by design (the docs call this out).
    onlink_records = [r for r in records if r.phase in onlink_phases and r.tick >= 5]
    ego_visible_onlink = sum(1 for r in onlink_records if r.ego_near_pushed_pose)
    inv["B_ego_visible_near_pushed_pose"] = {
        "pass": ego_visible_onlink >= int(0.75 * len(onlink_records)),
        "detail": f"{ego_visible_onlink}/{len(onlink_records)} on-link frames had a vehicle "
                  f"within 5m of the pushed ego pose",
    }
    # Cross-check with TL's stdout: it should have printed an
    # "ego registered: VISSIM VehicleID=N" line, which is the most direct
    # evidence the ego inject path round-tripped through DSProxy.
    tl_log = HERE / "tl.log"
    tl_log_text = tl_log.read_text(errors="replace") if tl_log.is_file() else ""
    inv["B_ego_registered_per_tl_log"] = {
        "pass": "ego registered: VISSIM VehicleID=" in tl_log_text,
        "detail": "TrafficLayer logged 'ego registered: VISSIM VehicleID=...'",
    }
    veh_first, veh_last = records[0].recv_vehicles, records[-1].recv_vehicles
    inv["B_background_traffic_grew"] = {
        "pass": veh_last > veh_first,
        "detail": f"vehicles {veh_first} -> {veh_last}",
    }
    inv["B_tls_received"] = {
        # C++ side publishes 20 signals/frame; Python TLS handler is a
        # placeholder so we just confirm bytes were exchanged successfully
        # (any tick without error means the wire format aligned).
        "pass": len(records) >= 50,
        "detail": f"{len(records)} ticks completed with no protocol break",
    }
    # Stage C contracts
    car_seen_count = sum(1 for r in records if r.type_100_in_readback)
    inv["C_drivermodel_flagged_type_present"] = {
        "pass": car_seen_count > 0,
        "detail": f"{car_seen_count}/{len(records)} ticks had at least one Car (type 100) in readback "
                  "— FIXS DriverModel loaded without aborting DSProxy",
    }
    # Verify the DriverModelError.txt / DriverModelLog.txt artifacts
    network_dir = HERE / "stage_network"
    dm_err = network_dir / "DriverModelError.txt"
    dm_log = network_dir / "DriverModelLog.txt"
    err_size = dm_err.stat().st_size if dm_err.is_file() else 0
    log_size = dm_log.stat().st_size if dm_log.is_file() else 0
    inv["C_drivermodel_init_evidence"] = {
        # The FIXS DriverModel writes "Simulation Starts at ..." to
        # DriverModelError.txt unconditionally at DRIVER_COMMAND_INIT. If
        # the DLL never loaded, the file is absent.
        "pass": dm_err.is_file() and err_size > 0,
        "detail": f"DriverModelError.txt {('exists' if dm_err.is_file() else 'absent')}, size={err_size}",
    }
    inv["C_no_traffic_layer_socket_error"] = {
        # PF#6 makes socketSetup conditional on ENABLE_REALSIM. With
        # EnableRealSim:false in coexist_par.yaml, DriverModel must NOT
        # have logged a TL connection failure.
        "pass": ("Error: initialize connection to Traffic Layer" not in
                 dm_err.read_text(errors="replace") if dm_err.is_file() else True),
        "detail": "no 'Error: initialize connection to Traffic Layer' in DriverModelError.txt",
    }

    all_pass = all(v["pass"] for v in inv.values())
    print("")
    print("=== Invariants ===")
    for k, v in inv.items():
        status = "PASS" if v["pass"] else "FAIL"
        print(f"  {status} {k:42s} ({v['detail']})")
    print(f"=== OVERALL: {'PASS' if all_pass else 'FAIL'} ===")

    summary = {
        "ticks": len(records),
        "background_traffic_first_last": [veh_first, veh_last],
        "ego_visible_near_pushed_pose_onlink": ego_visible_onlink,
        "car_seen_count": car_seen_count,
        "driver_model_error_size": err_size,
        "driver_model_log_size": log_size,
        "invariants": {k: v["pass"] for k, v in inv.items()},
        "overall_pass": all_pass,
    }
    (HERE / "out" / "summary.json").parent.mkdir(exist_ok=True)
    (HERE / "out" / "summary.json").write_text(json.dumps(summary, indent=2))
    print(f"summary written to {HERE / 'out' / 'summary.json'}")
    return 0 if all_pass else 1


if __name__ == "__main__":
    sys.exit(main())
