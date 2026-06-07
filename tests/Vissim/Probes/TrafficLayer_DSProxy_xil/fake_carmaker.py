"""
Stage B fake CarMaker client for issue #158.

Connects to TrafficLayer.exe (running in DSProxy mode) on the application
port from config.yaml and exchanges VehFullData_t messages every tick:

  - Receives: VehFullData_t per VISSIM vehicle + TrafficLightData_t per signal
  - Sends:    a single ego VehFullData_t with id='ego' driving east at 10 m/s
              starting near the shipped DS example's first link

Validates the bidirectional pipeline introduced in Stage B without
requiring an actual CarMaker install. Real CarMaker integration arrives
when tests/Vissim/Ipg/ is refurbished (also Stage B scope).

PASS/FAIL invariants + summary.json output mirror the pattern used by
the integrated B+C probe (tests/Vissim/Probes/TrafficLayer_DSProxy_coexist_xil/),
so this probe is now CI-shaped rather than a one-off smoke test.
"""

from __future__ import annotations

import json
import math
import os
import pathlib
import socket
import sys
import time
from dataclasses import dataclass

REPO_ROOT = pathlib.Path(__file__).resolve().parents[4]
sys.path.insert(0, str(REPO_ROOT))

from CommonLib.SocketHelper import SocketHelper  # noqa: E402
from CommonLib.MsgHelper import MsgHelper  # noqa: E402
from CommonLib.ConfigHelper import ConfigHelper  # noqa: E402
from CommonLib.VehDataMsgDefs import VehData  # noqa: E402

EGO_ID = "ego"
START_X = 397.88
START_Y = 167.46
EGO_SPEED_MPS = 10.0
DT = 0.1  # 10 Hz


@dataclass
class TickSummary:
    tick: int
    received_vehicles: int
    received_tls: int
    ego_x_sent: float


def make_ego(tick: int) -> VehData:
    """Deterministic eastbound ego trajectory starting near the .inpx's first link."""
    v = VehData()
    v.id = EGO_ID
    v.type = "100"
    v.speed = EGO_SPEED_MPS
    v.positionX = START_X + EGO_SPEED_MPS * DT * tick
    v.positionY = START_Y
    v.positionZ = 0.0
    v.heading = 0.0   # eastbound; PTV math convention (east = 0 rad)
    v.linkId = ""
    v.laneId = 0
    v.grade = 0.0
    return v


def main() -> int:
    cfg_path = os.path.join(os.path.dirname(__file__), "config.yaml")
    cfg = ConfigHelper()
    cfg.getConfig(cfg_path)

    msg = MsgHelper()
    fields = cfg.simulation_setup.get("VehicleMessageField", ["id", "speed", "positionX", "positionY", "heading"])
    msg.set_vehicle_message_field(fields)
    sh = SocketHelper(cfg, msg)

    sub = cfg.application_setup["VehicleSubscription"][0]
    server_ip = sub["ip"][0]
    server_port = sub["port"][0]

    print(f"[fake_carmaker] connecting to {server_ip}:{server_port}")
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        sock.connect((server_ip, server_port))
    except OSError as e:
        print(f"[fake_carmaker] connect failed: {e}", file=sys.stderr)
        return 2
    print("[fake_carmaker] connected")

    tick = 0
    summaries: list[TickSummary] = []
    try:
        while True:
            sh.clear_data()
            sim_state, sim_time = sh.recv_data(sock)

            n_veh = len(sh.vehicle_data_receive_list)
            n_tls = len(sh.traffic_light_data_receive_list)

            if sim_state == 0:
                print(f"[fake_carmaker] received shutdown signal at tick {tick}")
                break

            ego = make_ego(tick)
            sh.vehicle_data_send_list.append(ego)
            sh.sendData(sim_state, sim_time, sock)

            summaries.append(TickSummary(tick, n_veh, n_tls, ego.positionX))
            if tick % 25 == 0:
                print(f"[fake_carmaker] tick {tick:4d} sim_time={sim_time:5.1f} "
                      f"recv_vehicles={n_veh:3d} recv_tls={n_tls:3d} "
                      f"ego_x_sent={ego.positionX:.2f}")
            tick += 1
    except (ConnectionResetError, ConnectionAbortedError):
        print(f"[fake_carmaker] server closed connection at tick {tick}")
    except KeyboardInterrupt:
        print(f"[fake_carmaker] interrupted at tick {tick}")
    finally:
        try:
            sock.close()
        except OSError:
            pass

    # ----- INVARIANT CHECKS -----
    here = pathlib.Path(__file__).resolve().parent
    if not summaries:
        print("FAIL — no ticks completed")
        return 4

    n_ticks = len(summaries)
    peak_veh = max(s.received_vehicles for s in summaries)
    final = summaries[-1]
    first_veh = summaries[0].received_vehicles

    # TL stdout evidence — `ego registered: VISSIM VehicleID=...` printed
    # once when the CreateID round-trip resolves.
    tl_log = here / "tl.log"
    tl_log_text = tl_log.read_text(errors="replace") if tl_log.is_file() else ""

    inv = {
        "B_minimum_ticks_completed": {
            # Stage B target was 300 ticks; allow 90% as the floor.
            "pass": n_ticks >= 270,
            "detail": f"{n_ticks} ticks completed (target >=270)",
        },
        "B_ego_registered_per_tl_log": {
            "pass": "ego registered: VISSIM VehicleID=" in tl_log_text,
            "detail": "TL stdout contains 'ego registered: VISSIM VehicleID=...'",
        },
        "B_background_traffic_grew": {
            "pass": final.received_vehicles > first_veh,
            "detail": f"vehicles {first_veh} -> {final.received_vehicles} "
                      f"(peak {peak_veh})",
        },
        "B_ego_moved_east": {
            # ego.positionX starts at START_X and grows by EGO_SPEED * DT
            # per tick. final.ego_x_sent should be > START_X + 1 m at any
            # tick past tick 10.
            "pass": final.ego_x_sent > START_X + 1.0,
            "detail": f"ego_x_sent {START_X:.2f} -> {final.ego_x_sent:.2f}",
        },
    }

    all_pass = all(v["pass"] for v in inv.values())
    print("")
    print("=== Invariants ===")
    for k, v in inv.items():
        print(f"  {'PASS' if v['pass'] else 'FAIL'} {k:35s} ({v['detail']})")
    print(f"=== OVERALL: {'PASS' if all_pass else 'FAIL'} ===")

    summary = {
        "ticks": n_ticks,
        "peak_received_vehicles": peak_veh,
        "first_received_vehicles": first_veh,
        "final_received_vehicles": final.received_vehicles,
        "final_recv_tls": final.received_tls,
        "final_ego_x_sent": final.ego_x_sent,
        "invariants": {k: v["pass"] for k, v in inv.items()},
        "overall_pass": all_pass,
    }
    (here / "out").mkdir(exist_ok=True)
    (here / "out" / "summary.json").write_text(json.dumps(summary, indent=2))
    print(f"summary -> {here / 'out' / 'summary.json'}")
    return 0 if all_pass else 1


if __name__ == "__main__":
    sys.exit(main())
