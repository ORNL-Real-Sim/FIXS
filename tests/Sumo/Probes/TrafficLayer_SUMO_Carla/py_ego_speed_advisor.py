"""L2 external speed-advisory controller (FIXS TrafficLayer client).

A standalone Python controller that connects to TrafficLayer as a client, receives
the ego state each tick, and streams back a desired ego speed (speedDesired) on
the ego's record. TrafficLayer's sequential-client path overlays that record into
the feed VirCarlaEnv receives, so the Carla ego (real PhysX) tracks the advisory
-- the genuine "L2 via FIXS" path, with no bespoke ego-forwarding in TL. This is
the artificial-controller stand-in for a real external CAV speed planner.

It MUST connect on a LOWER port than VirCarlaEnv so TL processes it first
(ascending-port order), i.e. its advisory is in bucket B before VirCarlaEnv is
published. Port comes from VehicleSubscription[0] in the config.

Run (resolved python with CommonLib deps):
    <python> py_ego_speed_advisor.py [config.yaml]
"""
from __future__ import annotations

import pathlib
import socket
import sys
import time

REPO_ROOT = pathlib.Path(__file__).resolve().parents[4]
sys.path.insert(0, str(REPO_ROOT))

from CommonLib.SocketHelper import SocketHelper  # noqa: E402
from CommonLib.MsgHelper import MsgHelper  # noqa: E402
from CommonLib.ConfigHelper import ConfigHelper  # noqa: E402
from CommonLib.VehDataMsgDefs import VehData  # noqa: E402

HERE = pathlib.Path(__file__).resolve().parent
EGO_ID = "ego"

# Demo speed profile (trapezoid): accelerate 4->12 (0-8s), cruise 12 (8-16s),
# decelerate 12->4 (16-24s), cruise 4 (24-32s), then loop.
PROFILE = [(0.0, 4.0), (8.0, 12.0), (16.0, 12.0), (24.0, 4.0), (32.0, 4.0)]
PERIOD = PROFILE[-1][0]


def desired_speed(t: float) -> float:
    tt = t % PERIOD if PERIOD > 1e-9 else t
    for i in range(len(PROFILE) - 1):
        t0, v0 = PROFILE[i]
        t1, v1 = PROFILE[i + 1]
        if t0 <= tt <= t1:
            f = (tt - t0) / (t1 - t0) if t1 > t0 else 0.0
            return v0 + (v1 - v0) * f
    return PROFILE[-1][1]


def main() -> int:
    cfg_path = sys.argv[1] if len(sys.argv) > 1 else str(HERE / "config_l2_wire.yaml")
    cfg = ConfigHelper()
    cfg.getConfig(cfg_path)

    msg = MsgHelper()
    msg.set_vehicle_message_field(cfg.simulation_setup["VehicleMessageField"])
    sh = SocketHelper(cfg, msg)

    sub = cfg.application_setup["VehicleSubscription"][0]   # this controller's port
    server_ip, server_port = sub["ip"][0], sub["port"][0]

    print(f"[advisor] connecting to {server_ip}:{server_port} (must be < VirCarlaEnv's port)")
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    for attempt in range(30):
        try:
            sock.connect((server_ip, server_port))
            break
        except OSError:
            if attempt == 29:
                print("[advisor] connect failed after 30 attempts", file=sys.stderr)
                return 2
            time.sleep(0.5)
    print("[advisor] connected")

    last_adv = 0.0
    try:
        tick = 0
        while True:
            sh.clear_data()
            sim_state, sim_time = sh.recv_data(sock)
            if sim_state == 0:
                print(f"[advisor] shutdown at tick {tick}")
                break

            adv = desired_speed(sim_time)
            last_adv = adv

            ego = next((v for v in sh.vehicle_data_receive_list if v.id.strip() == EGO_ID), None)
            if ego is not None:
                # echo the ego's full record (per-record last-wins in TL) and
                # override speedDesired -- the advisory VirCarlaEnv will read.
                cmd = VehData()
                cmd.id = EGO_ID
                cmd.type = ego.type
                cmd.speed = ego.speed
                cmd.speedDesired = adv
                cmd.positionX = ego.positionX
                cmd.positionY = ego.positionY
                cmd.positionZ = ego.positionZ
                cmd.heading = ego.heading
                cmd.grade = ego.grade
                cmd.linkId = ego.linkId
                cmd.laneId = ego.laneId
                sh.vehicle_data_send_list.append(cmd)
            # (if the ego isn't visible yet -- pre-injection -- send nothing this tick)
            sh.sendData(sim_state, sim_time, sock)

            if tick % 10 == 0:
                seen = "ego" if ego is not None else "no-ego-yet"
                print(f"[advisor] t={sim_time:6.1f} cmd={adv:5.2f} m/s  ({seen}, "
                      f"{len(sh.vehicle_data_receive_list)} veh)", flush=True)
            tick += 1
    except (ConnectionResetError, ConnectionAbortedError):
        print("[advisor] server closed connection")
    finally:
        try:
            sock.close()
        except OSError:
            pass
    print(f"[advisor] done (last advisory {last_adv:.2f} m/s)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
