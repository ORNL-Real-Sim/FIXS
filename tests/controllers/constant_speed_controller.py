"""
Constant-speed ego controller (#189) -- a test-double controller.

Controllers are the USER's to provide (out of FIXS scope); FIXS ships only
trivial ones to exercise the orchestrator. This is the simplest: an XIL-style
TrafficLayer client that owns the ego and commands a constant target speed.

Role in the co-sim (usage 2):  Controller: (s_k, x_k) -> u_k
    recv the ego (+ bubble) state from TrafficLayer -> emit an ego command
    (speedDesired) -> send it back. TrafficLayer writes the ego; the World Model
    (a separate client) owns the surrounding traffic.

NOTE: the shipped WorldModels/Demo uses a SUMO-*routed* looping ego (no external
controller), so this client is not part of that demo. It is the scaffold for the
usage-2 "controller owns the ego" seam. Point it at a config whose ego bubble is
served on the controller's port, then run alongside the WM translator.
"""

from __future__ import annotations

import argparse
import os
import pathlib
import socket
import sys

HERE = pathlib.Path(__file__).resolve().parent
REPO = HERE.parents[1]                       # tests/controllers -> repo root
sys.path.insert(0, str(REPO))

from CommonLib.ConfigHelper import ConfigHelper   # noqa: E402
from CommonLib.MsgHelper import MsgHelper          # noqa: E402
from CommonLib.SocketHelper import SocketHelper    # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser(description="Constant-speed ego controller (#189)")
    ap.add_argument("-f", "--config", required=True)
    ap.add_argument("--ego", default="egoCm123", help="ego id this controller owns")
    ap.add_argument("--speed", type=float, default=8.0, help="constant target speed (m/s)")
    ap.add_argument("--observe-only", action="store_true",
                    help="stay online (recv the scene) but send NOTHING back, so SUMO keeps "
                         "driving the ego. Use for the 'ego by SUMO, surrounding by WM' mode "
                         "where the controller is present but must not change SUMO car states.")
    ap.add_argument("--max-steps", type=int, default=0)
    args = ap.parse_args()

    cfg = ConfigHelper()
    cfg.getConfig(args.config)
    msg = MsgHelper()
    msg.set_vehicle_message_field(
        cfg.simulation_setup.get("VehicleMessageField", ["id", "speed", "speedDesired"])
    )
    sock_helper = SocketHelper(cfg, msg)

    subs = cfg.application_setup.get("VehicleSubscription", [])
    if not subs:
        print("Error: no VehicleSubscription in ApplicationSetup", file=sys.stderr)
        return 2
    server_ip, server_port = subs[0]["ip"][0], subs[0]["port"][0]

    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client.connect((server_ip, server_port))
    print(f"[ctrl] connected to {server_ip}:{server_port}; ego={args.ego} v={args.speed}", file=sys.stderr)

    step = 0
    try:
        while True:
            sock_helper.clear_data()
            sim_state, sim_time = sock_helper.recv_data(client)
            if sim_state == 0:
                break
            step += 1
            # observe-only: recv the scene, command nothing -> SUMO keeps the ego.
            if not args.observe_only:
                for v in sock_helper.vehicle_data_receive_list:
                    if v.id.strip() == args.ego:
                        v.speedDesired = args.speed   # (s_k, x_k) -> u_k  (trivial)
                        sock_helper.vehicle_data_send_list.append(v)
            sock_helper.sendData(sim_state, sim_time, client)
            if args.max_steps and step >= args.max_steps:
                print(f"[ctrl] SUCCESS: ran {step} steps", file=sys.stderr)
                break
    except (ConnectionResetError, BrokenPipeError):
        print("[ctrl] server closed connection", file=sys.stderr)
    finally:
        client.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
