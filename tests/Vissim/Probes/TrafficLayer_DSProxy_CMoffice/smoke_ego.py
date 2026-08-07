"""
Headless smoke test for the #168 CarMaker-VISSIM demo pipeline.

Stands in for real CarMaker: connects to TrafficLayer (DSProxy mode) on the
configured app port, and every tick sends one ego 'egoCm' driving east along
the SimpleEcho loop's road 40, while receiving back the VISSIM vehicle list.

Proves, without the CarMaker GUI:
  - TrafficLayer spawns VISSIM via DSProxy and ticks it
  - ego 'egoCm' is injected and round-trips (Create -> VehicleID assigned)
  - background traffic streams back
  - the ego is re-stamped 'egoCm' on publish AND excluded from being a
    phantom (it should come back with id 'egoCm', not a bare VISSIM int id)

Usage:  python smoke_ego.py [--ticks 200]
"""
from __future__ import annotations
import argparse, os, socket, sys, pathlib

REPO_ROOT = pathlib.Path(__file__).resolve().parents[4]
sys.path.insert(0, str(REPO_ROOT))
from CommonLib.SocketHelper import SocketHelper      # noqa: E402
from CommonLib.MsgHelper import MsgHelper            # noqa: E402
from CommonLib.ConfigHelper import ConfigHelper      # noqa: E402
from CommonLib.VehDataMsgDefs import VehData         # noqa: E402

EGO_ID = "egoCm"
# SimpleEcho road 40: straight edge from (0,0) heading east (hdg=0), length 200m.
START_X, START_Y = 10.0, 0.0
EGO_SPEED = 8.0
DT = 0.1


def make_ego(tick: int) -> VehData:
    v = VehData()
    v.id = EGO_ID
    v.type = "100"
    v.speed = EGO_SPEED
    v.positionX = START_X + EGO_SPEED * DT * tick
    v.positionY = START_Y
    v.positionZ = 0.0
    v.heading = 0.0           # east, PTV math convention
    v.linkId = ""
    v.laneId = 0
    v.grade = 0.0
    return v


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--ticks", type=int, default=200)
    ap.add_argument("--config", default=os.path.join(os.path.dirname(__file__), "config.runtime.yaml"))
    args = ap.parse_args()

    cfg = ConfigHelper()
    cfg.getConfig(args.config)
    msg = MsgHelper()
    msg.set_vehicle_message_field(cfg.simulation_setup.get(
        "VehicleMessageField", ["id", "speed", "positionX", "positionY", "heading"]))
    SocketHelper(cfg, msg)  # parity with fake_carmaker; not used directly

    sub = cfg.application_setup["VehicleSubscription"][0]
    ip, port = sub["ip"][0], sub["port"][0]
    print(f"[smoke_ego] connecting to {ip}:{port} as ego '{EGO_ID}'")

    sh = SocketHelper(cfg, msg)
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((ip, port))
    print("[smoke_ego] connected")

    ego_seen_back = False
    peak_veh = 0
    tick = 0
    try:
        while tick < args.ticks:
            sh.clear_data()
            sim_state, sim_time = sh.recv_data(sock)
            if sim_state == 0:
                print(f"[smoke_ego] shutdown at tick {tick}")
                break
            n = len(sh.vehicle_data_receive_list)
            peak_veh = max(peak_veh, n)
            ids = [getattr(v, "id", "?") for v in sh.vehicle_data_receive_list]
            if EGO_ID in ids:
                ego_seen_back = True

            sh.vehicle_data_send_list.append(make_ego(tick))
            sh.sendData(sim_state, sim_time, sock)

            if tick % 25 == 0:
                print(f"[smoke_ego] tick {tick:4d} t={sim_time:5.1f} recv_veh={n:3d} "
                      f"ego_in_recv={'yes' if EGO_ID in ids else 'no '} "
                      f"sample_ids={ids[:5]}")
            tick += 1
    except (ConnectionResetError, ConnectionAbortedError):
        print(f"[smoke_ego] server closed at tick {tick}")
    finally:
        sock.close()

    print("\n=== smoke_ego summary ===")
    print(f"ticks run:           {tick}")
    print(f"peak vehicles recv:  {peak_veh}")
    print(f"ego 'egoCm' echoed back with its string id: {ego_seen_back} "
          f"(expected True -> re-stamp works)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
