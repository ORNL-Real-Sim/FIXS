"""
World Model traffic-agent translator (#189) -- FIXS-side loop.

Usage 1 (current):  SUMO -> TrafficLayer -> WM
    A multi-resolution simulator. SUMO simulates the whole network at low
    fidelity; a learned world model re-simulates the high-fidelity agents
    *surrounding* the ego. The ego is a plain SUMO-driven vehicle (bubble
    centre + WM conditioning agent) and is never moved by the WM. The WM owns
    only the non-ego bubble vehicles; their updated poses are written back into
    SUMO via TrafficLayer's model-agnostic external-pose branch (moveToXY).

Usage 2 (later):  SUMO -> TrafficLayer -> controller -> WM
    A controller plans the ego's action and passes it to the WM as the ego
    conditioning. Only the source of ``BevAgentSet.ego_future`` changes.

Structurally this is a ``tests/Python/SimpleEchoClient`` client: recv vehicle
list -> transform -> send updated poses back. The transform is:
    crop to square BEV bubble -> push rolling history -> backend.step() -> poses

The default 'stub' backend imports neither torch nor jax, so this loop runs in
the plain ``realsim`` env.
"""

from __future__ import annotations

import argparse
import os
import pathlib
import socket
import sys

import yaml

HERE = pathlib.Path(__file__).resolve().parent
REPO = HERE.parents[1]                      # tests/WorldModels -> repo root
sys.path.insert(0, str(REPO))               # CommonLib.*
sys.path.insert(0, str(HERE))               # bev, backends

from CommonLib.ConfigHelper import ConfigHelper   # noqa: E402
from CommonLib.MsgHelper import MsgHelper          # noqa: E402
from CommonLib.SocketHelper import SocketHelper    # noqa: E402
from CommonLib.VehDataMsgDefs import VehData       # noqa: E402

from bev import (  # noqa: E402
    BevAgentSet,
    RollingHistory,
    crop_square_bev,
    make_ego_future,
    push_frame,
)
from backends import make_backend  # noqa: E402


def load_world_model_setup(config_file: str) -> dict:
    """Read the ``WorldModelSetup`` section directly from YAML.

    NOTE: the C++/Python ConfigHelper does not parse ``WorldModelSetup`` yet
    (that wiring -- a ``WorldModelSetup_t`` mirroring ``CarlaSetup_t`` -- is the
    real-implementation follow-up). For the runnable skeleton we read it here.
    """
    with open(config_file, "r", encoding="utf-8") as f:
        raw = yaml.safe_load(f)
    wm = raw.get("WorldModelSetup", {}) or {}
    cfg_dir = os.path.dirname(os.path.abspath(config_file))

    def _resolve(p):
        if p and not os.path.isabs(p):
            return os.path.normpath(os.path.join(cfg_dir, p))
        return p

    return {
        "Enable": wm.get("Enable", True),
        "Backend": wm.get("Backend", "constant_velocity"),
        "EgoId": wm.get("EgoId", "ego"),
        "BevHalfExtent": float(wm.get("BevHalfExtent", 50.0)),
        "StepLength": float(wm.get("StepLength", 0.1)),
        "HistoryFrames": int(wm.get("HistoryFrames", 16)),
        "EgoFutureHorizon": int(wm.get("EgoFutureHorizon", 5)),
        "NetFile": _resolve(wm.get("NetFile", "")),
        "ModelDir": _resolve(wm.get("ModelDir", "")),
        "TokenDir": _resolve(wm.get("TokenDir", "")),
        "Device": wm.get("Device", "cuda"),
        # Hand vehicles back to SUMO (don't WM-drive them) when they are
        # signal-constrained, so SUMO's signal + car-following own the stop-and-go
        # (the WMs are not signal-aware, and moveToXY would otherwise pin a stopped
        # vehicle so it can never launch at green). The WM keeps free-flow traffic.
        "MinControlSpeed": float(wm.get("MinControlSpeed", 2.0)),   # m/s
    }


def build_send_veh(recv_map: dict, aid: str, pose) -> VehData:
    """Build the VehData to send back for one WM-owned vehicle.

    Reuses the received VehData (type/class/extents) and overwrites the pose so
    TrafficLayer's external-pose branch does moveToXY + setSpeed. ``speedDesired``
    is what the C++ write-back reads for SUMO speed.
    """
    v = recv_map[aid]
    v.positionX = pose.x
    v.positionY = pose.y
    v.heading = pose.heading_deg
    v.speed = pose.speed
    v.speedDesired = pose.speed
    return v


def main() -> int:
    ap = argparse.ArgumentParser(description="FIXS World Model translator (#189)")
    ap.add_argument("-f", "--config", default=os.path.join(HERE, "Demo", "config.yaml"))
    ap.add_argument("--backend", default=None, help="override WorldModelSetup.Backend")
    ap.add_argument("--max-steps", type=int, default=0, help="stop after N steps (0 = run forever)")
    args = ap.parse_args()

    # --- FIXS socket plumbing (identical to SimpleEchoClient) ----------------
    config_helper = ConfigHelper()
    config_helper.getConfig(args.config)
    msg_helper = MsgHelper()
    msg_helper.set_vehicle_message_field(
        config_helper.simulation_setup.get(
            "VehicleMessageField", ["id", "speed", "positionX", "positionY"]
        )
    )
    socket_helper = SocketHelper(config_helper, msg_helper)

    subs = config_helper.application_setup.get("VehicleSubscription", [])
    if not subs:
        print("Error: no VehicleSubscription in ApplicationSetup", file=sys.stderr)
        return 2
    server_ip, server_port = subs[0]["ip"][0], subs[0]["port"][0]

    # --- World model setup ---------------------------------------------------
    wm = load_world_model_setup(args.config)
    if args.backend:
        wm["Backend"] = args.backend
    ego_id = wm["EgoId"]
    half = wm["BevHalfExtent"]
    dt = wm["StepLength"]
    horizon = wm["EgoFutureHorizon"]
    min_control_speed = wm["MinControlSpeed"]

    history = RollingHistory(depth=wm["HistoryFrames"])

    # Local vector map (parsed once) for map-consuming backends.
    vmap = None
    if wm["NetFile"] and os.path.exists(wm["NetFile"]):
        from vector_map import VectorMap
        vmap = VectorMap.from_sumo_net(wm["NetFile"])
        print(f"[wm] vector map: {len(vmap.lanes)} lane polylines from {os.path.basename(wm['NetFile'])}",
              file=sys.stderr)

    backend = make_backend(wm["Backend"])
    backend.reset(wm, vector_map=vmap)
    print(f"[wm] backend={wm['Backend']} ego={ego_id} bubble=+/-{half}m dt={dt}s", file=sys.stderr)

    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client.connect((server_ip, server_port))
    print(f"[wm] connected to {server_ip}:{server_port}", file=sys.stderr)

    step = 0
    try:
        while True:
            socket_helper.clear_data()
            sim_state, sim_time = socket_helper.recv_data(client)
            if sim_state == 0:
                print("[wm] shutdown signal from server", file=sys.stderr)
                break
            step += 1

            recv = socket_helper.vehicle_data_receive_list
            recv_map = {v.id.strip(): v for v in recv}

            bev = crop_square_bev(recv, ego_id, half, history)
            if bev is None:
                # ego not in the network this step: pass everything through
                socket_helper.vehicle_data_send_list.extend(recv)
                socket_helper.sendData(sim_state, sim_time, client)
                continue

            push_frame(bev)
            bev.ego_future = make_ego_future(bev, horizon, dt)

            poses = backend.step(bev)  # {id: AgentPose} for non-ego only
            for aid, pose in poses.items():
                if aid not in recv_map:
                    continue
                i = bev.ids.index(aid)
                spd = float(bev.speed[i])
                v = recv_map[aid]
                sig = int(getattr(v, "signalLightColor", 0))       # 1=red 2=yellow 3=green
                sig_dist = float(getattr(v, "signalLightDistance", -1.0))
                # comfortable stopping distance (+margin) at the current speed
                stop_dist = spd * spd / (2.0 * 3.0) + 5.0
                # Hand back to SUMO if signal-constrained: stopped/slow (would
                # deadlock at 0), or approaching a red/yellow within stopping range.
                # RS_WM_NO_HANDBACK disables this (to observe the WM's OWN signal
                # response, e.g. the CATK light_type experiment).
                if not os.environ.get("RS_WM_NO_HANDBACK"):
                    if spd < min_control_speed:
                        continue
                    if sig in (1, 2) and 0.0 <= sig_dist <= stop_dist:
                        continue
                socket_helper.vehicle_data_send_list.append(
                    build_send_veh(recv_map, aid, pose)
                )

            if os.environ.get("RS_WM_DEBUG") and poses:
                dbg = os.environ.get("RS_WM_DEBUG_CSV", "wm_debug.csv")
                newf = not os.path.exists(dbg)
                with open(dbg, "a", encoding="utf-8") as f:
                    if newf:
                        f.write("t,id,recvX,recvY,recvSpeed,outSpeed,sigColor,sigDist\n")
                    for aid, p in poses.items():
                        if aid not in recv_map:
                            continue
                        i = bev.ids.index(aid)
                        v = recv_map[aid]
                        f.write(f"{sim_time:.2f},{aid},{bev.xy[i,0]:.3f},{bev.xy[i,1]:.3f},"
                                f"{float(bev.speed[i]):.3f},{p.speed:.3f},"
                                f"{int(getattr(v,'signalLightColor',0))},"
                                f"{float(getattr(v,'signalLightDistance',0.0)):.1f}\n")

            socket_helper.sendData(sim_state, sim_time, client)

            if step % 100 == 0:
                print(
                    f"[wm] step {step} t={sim_time:.1f}s bubble={len(bev.ids)} "
                    f"moved={len(poses)} warm={history.is_warm}",
                    file=sys.stderr,
                )
            if args.max_steps and step >= args.max_steps:
                print(f"[wm] SUCCESS: ran {step} steps", file=sys.stderr)
                break
    except (ConnectionResetError, BrokenPipeError):
        print("[wm] server closed connection", file=sys.stderr)
    except KeyboardInterrupt:
        print("[wm] interrupted", file=sys.stderr)
    finally:
        client.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
