"""
Stage B multi-port routing probe (issue #158, PR #165).

Spawns two Python clients in threads that connect to TrafficLayer
(DSProxy mode) on ports 2444 and 2445 simultaneously. Validates that
per-port filtering in DSProxyMode.cpp::publishesVehicle works:

  - port 2444 (type: ego) sees the full vehicle list
  - port 2445 (type: vehicleType, id: ['100']) sees ONLY type=='100'
  - both ports receive the same signal table size each tick
  - both clients stay tick-aligned (TrafficLayer drives them in lockstep)
"""

from __future__ import annotations

import json
import os
import pathlib
import socket
import sys
import threading
import time
from dataclasses import dataclass, field
from typing import List

REPO_ROOT = pathlib.Path(__file__).resolve().parents[4]
sys.path.insert(0, str(REPO_ROOT))

from CommonLib.SocketHelper import SocketHelper  # noqa: E402
from CommonLib.MsgHelper import MsgHelper  # noqa: E402
from CommonLib.ConfigHelper import ConfigHelper  # noqa: E402
from CommonLib.VehDataMsgDefs import VehData  # noqa: E402

HERE = pathlib.Path(__file__).resolve().parent

EGO_ID = "ego"
EGO_TYPE = "200"      # HGV — distinct from the filter target ("100" = Car)
CAR_TYPE = "100"      # the type the port-2445 client subscribes to
DT = 0.1
START_X, START_Y = 397.88, 167.46
EGO_SPEED = 10.0
TICKS_TARGET = 80     # enough to see ~10 vehicles spawn on the test network


def make_ego(tick: int) -> VehData:
    v = VehData()
    v.id = EGO_ID
    v.type = EGO_TYPE
    v.speed = EGO_SPEED
    v.speedDesired = EGO_SPEED
    v.positionX = START_X + EGO_SPEED * DT * tick
    v.positionY = START_Y
    v.positionZ = 0.0
    v.heading = 0.0
    v.linkId = ""
    v.laneId = 0
    v.grade = 0.0
    return v


@dataclass
class TickSnapshot:
    tick: int
    recv_veh_types: List[str]
    recv_tls: int


@dataclass
class ClientResult:
    name: str
    port: int
    connected: bool = False
    ticks: List[TickSnapshot] = field(default_factory=list)
    error: str = ""


def run_client(name: str, port: int, role: str, result: ClientResult,
               start_barrier: threading.Barrier) -> None:
    """role in {'ego', 'observer'}."""
    cfg = ConfigHelper()
    cfg.getConfig(str(HERE / "config.yaml"))

    msg = MsgHelper()
    msg.set_vehicle_message_field(cfg.simulation_setup["VehicleMessageField"])
    sh = SocketHelper(cfg, msg)

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    for attempt in range(40):
        try:
            sock.connect(("127.0.0.1", port))
            result.connected = True
            break
        except OSError:
            if attempt == 39:
                result.error = f"connect failed after 40 attempts to port {port}"
                start_barrier.abort()
                return
            time.sleep(0.5)

    print(f"[{name}] connected to port {port} (role={role})", flush=True)
    try:
        start_barrier.wait(timeout=20)
    except threading.BrokenBarrierError:
        result.error = "start barrier broken (other client never connected)"
        sock.close()
        return

    try:
        for tick in range(TICKS_TARGET):
            sh.clear_data()
            sim_state, sim_time = sh.recv_data(sock)
            if sim_state == 0:
                print(f"[{name}] shutdown at tick {tick}", flush=True)
                break

            recv_types = [v.type.strip() for v in sh.vehicle_data_receive_list]
            recv_tls = len(sh.traffic_light_data_receive_list)
            result.ticks.append(TickSnapshot(tick=tick,
                                             recv_veh_types=recv_types,
                                             recv_tls=recv_tls))

            if role == "ego":
                sh.vehicle_data_send_list.append(make_ego(tick))
            # observer: send nothing — TL still recvs a header with empty
            # body and that's fine for the per-tick handshake.
            sh.sendData(sim_state, sim_time, sock)

            if tick % 20 == 0:
                print(f"[{name}] tick {tick:3d} veh={len(recv_types):3d} "
                      f"tls={recv_tls:3d} cars={sum(1 for t in recv_types if t == CAR_TYPE):3d}",
                      flush=True)
    except (ConnectionResetError, ConnectionAbortedError):
        print(f"[{name}] server closed connection", flush=True)
    except Exception as ex:
        result.error = f"{type(ex).__name__}: {ex}"
        print(f"[{name}] EXCEPTION: {result.error}", flush=True)
    finally:
        try:
            sock.close()
        except OSError:
            pass


def main() -> int:
    barrier = threading.Barrier(2, timeout=20)
    res_ego = ClientResult(name="ego_ctrl", port=2444)
    res_obs = ClientResult(name="observer", port=2445)

    t_ego = threading.Thread(target=run_client,
                             args=("ego_ctrl", 2444, "ego", res_ego, barrier),
                             daemon=True)
    t_obs = threading.Thread(target=run_client,
                             args=("observer", 2445, "observer", res_obs, barrier),
                             daemon=True)
    t_ego.start()
    t_obs.start()
    t_ego.join()
    t_obs.join()

    # ----- INVARIANTS -----
    inv = {}

    inv["both_clients_connected"] = {
        "pass": res_ego.connected and res_obs.connected,
        "detail": f"ego={res_ego.connected} observer={res_obs.connected}",
    }

    inv["both_clients_no_error"] = {
        "pass": not res_ego.error and not res_obs.error,
        "detail": f"ego_err='{res_ego.error}' obs_err='{res_obs.error}'",
    }

    n_ego = len(res_ego.ticks)
    n_obs = len(res_obs.ticks)
    inv["tick_aligned"] = {
        "pass": n_ego > 0 and n_obs > 0 and abs(n_ego - n_obs) <= 1,
        "detail": f"ego_ticks={n_ego} obs_ticks={n_obs}",
    }

    # Observer should ONLY see type==100 (Car). Ego should see ALL types
    # (mix of 100, 200, 300 in the shipped DS example).
    obs_types_seen = set()
    for t in res_obs.ticks:
        obs_types_seen.update(t.recv_veh_types)
    inv["observer_filtered_to_Car"] = {
        "pass": bool(obs_types_seen) and obs_types_seen.issubset({CAR_TYPE, ""}),
        "detail": f"observer saw types: {sorted(obs_types_seen)} (expected only '{CAR_TYPE}')",
    }

    # Ego port should see at least one vehicle type other than Car (the
    # shipped DS example has Bus/HGV; without that diversity the filter
    # test isn't meaningful).
    ego_types_seen = set()
    for t in res_ego.ticks:
        ego_types_seen.update(t.recv_veh_types)
    inv["ego_port_unfiltered"] = {
        "pass": len(ego_types_seen - {CAR_TYPE, ""}) >= 1,
        "detail": f"ego port saw types: {sorted(ego_types_seen)}",
    }

    # Both ports should see the same number of TLS messages per tick
    # (signals are global, not filtered per port). This invariant checks
    # that the count is IDENTICAL across ports, whatever it is — on the
    # shipped DS example that is 20 signal groups per tick.
    tls_aligned = 0
    n_pair = min(n_ego, n_obs)
    for i in range(n_pair):
        if res_ego.ticks[i].recv_tls == res_obs.ticks[i].recv_tls:
            tls_aligned += 1
    inv["signals_count_aligned_across_ports"] = {
        "pass": n_pair > 0 and tls_aligned == n_pair,
        "detail": f"{tls_aligned}/{n_pair} ticks had identical TLS count; "
                  f"py-side tls/tick={res_ego.ticks[0].recv_tls if res_ego.ticks else 0}",
    }

    all_pass = all(v["pass"] for v in inv.values())
    print("")
    print("=== Invariants ===")
    for k, v in inv.items():
        print(f"  {'PASS' if v['pass'] else 'FAIL'} {k:35s} ({v['detail']})")
    print(f"=== OVERALL: {'PASS' if all_pass else 'FAIL'} ===")

    summary = {
        "ego_ticks": n_ego,
        "observer_ticks": n_obs,
        "ego_types_seen": sorted(ego_types_seen),
        "observer_types_seen": sorted(obs_types_seen),
        "invariants": {k: v["pass"] for k, v in inv.items()},
        "overall_pass": all_pass,
    }
    out_dir = HERE / "out"
    out_dir.mkdir(exist_ok=True)
    (out_dir / "summary.json").write_text(json.dumps(summary, indent=2))
    print(f"summary -> {out_dir / 'summary.json'}")
    return 0 if all_pass else 1


if __name__ == "__main__":
    sys.exit(main())
