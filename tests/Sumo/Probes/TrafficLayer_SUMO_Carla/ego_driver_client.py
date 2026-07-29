"""Unified EgoDriver -- standalone FIXS TrafficLayer client (L0 / L2 / L4).

Connects to TrafficLayer as a client, reads the ego's state each tick, and returns
an ACTUATION command on the ego's record: acceleratorPedalDesired [0,1],
brakePedalDesired [0,1], steerAngleDesired [rad]. TrafficLayer's sequential overlay
merges that record into the feed VirCarlaEnv receives; VirCarlaEnv (EgoL0Driver:
Actuation) applies it to the physics ego via ApplyControl. One command channel, three
levels:

  L0  no external speed input      -> EgoDriver uses an internal target speed
  L2  advisor sets speedDesired    -> EgoDriver tracks that target (still its own lateral)
  L4  external fills the 3 fields   -> a real controller replaces this client

The lateral+longitudinal logic is a 1:1 port of CommonLib/EgoDriver.cpp (pure-pursuit
steer + proportional speed hold). Heading is derived from motion (velocity direction
in the SUMO x/y frame) so it's independent of any yaw-convention on the wire.

Ports: must connect on a LOWER port than VirCarlaEnv (ascending-port order in TL), and
if an L2 advisor is present, ABOVE the advisor so its speedDesired is already in bucket B.

Run:  <python-with-CommonLib> ego_driver_client.py [config.yaml]
"""
from __future__ import annotations

import math
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

# simple_loop centreline (SUMO frame), the SAME EgoRoutePoints the C++ Pursuit driver
# uses (config_l0_egodriver.yaml). Python ConfigHelper doesn't parse EgoRoutePoints, so
# the loop route is kept here; EgoDriver densifies it to 3 m spacing.
DEFAULT_ROUTE = [(15.0, -3.0), (185.0, -3.0), (192.9, -1.9), (198.5, 1.5), (201.9, 7.1),
                 (203.0, 15.0), (203.0, 185.0), (201.9, 192.9), (198.5, 198.5),
                 (192.9, 201.9), (185.0, 203.0), (15.0, 203.0), (7.1, 201.9), (1.5, 198.5),
                 (-1.9, 192.9), (-3.0, 185.0), (-3.0, 15.0), (-1.9, 7.1), (1.5, 1.5), (7.1, -1.9)]

DEFAULT_TARGET_SPEED = 10.0  # m/s, L0 internal target when no advisor is present


class EgoDriverParams:
    wheelbase = 2.9
    max_steer_rad = 0.7
    lookahead_min = 6.0
    lookahead_time = 1.2
    curve_slow_gain = 1.2
    curve_slow_min = 0.4
    throttle_gain = 0.25
    throttle_bias = 0.15
    throttle_max = 0.75
    brake_gain = 0.30
    search_window = 40


class EgoDriver:
    """1:1 port of CommonLib/EgoDriver.cpp: pure-pursuit steer + speed hold."""

    def __init__(self, route_pts, closed=True, params=EgoDriverParams()):
        self.p = params
        self.closed = closed
        self.cursor = 0
        self.route = self._densify(route_pts, closed)

    def _densify(self, pts, closed):
        route = []
        if len(pts) < 2:
            return list(pts)
        segs = len(pts) if closed else len(pts) - 1
        for i in range(segs):
            a = pts[i]
            b = pts[(i + 1) % len(pts)]
            dx, dy = b[0] - a[0], b[1] - a[1]
            n = max(1, int(math.hypot(dx, dy) / 3.0))
            for k in range(n):
                route.append((a[0] + dx * k / n, a[1] + dy * k / n))
        if not closed:
            route.append(pts[-1])
        return route

    def compute(self, x, y, heading_rad, speed, target_speed):
        """Returns (throttle[0,1], brake[0,1], steer[-1,1])."""
        n = len(self.route)
        if n < 2:
            return 0.0, 0.0, 0.0
        # advance cursor to nearest point ahead within a local window
        best, best_ix = 1e18, self.cursor
        for k in range(self.p.search_window):
            ix = self.cursor + k
            if ix >= n:
                if self.closed:
                    ix %= n
                else:
                    break
            dx = self.route[ix][0] - x
            dy = self.route[ix][1] - y
            d = dx * dx + dy * dy
            if d < best:
                best, best_ix = d, ix
        self.cursor = best_ix
        # lookahead point ~max(6 m, 1.2 s of travel)
        lookahead = max(self.p.lookahead_min, self.p.lookahead_time * speed)
        tgt_ix, acc = self.cursor, 0.0
        while acc < lookahead:
            nxt = tgt_ix + 1
            if nxt >= n:
                if self.closed:
                    nxt = 0
                else:
                    tgt_ix = n - 1
                    break
            dx = self.route[nxt][0] - self.route[tgt_ix][0]
            dy = self.route[nxt][1] - self.route[tgt_ix][1]
            acc += math.hypot(dx, dy)
            tgt_ix = nxt
        tx = self.route[tgt_ix][0] - x
        ty = self.route[tgt_ix][1] - y
        alpha = math.atan2(ty, tx) - heading_rad
        while alpha > math.pi:
            alpha -= 2.0 * math.pi
        while alpha < -math.pi:
            alpha += 2.0 * math.pi
        delta = math.atan2(2.0 * self.p.wheelbase * math.sin(alpha), lookahead)
        steer = max(-1.0, min(1.0, delta / self.p.max_steer_rad))
        curve_slow = max(self.p.curve_slow_min, 1.0 - self.p.curve_slow_gain * abs(alpha))
        dv = target_speed * curve_slow - speed
        if dv >= 0:
            throttle = min(self.p.throttle_max, self.p.throttle_gain * dv + self.p.throttle_bias)
            brake = 0.0
        else:
            throttle = 0.0
            brake = min(1.0, -self.p.brake_gain * dv)
        return throttle, brake, steer


def load_route(cfg):
    """Route waypoints in the SUMO frame, from config EgoRoutePoints if present."""
    try:
        pts = cfg.xil_setup.get("EgoRoutePoints") if hasattr(cfg, "xil_setup") else None
        if pts:
            return [(float(p[0]), float(p[1])) for p in pts]
    except Exception:
        pass
    return DEFAULT_ROUTE


def main() -> int:
    cfg_path = sys.argv[1] if len(sys.argv) > 1 else str(HERE / "config_l0_egodriver_client.yaml")
    cfg = ConfigHelper()
    cfg.getConfig(cfg_path)

    msg = MsgHelper()
    msg.set_vehicle_message_field(cfg.simulation_setup["VehicleMessageField"])
    sh = SocketHelper(cfg, msg)

    sub = cfg.application_setup["VehicleSubscription"][0]   # this client's port
    server_ip, server_port = sub["ip"][0], sub["port"][0]

    driver = EgoDriver(load_route(cfg), closed=True)
    print(f"[egodriver] route {len(driver.route)} densified pts; connecting to "
          f"{server_ip}:{server_port} (must be < VirCarlaEnv's port)")

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    for attempt in range(30):
        try:
            sock.connect((server_ip, server_port))
            break
        except OSError:
            if attempt == 29:
                print("[egodriver] connect failed after 30 attempts", file=sys.stderr)
                return 2
            time.sleep(0.5)
    print("[egodriver] connected")

    prev_xy = None       # for motion-derived heading
    heading = 0.0
    tick = 0
    try:
        while True:
            sh.clear_data()
            sim_state, sim_time = sh.recv_data(sock)
            if sim_state == 0:
                print(f"[egodriver] shutdown at tick {tick}")
                break

            ego = next((v for v in sh.vehicle_data_receive_list if v.id.strip() == EGO_ID), None)
            if ego is not None:
                # heading from motion (velocity direction in the SUMO x/y frame);
                # convention-free. Hold last heading below a small motion threshold.
                if prev_xy is not None:
                    dx, dy = ego.positionX - prev_xy[0], ego.positionY - prev_xy[1]
                    if math.hypot(dx, dy) > 0.05:
                        heading = math.atan2(dy, dx)
                prev_xy = (ego.positionX, ego.positionY)

                # L2 if an upstream advisor put a speedDesired on the record; else L0 internal target
                target = ego.speedDesired if ego.speedDesired and ego.speedDesired > 0.01 else DEFAULT_TARGET_SPEED
                throttle, brake, steer = driver.compute(ego.positionX, ego.positionY,
                                                        heading, ego.speed, target)

                cmd = VehData()
                cmd.id = EGO_ID
                cmd.type = ego.type
                cmd.speed = ego.speed
                cmd.positionX = ego.positionX
                cmd.positionY = ego.positionY
                cmd.positionZ = ego.positionZ
                cmd.heading = ego.heading
                cmd.speedDesired = target
                cmd.acceleratorPedalDesired = throttle
                cmd.brakePedalDesired = brake
                cmd.steerAngleDesired = steer * EgoDriverParams.max_steer_rad  # normalized -> rad
                sh.vehicle_data_send_list.append(cmd)

                if tick % 10 == 0:
                    print(f"[egodriver] t={sim_time:6.1f} tgt={target:5.2f} spd={ego.speed:5.2f} "
                          f"acc={throttle:.2f} brk={brake:.2f} steer={steer:+.2f} "
                          f"pos=({ego.positionX:6.1f},{ego.positionY:6.1f})", flush=True)
            sh.sendData(sim_state, sim_time, sock)
            tick += 1
    except (ConnectionResetError, ConnectionAbortedError):
        print("[egodriver] server closed connection")
    finally:
        try:
            sock.close()
        except OSError:
            pass
    print("[egodriver] done")
    return 0


if __name__ == "__main__":
    sys.exit(main())
