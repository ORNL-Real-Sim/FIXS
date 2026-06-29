"""
Eco-driving CAV controller for #173.

Connects to TrafficLayer's DEDICATED CAV-controller socket (DSProxyMode,
VissimSetup.CavControllerPort) -- NOT the app/CarMaker port. Each tick it:

  1. recvs the whole-network readback (all vehicles + signals) TL publishes,
  2. for every DriverModel-hooked CAV (vehicle type == CAV_TYPE), computes its
     distance to the next signal ahead and that signal's green window from the
     FIXED plan + current sim time,
  3. asks eco_driving.recommend_speed for a target speed,
  4. sends one VehFullData_t per CAV with speedDesired = target; TL relays it to
     the DriverModel (1337) -> DRIVER_DATA_DESIRED_VELOCITY -> Wiedemann.

The CarMaker ego rides in on the app port (2444) and is owned by DSProxy; this
controller never touches it (it is filtered out by vehicle type).

Signal geometry + timing come from a small, human-auditable signals.json so the
corridor model is config, not hardcoded (see build_eco_signals.py to scaffold
it, and the README for the one calibration step). The fixed-time assumption is
exactly what the issue specifies ("distance to the next signal + the fixed
signal timing").
"""

from __future__ import annotations

import argparse
import json
import pathlib
import socket
import sys
import time
from dataclasses import dataclass

import yaml

REPO_ROOT = pathlib.Path(__file__).resolve().parents[4]
sys.path.insert(0, str(REPO_ROOT))

from CommonLib.SocketHelper import SocketHelper  # noqa: E402
from CommonLib.MsgHelper import MsgHelper  # noqa: E402
from CommonLib.ConfigHelper import ConfigHelper  # noqa: E402
from CommonLib.VehDataMsgDefs import VehData  # noqa: E402

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from eco_driving import recommend_speed, GlosaParams, Advice  # noqa: E402


# --- Signal model -----------------------------------------------------------

@dataclass
class Signal:
    name: str
    pos: float          # position along travel_axis of the stop line (m)
    cycle: float        # signal cycle length (s)
    offset: float       # plan offset (s)
    green_start: float  # seconds into the cycle the through-green opens
    green_dur: float    # green duration (s)

    def window(self, sim_time: float) -> tuple[float, float]:
        """(time_to_green_start, time_to_green_end) for this sim_time."""
        phase = (sim_time - self.offset) % self.cycle
        g0 = self.green_start % self.cycle
        g1 = g0 + self.green_dur
        if g1 <= self.cycle and g0 <= phase < g1:
            # currently green
            return 0.0, (g1 - phase)
        if g1 > self.cycle and (phase >= g0 or phase < (g1 - self.cycle)):
            # green window wraps past the cycle boundary and we are inside it
            tge = (g1 - phase) if phase >= g0 else ((g1 - self.cycle) - phase)
            return 0.0, tge
        # currently red: time until the green opens
        ttgs = (g0 - phase) % self.cycle
        return ttgs, ttgs + self.green_dur


@dataclass
class SignalWorld:
    axis: str               # "x" or "y": the corridor travel coordinate
    direction: float        # +1.0 traffic travels increasing axis, -1.0 decreasing
    cav_type: str           # vehicle type string of DriverModel-hooked CAVs
    signals: list[Signal]

    @staticmethod
    def load(path: pathlib.Path) -> "SignalWorld":
        d = json.loads(path.read_text())
        sigs = [Signal(**s) for s in d["signals"]]
        return SignalWorld(
            axis=d.get("travel_axis", "x"),
            direction=float(d.get("direction", 1.0)),
            cav_type=str(d.get("cav_type", "100")),
            signals=sigs,
        )

    def coord(self, veh: VehData) -> float:
        return veh.positionX if self.axis == "x" else veh.positionY

    def next_signal(self, veh: VehData, p: GlosaParams):
        """Nearest signal AHEAD of `veh` (by travel direction), within range.

        Returns (signal, distance) or (None, inf).
        """
        c = self.coord(veh)
        best = None
        best_d = float("inf")
        for s in self.signals:
            d = (s.pos - c) * self.direction   # >0 means ahead
            if 0.0 < d <= p.range_m and d < best_d:
                best, best_d = s, d
        return best, best_d


def make_cav_cmd(veh: VehData, target_speed: float) -> VehData:
    """One behavior command for a CAV: echo state, override speedDesired."""
    cmd = VehData()
    cmd.id = veh.id                 # VISSIM VehicleID -> DriverModel matches on it
    cmd.type = veh.type
    cmd.speed = veh.speed
    cmd.speedDesired = target_speed
    cmd.positionX = veh.positionX
    cmd.positionY = veh.positionY
    cmd.positionZ = veh.positionZ
    cmd.heading = veh.heading
    cmd.linkId = veh.linkId
    cmd.laneId = veh.laneId
    cmd.grade = 0.0
    return cmd


def main() -> int:
    ap = argparse.ArgumentParser(description="Eco-driving CAV controller (#173)")
    ap.add_argument("--config", default=str(HERE / "config.yaml"),
                    help="config.yaml (for VehicleMessageField + CavControllerPort)")
    ap.add_argument("--signals", default=str(HERE / "signals.json"),
                    help="signals.json (corridor signal positions + fixed timing)")
    ap.add_argument("--ticks", type=int, default=0,
                    help="max ticks (0 = run until TL sends shutdown)")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=0,
                    help="CAV-controller port; 0 -> read VissimSetup.CavControllerPort from config")
    args = ap.parse_args()

    cfg_path = pathlib.Path(args.config)
    raw = yaml.safe_load(cfg_path.read_text())
    port = args.port or int(raw.get("VissimSetup", {}).get("CavControllerPort", 1338))

    # Reuse the CommonLib wire stack so the binary VehFullData_t format matches
    # TrafficLayer exactly (the field list gates each packed field).
    cfg = ConfigHelper()
    cfg.getConfig(str(cfg_path))
    msg = MsgHelper()
    msg.set_vehicle_message_field(cfg.simulation_setup["VehicleMessageField"])
    sh = SocketHelper(cfg, msg)

    world = SignalWorld.load(pathlib.Path(args.signals))
    glosa = GlosaParams()

    print(f"[cav_controller] connecting to {args.host}:{port} "
          f"(cav_type={world.cav_type}, {len(world.signals)} signals)", flush=True)
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    for attempt in range(60):
        try:
            sock.connect((args.host, port))
            break
        except OSError:
            if attempt == 59:
                print("[cav_controller] connect failed after 60 attempts", file=sys.stderr)
                return 2
            time.sleep(0.5)
    print("[cav_controller] connected", flush=True)

    n_active = 0
    n_cmds = 0
    tick = 0
    try:
        while True:
            if args.ticks and tick >= args.ticks:
                break
            sh.clear_data()
            sim_state, sim_time = sh.recv_data(sock)
            if sim_state == 0:
                print(f"[cav_controller] shutdown at tick {tick}", flush=True)
                break

            cavs = [v for v in sh.vehicle_data_receive_list
                    if v.type.strip() == world.cav_type]

            active_this_tick = 0
            for cav in cavs:
                sig, dist = world.next_signal(cav, glosa)
                if sig is None:
                    advice = Advice(glosa.v_max, False, "no_signal")
                else:
                    ttgs, ttge = sig.window(sim_time)
                    advice = recommend_speed(cav.speed, dist, ttgs, ttge, glosa)
                sh.vehicle_data_send_list.append(make_cav_cmd(cav, advice.target_speed))
                n_cmds += 1
                if advice.active:
                    active_this_tick += 1
            n_active += active_this_tick

            sh.sendData(sim_state, sim_time, sock)

            if tick % 10 == 0:
                print(f"[cav_controller] t={sim_time:6.1f}s tick={tick:4d} "
                      f"cavs={len(cavs):2d} eco_active={active_this_tick:2d}", flush=True)
            tick += 1
    except (ConnectionResetError, ConnectionAbortedError):
        print("[cav_controller] server closed connection", flush=True)
    finally:
        try:
            sock.close()
        except OSError:
            pass

    print(f"[cav_controller] done: {tick} ticks, {n_cmds} CAV cmds sent, "
          f"{n_active} eco-active CAV-ticks", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
