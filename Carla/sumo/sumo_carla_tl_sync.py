#!/usr/bin/env python3
import os
import sys
import csv
import math
import time
import argparse

# Get onto the interpreter carla.json names before importing carla/traci: this is a
# standalone tool (Carla/README.md), so it starts on whatever python is on PATH, and
# on the wrong one the imports below fail by naming a module rather than the env.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import carla_env_setup as env  # noqa: E402
if __name__ == "__main__":
    env.reexec_under_configured(__file__, tag="tlsync")

import carla  # noqa: E402  (deliberately after the re-exec above)

try:
    import traci
    from sumolib import checkBinary
except Exception as e:
    print("ERROR: Could not import traci/sumolib. Ensure SUMO_HOME is set and sumo/tools is on PYTHONPATH.")
    raise


def load_tl_table(csv_path):
    """
    Expected columns: junction_id, link_id, x, y, z, heading
    Returns:
      rows: list[(junction_id:str, link_id:int, x:float, y:float)]
      table: dict[junction_id][link_id] -> dict(x,y,z,heading, actor=None)
    """
    rows = []
    table = {}
    with open(csv_path, newline="", encoding="utf-8") as file:
        r = csv.DictReader(file)
        required = {"junction_id", "link_id", "x", "y"}
        if not required.issubset(set(r.fieldnames or [])):
            raise ValueError(f"CSV missing required columns. Found: {r.fieldnames}")

        for tl_head in r:
            junc_id = str(tl_head["junction_id"])
            link_id = int(tl_head["link_id"])
            x = float(tl_head["x"])
            y = float(tl_head["y"])
            z = float(tl_head.get("z", 0.0))
            heading = float(tl_head.get("heading", 0.0))

            rows.append((junc_id, link_id, x, y))
            table.setdefault(junc_id, {})[link_id] = {"x": x, "y": y, "z": z, "heading": heading, "actor": None}

    return rows, table


def carla_to_sumo_xy(loc, offset_x=0.0, offset_y=0.0):
    return (loc.x + offset_x, -loc.y + offset_y)


def find_closest(rows, sx, sy):
    best = None
    best_d2 = float("inf")
    for (j, k, x, y) in rows:
        dx = x - sx
        dy = y - sy
        d2 = dx * dx + dy * dy
        if d2 < best_d2:
            best_d2 = d2
            best = (j, k, math.sqrt(d2))
    return best  # (junction_id, link_id, dist_m)


def map_sumo_char_to_carla_state(c):
    if c in ("r", "u"):
        return carla.TrafficLightState.Red
    if c == "y":
        return carla.TrafficLightState.Yellow
    if c in ("G", "g", "s"):
        return carla.TrafficLightState.Green
    if c in ("O", "o"):
        return carla.TrafficLightState.Off
    return carla.TrafficLightState.Unknown


def build_actor_mapping(world, rows, table, offset_x=0.0, offset_y=0.0, max_match_dist=50.0):
    """
    For each CARLA TL actor, map to nearest (junction_id, link_id) row in the CSV.
    max_match_dist guards against bad matches (meters in SUMO XY space).
    """
    tls = world.get_actors().filter("traffic.traffic_light*")
    print(f"[CARLA] Found {len(tls)} traffic light actors")

    mapped = 0
    for tl in tls:
        tl.freeze(True)
        loc = tl.get_transform().location
        sx, sy = carla_to_sumo_xy(loc, offset_x, offset_y)
        hit = find_closest(rows, sx, sy)
        if hit is None:
            continue
        j, k, dist = hit
        if dist > max_match_dist:
            # probably bad match
            continue
        table[j][k]["actor"] = tl
        mapped += 1

    print(f"[MAP] Mapped {mapped}/{len(tls)} CARLA TL actors to SUMO (junction_id, link_id)")
    return table


def apply_sumo_states_to_carla(table):
    """
    For each junction_id in our mapping table:
      - fetch SUMO state string via TraCI
      - apply per-link char to the mapped CARLA actor
    """
    for junction_id, link_map in table.items():
        try:
            state = traci.trafficlight.getRedYellowGreenState(junction_id)
        except traci.TraCIException:
            # junction_id not in SUMO TL list (or SUMO not ready)
            continue

        for link_id, entry in link_map.items():
            tl = entry["actor"]
            if tl is None:
                continue
            if link_id < 0 or link_id >= len(state):
                continue
            tl.set_state(map_sumo_char_to_carla_state(state[link_id]))


def set_carla_sync(world, fixed_dt):
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = fixed_dt
    world.apply_settings(settings)
    world.tick()


def restore_carla_async(world):
    settings = world.get_settings()
    settings.synchronous_mode = False
    settings.fixed_delta_seconds = None
    world.apply_settings(settings)

def connect_or_start_sumo(args):
    """
    Active mode:
      - if --sumocfg is provided, start SUMO ourselves
      - else require --sumo-port and connect

    Passive mode:
      - require --sumo-port and connect to an already-running SUMO/TraCI server
      - do not launch or step SUMO from this script
    """
    if args.passive:
        if not args.sumo_port:
            raise RuntimeError("--passive requires --sumo-port so this script can attach to an external SUMO TraCI server.")
        print(f"[SUMO] Connecting to existing TraCI server on port {args.sumo_port}")
        traci.connect(port=int(args.sumo_port))
        return

    if args.sumocfg:
        sumo_bin = checkBinary("sumo-gui" if args.sumo_gui else "sumo")
        sumo_cmd = [sumo_bin, "-c", args.sumocfg]

        if args.sumo_step_length is not None:
            sumo_cmd += ["--step-length", str(args.sumo_step_length)]
        else:
            sumo_cmd += ["--step-length", str(args.fixed_dt)]

        print("[SUMO] Starting:", " ".join(sumo_cmd))
        traci.start(sumo_cmd)
        return

    if not args.sumo_port:
        raise RuntimeError("Either provide --sumocfg to launch SUMO, or --sumo-port to connect to an existing SUMO instance.")

    print(f"[SUMO] Connecting to existing TraCI server on port {args.sumo_port}")
    traci.connect(port=int(args.sumo_port))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--carla-host", default="localhost")
    ap.add_argument("--carla-port", type=int, default=2000)
    ap.add_argument("--fixed-dt", type=float, default=0.05)

    ap.add_argument("--sumocfg", required=False, help="Path to your SUMO .sumocfg")
    ap.add_argument("--sumo-gui", action="store_true")
    ap.add_argument("--sumo-step-length", type=float, default=None, help="Optional SUMO step length seconds")
    ap.add_argument("--sumo-port", required=False, help="Port on which SUMO TraCI is running")

    ap.add_argument("--tl-table", required=True, help="traffic_light_table.csv")
    ap.add_argument("--offset-x", type=float, default=0.0)
    ap.add_argument("--offset-y", type=float, default=0.0)
    ap.add_argument("--max-match-dist", type=float, default=50.0)

    ap.add_argument("--xodr", default=None, help="Optional: load standalone OpenDRIVE world from this .xodr file")

    ap.add_argument(
        "--passive",
        action="store_true",
        help="Passive mirror mode: do not tick CARLA or SUMO; assume an external component is advancing both sims."
    )
    ap.add_argument(
        "--poll-sleep",
        type=float,
        default=0.001,
        help="Sleep interval used only in passive mode if CARLA wait_for_tick is unavailable/fails."
    )

    args = ap.parse_args()

    # CARLA connect
    client = carla.Client(args.carla_host, args.carla_port)
    client.set_timeout(30.0)

    # Optional: load OpenDRIVE standalone world
    if args.xodr:
        with open(args.xodr, "r", encoding="utf-8") as f:
            xodr_xml = f.read()
        params = carla.OpendriveGenerationParameters()
        world = client.generate_opendrive_world(xodr_xml, params)
        world.wait_for_tick()
        print("[CARLA] Loaded OpenDRIVE standalone world")
    else:
        world = client.get_world()
        print(f"[CARLA] world: {world.get_map().name}")

    # Only force sync if we are the component driving CARLA ticks
    if not args.passive:
        set_carla_sync(world, args.fixed_dt)
    else:
        print("[MODE] Passive mode enabled: external component must tick CARLA and SUMO")

    # Load mapping table + map CARLA TL actors
    rows, table = load_tl_table(args.tl_table)
    table = build_actor_mapping(
        world, rows, table,
        offset_x=args.offset_x, offset_y=args.offset_y,
        max_match_dist=args.max_match_dist
    )

    # SUMO start/connect
    connect_or_start_sumo(args)

    try:
        step = 0

        if args.passive:
            last_frame = None
            while True:
                # Use CARLA frame advances as the pacing source when another component is ticking.
                try:
                    snapshot = world.wait_for_tick()
                    frame = snapshot.frame
                except Exception:
                    # Fallback if wait_for_tick is not usable in the current setup.
                    time.sleep(args.poll_sleep)
                    snapshot = world.get_snapshot()
                    frame = snapshot.frame

                    if last_frame is not None and frame == last_frame:
                        continue

                last_frame = frame

                # Read current SUMO TL state and mirror into CARLA.
                apply_sumo_states_to_carla(table)

                step += 1
                if step % 100 == 0:
                    try:
                        sim_time = traci.simulation.getTime()
                        print(f"[PASSIVE] updates={step} carla_frame={frame} sumo_time={sim_time:.2f}s")
                    except Exception:
                        print(f"[PASSIVE] updates={step} carla_frame={frame}")
        else:
            while True:
                # Step SUMO one tick
                traci.simulationStep()

                # Apply SUMO TL states to CARLA TL actors
                apply_sumo_states_to_carla(table)

                # Tick CARLA one tick
                world.tick()

                step += 1
                if step % 100 == 0:
                    sim_time = traci.simulation.getTime()
                    print(f"[SYNC] step={step} sumo_time={sim_time:.2f}s")

    except KeyboardInterrupt:
        print("\n[EXIT] Stopping...")
    finally:
        try:
            traci.close()
        except Exception:
            pass

        # Only restore async if this script changed CARLA settings.
        if not args.passive:
            try:
                restore_carla_async(world)
            except Exception:
                pass


if __name__ == "__main__":
    main()