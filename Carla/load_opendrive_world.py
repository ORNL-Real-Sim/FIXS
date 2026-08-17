"""
load_opendrive_world.py -- import an OpenDRIVE (.xodr) road into a running CARLA
server via generate_opendrive_world(), no map package / no Unreal cook.

This is how a signal-free scenario (e.g. SimpleLoop) gets its geometry into CARLA:
the same simple_loop.xodr the CarMaker side uses, so the two are coordinate-matched.
CARLA meshes the road procedurally on the fly -- a bare drivable road, no scenery,
which is exactly what a bridge test needs.

Run AFTER launch_carla.bat + wait_for_rpc.ps1 (the server must be accepting RPC),
and BEFORE the bridge (VirCarlaEnv.exe) connects, so the world is already the
OpenDRIVE road when the bridge calls GetWorld().

  python load_opendrive_world.py <path-to.xodr> [--host H] [--port P]
                                 [--sync] [--delta 0.1] [--vertex-distance 2.0]
                                 [--max-road-length 50.0] [--wall-height 0.0]
                                 [--extra-width 0.6]

Requires: a Python with the carla 0.9.15 package.
"""
from __future__ import annotations
import argparse
import sys

# carla_env_setup moved out of this folder (#313); make it importable
# before the flat import below.
import _cosim_path  # noqa: E402
_cosim_path.ensure()

import carla_env_setup as env

# Get onto the interpreter carla.json names BEFORE importing carla. This script is
# launched standalone (the demo .bat files, or by hand), so the python that started
# it is whatever was on PATH - and on the wrong one the next line is a bare
# ImportError naming a module, when the real answer is "you are in the wrong env".
if __name__ == "__main__":
    env.reexec_under_configured(__file__, tag="carla")

import carla  # noqa: E402  (deliberately after the re-exec above)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("xodr", help="path to the .xodr file")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--timeout", type=float, default=20.0)
    ap.add_argument("--sync", action="store_true",
                    help="put the new world in synchronous mode (match the bridge's tick)")
    ap.add_argument("--delta", type=float, default=0.1,
                    help="fixed_delta_seconds when --sync (default 0.1 = 10 Hz)")
    # generate_opendrive_world mesh params (sane defaults for a bare test road)
    ap.add_argument("--vertex-distance", type=float, default=2.0)
    ap.add_argument("--max-road-length", type=float, default=50.0)
    ap.add_argument("--wall-height", type=float, default=0.0,
                    help="0 = no boundary walls (keep the loop open for the camera)")
    ap.add_argument("--extra-width", type=float, default=0.6)
    args = ap.parse_args()

    try:
        with open(args.xodr, "r", encoding="utf-8") as fh:
            xodr = fh.read()
    except OSError as e:
        print(f"[load_opendrive_world] FAIL: cannot read xodr: {e}")
        return 2
    if "<OpenDRIVE" not in xodr:
        print(f"[load_opendrive_world] FAIL: {args.xodr} does not look like OpenDRIVE")
        return 2

    print(f"[load_opendrive_world] connecting to {args.host}:{args.port} ...")
    # On a cold start the RPC port can be open (TCP listen backlog) BEFORE Carla's
    # RPC server actually answers, so a single long-timeout call just hangs. Poll
    # get_server_version with a FRESH client each attempt until it responds (this
    # is what reliably catches the server becoming ready); --timeout is the overall
    # budget for the server to come up.
    import time
    deadline = time.time() + args.timeout
    while True:
        try:
            client = carla.Client(args.host, args.port)
            client.set_timeout(5.0)
            srv = client.get_server_version()
            print(f"[load_opendrive_world] client {client.get_client_version()} / "
                  f"server {srv}")
            break
        except RuntimeError:
            if time.time() >= deadline:
                print(f"[load_opendrive_world] FAIL: CARLA RPC not responsive within "
                      f"{args.timeout:.0f}s (server still starting up?)")
                return 3
            time.sleep(3)
    # generous timeout for the heavier generate_opendrive_world mesh build
    client.set_timeout(max(args.timeout, 60.0))

    params = carla.OpendriveGenerationParameters(
        vertex_distance=args.vertex_distance,
        max_road_length=args.max_road_length,
        wall_height=args.wall_height,
        additional_width=args.extra_width,
        smooth_junctions=True,
        enable_mesh_visibility=True,
        enable_pedestrian_navigation=False,
    )
    print(f"[load_opendrive_world] generating world from {args.xodr} ...")
    world = client.generate_opendrive_world(xodr, params)

    if args.sync:
        s = world.get_settings()
        s.synchronous_mode = True
        s.fixed_delta_seconds = args.delta
        world.apply_settings(s)
        print(f"[load_opendrive_world] synchronous mode on, delta={args.delta}s")

    m = world.get_map()
    print(f"[load_opendrive_world] OK: world map '{m.name}', "
          f"{len(m.get_spawn_points())} spawn points. Bridge can connect now.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
