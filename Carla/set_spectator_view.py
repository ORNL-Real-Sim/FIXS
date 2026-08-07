"""
set_spectator_view.py -- point the CARLA spectator camera at a scene.
Generalized from the ANL set_spectator_view.py (was hardcoded to Roosevelt).

Default is a top-down view; pass --x/--y to center it, or --follow <role_name>
to snap above a named actor (e.g. the ego).

  python set_spectator_view.py [--host H] [--port P]
                               [--x 0 --y 0 --z 120]      # top-down over a point
                               [--follow ego]             # or above an actor by role_name
                               [--pitch -90 --yaw 0]

Requires: a Python with the carla 0.9.15 package.
"""
from __future__ import annotations
import argparse
import sys

import carla_env_setup as env

# Get onto the interpreter carla.json names BEFORE importing carla - see
# load_opendrive_world.py for why this sits above the import rather than in main().
if __name__ == "__main__":
    env.reexec_under_configured(__file__, tag="carla")

import carla  # noqa: E402  (deliberately after the re-exec above)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--timeout", type=float, default=20.0)
    ap.add_argument("--x", type=float, default=0.0)
    ap.add_argument("--y", type=float, default=0.0)
    ap.add_argument("--z", type=float, default=120.0, help="camera height (m)")
    ap.add_argument("--pitch", type=float, default=-90.0)
    ap.add_argument("--yaw", type=float, default=0.0)
    ap.add_argument("--follow", default=None,
                    help="role_name of an actor to center above (overrides --x/--y)")
    args = ap.parse_args()

    client = carla.Client(args.host, args.port)
    client.set_timeout(args.timeout)
    world = client.get_world()
    spectator = world.get_spectator()

    x, y = args.x, args.y
    if args.follow:
        match = None
        for a in world.get_actors().filter("vehicle.*"):
            if a.attributes.get("role_name") == args.follow:
                match = a
                break
        if match is None:
            print(f"[set_spectator_view] WARN: no vehicle with role_name '{args.follow}'; "
                  f"using --x/--y instead")
        else:
            loc = match.get_location()
            x, y = loc.x, loc.y

    tf = carla.Transform(carla.Location(x=x, y=y, z=args.z),
                         carla.Rotation(pitch=args.pitch, yaw=args.yaw, roll=0.0))
    spectator.set_transform(tf)
    print(f"[set_spectator_view] camera at ({x:.1f},{y:.1f},{args.z:.1f}) "
          f"pitch={args.pitch} yaw={args.yaw}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
