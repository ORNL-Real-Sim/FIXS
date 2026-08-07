"""
set_spectator_view.py - move the CARLA spectator to a useful overhead view.

In -game mode the spectator spawns at the world origin, far from the map, so the
scene looks like a tiny floating cluster. This points the spectator at the centre
of the traffic-light actors (the intersection area) from an angled overhead view.

Usage:
  python set_spectator_view.py [--host H] [--port P] [--height M] [--pitch DEG]
"""
import argparse
import os
import sys

# Get onto the interpreter carla.json names before importing carla - see
# sumo_carla_tl_sync.py for why this sits above the import rather than in main().
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import carla_env_setup as env  # noqa: E402
if __name__ == "__main__":
    env.reexec_under_configured(__file__, tag="carla")

import carla  # noqa: E402  (deliberately after the re-exec above)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="localhost")
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--height", type=float, default=180.0, help="metres above the scene")
    ap.add_argument("--pitch", type=float, default=-55.0, help="camera pitch (deg, negative looks down)")
    ap.add_argument("--load-map", default=None,
                    help="load this CARLA map (load_world) before positioning - use when -game "
                         "fell back to a default map after a fresh import")
    args = ap.parse_args()

    client = carla.Client(args.host, args.port)
    client.set_timeout(60.0)  # load_world can take a while
    if args.load_map:
        print(f"[MAP] loading {args.load_map} ...")
        world = client.load_world(args.load_map)
    else:
        world = client.get_world()

    tls = world.get_actors().filter("traffic.traffic_light*")
    if len(tls) > 0:
        locs = [t.get_transform().location for t in tls]
        cx = sum(l.x for l in locs) / len(locs)
        cy = sum(l.y for l in locs) / len(locs)
        cz = sum(l.z for l in locs) / len(locs)
        anchor = f"{len(tls)} traffic lights"
    else:
        spawns = world.get_map().get_spawn_points()
        if spawns:
            cx, cy, cz = spawns[0].location.x, spawns[0].location.y, spawns[0].location.z
            anchor = "first spawn point"
        else:
            cx = cy = cz = 0.0
            anchor = "world origin (no actors/spawns found)"

    spectator = world.get_spectator()
    transform = carla.Transform(
        carla.Location(x=cx, y=cy, z=cz + args.height),
        carla.Rotation(pitch=args.pitch, yaw=0.0, roll=0.0),
    )
    spectator.set_transform(transform)
    print(f"[VIEW] spectator -> ({cx:.1f}, {cy:.1f}, {cz + args.height:.1f}) "
          f"pitch={args.pitch} (anchored on {anchor})")


if __name__ == "__main__":
    main()
