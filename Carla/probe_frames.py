"""probe_frames.py -- one row per RENDERED CARLA frame, for the flicker question.

probe_spectator.py samples on the wall clock (2 Hz by default), which cannot see
anything that happens frame to frame. This attaches an on_tick callback instead,
so every server tick -- i.e. every frame the window draws -- produces exactly one
row, and records BOTH the camera and the vehicle it is following from the SAME
snapshot (no RPC skew between the two).

Per frame:
    frame, sim_t, dsim        server frame id / clock
    wall_dt                   wall-clock gap since the previous frame -> is the
                              display evenly paced, or does it judder?
    cam_x/y/z, cam_yaw        the spectator as the bridge left it
    ego_x/y/z, ego_yaw        the followed vehicle, same snapshot
    d_xy                      camera-to-vehicle horizontal offset. The follow is
                              a rigid snap, so this is 0 when it is correct; any
                              nonzero value is the vehicle sliding off centre.
    agl                       cam_z - ego_z. Constant = no zoom breathing; this
                              is the "is it the camera zoom?" measurement.
    ego_step, cam_step        metres each moved since the previous frame -> do
                              they move on the same frame, and does the world
                              advance smoothly or in bursts?

READ-ONLY: connects as a second client and only listens. No spawn, no tick, no
world-settings change.
"""
from __future__ import annotations

import argparse
import csv
import math
import statistics
import sys
import time

import carla


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--timeout", type=float, default=60.0)
    ap.add_argument("--seconds", type=float, default=60.0,
                    help="how long to record once the follow engages")
    ap.add_argument("--wait", type=float, default=0.0,
                    help="wait this long for the camera to start moving before "
                         "recording (0 = record immediately)")
    ap.add_argument("--csv", required=True)
    args = ap.parse_args()

    client = carla.Client(args.host, args.port)
    client.set_timeout(args.timeout)
    world = client.get_world()
    world.wait_for_tick(args.timeout)
    spectator = world.get_spectator()
    spec_id = spectator.id
    print(f"[frames] map {world.get_map().name}, spectator actor {spec_id}", flush=True)

    # The bridge leaves every mirrored vehicle at the blueprint default role_name,
    # so the followed actor is found geometrically -- nearest vehicle to the
    # camera -- and then LOCKED by id, so a row is never silently about a
    # different car when two pass close.
    def followed_id():
        sloc = spectator.get_transform().location
        best, best_d = None, float("inf")
        for a in world.get_actors().filter("vehicle.*"):
            p = a.get_location()
            d = math.hypot(p.x - sloc.x, p.y - sloc.y)
            if d < best_d:
                best, best_d = a.id, d
        return best, best_d

    if args.wait > 0:
        print(f"[frames] waiting up to {args.wait:g}s for the camera to move "
              f"(follow engages when the ego departs)", flush=True)
        t0, prev = time.time(), None
        while time.time() - t0 < args.wait:
            loc = spectator.get_transform().location
            cur = (loc.x, loc.y, loc.z)
            if prev is not None and math.dist(cur, prev) > 0.05:
                print(f"[frames] camera moving at {cur}", flush=True)
                break
            prev = cur
            time.sleep(0.25)
        else:
            print("[frames] camera never moved; recording anyway", flush=True)

    ego_id, d0 = followed_id()
    print(f"[frames] following actor {ego_id} ({d0:.2f} m from the camera)", flush=True)

    rows = []
    t_start = time.perf_counter()

    def on_tick(snap):
        rows.append((time.perf_counter(), snap))

    cb = world.on_tick(on_tick)
    try:
        time.sleep(args.seconds)
    except KeyboardInterrupt:
        pass
    world.remove_on_tick(cb)
    print(f"[frames] {len(rows)} frames in {time.perf_counter() - t_start:.1f}s", flush=True)

    out, prev = [], None
    for wall, snap in rows:
        s = snap.find(spec_id)
        e = snap.find(ego_id)
        if s is None or e is None:
            continue
        st, et = s.get_transform(), e.get_transform()
        d_xy = math.hypot(st.location.x - et.location.x, st.location.y - et.location.y)
        agl = st.location.z - et.location.z
        ego_step = cam_step = wall_dt = float("nan")
        if prev is not None:
            pw, pst, pet = prev
            wall_dt = wall - pw
            ego_step = math.hypot(et.location.x - pet.location.x,
                                  et.location.y - pet.location.y)
            cam_step = math.hypot(st.location.x - pst.location.x,
                                  st.location.y - pst.location.y)
        out.append({
            "frame": snap.frame,
            "sim_t": round(snap.timestamp.elapsed_seconds, 4),
            "dsim": round(snap.timestamp.delta_seconds, 5),
            "wall_dt": round(wall_dt, 5),
            "cam_x": round(st.location.x, 3), "cam_y": round(st.location.y, 3),
            "cam_z": round(st.location.z, 3), "cam_yaw": round(st.rotation.yaw, 3),
            "cam_pitch": round(st.rotation.pitch, 3),
            "ego_x": round(et.location.x, 3), "ego_y": round(et.location.y, 3),
            "ego_z": round(et.location.z, 3), "ego_yaw": round(et.rotation.yaw, 3),
            "d_xy": round(d_xy, 4), "agl": round(agl, 4),
            "ego_step": round(ego_step, 4), "cam_step": round(cam_step, 4),
        })
        prev = (wall, st, et)

    if not out:
        print("[frames] nothing recorded", flush=True)
        return 1

    with open(args.csv, "w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=list(out[0].keys()))
        w.writeheader()
        w.writerows(out)

    def stats(key, skip_first=True):
        v = [r[key] for r in out[1:] if not math.isnan(r[key])] if skip_first \
            else [r[key] for r in out if not math.isnan(r[key])]
        if not v:
            return "n/a"
        return (f"mean {statistics.mean(v):8.4f}  sd {statistics.pstdev(v):8.4f}  "
                f"min {min(v):8.4f}  max {max(v):8.4f}")

    print(f"\n[frames] {len(out)} frames -> {args.csv}")
    print(f"  d_xy     (camera off the vehicle, m)   {stats('d_xy', False)}")
    print(f"  agl      (camera height above it, m)   {stats('agl', False)}")
    print(f"  dsim     (server step, s)              {stats('dsim', False)}")
    print(f"  wall_dt  (wall gap between frames, s)  {stats('wall_dt')}")
    print(f"  ego_step (m/frame)                     {stats('ego_step')}")
    print(f"  cam_step (m/frame)                     {stats('cam_step')}")
    still = sum(1 for r in out[1:] if r["ego_step"] < 1e-4)
    print(f"  frames where the ego did not move:     {still}/{len(out) - 1}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
