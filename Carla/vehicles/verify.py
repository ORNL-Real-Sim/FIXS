"""Spawn-and-drive check of an installed library vehicle on a running CARLA server.

Run through `import_vehicle.py --verify <id> --port <rpc>`. Use a server nobody else is
driving: the check switches the world to synchronous mode for its duration and puts the
previous settings back afterwards. It refuses a world that is already synchronous.

Checks, against the vehicle's manifest: the blueprint exists and takes `color`; the car
settles on its wheels; wheelbase, track and wheel radius after applying the manifest
physics; launch 0-50 km/h and a full stop; full-lock front wheel angle.
"""
import json
import math
import os

from . import catalog
from . import manifest as vmanifest

DT = 0.02


def geometry_rows(m, wheel_positions_cm, radius_cm):
    """Rows like manifest.check_dimensions, from CARLA's wheel positions (FL, FR, RL, RR)."""
    fl, fr, rl, rr = [[c / 100.0 for c in p] for p in wheel_positions_cm]

    def dist(a, b):
        return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))
    measured = {"wheelbase_m": (dist(fl, rl) + dist(fr, rr)) / 2, "track_front_m": dist(fl, fr),
                "track_rear_m": dist(rl, rr), "wheel_radius_m": radius_cm / 100.0}
    return vmanifest.check_dimensions(measured, m)


def main_check(bp_id, host, port, tm_port, color=None, out_dir=None):
    import carla

    path = catalog.installed_manifest(bp_id)
    if not path:
        print("[verify] {} has no installed manifest under {}".format(bp_id, catalog.home()))
        return 2
    m = vmanifest.load(path)
    client = carla.Client(host, port)
    client.set_timeout(60.0)
    world = client.get_world()
    orig = world.get_settings()
    if orig.synchronous_mode:
        print("[verify] the world on {}:{} is already synchronous - someone else drives it. "
              "Run the check on your own server.".format(host, port))
        return 2
    R = {"id": bp_id, "server": "{}:{}".format(host, port)}
    actor = None
    tm = None
    try:
        lib = world.get_blueprint_library()
        try:
            bp = lib.find(bp_id)
        except (IndexError, RuntimeError):
            print("[verify] {} is not in this server's blueprint library (restart it after "
                  "installing?)".format(bp_id))
            return 1
        R["has_color"] = bp.has_attribute("color")
        col = color or ",".join(str(c) for c in m["factory"]["recommended_colors"][0])
        if R["has_color"]:
            bp.set_attribute("color", col)
        s = world.get_settings()
        s.synchronous_mode, s.fixed_delta_seconds = True, DT
        world.apply_settings(s)
        cmap = world.get_map()
        sp = _straight_spawn(cmap)
        sp.location.z += 0.3
        actor = world.spawn_actor(bp, sp)
        pc = actor.get_physics_control()
        vmanifest.apply_physics(pc, vmanifest.physics_plan(m), carla)
        actor.apply_physics_control(pc)
        for _ in range(150):
            world.tick()
        t = actor.get_transform()
        road = cmap.get_waypoint(t.location).transform.location.z
        pc = actor.get_physics_control()
        R["settle"] = {"z_above_road": t.location.z - road, "pitch": t.rotation.pitch,
                       "roll": t.rotation.roll, "speed": _speed(actor)}
        R["physics"] = {"mass": pc.mass, "drag": pc.drag_coefficient,
                        "max_steer": [w.max_steer_angle for w in pc.wheels],
                        "radius_cm": [w.radius for w in pc.wheels]}
        rows = geometry_rows(m, [(w.position.x, w.position.y, w.position.z) for w in pc.wheels],
                             pc.wheels[0].radius)
        R["geometry"] = [list(r) for r in rows]

        t0, start, t50 = world.get_snapshot().timestamp.elapsed_seconds, actor.get_location(), None
        for _ in range(int(20 / DT)):
            actor.apply_control(carla.VehicleControl(throttle=1.0))
            world.tick()
            if _speed(actor) >= 50 / 3.6:
                t50 = world.get_snapshot().timestamp.elapsed_seconds - t0
                break
        R["t_0_50_s"], R["dist_0_50_m"] = t50, actor.get_location().distance(start)
        b0, bloc = world.get_snapshot().timestamp.elapsed_seconds, actor.get_location()
        for _ in range(int(15 / DT)):
            actor.apply_control(carla.VehicleControl(brake=1.0))
            world.tick()
            if _speed(actor) < 0.05:
                break
        R["stop_s"] = world.get_snapshot().timestamp.elapsed_seconds - b0
        R["stop_m"] = actor.get_location().distance(bloc)
        for name, st in (("full_left", -1.0), ("full_right", 1.0)):
            actor.apply_control(carla.VehicleControl(steer=st, brake=1.0))
            for _ in range(50):
                world.tick()
            R["steer_{}_FL_FR_deg".format(name)] = [
                round(actor.get_wheel_steer_angle(carla.VehicleWheelLocation.FL_Wheel), 2),
                round(actor.get_wheel_steer_angle(carla.VehicleWheelLocation.FR_Wheel), 2)]

        tm = client.get_trafficmanager(tm_port)
        tm.set_synchronous_mode(True)
        actor.apply_control(carla.VehicleControl(brake=0.0))
        actor.set_autopilot(True, tm_port)
        dist, last, zmin = 0.0, actor.get_location(), 1e9
        for _ in range(int(20 / DT)):
            world.tick()
            loc = actor.get_location()
            dist += loc.distance(last)
            last = loc
            zmin = min(zmin, loc.z - cmap.get_waypoint(loc).transform.location.z)
        actor.set_autopilot(False, tm_port)
        R["autopilot_m"], R["autopilot_min_z_above_lane"] = dist, zmin
    finally:
        if actor is not None:
            actor.destroy()
        if tm is not None:
            tm.set_synchronous_mode(False)
        world.apply_settings(orig)
    _report(R, rows)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
        with open(os.path.join(out_dir, "verify_{}.json".format(bp_id)), "w", encoding="utf-8") as fh:
            json.dump(R, fh, indent=1, default=str)
    ok = all(r[4] for r in rows) and abs(R["settle"]["z_above_road"]) < 0.05 and R["autopilot_m"] > 20
    return 0 if ok else 1


def _report(R, rows):
    print("[verify] {}  colour attribute: {}".format(R["id"], R["has_color"]))
    print("[verify] settle: {}".format({k: round(v, 3) for k, v in R["settle"].items()}))
    print(vmanifest.format_checks(rows))
    print("[verify] 0-50 km/h {} s over {:.1f} m; stop {:.2f} s over {:.1f} m".format(
        None if R["t_0_50_s"] is None else round(R["t_0_50_s"], 2), R["dist_0_50_m"], R["stop_s"], R["stop_m"]))
    print("[verify] full lock FL/FR deg: left {} right {}".format(
        R["steer_full_left_FL_FR_deg"], R["steer_full_right_FL_FR_deg"]))
    print("[verify] autopilot drove {:.0f} m, lowest body z above lane {:.3f} m".format(
        R["autopilot_m"], R["autopilot_min_z_above_lane"]))


def _speed(a):
    v = a.get_velocity()
    return math.sqrt(v.x ** 2 + v.y ** 2 + v.z ** 2)


def _straight_spawn(cmap, min_len=70.0):
    for sp in cmap.get_spawn_points():
        wp = cmap.get_waypoint(sp.location)
        d, cur, ok = 0.0, wp, True
        while d < min_len:
            nx = cur.next(2.0)
            if not nx or nx[0].is_junction or abs(
                    (nx[0].transform.rotation.yaw - wp.transform.rotation.yaw + 180) % 360 - 180) > 3:
                ok = False
                break
            cur, d = nx[0], d + 2.0
        if ok:
            return sp
    raise RuntimeError("no straight 70 m stretch from any spawn point on this map")
