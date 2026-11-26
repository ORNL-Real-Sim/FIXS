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
        R["color_attribute"] = actor.attributes.get("color")
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

        R["turning_circle"] = _turning_circle(world, cmap, actor, carla, m)

        tm = client.get_trafficmanager(tm_port)
        tm.set_synchronous_mode(True)
        actor.apply_control(carla.VehicleControl(brake=0.0))
        actor.set_autopilot(True, tm_port)
        tm.ignore_lights_percentage(actor, 100)
        tm.set_desired_speed(actor, 35.0)
        dist, last, zmin = 0.0, actor.get_location(), 1e9
        for _ in range(int(25 / DT)):
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
    R["accepted_mismatch"] = _accepted_mismatch(m)
    _report(R, rows)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
        with open(os.path.join(out_dir, "verify_{}.json".format(bp_id)), "w", encoding="utf-8") as fh:
            json.dump(R, fh, indent=1, default=str)
    accepted = R["accepted_mismatch"]
    ok = (all(r[4] or r[0] in accepted for r in rows) and abs(R["settle"]["z_above_road"]) < 0.05
          and R["autopilot_m"] > 100)
    return 0 if ok else 1


def _accepted_mismatch(m):
    """Dimension checks prep was told to accept (--allow-mismatch), from rig/prep_report.json."""
    try:
        with open(os.path.join(m["dir"], vmanifest.RIG_REPORT), "r", encoding="utf-8") as fh:
            return list(json.load(fh).get("accepted_mismatch") or [])
    except (OSError, ValueError):
        return []


def _report(R, rows):
    print("[verify] {}  colour attribute: {} (spawned with {})".format(
        R["id"], R["has_color"], R.get("color_attribute")))
    print("[verify] settle: {}".format({k: round(v, 3) for k, v in R["settle"].items()}))
    print(vmanifest.format_checks(rows))
    if R.get("accepted_mismatch"):
        print("[verify] accepted at prep (rig/prep_report.json): {}".format(", ".join(R["accepted_mismatch"])))
    print("[verify] 0-50 km/h {} s over {:.1f} m; stop {:.2f} s over {:.1f} m".format(
        None if R["t_0_50_s"] is None else round(R["t_0_50_s"], 2), R["dist_0_50_m"], R["stop_s"], R["stop_m"]))
    print("[verify] full lock FL/FR deg: left {} right {}".format(
        R["steer_full_left_FL_FR_deg"], R["steer_full_right_FL_FR_deg"]))
    tc = R.get("turning_circle") or {}
    print("[verify] full-lock circle at ~5 km/h: centre path {} m, kerb to kerb {} m ({} deg swept)".format(
        tc.get("centre_path_diameter_m"), tc.get("kerb_to_kerb_diameter_m"), tc.get("heading_swept_deg")))
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


def fit_circle(pts):
    """(radius, max residual) of the least-squares circle through [(x, y)]."""
    import numpy as np
    p = np.asarray(pts, float)
    A = np.c_[2 * p[:, 0], 2 * p[:, 1], np.ones(len(p))]
    cx, cy, k = np.linalg.lstsq(A, (p ** 2).sum(1), rcond=None)[0]
    r = math.sqrt(k + cx * cx + cy * cy)
    return r, float(np.abs(np.hypot(p[:, 0] - cx, p[:, 1] - cy) - r).max())


def _open_site(cmap, carla):
    best = None
    for a, _ in cmap.get_topology():
        if a.is_junction:
            e = a.get_junction().bounding_box.extent
            if best is None or min(e.x, e.y) > best[0]:
                best = (min(e.x, e.y), a.get_junction())
    loc = best[1].bounding_box.location
    wp = cmap.get_waypoint(loc)
    return carla.Transform(carla.Location(loc.x, loc.y, wp.transform.location.z + 0.3),
                           carla.Rotation(yaw=wp.transform.rotation.yaw))


def _turning_circle(world, cmap, actor, carla, m):
    """Full right lock at ~5 km/h in the widest junction: centre-path and kerb-to-kerb diameters."""
    actor.set_transform(_open_site(cmap, carla))
    actor.apply_control(carla.VehicleControl(brake=1.0))
    for _ in range(100):
        world.tick()
    inv = actor.get_transform().get_inverse_matrix()
    fl = actor.get_physics_control().wheels[0].position  # world cm
    fl_local = [sum(inv[r][k] * c for k, c in enumerate((fl.x / 100, fl.y / 100, fl.z / 100, 1.0)))
                for r in range(3)]
    # the outer (left) front tyre's outer edge, in the vehicle frame
    front_x, edge_y = fl_local[0], fl_local[1] - m["wheels"]["width_m"] / 2
    target, centre, kerb, swept, last = 5 / 3.6, [], [], 0.0, None
    for n in range(int(60 / DT)):
        v = _speed(actor)
        brk = 0.3 if v > target * 1.3 else 0.0
        thr = 0.0 if brk else max(0.0, min(0.6, 0.35 + 0.8 * (target - v)))
        actor.apply_control(carla.VehicleControl(throttle=thr, brake=brk, steer=1.0))
        world.tick()
        t = actor.get_transform()
        if n * DT < 6.0:
            last = t.rotation.yaw
            continue
        swept += abs((t.rotation.yaw - last + 180) % 360 - 180)
        last = t.rotation.yaw
        o = t.transform(carla.Location(front_x, edge_y, 0))
        centre.append((t.location.x, t.location.y))
        kerb.append((o.x, o.y))
        if swept >= 360.0:
            break
    actor.apply_control(carla.VehicleControl(brake=1.0))
    for _ in range(50):
        world.tick()
    if len(centre) < 10:
        return {"heading_swept_deg": swept}
    rc, ec = fit_circle(centre)
    rk, ek = fit_circle(kerb)
    return {"heading_swept_deg": round(swept, 1), "centre_path_diameter_m": round(2 * rc, 2),
            "centre_fit_resid_m": round(ec, 3), "kerb_to_kerb_diameter_m": round(2 * rk, 2),
            "kerb_fit_resid_m": round(ek, 3)}
