"""Where does m*dv/dt = sum(Fx) - drag hold, and where does it break?

Motivation: the previous sweep gave a 0.09 % median error but a 2492 % worst
case at v = 0. PhysX enforces standstill with a velocity constraint ("sticky
tire"), not a force, so the reported long_force there is a constraint impulse,
not a tire force. That is precisely the regime where the dyno has to hand over,
so it needs to be bounded rather than averaged away.
"""
import math
import carla

DT = 0.005


def fwd_speed(a):
    v = a.get_velocity()
    f = a.get_transform().get_forward_vector()
    return v.x * f.x + v.y * f.y + v.z * f.z


c = carla.Client("127.0.0.1", 2000)
c.set_timeout(60.0)
world = c.get_world()
s = world.get_settings()
s.synchronous_mode = True
s.fixed_delta_seconds = DT
world.apply_settings(s)

bl = world.get_blueprint_library()
wps = world.get_map().generate_waypoints(2.0)
best, bestlen = None, 0
for wp in wps:
    n, run = wp, 0.0
    while True:
        nxt = n.next(2.0)
        if len(nxt) != 1 or nxt[0].is_junction:
            break
        if abs(nxt[0].transform.rotation.yaw - wp.transform.rotation.yaw) > 3:
            break
        n = nxt[0]; run += 2.0
        if run > 200:
            break
    if run > bestlen:
        bestlen, best = run, wp
tf = best.transform
tf.location.z += 0.5
ego = world.spawn_actor(bl.filter("vehicle.tesla.model3")[0], tf)
hits = []
col = world.spawn_actor(bl.find("sensor.other.collision"), carla.Transform(), attach_to=ego)
col.listen(lambda e: hits.append(e))
for _ in range(100):
    world.tick()

try:
    m = ego.get_physics_control().mass
    rows = []
    vprev = fwd_speed(ego)
    for k in range(1600):
        if k < 700:
            ctl = carla.VehicleControl(throttle=0.35, brake=0.0)
        elif k < 1000:
            ctl = carla.VehicleControl(throttle=0.0, brake=0.0)
        else:                                   # brake all the way to standstill
            ctl = carla.VehicleControl(throttle=0.0, brake=0.5)
        ego.apply_control(ctl)
        world.tick()
        t = ego.get_telemetry_data()
        v = fwd_speed(ego)
        rows.append((v, (v - vprev) / DT, t.drag,
                     sum(w.long_force for w in t.wheels)))
        vprev = v
    print("collisions: %d   final speed %.3f m/s" % (len(hits), rows[-1][0]))

    bins = [(0.0, 0.1), (0.1, 0.5), (0.5, 1.0), (1.0, 3.0),
            (3.0, 6.0), (6.0, 10.0), (10.0, 99.0)]
    print("\n  residual  r = (m*a) - (sum Fx - drag)   [N]\n")
    print("  %-14s %7s %12s %12s %12s" %
          ("speed [m/s]", "n", "median |r|", "max |r|", "med |m*a|"))
    for lo, hi in bins:
        sel = [(abs(m * a - (f - d)), abs(m * a))
               for (v, a, d, f) in rows if lo <= abs(v) < hi]
        if not sel:
            print("  %-14s %7d %12s" % ("%.1f - %.1f" % (lo, hi), 0, "-"))
            continue
        r = sorted(x[0] for x in sel)
        ma = sorted(x[1] for x in sel)
        print("  %-14s %7d %12.1f %12.1f %12.1f"
              % ("%.1f - %.1f" % (lo, hi), len(sel),
                 r[len(r) // 2], r[-1], ma[len(ma) // 2]))

    print("\n  Interpretation: a residual that stays small in absolute terms at all")
    print("  speeds means sum(Fx) - drag really is the whole longitudinal force.")
    print("  A residual that blows up only in the lowest bin is the standstill")
    print("  constraint, not a modelling gap.")
finally:
    col.stop(); col.destroy(); ego.destroy()
    s = world.get_settings()
    s.synchronous_mode = False
    s.fixed_delta_seconds = None
    world.apply_settings(s)
    print("\ndone")
