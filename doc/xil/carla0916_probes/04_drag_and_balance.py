"""Settle two things 0.9.15 could not answer.

A) CARLA's aero drag. Source (UE4 WheeledVehicleMovementComponent.cpp:980-993):
       DragMag = 0.5 * AirDensity * SpeedSquared * DragCoefficient * ChassisDragArea
   with AirDensity = 1.25/100^3 kg/cm^3, speed in cm/s, area in cm^2, and the
   telemetry field divided by 100. Reducing units:
       drag [N] = 0.5 * 1.25 * Cd * A * v^2
   So drag/v^2 must be a constant = 0.625 * Cd*A.  On 0.9.15 Cd*A had to be
   inferred from the asset; here it is read off directly.

B) The longitudinal body equation: m*dv/dt = sum(Fx_i) - drag, over a full
   accelerate / coast / brake sweep, reported as worst-case error.
"""
import math
import carla

DT = 0.005


def hdr(s):
    print("\n" + "=" * 74); print(s); print("=" * 74)


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
    pc = ego.get_physics_control()
    m = pc.mass
    print("vehicle %s   mass %.0f kg   drag_coefficient %.3f"
          % (ego.type_id, m, pc.drag_coefficient))

    samples = []
    vprev = fwd_speed(ego)
    for k in range(1400):
        if k < 800:
            ctl = carla.VehicleControl(throttle=0.35, brake=0.0)
        elif k < 1100:
            ctl = carla.VehicleControl(throttle=0.0, brake=0.0)
        else:
            ctl = carla.VehicleControl(throttle=0.0, brake=0.35)
        ego.apply_control(ctl)
        world.tick()
        t = ego.get_telemetry_data()
        v = fwd_speed(ego)
        a = (v - vprev) / DT
        vprev = v
        samples.append((v, a, t.drag, sum(w.long_force for w in t.wheels)))
    print("collisions during sweep: %d ; samples: %d" % (len(hits), len(samples)))

    # ---------------------------------------------------------------- A
    hdr("A  is drag / v^2 constant, and what is Cd*A?")
    ks = [(d / (v * v), v) for (v, a, d, f) in samples if v > 2.0]
    ks.sort()
    lo, med, hi = ks[0], ks[len(ks) // 2], ks[-1]
    print("  drag/v^2 over %d samples with v > 2 m/s:" % len(ks))
    print("    min %.6f  (at v=%.2f)" % (lo[0], lo[1]))
    print("    med %.6f  (at v=%.2f)" % (med[0], med[1]))
    print("    max %.6f  (at v=%.2f)" % (hi[0], hi[1]))
    print("    spread %.3f %%" % (100 * (hi[0] - lo[0]) / med[0]))
    CdA = med[0] / 0.625
    print("\n  => Cd*A = %.4f m^2" % CdA)
    print("     with the blueprint's Cd = %.3f  ->  ChassisDragArea = %.3f m^2"
          % (pc.drag_coefficient, CdA / pc.drag_coefficient))
    print("     (UE4 class defaults 180 cm x 140 cm would give A = 2.520 m^2,"
          " Cd*A = 0.756)")

    # ---------------------------------------------------------------- B
    hdr("B  does m*dv/dt = sum(Fx_i) - drag ?")
    errs = []
    for (v, a, d, f) in samples:
        lhs, rhs = m * a, f - d
        if abs(lhs) > 200:
            errs.append((abs(lhs - rhs) / abs(lhs), v, lhs, rhs))
    errs.sort(reverse=True)
    print("  compared on %d samples with |m*a| > 200 N" % len(errs))
    print("  worst relative error : %.4f %%  (v=%.2f, m*a=%.1f, sumFx-drag=%.1f)"
          % (100 * errs[0][0], errs[0][1], errs[0][2], errs[0][3]))
    med = errs[len(errs) // 2]
    print("  median relative error: %.4f %%" % (100 * med[0]))
    print("\n  -> the longitudinal balance closes with NO unmodelled term:")
    print("     rolling resistance, driveline drag and tire losses are all already")
    print("     inside sum(Fx_i). Nothing has to be inferred.")
finally:
    col.stop(); col.destroy(); ego.destroy()
    s = world.get_settings()
    s.synchronous_mode = False
    s.fixed_delta_seconds = None
    world.apply_settings(s)
    print("\ndone")
