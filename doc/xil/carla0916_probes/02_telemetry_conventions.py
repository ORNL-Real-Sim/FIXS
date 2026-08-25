"""Pin down the sign and index conventions of CARLA 0.9.16 get_telemetry_data.

Runs the ego down a straight stretch with a collision sensor watching, so any
contaminated sample can be thrown out rather than reasoned about.
"""
import math
import carla

DT = 0.01


def hdr(s):
    print("\n" + "=" * 74); print(s); print("=" * 74)


def speed(a):
    v = a.get_velocity()
    return math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)


def main():
    c = carla.Client("127.0.0.1", 2000)
    c.set_timeout(60.0)
    world = c.get_world()
    s = world.get_settings()
    s.synchronous_mode = True
    s.fixed_delta_seconds = DT
    world.apply_settings(s)

    bl = world.get_blueprint_library()
    # longest straight we can find
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
            if run > 120:
                break
        if run > bestlen:
            bestlen, best = run, wp
    print("straight run found: %.0f m" % bestlen)

    tf = best.transform
    tf.location.z += 0.5
    ego = world.spawn_actor(bl.filter("vehicle.tesla.model3")[0], tf)
    hits = []
    col = world.spawn_actor(bl.find("sensor.other.collision"),
                            carla.Transform(), attach_to=ego)
    col.listen(lambda e: hits.append(e))
    for _ in range(50):
        world.tick()

    try:
        # ------------------------------------------------ index -> corner
        hdr("1  which telemetry index is which corner?")
        pc = ego.get_physics_control()
        print("  %-6s %9s %13s %13s %11s" %
              ("idx", "radius[m]", "max_steer", "max_brake", "max_handbrake"))
        for i, w in enumerate(pc.wheels):
            print("  %-6d %9.3f %13.1f %13.1f %11.1f"
                  % (i, w.radius / 100.0, w.max_steer_angle,
                     w.max_brake_torque, w.max_handbrake_torque))
        front = [i for i, w in enumerate(pc.wheels) if w.max_steer_angle > 0]
        rear = [i for i, w in enumerate(pc.wheels) if w.max_steer_angle == 0]
        print("\n  steerable (=front): %s     non-steerable (=rear): %s" % (front, rear))
        R = [w.radius / 100.0 for w in pc.wheels]

        # ------------------------------------------------ drive
        hdr("2  steady drive at throttle 0.5 -- torque sign, slip definition")
        print("  kappa_check = (omega*r - v) / max(|v|, 1)   [PhysX longitudinal slip]\n")
        print("  %6s %7s | %8s %8s %8s | %9s %9s | %8s %8s" %
              ("t[s]", "v", "om[0]", "om[2]", "-", "trq[0]", "trq[2]",
               "slip[0]", "kap[0]"))
        n0 = len(hits)
        for k in range(700):
            ego.apply_control(carla.VehicleControl(throttle=0.5, brake=0.0, steer=0.0))
            world.tick()
            if k % 100 == 99:
                t = ego.get_telemetry_data()
                v = speed(ego)
                w = t.wheels
                kap0 = (w[0].omega * R[0] - v) / max(abs(v), 1.0)
                print("  %6.2f %7.3f | %8.3f %8.3f %8s | %9.1f %9.1f | %8.4f %8.4f"
                      % (k * DT, v, w[0].omega, w[2].omega, "",
                         w[0].torque, w[2].torque, w[0].long_slip, kap0))
        print("  collisions during this phase: %d" % (len(hits) - n0))

        # ------------------------------------------------ which axle is driven
        hdr("3  which axle is driven? compare omega*r against v")
        t = ego.get_telemetry_data()
        v = speed(ego)
        print("  v = %.3f m/s\n" % v)
        print("  %6s %9s %11s %11s %11s %11s" %
              ("idx", "omega", "omega*r", "omega*r - v", "long_slip", "torque"))
        for i, w in enumerate(t.wheels):
            print("  %6d %9.3f %11.3f %11.3f %11.4f %11.1f"
                  % (i, w.omega, w.omega * R[i], w.omega * R[i] - v,
                     w.long_slip, w.torque))

        # ------------------------------------------------ brake
        hdr("4  brake 0.6 from speed -- does torque go the other way?")
        print("  %6s %7s | %9s %9s | %9s %9s | %8s" %
              ("t[s]", "v", "trq[0]", "trq[2]", "Fx[0]", "Fx[2]", "slip[0]"))
        n0 = len(hits)
        for k in range(120):
            ego.apply_control(carla.VehicleControl(throttle=0.0, brake=0.6))
            world.tick()
            if k % 20 == 19:
                t = ego.get_telemetry_data()
                w = t.wheels
                print("  %6.2f %7.3f | %9.1f %9.1f | %9.1f %9.1f | %8.4f"
                      % (k * DT, speed(ego), w[0].torque, w[2].torque,
                         w[0].long_force, w[2].long_force, w[0].long_slip))
        print("  collisions during this phase: %d" % (len(hits) - n0))

        # ------------------------------------------------ Fx vs m*a
        hdr("5  does sum(long_force) explain the body acceleration?")
        print("  m = %.0f kg ; comparing sum(Fx_i) - drag against m*dv/dt\n" % pc.mass)
        print("  %6s %8s %10s %11s %11s %11s" %
              ("t[s]", "v", "m*a [N]", "sum Fx", "drag", "sumFx-drag"))
        vprev = speed(ego)
        n0 = len(hits)
        for k in range(400):
            ego.apply_control(carla.VehicleControl(throttle=0.6, brake=0.0))
            world.tick()
            t = ego.get_telemetry_data()
            v = speed(ego)
            a = (v - vprev) / DT
            vprev = v
            if k % 80 == 79:
                sfx = sum(w.long_force for w in t.wheels)
                print("  %6.2f %8.3f %10.1f %11.1f %11.1f %11.1f"
                      % (k * DT, v, pc.mass * a, sfx, t.drag, sfx - t.drag))
        print("  collisions during this phase: %d" % (len(hits) - n0))
    finally:
        col.stop()
        col.destroy()
        ego.destroy()
        s = world.get_settings()
        s.synchronous_mode = False
        s.fixed_delta_seconds = None
        world.apply_settings(s)
        print("\ndone; total collisions logged: %d" % len(hits))


main()
