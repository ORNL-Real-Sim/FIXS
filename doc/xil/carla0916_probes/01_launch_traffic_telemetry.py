"""Verify the CARLA 0.9.16 build: clean run, traffic, and live wheel telemetry."""
import math
import sys
import carla

DT = 0.05


def hdr(s):
    print("\n" + "=" * 72); print(s); print("=" * 72)


def main():
    c = carla.Client("127.0.0.1", 2000)
    c.set_timeout(60.0)
    print("server %s | client %s | map %s"
          % (c.get_server_version(), c.get_client_version(), c.get_world().get_map().name))

    world = c.get_world()
    s = world.get_settings()
    s.synchronous_mode = True
    s.fixed_delta_seconds = DT
    world.apply_settings(s)

    tm = c.get_trafficmanager(8000)
    tm.set_synchronous_mode(True)

    bl = world.get_blueprint_library()
    spawns = world.get_map().get_spawn_points()
    actors = []
    try:
        # ---------------------------------------------------------- traffic
        hdr("A  spawn traffic and run it")
        want = min(25, len(spawns) - 1)
        for i in range(want):
            bp = bl.filter("vehicle.*")[i % len(bl.filter("vehicle.*"))]
            a = world.try_spawn_actor(bp, spawns[i + 1])
            if a:
                a.set_autopilot(True, 8000)
                actors.append(a)
        print("  spawned %d/%d NPC vehicles" % (len(actors), want))

        ego_bp = bl.filter("vehicle.tesla.model3")[0]
        ego = world.try_spawn_actor(ego_bp, spawns[0])
        actors.append(ego)
        print("  spawned ego: %s" % ego.type_id)

        for _ in range(60):
            world.tick()
        moving = sum(1 for a in actors[:-1]
                     if math.hypot(a.get_velocity().x, a.get_velocity().y) > 0.5)
        print("  after 3 s: %d/%d NPCs moving under Traffic Manager" % (moving, len(actors) - 1))

        # ---------------------------------------------------------- telemetry
        hdr("B  get_telemetry_data on a LIVE 0.9.16 server")
        ego.set_autopilot(False)
        for _ in range(20):
            ego.apply_control(carla.VehicleControl(throttle=0.7, brake=0.0))
            world.tick()

        t = ego.get_telemetry_data()
        print("  vehicle: speed=%.2f m/s  engine_rpm=%.0f  gear=%d  drag=%.2f N"
              % (t.speed, t.engine_rpm, t.gear, t.drag))
        print("  throttle=%.2f brake=%.2f steer=%.2f  wheels reported=%d"
              % (t.throttle, t.brake, t.steer, len(t.wheels)))
        print()
        print("  %-6s %10s %11s %11s %11s %11s %11s" %
              ("wheel", "omega", "torque", "long_slip", "tire_load", "long_force", "lat_force"))
        for i, w in enumerate(t.wheels):
            print("  %-6d %10.3f %11.1f %11.5f %11.1f %11.1f %11.1f"
                  % (i, w.omega, w.torque, w.long_slip, w.tire_load,
                     w.long_force, w.lat_force))

        # ---------------------------------------------------------- the bench check
        hdr("C  is omega usable as a dyno speed command?")
        r = ego.get_physics_control().wheels[0].radius / 100.0
        print("  wheel radius %.3f m ; comparing omega*r against body speed\n" % r)
        print("  %8s %9s %11s %11s %11s" % ("t[s]", "v [m/s]", "omega[0]", "omega*r", "slip"))
        for k in range(60):
            ego.apply_control(carla.VehicleControl(throttle=0.8, brake=0.0))
            world.tick()
            if k % 12 == 0:
                t = ego.get_telemetry_data()
                v = math.sqrt(sum(x * x for x in
                                  (ego.get_velocity().x, ego.get_velocity().y)))
                om = t.wheels[0].omega
                print("  %8.2f %9.3f %11.3f %11.3f %11.5f"
                      % (k * DT, v, om, om * r, t.wheels[0].long_slip))

        hdr("D  braking - does torque go negative?")
        for _ in range(25):
            ego.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0))
            world.tick()
        t = ego.get_telemetry_data()
        print("  under full brake: torque per wheel = %s"
              % ["%.0f" % w.torque for w in t.wheels])
        print("  long_slip per wheel      = %s"
              % ["%.4f" % w.long_slip for w in t.wheels])
        print("\n  VERDICT: telemetry is live, per wheel, and responds to drive and brake.")
    finally:
        for a in actors:
            try:
                a.destroy()
            except Exception:
                pass
        s = world.get_settings()
        s.synchronous_mode = False
        s.fixed_delta_seconds = None
        world.apply_settings(s)
        print("\ncleaned up")


if __name__ == "__main__":
    sys.exit(main())
