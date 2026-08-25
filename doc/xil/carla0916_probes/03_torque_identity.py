"""Two decisive checks on CARLA 0.9.16 telemetry.

H1: telemetry.wheels[i].torque is identically -long_force * radius, i.e. it is the
    tire-to-wheel reaction torque (our Trq_T2W), NOT the drivetrain torque.

H2: during a coast (throttle 0, brake 0) the wheel-spin DOF obeys
        J_w * dw/dt = torque
    which is the same structure as the CarMaker wheel equation with Taxl = 0.
    If it closes, J_w is recoverable and the dyno can be coupled by supplying Taxl.
"""
import math
import carla

DT = 0.005


def hdr(s):
    print("\n" + "=" * 74); print(s); print("=" * 74)


def speed(a):
    v = a.get_velocity()
    return math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)


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
    R = [w.radius / 100.0 for w in pc.wheels]

    # --------------------------------------------------------------- H1
    hdr("H1  torque == -long_force * radius ?")
    print("  gentle throttle then brake, checking every tick, 4 wheels\n")
    worst, n = 0.0, 0
    for k in range(600):
        thr, brk = (0.3, 0.0) if k < 400 else (0.0, 0.4)
        ego.apply_control(carla.VehicleControl(throttle=thr, brake=brk))
        world.tick()
        t = ego.get_telemetry_data()
        for i, w in enumerate(t.wheels):
            pred = -w.long_force * R[i]
            if abs(pred) > 1.0:
                worst = max(worst, abs(w.torque - pred) / abs(pred))
                n += 1
    print("  samples compared : %d" % n)
    print("  worst relative error between torque and -Fx*r : %.3e" % worst)
    print("  -> %s" % ("IDENTITY: torque carries no information beyond long_force"
                       if worst < 1e-4 else "NOT an identity"))

    # --------------------------------------------------------------- H2
    hdr("H2  does J_w * dw/dt = torque during a coast?")
    # get up to speed, then release everything
    for _ in range(500):
        ego.apply_control(carla.VehicleControl(throttle=0.45, brake=0.0))
        world.tick()
    print("  released throttle at v = %.2f m/s\n" % speed(ego))

    prev = [w.omega for w in ego.get_telemetry_data().wheels]
    rows = []
    n0 = len(hits)
    for k in range(600):
        ego.apply_control(carla.VehicleControl(throttle=0.0, brake=0.0))
        world.tick()
        t = ego.get_telemetry_data()
        cur = [w.omega for w in t.wheels]
        dwdt = [(cur[i] - prev[i]) / DT for i in range(4)]
        rows.append((k * DT, speed(ego), list(cur), dwdt,
                     [w.torque for w in t.wheels]))
        prev = cur
    print("  collisions during coast: %d" % (len(hits) - n0))

    print("\n  implied J_w = torque / (dw/dt), per wheel, sampled:\n")
    print("  %7s %7s | %8s %9s %9s | %9s" %
          ("t[s]", "v", "om[2]", "dw/dt[2]", "trq[2]", "J_w[2]"))
    Js = [[], [], [], []]
    for (tt, v, om, dw, tq) in rows:
        for i in range(4):
            if abs(dw[i]) > 1.0:
                Js[i].append(tq[i] / dw[i])
        if abs(dw[2]) > 1.0 and len(rows) and abs(tt * 1000) % 500 < 5:
            print("  %7.3f %7.3f | %8.3f %9.2f %9.2f | %9.3f"
                  % (tt, v, om[2], dw[2], tq[2], tq[2] / dw[2]))

    print()
    for i in range(4):
        if Js[i]:
            Js[i].sort()
            med = Js[i][len(Js[i]) // 2]
            print("  wheel %d : median implied J_w = %8.3f kg m^2   (n=%d)"
                  % (i, med, len(Js[i])))

    m = pc.mass
    print("\n  for reference, UE4 UVehicleWheel default: J = 0.5*m_w*r^2 "
          "= 0.5*20*%.3f^2 = %.3f kg m^2" % (R[0], 0.5 * 20 * R[0] ** 2))
    print("  vehicle mass %.0f kg" % m)
    print("\n  NOTE: during a coast the drivetrain is still attached, so any")
    print("  discrepancy is the reflected engine/gearbox inertia, not an error.")

    # --------------------------------------------------------------- extra
    hdr("H2b  same test with the vehicle disengaged (handbrake off, gear neutral)")
    ego.apply_control(carla.VehicleControl(throttle=0.0, brake=0.0,
                                           manual_gear_shift=True, gear=0))
    for _ in range(100):
        world.tick()
    prev = [w.omega for w in ego.get_telemetry_data().wheels]
    Js2 = [[], [], [], []]
    for k in range(400):
        ego.apply_control(carla.VehicleControl(throttle=0.0, brake=0.0,
                                               manual_gear_shift=True, gear=0))
        world.tick()
        t = ego.get_telemetry_data()
        cur = [w.omega for w in t.wheels]
        for i in range(4):
            dw = (cur[i] - prev[i]) / DT
            if abs(dw) > 1.0:
                Js2[i].append(t.wheels[i].torque / dw)
        prev = cur
    print("  v = %.2f m/s, gear = %d\n" % (speed(ego),
                                           ego.get_telemetry_data().gear))
    for i in range(4):
        if Js2[i]:
            Js2[i].sort()
            print("  wheel %d : median implied J_w = %8.3f kg m^2   (n=%d)"
                  % (i, Js2[i][len(Js2[i]) // 2], len(Js2[i])))
        else:
            print("  wheel %d : no usable samples" % i)
finally:
    col.stop(); col.destroy(); ego.destroy()
    s = world.get_settings()
    s.synchronous_mode = False
    s.fixed_delta_seconds = None
    world.apply_settings(s)
    print("\ndone; collisions total %d" % len(hits))
