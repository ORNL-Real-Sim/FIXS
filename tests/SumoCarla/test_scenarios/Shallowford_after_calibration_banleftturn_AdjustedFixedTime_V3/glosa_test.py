import traci
import os
from math import sqrt

# === CONFIG ===
SUMO_BINARY = "sumo-gui"  # Change to "sumo" for headless
SUMO_CONFIG = "chattCavMpr.sumocfg"

# === GLOSA CONSTANTS ===
A_MAX = 2.6
D_MAX = 4.5
V_MAX = 13.89
GLOSA_MIN_SPEED = 5.0
GLOSA_RANGE = 100.0
GLOSA_DECEL = 2.5

# === Motion Utilities ===
def distance_at_continuous_accel(speed, time):
    return speed * time + A_MAX * time ** 2 / 2

def time_to_junction_at_continuous_accel(d, v):
    a = A_MAX
    p_half = v / a
    return -p_half + sqrt(p_half ** 2 + 2 * d / a)

def earliest_arrival(speed, distance):
    max_speed = max(V_MAX, speed)
    accel_time = min((max_speed - speed) / A_MAX,
                     time_to_junction_at_continuous_accel(distance, speed))
    remaining_dist = distance - distance_at_continuous_accel(speed, accel_time)
    remaining_time = remaining_dist / max_speed
    return accel_time + remaining_time

def glosa_maneuver(speed, distance, time_till_switch):
    t = time_till_switch
    a = A_MAX
    d = D_MAX
    u = GLOSA_MIN_SPEED
    w = max(V_MAX, speed)
    s = distance
    v = speed
    sign0 = -1
    root_argument = a * d * ((2.0 * d * (s - (w * t))) - ((v - w) ** 2) + (a * ((d * (t ** 2)) + (2.0 * (s - (t * v))))))
    if root_argument < 0:
        return v, 0
    x = (((a * (v - (d * t))) + (d * w) - sign0 * sqrt(root_argument)) / (d + a))
    y = t - (w - x) / d
    if x >= u and x <= w and y > 0 and y < t:
        return x, y
    else:
        return v, 0

def adapt_speed(vehicle_id, distance, time_to_switch):
    speed = traci.vehicle.getSpeed(vehicle_id)
    if earliest_arrival(speed, distance) < time_to_switch:
        if speed > GLOSA_MIN_SPEED:
            target_speed, duration = glosa_maneuver(speed, distance, time_to_switch)
            traci.vehicle.slowDown(vehicle_id, target_speed, int(duration * 1000))
            traci.vehicle.setColor(vehicle_id, (255, 0, 0))  # RED: GLOSA active
    else:
        traci.vehicle.slowDown(vehicle_id, V_MAX, 0)
        traci.vehicle.setColor(vehicle_id, (0, 255, 0))  # GREEN: normal

# === Main Simulation Loop ===
def run_glosa_simulation():
    traci.start([SUMO_BINARY, "-c", SUMO_CONFIG])
    print("Simulation started with GLOSA.")

    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        sim_time = traci.simulation.getTime()
        tls_cache = {}

        for veh_id in traci.vehicle.getIDList():
            tls_data = traci.vehicle.getNextTLS(veh_id)
            if tls_data:
                tls_id, distance, index, _ = tls_data[0]
                if distance < GLOSA_RANGE:
                    if tls_id not in tls_cache:
                        next_switch = traci.trafficlight.getNextSwitch(tls_id) / 1000.0
                        tls_cache[tls_id] = next_switch - sim_time
                    adapt_speed(veh_id, distance, tls_cache[tls_id])

    traci.close()
    print("Simulation finished.")

if __name__ == "__main__":
    run_glosa_simulation()
