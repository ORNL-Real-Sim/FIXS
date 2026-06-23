"""
FIXS #172: let SUMO drive ONE vehicle around the loop with NO lane change, and
record the exact (edge, lane) sequence it traverses. SUMO is the authority on
lane connectivity, so this is the clean lane-specific loop chain -- no fragile
graph parsing. Output -> sumo_lane_seq.txt (consecutive-deduped lane ids).

Loop = findRoute(corridor_w_in -> corridor_w_out)  [E-bound + east roundabout +
W-bound] + findRoute(corridor_w_out -> corridor_w_in)  [west roundabout].
Tries each departure lane; keeps the one that completes the loop in a single lane
(no change) and covers the most distance.
"""
from __future__ import annotations
import os, sys, math, pathlib
sys.path.append(os.path.join(os.environ["SUMO_HOME"], "tools"))
import traci

HERE = pathlib.Path(__file__).resolve().parent
NET = str(HERE / "simple_traffic_light.net.xml")
SUMO = os.path.join(os.environ["SUMO_HOME"], "bin", "sumo.exe")


def run(dep_lane):
    traci.start([SUMO, "-n", NET, "--no-step-log", "true", "-W", "true", "--step-length", "0.5"])
    try:
        r1 = list(traci.simulation.findRoute("corridor_w_in", "corridor_w_out").edges)
        r2 = list(traci.simulation.findRoute("corridor_w_out", "corridor_w_in").edges)
        loop = r1 + r2[1:]
        traci.route.add("loop", loop)
        traci.vehicle.add("ego", "loop", departLane=str(dep_lane), departSpeed="max")
        traci.vehicle.setLaneChangeMode("ego", 0)   # 0 = forbid all lane changes
        lanes, pos = [], []
        for _ in range(4000):
            traci.simulationStep()
            ids = traci.vehicle.getIDList()
            if "ego" in ids:
                lid = traci.vehicle.getLaneID("ego")
                if lid:
                    lanes.append(lid)
                x, y = traci.vehicle.getPosition("ego")
                ang_deg = traci.vehicle.getAngle("ego")  # SUMO angle: deg, 0=North, CW
                spd = traci.vehicle.getSpeed("ego")      # SUMO slows to the loop's speed limit in the loop
                pos.append((x, y, ang_deg, spd))
            elif lanes:
                break
        seq = []
        for l in lanes:
            if not seq or seq[-1] != l:
                seq.append(l)
        # detect lane change on the main (non-internal) corridor edges
        main = [l for l in seq if not l.startswith(":")]
        edge_lane = {}
        for l in main:
            e, _, idx = l.rpartition("_")
            edge_lane.setdefault(e, set()).add(idx)
        multi = {e: v for e, v in edge_lane.items() if len(v) > 1}
        return loop, seq, multi, pos
    finally:
        traci.close()


def main():
    best = None
    for dl in ("free", "0", "1", "2"):
        try:
            loop, seq, multi, pos = run(dl)
        except traci.TraCIException as e:
            print(f"departLane {dl}: failed ({e})"); continue
        print(f"departLane {dl}: {len(seq)} lane-segments, {len(pos)} path pts, edges-with-lane-change={list(multi)}")
        if best is None or (len(multi) < len(best[3])) or (len(multi) == len(best[3]) and len(seq) > len(best[2])):
            best = (dl, loop, seq, multi, pos)
    if not best:
        print("no run completed"); return
    dl, loop, seq, multi, pos = best
    print(f"\nBEST departLane {dl}: lane-change edges={list(multi)}, {len(pos)} path points")
    (HERE / "sumo_lane_seq.txt").write_text("\n".join(seq) + "\n", encoding="utf-8")
    # path: x y angle_deg speed (SUMO angle 0=N, CW; speed m/s -> slows in the loop).
    # thin to ~every 3 m for a clean trajectory.
    thinned, last = [], None
    for (x, y, ad, sp) in pos:
        if last is None or math.hypot(x-last[0], y-last[1]) >= 3.0:
            thinned.append((x, y, ad, sp)); last = (x, y)
    (HERE / "sumo_path.txt").write_text(
        "\n".join(f"{x:.3f} {y:.3f} {ad:.2f} {sp:.3f}" for x, y, ad, sp in thinned) + "\n", encoding="utf-8")
    print(f"wrote sumo_lane_seq.txt ({len(seq)}) + sumo_path.txt ({len(thinned)} pts)")


if __name__ == "__main__":
    main()
