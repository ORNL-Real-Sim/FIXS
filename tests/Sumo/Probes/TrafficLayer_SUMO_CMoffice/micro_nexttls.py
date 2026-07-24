#!/usr/bin/env python3
# =============================================================================
# FIXS #177 -- WHY is per-vehicle getNextTLS ~2.4 ms/call on this network?
#
# Hypothesis: vehicles ride the rou.xml 'loop' route declared with
# repeat="100000" (-> ~400k internal edges). getNextTLS scans the upcoming
# route/best-lane continuation for traffic lights; on a no-signal network it
# finds none and walks a lookahead proportional to the (huge) remaining route.
#
# This isolates getNextTLS cost as a function of the route length ahead, with a
# SINGLE vehicle (no traffic, no subscription) so nothing else contributes.
# In-process libsumo so the number is pure compute (no socket).
# =============================================================================
import os, time, statistics
import libsumo

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", "..", "..", ".."))
NET = os.path.join(REPO, "tests", "Sumo", "networks", "simple_loop", "simple_loop.net.xml")
LOOP = ["-1", "-2", "-3", "-4"]


def time_nexttls(route_edges, repeat, n_calls=300, warm=30):
    # empty-demand load of just the net
    libsumo.start(["sumo", "-n", NET, "--step-length", "0.1",
                   "--no-step-log", "true", "--no-warnings", "true"])
    libsumo.simulationStep()
    edges = route_edges * repeat
    libsumo.route.add("r", edges)
    libsumo.vehicle.add("v", "r", typeID="DEFAULT_VEHTYPE")
    libsumo.vehicle.setSpeedMode("v", 32)
    # let it get onto the lane and build best-lanes
    for _ in range(warm):
        libsumo.simulationStep()
    route_len = len(libsumo.vehicle.getRoute("v"))
    ts = []
    for _ in range(n_calls):
        t0 = time.perf_counter()
        libsumo.vehicle.getNextTLS("v")
        ts.append((time.perf_counter() - t0) * 1e3)
        libsumo.simulationStep()
    libsumo.close()
    return route_len, statistics.median(ts), max(ts)


def main():
    print("route_repeat  route_edges   med_ms   max_ms   (single veh, in-process)")
    for repeat in [1, 10, 100, 1000, 10000, 100000]:
        try:
            rlen, med, mx = time_nexttls(LOOP, repeat)
            print(f"{repeat:>11d}  {rlen:>11d}  {med:8.3f} {mx:8.3f}")
        except Exception as e:
            print(f"{repeat:>11d}  ERROR {e}")


if __name__ == "__main__":
    main()
