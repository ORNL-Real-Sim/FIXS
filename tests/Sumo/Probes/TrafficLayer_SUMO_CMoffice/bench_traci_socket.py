#!/usr/bin/env python3
# =============================================================================
# FIXS #177 -- SUMO TraCI cost over the SOCKET transport (libtraci), the path
# the SHIPPING build actually uses.
#
# CommonLib/TrafficHelper.h: ENABLE_LIBSUMO is commented out, so the build
# compiles the libtraci namespace -> TrafficLayer talks to a SEPARATE
# sumo-gui.exe over TCP (run_sumo_cm_demo.bat: sumo-gui --remote-port 1337).
# Under libtraci EVERY individual TraCI getter is a synchronous request/response
# round-trip. Python's `traci` module is the SAME socket client, so it
# reproduces the real transport (unlike bench_traci_sub.py, which used the
# in-process libsumo binding and therefore UNDER-counts -- it skips the socket).
#
# This isolates the two cost regimes:
#   * context subscription  -> ONE round-trip/step for ALL N vehicles (cheap)
#   * per-vehicle getters    -> N x (getLeader + getNextTLS + getSpeed)
#                               round-trips/step  (the parserSumoSubscription
#                               path, TrafficHelper.cpp L1512/1519/1525)
#
# python bench_traci_socket.py            # full suite
# =============================================================================
import os, sys, time, csv, statistics, argparse

import traci
from traci import constants as tc

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", "..", "..", ".."))
SUMOCFG = os.path.join(REPO, "tests", "Sumo", "networks", "simple_loop", "simple_loop.sumocfg")
OUTDIR = os.path.join(HERE, "_perf")
SUMO_BIN = os.path.join(os.environ.get("SUMO_HOME", r"C:\Program Files (x86)\Eclipse\Sumo"),
                        "bin", "sumo.exe")

FULL = [tc.VAR_TYPE, tc.VAR_SPEED, tc.VAR_POSITION3D, tc.VAR_ANGLE, tc.VAR_COLOR,
        tc.VAR_ROAD_ID, tc.VAR_LANE_INDEX, tc.VAR_DISTANCE, tc.VAR_LANEPOSITION,
        tc.VAR_LANE_ID, tc.VAR_VEHICLECLASS, tc.VAR_ROUTE_INDEX, tc.VAR_ACCELERATION,
        tc.VAR_ALLOWED_SPEED, tc.VAR_SPEED_FACTOR, tc.VAR_VIA, tc.VAR_SLOPE,
        tc.VAR_SIGNALS, tc.VAR_LENGTH, tc.VAR_WIDTH, tc.VAR_HEIGHT,
        tc.VAR_SPEED_WITHOUT_TRACI]
MINIMAL = [tc.VAR_POSITION3D, tc.VAR_SPEED, tc.VAR_ANGLE]

EGO = "egoCm"
STEP_LEN = 0.1
SPEED_MODE = 32


def start_sumo():
    traci.start([SUMO_BIN, "-c", SUMOCFG, "--step-length", str(STEP_LEN),
                 "--no-step-log", "true", "--no-warnings", "true"])


def inject_ego():
    traci.simulationStep()
    traci.vehicle.add(EGO, "loop", typeID="car")   # repeating loop route (persists)
    traci.vehicle.setColor(EGO, (255, 0, 0, 255))
    traci.simulationStep()
    try:
        traci.vehicle.setSpeedMode(EGO, SPEED_MODE)
    except traci.TraCIException:
        pass


def run_config(name, var_list, radius, n_steps, per_veh=(), hold_n=35):
    start_sumo()
    inject_ego()
    sub = var_list is not None
    if sub:
        traci.vehicle.subscribeContext(
            EGO, tc.CMD_GET_VEHICLE_VARIABLE, float(radius), var_list, 0, 100000)

    rows = []
    for i in range(n_steps):
        t0 = time.perf_counter()
        traci.simulationStep()
        t1 = time.perf_counter()
        if sub:
            res = traci.vehicle.getContextSubscriptionResults(EGO) or {}
            t2 = time.perf_counter()
            nctx = len(res)
            # per-veh individual getters == N socket round-trips (the real path)
            for vid in res.keys():
                if "leader" in per_veh:
                    ld = traci.vehicle.getLeader(vid, 1000)
                    if ld and ld[0]:
                        traci.vehicle.getSpeed(ld[0])
                if "nexttls" in per_veh:
                    traci.vehicle.getNextTLS(vid)
            t3 = time.perf_counter()
        else:
            t2 = t3 = t1
            nctx = 0
        n_active = traci.vehicle.getIDCount()
        rows.append({
            "step": i, "simtime": round((i + 1) * STEP_LEN, 3),
            "n_active": n_active, "n_ctx": nctx,
            "t_step_ms": (t1 - t0) * 1e3,
            "t_subread_ms": (t2 - t1) * 1e3,
            "t_perveh_ms": (t3 - t2) * 1e3,
        })
    traci.close()

    os.makedirs(OUTDIR, exist_ok=True)
    with open(os.path.join(OUTDIR, f"sock_{name}.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader(); w.writerows(rows)

    steady = [r for r in rows if r["n_active"] >= hold_n]
    if len(steady) < 5:
        steady = sorted(rows, key=lambda r: r["n_active"])[-max(5, len(rows)//5):]

    def med(k):
        return statistics.median(r[k] for r in steady)
    n_med = statistics.median(r["n_active"] for r in steady)
    total = med("t_step_ms") + med("t_subread_ms") + med("t_perveh_ms")
    summ = {
        "config": name, "n_vars": (len(var_list) if sub else 0), "radius": radius,
        "per_veh": "+".join(per_veh) if per_veh else "-",
        "steady_n": round(n_med, 1),
        "step_ms": round(med("t_step_ms"), 3),
        "subread_ms": round(med("t_subread_ms"), 3),
        "perveh_ms": round(med("t_perveh_ms"), 3),
        "total_ms": round(total, 3),
        "per_veh_us": round(total / n_med * 1e3, 1) if n_med else 0,
        "rtf": round(STEP_LEN * 1000.0 / total, 2) if total else 0,
    }
    print(f"  {name:22s} vars={summ['n_vars']:2d} pv={summ['per_veh']:14s} "
          f"N={summ['steady_n']:>4} step={summ['step_ms']:6.3f} "
          f"subread={summ['subread_ms']:6.3f} perveh={summ['perveh_ms']:7.3f} "
          f"total={summ['total_ms']:7.3f}ms RTF={summ['rtf']:.2f}")
    return summ


def build_suite():
    return [
        ("nosub",       None,    500, ()),
        ("sub_full",    FULL,    500, ()),
        ("sub_minimal", MINIMAL, 500, ()),
        ("leader",      FULL,    500, ("leader",)),
        ("nexttls",     FULL,    500, ("nexttls",)),
        ("faithful",    FULL,    500, ("leader", "nexttls")),
    ]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--only", nargs="*", default=None)
    ap.add_argument("--steps", type=int, default=2300)
    args = ap.parse_args()
    suite = build_suite()
    if args.only:
        suite = [s for s in suite if s[0] in args.only]
    print(f"#177 SUMO TraCI SOCKET benchmark (traci {traci.__version__ if hasattr(traci,'__version__') else '?'})")
    print(f"sumo: {SUMO_BIN}\nsteps/config: {args.steps}  step-len {STEP_LEN}s  "
          f"(real-time budget = {STEP_LEN*1000:.0f} ms/step)\n")
    out = []
    for name, vl, r, pv in suite:
        out.append(run_config(name, vl, r, args.steps, per_veh=pv))
    with open(os.path.join(OUTDIR, "sock_summary.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(out[0].keys()))
        w.writeheader(); w.writerows(out)
    print(f"\nwrote {os.path.join(OUTDIR, 'sock_summary.csv')}")


if __name__ == "__main__":
    main()
