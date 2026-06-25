#!/usr/bin/env python3
# =============================================================================
# FIXS #177 -- optimization bench. Verifies, head-to-head over BOTH transports
# (in-process libsumo vs socket traci==libtraci==the shipping build), three
# independent levers on the SUMO-CM per-step cost:
#
#   1. getNextTLS strategy:  everystep | cached | none
#        cached = recompute the next-TLS IDENTITY only when the vehicle's
#        VAR_ROAD_ID (already in the subscription) changes; between edge changes
#        the identity is constant and the distance is an O(1) update from the
#        subscribed lane position (modelled here as free -- no TraCI call).
#        This is the "make getNextTLS efficient" fix, caller-side, no SUMO patch.
#   2. subscription variable set:  full(22) | consumed(12)
#   3. transport:  libsumo (in-process, no TCP) | traci (socket, per-call RTT)
#
# Every config drives the same simple_loop scenario to steady state (~37 veh).
#
#   python bench_opt.py                       # full matrix
#   python bench_opt.py --only L_opt T_opt    # specific configs
# =============================================================================
import os, sys, time, csv, statistics, argparse, importlib

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", "..", "..", ".."))
SUMOCFG = os.path.join(REPO, "tests", "Sumo", "network", "simple_loop", "simple_loop.sumocfg")
OUTDIR = os.path.join(HERE, "_perf")
SUMO_BIN = os.path.join(os.environ.get("SUMO_HOME", r"C:\Program Files (x86)\Eclipse\Sumo"),
                        "bin", "sumo.exe")
EGO, STEP_LEN, SPEED_MODE = "egoCm", 0.1, 32


def load(transport):
    """Return (module, const-namespace). libsumo and traci share the surface we use."""
    if transport == "libsumo":
        S = importlib.import_module("libsumo")
        return S, S
    else:
        S = importlib.import_module("traci")
        from traci import constants as tc
        return S, tc


def varset(C, which):
    full = [C.VAR_TYPE, C.VAR_SPEED, C.VAR_POSITION3D, C.VAR_ANGLE, C.VAR_COLOR,
            C.VAR_ROAD_ID, C.VAR_LANE_INDEX, C.VAR_DISTANCE, C.VAR_LANEPOSITION,
            C.VAR_LANE_ID, C.VAR_VEHICLECLASS, C.VAR_ROUTE_INDEX, C.VAR_ACCELERATION,
            C.VAR_ALLOWED_SPEED, C.VAR_SPEED_FACTOR, C.VAR_VIA, C.VAR_SLOPE,
            C.VAR_SIGNALS, C.VAR_LENGTH, C.VAR_WIDTH, C.VAR_HEIGHT, C.VAR_SPEED_WITHOUT_TRACI]
    consumed = [C.VAR_TYPE, C.VAR_SPEED, C.VAR_POSITION3D, C.VAR_ANGLE, C.VAR_COLOR,
                C.VAR_ROAD_ID, C.VAR_LANE_ID, C.VAR_VEHICLECLASS, C.VAR_SLOPE,
                C.VAR_LENGTH, C.VAR_WIDTH, C.VAR_HEIGHT, C.VAR_LANE_INDEX]
    return full if which == "full" else consumed


def run(name, transport, var_which, tls_mode, n_steps, hold_n=35):
    S, C = load(transport)
    S.start([SUMO_BIN if transport == "traci" else "sumo", "-c", SUMOCFG,
             "--step-length", str(STEP_LEN), "--no-step-log", "true", "--no-warnings", "true"])
    S.simulationStep()
    S.vehicle.add(EGO, "loop", typeID="car")
    S.simulationStep()
    try: S.vehicle.setSpeedMode(EGO, SPEED_MODE)
    except Exception: pass
    vlist = varset(C, var_which)
    S.vehicle.subscribeContext(EGO, C.CMD_GET_VEHICLE_VARIABLE, 500.0, vlist, 0, 100000)

    tls_road = {}            # vid -> last road id (cache-on-edge-change)
    rows = []
    for i in range(n_steps):
        t0 = time.perf_counter()
        S.simulationStep()
        t1 = time.perf_counter()
        res = S.vehicle.getContextSubscriptionResults(EGO) or {}
        t2 = time.perf_counter()
        ncalls = 0
        for vid, vd in res.items():
            if tls_mode == "none":
                continue
            if tls_mode == "everystep":
                S.vehicle.getNextTLS(vid); ncalls += 1
            elif tls_mode == "cached":
                road = vd.get(C.VAR_ROAD_ID)
                if tls_road.get(vid) != road:      # only on edge change
                    S.vehicle.getNextTLS(vid); ncalls += 1
                    tls_road[vid] = road
        t3 = time.perf_counter()
        rows.append({"step": i, "n_active": S.vehicle.getIDCount(), "n_ctx": len(res),
                     "tls_calls": ncalls,
                     "t_step_ms": (t1-t0)*1e3, "t_subread_ms": (t2-t1)*1e3,
                     "t_tls_ms": (t3-t2)*1e3})
    S.close()

    os.makedirs(OUTDIR, exist_ok=True)
    with open(os.path.join(OUTDIR, f"opt_{name}.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)

    st = [r for r in rows if r["n_active"] >= hold_n] or \
         sorted(rows, key=lambda r: r["n_active"])[-max(5, len(rows)//5):]
    def m(k): return statistics.median(r[k] for r in st)
    N = statistics.median(r["n_active"] for r in st)
    calls = statistics.median(r["tls_calls"] for r in st)
    total = m("t_step_ms") + m("t_subread_ms") + m("t_tls_ms")
    summ = {"config": name, "transport": transport, "vars": var_which, "tls": tls_mode,
            "N": round(N, 1), "tls_calls_step": round(calls, 1),
            "step_ms": round(m("t_step_ms"), 3), "subread_ms": round(m("t_subread_ms"), 3),
            "tls_ms": round(m("t_tls_ms"), 3), "total_ms": round(total, 3),
            "rtf": round(100.0/total, 1) if total else 0}
    print(f"  {name:16s} {transport:7s} {var_which:8s} tls={tls_mode:9s} N={summ['N']:>4} "
          f"calls/step={summ['tls_calls_step']:>4} step={summ['step_ms']:6.3f} "
          f"sub={summ['subread_ms']:6.3f} tls={summ['tls_ms']:7.3f} "
          f"TOTAL={summ['total_ms']:7.3f}ms RTF={summ['rtf']}")
    return summ


def build():
    # name, transport, varset, tls_mode
    return [
        ("T_base",   "traci",   "full",     "everystep"),  # current shipping build
        ("T_cached", "traci",   "full",     "cached"),     # +getNextTLS cache
        ("T_opt",    "traci",   "consumed", "cached"),     # +trim sub
        ("T_floor",  "traci",   "consumed", "none"),       # sub-only floor (socket)
        ("L_base",   "libsumo", "full",     "everystep"),  # in-process, current calls
        ("L_cached", "libsumo", "full",     "cached"),
        ("L_opt",    "libsumo", "consumed", "cached"),     # full optimized stack
        ("L_floor",  "libsumo", "consumed", "none"),       # sub-only floor (in-proc)
    ]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--only", nargs="*", default=None)
    ap.add_argument("--steps", type=int, default=2100)
    a = ap.parse_args()
    suite = [s for s in build() if (not a.only or s[0] in a.only)]
    print(f"#177 optimization matrix  steps/config={a.steps}  (RT budget {STEP_LEN*1000:.0f} ms/step, CM step 1 ms)\n")
    out = [run(*cfg, n_steps=a.steps) for cfg in suite]
    with open(os.path.join(OUTDIR, "opt_summary.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(out[0].keys())); w.writeheader(); w.writerows(out)
    print(f"\nwrote {os.path.join(OUTDIR,'opt_summary.csv')}")


if __name__ == "__main__":
    main()
