#!/usr/bin/env python3
# =============================================================================
# FIXS #177 -- SUMO TraCI context-subscription cost, measured IN ISOLATION.
#
# Reproduces the EXACT CarMaker-side SUMO path from CommonLib/TrafficHelper.cpp
# but with NO CarMaker, NO TrafficLayer, NO socket -- just the in-process
# libsumo 1.21 core (same library the .lib links). This localizes the
# per-step cost to the SUMO subscription layer and ablates it cleanly.
#
# What it replicates (TrafficHelper.cpp):
#   - Inject an ego 'egoCm' (Vehicle.add on a dummy route)         (~L483)
#   - ONE radius context subscription centred on the ego           (~L1049)
#       Vehicle.subscribeContext(ego, CMD_GET_VEHICLE_VARIABLE, radius, vars)
#   - each step: Simulation.step(); getAllContextSubscriptionResults() (~L1083)
#       then parse every returned vehicle's variables                 (~L1446)
#   - 0.1 s step length, SpeedMode 32, simple_loop net + 40-veh demand
#
# Per-step timing is decomposed because the subscription RESULTS are computed
# INSIDE Simulation.step() in libsumo (the getter only returns the cache):
#   t_step   = Simulation.step()                  <- where SUMO's "TraCI: ms" lives
#   t_getctx = getAllContextSubscriptionResults() <- should be ~0 (cache return)
#   t_parse  = read every var of every veh        <- client-side parse cost
#
# Output: _perf/bench_<config>.csv (per-step rows) + _perf/summary.csv
#         (steady-state N=40 mean of each phase, per config).
#
#   python bench_traci_sub.py                 # run the full ablation suite
#   python bench_traci_sub.py --only full nosub
# =============================================================================
import os, sys, time, argparse, csv, statistics

# in-process libsumo (matches the .lib path; NOT socket traci)
import libsumo

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", "..", "..", ".."))
SUMOCFG = os.path.join(REPO, "tests", "Sumo", "network", "simple_loop", "simple_loop.sumocfg")
OUTDIR = os.path.join(HERE, "_perf")

# ---- the EXACT VehDataSubscribeList from TrafficHelper.cpp L126-192 ----------
FULL = [
    ("VAR_TYPE",               libsumo.VAR_TYPE),
    ("VAR_SPEED",              libsumo.VAR_SPEED),
    ("VAR_POSITION3D",         libsumo.VAR_POSITION3D),
    ("VAR_ANGLE",              libsumo.VAR_ANGLE),
    ("VAR_COLOR",              libsumo.VAR_COLOR),
    ("VAR_ROAD_ID",            libsumo.VAR_ROAD_ID),
    ("VAR_LANE_INDEX",         libsumo.VAR_LANE_INDEX),
    ("VAR_DISTANCE",           libsumo.VAR_DISTANCE),
    ("VAR_LANEPOSITION",       libsumo.VAR_LANEPOSITION),
    ("VAR_LANE_ID",            libsumo.VAR_LANE_ID),
    ("VAR_VEHICLECLASS",       libsumo.VAR_VEHICLECLASS),
    ("VAR_ROUTE_INDEX",        libsumo.VAR_ROUTE_INDEX),
    ("VAR_ACCELERATION",       libsumo.VAR_ACCELERATION),
    ("VAR_ALLOWED_SPEED",      libsumo.VAR_ALLOWED_SPEED),
    ("VAR_SPEED_FACTOR",       libsumo.VAR_SPEED_FACTOR),
    ("VAR_VIA",                libsumo.VAR_VIA),
    ("VAR_SLOPE",              libsumo.VAR_SLOPE),
    ("VAR_SIGNALS",            libsumo.VAR_SIGNALS),
    ("VAR_LENGTH",             libsumo.VAR_LENGTH),
    ("VAR_WIDTH",              libsumo.VAR_WIDTH),
    ("VAR_HEIGHT",             libsumo.VAR_HEIGHT),
    ("VAR_SPEED_WITHOUT_TRACI", libsumo.VAR_SPEED_WITHOUT_TRACI),
]
FULL_MAP = dict(FULL)

# Vars the CarMaker bridge actually consumes (VehicleMessageField in config.yaml:
# id,type,vehicleClass,speed,speedDesired,position,heading,linkId,laneId,grade
# + length/width/height for the box). speedDesired ~ VAR_ALLOWED_SPEED*factor.
CONSUMED = ["VAR_TYPE","VAR_SPEED","VAR_POSITION3D","VAR_ANGLE","VAR_COLOR",
            "VAR_ROAD_ID","VAR_LANE_ID","VAR_VEHICLECLASS","VAR_SLOPE",
            "VAR_LENGTH","VAR_WIDTH","VAR_HEIGHT"]
MINIMAL = ["VAR_POSITION3D","VAR_SPEED","VAR_ANGLE"]

EGO = "egoCm"
STEP_LEN = 0.1
SPEED_MODE = 32


def varlist(names):
    return [FULL_MAP[n] for n in names]


def start_sumo():
    libsumo.start(["sumo", "-c", SUMOCFG,
                   "--step-length", str(STEP_LEN),
                   "--no-step-log", "true",
                   "--no-warnings", "true"])


def inject_ego():
    # step once so the net is live
    libsumo.simulationStep()
    # Ego must PERSIST for the whole run so the ego-centred context subscription
    # stays populated (a one-lap route would let it exit and the context goes
    # empty). Reuse the rou.xml 'loop' route (repeat=100000) so it circulates
    # forever, exactly like the background vehicles -- and so its getLeader/
    # getNextTLS best-lane lookahead sees the same huge repeated route the real
    # background vehicles carry.
    libsumo.vehicle.add(EGO, "loop", typeID="car")
    libsumo.vehicle.setColor(EGO, (255, 0, 0, 255))
    libsumo.simulationStep()
    try:
        libsumo.vehicle.setSpeedMode(EGO, SPEED_MODE)
    except Exception:
        pass


def run_config(name, var_names, radius, n_steps=400, hold_n=35, per_veh=()):
    """Run one subscription config; record per-step decomposed timing.

    per_veh: tuple of extra individual TraCI calls made PER context vehicle PER
    step, replicating parserSumoSubscription (TrafficHelper.cpp L1446). Each is
    one of: 'leader' (getLeader(v,1000)+getSpeed(leader)), 'nexttls'
    (getNextTLS(v)). This is the path the subscription does NOT cover.
    """
    start_sumo()
    inject_ego()
    sub = var_names is not None
    if sub:
        libsumo.vehicle.subscribeContext(
            EGO, libsumo.CMD_GET_VEHICLE_VARIABLE, float(radius),
            varlist(var_names), 0, 100000)

    rows = []
    for i in range(n_steps):
        t0 = time.perf_counter()
        libsumo.simulationStep()
        t1 = time.perf_counter()
        if sub:
            res = libsumo.vehicle.getAllContextSubscriptionResults()
            t2 = time.perf_counter()
            # client-side parse: touch every variable of every returned veh
            nctx = 0
            sink = 0
            for center, members in res.items():
                for vid, vardict in members.items():
                    nctx += 1
                    for k, v in vardict.items():
                        sink ^= hash(k)  # force read
            t3 = time.perf_counter()
            # per-veh individual TraCI calls (the parserSumoSubscription path)
            for center, members in res.items():
                for vid in members.keys():
                    if "leader" in per_veh:
                        ld = libsumo.vehicle.getLeader(vid, 1000)
                        if ld and ld[0]:
                            libsumo.vehicle.getSpeed(ld[0])
                    if "nexttls" in per_veh:
                        libsumo.vehicle.getNextTLS(vid)
            t4 = time.perf_counter()
        else:
            t2 = t3 = t4 = t1
            nctx = 0
        n_active = libsumo.vehicle.getIDCount()
        rows.append({
            "step": i,
            "simtime": round((i + 1) * STEP_LEN, 3),
            "n_active": n_active,
            "n_ctx": nctx,
            "t_step_ms": (t1 - t0) * 1e3,
            "t_getctx_ms": (t2 - t1) * 1e3,
            "t_parse_ms": (t3 - t2) * 1e3,
            "t_perveh_ms": (t4 - t3) * 1e3,
        })
    libsumo.close()

    os.makedirs(OUTDIR, exist_ok=True)
    csvpath = os.path.join(OUTDIR, f"bench_{name}.csv")
    with open(csvpath, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)

    # steady-state = rows where n_active >= hold_n (population filled to ~40)
    steady = [r for r in rows if r["n_active"] >= hold_n]
    if len(steady) < 5:                       # demand never reached hold_n
        steady = sorted(rows, key=lambda r: r["n_active"])[-max(5, len(rows)//5):]

    def med(key):
        return statistics.median(r[key] for r in steady)

    n_med = statistics.median(r["n_active"] for r in steady)
    total = med("t_step_ms") + med("t_getctx_ms") + med("t_parse_ms") + med("t_perveh_ms")
    summ = {
        "config": name,
        "n_vars": (len(var_names) if sub else 0),
        "radius": (radius if sub else 0),
        "per_veh": "+".join(per_veh) if per_veh else "-",
        "steady_n": round(n_med, 1),
        "step_ms": round(med("t_step_ms"), 3),
        "getctx_ms": round(med("t_getctx_ms"), 4),
        "parse_ms": round(med("t_parse_ms"), 4),
        "perveh_ms": round(med("t_perveh_ms"), 3),
        "total_ms": round(total, 3),
        "per_veh_us": round(total / n_med * 1e3, 1) if n_med else 0,
    }
    print(f"  {name:26s} vars={summ['n_vars']:2d} r={summ['radius']:>4} "
          f"pv={summ['per_veh']:14s} N={summ['steady_n']:>4}  "
          f"step={summ['step_ms']:6.3f} parse={summ['parse_ms']:6.3f} "
          f"perveh={summ['perveh_ms']:7.3f}  total={summ['total_ms']:7.3f}ms "
          f"({summ['per_veh_us']:.1f}us/veh)")
    return summ


def build_suite():
    ALL = [n for n, _ in FULL]
    suite = []
    # entries: (name, var_names|None, radius, per_veh tuple)
    # --- baselines & whole-list configs (radius 500 like the #174 config) ---
    suite.append(("nosub",    None, 500, ()))
    suite.append(("full",     ALL,  500, ()))
    suite.append(("consumed", CONSUMED, 500, ()))
    suite.append(("minimal",  MINIMAL,  500, ()))
    # --- radius sweep on the FULL list (hypothesis 2) -----------------------
    suite.append(("full_r50",  ALL, 50, ()))
    suite.append(("full_r100", ALL, 100, ()))
    suite.append(("full_r5000",ALL, 5000, ()))
    # --- THE parserSumoSubscription per-veh calls (hypothesis 3, real cost) -
    suite.append(("full_leader",      ALL, 500, ("leader",)))
    suite.append(("full_nexttls",     ALL, 500, ("nexttls",)))
    suite.append(("full_faithful",    ALL, 500, ("leader", "nexttls")))
    # --- add-one var sweep (hypothesis 1, per-var cost) ---------------------
    rest = [n for n in ALL if n not in MINIMAL]
    for n in rest:
        suite.append((f"add_{n}", MINIMAL + [n], 500, ()))
    return suite


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--only", nargs="*", default=None,
                    help="run only these config names")
    ap.add_argument("--steps", type=int, default=400)
    args = ap.parse_args()

    suite = build_suite()
    if args.only:
        suite = [s for s in suite if s[0] in args.only]

    print(f"#177 SUMO TraCI subscription benchmark (libsumo {getattr(libsumo,'__version__','?')})")
    print(f"net: {SUMOCFG}")
    print(f"steps/config: {args.steps}  step-len: {STEP_LEN}s\n")

    summaries = []
    for name, vnames, radius, per_veh in suite:
        summaries.append(run_config(name, vnames, radius,
                                    n_steps=args.steps, per_veh=per_veh))

    os.makedirs(OUTDIR, exist_ok=True)
    spath = os.path.join(OUTDIR, "summary.csv")
    with open(spath, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(summaries[0].keys()))
        w.writeheader()
        w.writerows(summaries)
    print(f"\nwrote {spath}")


if __name__ == "__main__":
    main()
