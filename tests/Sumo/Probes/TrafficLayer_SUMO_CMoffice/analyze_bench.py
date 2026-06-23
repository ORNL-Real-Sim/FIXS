#!/usr/bin/env python3
# #177 -- fit per-step total cost vs active-vehicle count (slope = ms/veh) for
# each bench_<config>.csv, so the measured slope is directly comparable to the
# issue's claimed ~7.5 ms/active-vehicle/step. Binned means + OLS slope.
import os, csv, glob, statistics

HERE = os.path.dirname(os.path.abspath(__file__))
OUTDIR = os.path.join(HERE, "_perf")


def ols(xs, ys):
    n = len(xs)
    mx = sum(xs) / n
    my = sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    sxy = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    slope = sxy / sxx if sxx else 0
    intercept = my - slope * mx
    return slope, intercept


def main():
    files = sorted(glob.glob(os.path.join(OUTDIR, "bench_*.csv")))
    print(f"{'config':26s} {'slope us/veh':>12s} {'intcpt ms':>10s} "
          f"{'ms@N=27':>9s} {'ms@N=40':>9s}  (warmup-trimmed)")
    for fp in files:
        name = os.path.basename(fp)[len("bench_"):-len(".csv")]
        rows = list(csv.DictReader(open(fp)))
        # total = step+getctx+parse(+perveh if present)
        def total(r):
            t = float(r["t_step_ms"]) + float(r["t_getctx_ms"]) + float(r["t_parse_ms"])
            if "t_perveh_ms" in r:
                t += float(r["t_perveh_ms"])
            return t
        # drop first 50 steps (warmup / JIT / route build) and any N<2
        data = [(int(r["n_active"]), total(r)) for r in rows[50:]
                if int(r["n_active"]) >= 2]
        if len(data) < 20:
            continue
        xs = [d[0] for d in data]
        ys = [d[1] for d in data]
        slope, intercept = ols(xs, ys)
        print(f"{name:26s} {slope*1e3:12.2f} {intercept:10.4f} "
              f"{slope*27+intercept:9.3f} {slope*40+intercept:9.3f}")


if __name__ == "__main__":
    main()
