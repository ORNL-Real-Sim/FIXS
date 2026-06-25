#!/usr/bin/env python3
# #177 -- burst-aware view of the optimization matrix. The 'cached' getNextTLS
# strategy is bursty (a 2.7 ms call only on edge-change steps), so the median
# per-step total hides the spikes. Report median, mean and p95 of the per-step
# total at steady state (N>=35), plus mean getNextTLS calls/step.
import os, csv, glob, statistics

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "_perf")


def pct(xs, p):
    xs = sorted(xs)
    return xs[min(len(xs) - 1, int(p / 100 * len(xs)))]


def main():
    print(f"{'config':16s} {'N':>4} {'calls/step(mean)':>16} "
          f"{'med_ms':>8} {'mean_ms':>8} {'p95_ms':>8}")
    for fp in sorted(glob.glob(os.path.join(OUT, "opt_*.csv"))):
        if fp.endswith("summary.csv"):
            continue
        rows = list(csv.DictReader(open(fp)))
        st = [r for r in rows if int(r["n_active"]) >= 35]
        if len(st) < 5:
            continue
        tot = [float(r["t_step_ms"]) + float(r["t_subread_ms"]) + float(r["t_tls_ms"]) for r in st]
        calls = [int(r["tls_calls"]) for r in st]
        name = os.path.basename(fp)[len("opt_"):-len(".csv")]
        print(f"{name:16s} {statistics.median(int(r['n_active']) for r in st):>4.0f} "
              f"{statistics.mean(calls):>16.2f} "
              f"{statistics.median(tot):>8.3f} {statistics.mean(tot):>8.3f} {pct(tot,95):>8.3f}")


if __name__ == "__main__":
    main()
