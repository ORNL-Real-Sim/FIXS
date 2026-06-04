"""
Compare SpeedLimitLite's trace against the SpeedLimit golden reference.

Default reference is the Parquet next to the original .mat:
  ../SpeedLimit/speedLimitTest{1,2,3}_orig.parquet  (~340-450 KB each)

Reference originated as a Simulink.SimulationOutput object inside the
.mat file (PSCAD-style opaque binary, unreadable from Python). We
converted it via archive/export_mat_to_csv.m → write parquet here:
~46x smaller than CSV, preserves dtypes, language-agnostic.

Reference is sampled at Simulink's 1 ms; SpeedLimitLite at VISSIM's
100 ms. Reference is resampled (nearest) to the VISSIM grid before
per-field comparison. Tolerances are per-field.

Exit code 0 if every compared field is within tolerance; 1 otherwise.
"""
from __future__ import annotations
import argparse
import pathlib
import sys
import numpy as np
import pandas as pd

HERE = pathlib.Path(__file__).parent.resolve()
DEFAULT_REF = HERE.parent / 'SpeedLimit' / 'speedLimitTest1_orig.parquet'
DEFAULT_MOD = HERE / 'speed_limit_lite_trace.parquet'

COMPARE_FIELDS = ['speed', 'speedLimit', 'speedLimitNext',
                  'speedLimitChangeDistance', 'signalLightHeadId',
                  'precedingVehicleDistance', 'precedingVehicleSpeed']

TOLERANCES = {
    '_default': 1.0,
    'speed':                     1.0,
    'speedLimit':                0.5,
    'speedLimitNext':            0.5,
    'speedLimitChangeDistance':  5.0,
    'signalLightHeadId':         0.5,
    'precedingVehicleDistance':  5.0,
    'precedingVehicleSpeed':     1.0,
}


def load_trace(path: pathlib.Path, time_col: str | None = None) -> pd.DataFrame:
    if path.suffix == '.parquet':
        df = pd.read_parquet(path)
    elif path.suffix == '.csv':
        df = pd.read_csv(path)
    else:
        raise ValueError(f"Unsupported format: {path.suffix}")
    # Pick a uniform time column name
    if time_col is None:
        time_col = 'Time' if 'Time' in df.columns else 'simTime'
    return df.rename(columns={time_col: 'Time'})


def align_nearest(ref: pd.DataFrame, t_mod: np.ndarray) -> pd.DataFrame:
    ref = ref.sort_values('Time').reset_index(drop=True)
    t_ref = ref['Time'].to_numpy()
    idx = np.searchsorted(t_ref, t_mod, side='left')
    idx = np.clip(idx, 0, len(t_ref) - 1)
    # Pick the closer of idx and idx-1
    for k in range(len(idx)):
        if idx[k] > 0 and (idx[k] == len(t_ref) or
                           abs(t_ref[idx[k]-1] - t_mod[k]) < abs(t_ref[idx[k]] - t_mod[k])):
            idx[k] -= 1
    return ref.iloc[idx].reset_index(drop=True)


def compare(ref: pd.DataFrame, mod: pd.DataFrame) -> tuple[bool, list[str]]:
    t_mod = mod['Time'].to_numpy()
    ref_aligned = align_nearest(ref, t_mod)

    lines = [
        f"Reference: {len(ref)} rows, t in [{ref['Time'].min():.3f}, {ref['Time'].max():.3f}]",
        f"SpeedLimitLite: {len(mod)} rows, t in [{t_mod.min():.3f}, {t_mod.max():.3f}]",
        "",
        f"{'field':<28} {'tol':>8} {'max|diff|':>10} {'mean|diff|':>10} {'verdict':>10}",
        "-" * 70,
    ]
    ok = True
    for fn in COMPARE_FIELDS:
        if fn not in ref.columns or fn not in mod.columns:
            lines.append(f"{fn:<28} {'(missing in one side)':>40}")
            ok = False
            continue
        a = pd.to_numeric(ref_aligned[fn], errors='coerce').to_numpy()
        b = pd.to_numeric(mod[fn], errors='coerce').to_numpy()
        n = min(len(a), len(b))
        a, b = a[:n], b[:n]
        mask = ~(np.isnan(a) | np.isnan(b))
        if not mask.any():
            lines.append(f"{fn:<28} {'(all-nan)':>40}")
            continue
        diff = np.abs(a[mask] - b[mask])
        f_tol = TOLERANCES.get(fn, TOLERANCES['_default'])
        ok_field = float(diff.max()) <= f_tol
        ok &= ok_field
        verdict = 'OK' if ok_field else 'FAIL'
        lines.append(
            f"{fn:<28} {f_tol:>8.3f} {diff.max():>10.4f} {diff.mean():>10.4f} {verdict:>10}"
        )
    return ok, lines


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--ref', type=pathlib.Path, default=DEFAULT_REF,
                   help=f"Golden reference (.parquet or .csv). Default: {DEFAULT_REF.name}")
    p.add_argument('--mod', type=pathlib.Path, default=DEFAULT_MOD,
                   help=f"SpeedLimitLite trace (.parquet or .csv). Default: {DEFAULT_MOD.name}")
    args = p.parse_args()

    if not args.ref.is_file():
        sys.exit(f"ERROR: reference not found: {args.ref}")
    if not args.mod.is_file():
        # Fall back to CSV if parquet not written yet
        csv_alt = args.mod.with_suffix('.csv')
        if csv_alt.is_file():
            args.mod = csv_alt
        else:
            sys.exit(f"ERROR: trace not found: {args.mod} (or .csv)")

    ref = load_trace(args.ref)
    mod = load_trace(args.mod)
    ok, lines = compare(ref, mod)
    for ln in lines:
        print(ln)
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
