# DSProxy_egoctrl — Stage 1.5 ego-control validation for #156

Asks one focused question: does DSProxy give us XIL-style **full control**
over the ego's pose — not just trajectory replay along VISSIM's natural
network — to the standard a CarMaker/Carla/Simulink dyno expects?

Scope: **VISSIM 2022 only.** Stage 1 already established 2022/2026 parity
for the basic flow ([../DSProxy_smoke/](../DSProxy_smoke/)); we'll port
this validation once the pipeline is locked.

## Trajectory phases

| Phase | Frames | Pose pattern | What it stresses |
| --- | --- | --- | --- |
| A | 0–49 | straight east, constant 10 m/s | baseline link-following |
| B | 50–69 | hold (speed 0, no advance) | DS holds ego mid-link without VISSIM auto-removing it |
| C | 70–99 | reverse west, constant 3 m/s, heading π | DS can push poses VISSIM's internal model would never produce |
| D | 100–119 | lateral teleport ±5 m on Y | XIL-style off-link excursion |
| E | 120–149 | straight east, constant 10 m/s | recovery after the abuse |

## Invariants (pass/fail)

| Check | Pass criterion |
| --- | --- |
| `ego_assigned_unique_id` | Exactly one VISSIM `VehicleID` was ever assigned to the ego |
| `ego_never_deleted` | ego never appeared in `VISSIM_GetVehicleLists().deleted` |
| `ego_always_ds_controlled` | ego's `ControlledByVissim` stayed false on every frame it was visible |
| `background_traffic_grew` | VISSIM-internal vehicles spawned and persisted during the run |
| `ego_present_in_onlink_phases` | ego present in `VISSIM_GetTrafficVehicles` for **every** frame of phases A/B/C/E |
| `pushed_x_honored_in_onlink_phases` | after the first 5 snap-in frames, |pushed X − readback X| < 2 m |

Phase D **explicitly does not assert presence** — VISSIM drops the ego from
its snapshot on frames where the pushed Y is outside the link width. This
is documented behavior, not a defect. See "Empirical findings" below.

## Run

```powershell
conda activate realsim_dev

cd tests\Vissim\Probes\DSProxy_egoctrl
python egoctrl_test.py    # writes to out_2022/
```

VISSIM 2022 GUI opens; total run is ~30 s wall-clock for 150 frames at 10 Hz.

## Empirical findings (last run: 2026-06-06)

All invariants PASS. Observations:

- **max steady-state X-drift in on-link phases: 0.46 m.** This is VISSIM's
  sub-frame interpolation, not a control gap. At 10 Hz simulator rate
  with VISSIM's internal step likely 0.1s, the readback is one tick
  behind the push. For CarMaker running at 1 kHz visualizing the ego,
  this is smoothed away.
- **Phase D off-link presence: 50 % of frames (10/20).** Frames with
  positive lateral offset (+5 m) → ego present, snapped to lane 1 of the
  same link. Frames with negative offset (-5 m) → ego absent from
  `VISSIM_GetTrafficVehicles` snapshot, but **never appears in the
  `deleted` list** — VISSIM treats off-link as "not currently snappable",
  not "destroyed". The next frame back on-link, the ego reappears.
- **Background traffic grew 6 → 139 vehicles** over 150 frames. VISSIM-
  internal spawning is unaffected by our ego antics.
- **Ego retained VehicleID=8 throughout.** No re-creation, no churn.
- **Phase C (reverse-direction push with heading π)** worked — VISSIM
  honored the X decrease. Heading readback was ~0 rather than π
  (documented in PDF §1.2 — heading is not a reliable readback for
  `ControlledByVissim=false` vehicles; DS owns its own heading).

## XIL-readiness verdict

The DSProxy contract supports IPG / Carla / Simulink dyno coupling **as long
as the ego's pose stays within the VISSIM network's link/lane geometry**.
That constraint is the same constraint these tools already operate under
when driving on a modeled road. The pushed pose round-trips tightly
(<0.5 m) and the ego persists across stop/reverse/teleport patterns that
VISSIM's internal Wiedemann model would never produce — i.e. we have real
control, not narrow trajectory replay.

## Outputs

Under `out_2022/`:
- `run.log` — invariant results + observations (gitignored, regenerated)
- `pose_compare.csv` — per-frame pushed pose vs. readback pose (gitignored)
- `network/` — staged copy of PTV's shipped .inpx + companion files (gitignored)
- **`baseline_observations.json`** — checked-in numeric snapshot of the
  observations table above, for drift detection

Re-baseline only after intentional changes (e.g., VISSIM patch update,
DSProxy DLL bump) and confirm visual inspection.
