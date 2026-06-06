# Vissim/Ipg — CarMaker-VISSIM co-simulation scenario

Intended dock point for the canonical **CarMaker-VISSIM** test scenario tracked
by [#101](https://github.com/ORNL-Real-Sim/FIXS/issues/101) (lib changes) and
gated on the design decisions in
[#156](https://github.com/ORNL-Real-Sim/FIXS/issues/156) (VISSIM DLL path,
signal routing).

## Status — not runnable as-is (2026-06-05)

This directory is a stale port from the original SUMO CoordMerge scenario.
Files present:

| File | State |
| --- | --- |
| `config_vissim.yaml` | `SelectedTrafficSimulator: VISSIM`, but `CarMakerSetup.EnableCosimulation: false`. CarMaker side is therefore **inactive**. |
| `runCoordMergeVissim.bat` | Despite the name, launches **SUMO** (`sumo-gui -c coordMerge.sumocfg ... --num-clients 1`) and references `config_SUMO.yaml`. Neither file exists in this directory. |
| `runCoordMergeVissim.m` | Loads `config_vissim.yaml` and the `RealSimGeneric.mdl` Simulink model, but the post-init `load_system` / `sim` block is commented out. Does not start VISSIM. |
| `cmproject.txt` | Points at `../../../CM11_Proj`. Current supported target is **CM13.1.2** (per repo `CLAUDE.md`). |
| `RealSimGeneric.mdl` | CarMaker-for-Simulink generic harness; unchanged from SUMO scenario. |

Running any of these today will not produce a CarMaker-VISSIM co-simulation.
Treat this directory as a **placeholder dock point**, not a working example.

## What needs to land before it can be made runnable

1. **#156** — pick the VISSIM DLL path (DriverModel.dll vs DrivingSimulator.dll)
   and the signal-routing option. This determines whether `VirtualEnvironment.lib`
   needs a contract change and whether existing FIXS-side glue can be reused.
2. **#101** — once #156 lands, update `VirtualEnvironment.lib` as needed and
   build a working batch / MATLAB launcher pair here, target CM13.1.2, and
   bump `cmproject.txt` accordingly.
3. Add Parquet reference traces alongside the scenario for regression checks,
   mirroring the pattern established in
   [tests/Vissim/SpeedLimit/](../SpeedLimit/) (`*_orig.parquet`).

## See also

- [doc/CarMakerDoc.md](../../../doc/CarMakerDoc.md) — CarMaker integration guide
  (currently SUMO-only; VISSIM section is stubbed pending #101 + #156).
- [doc/VISSIMdoc.md](../../../doc/VISSIMdoc.md) — VISSIM integration notes.
- [CLAUDE.md](../../../CLAUDE.md) — repo-wide notes, including the VISSIM 2022
  dispatch playbook for Win11 24H2.
