# Cross-simulator probes (FIXS #174)

One **shared scenario** (`tests/Sumo/SimpleLoop`) driving the FIXS
VirtualEnvironment pipeline through **different backends**, each with a
**one-click** launcher. This is the test harness #174 PR 1 stands up so the
later VirEnvCore consolidation has a green baseline to refactor against.

| Probe | Traffic | Virtual env | One-click | Status |
| --- | --- | --- | --- | --- |
| [TrafficLayer_SUMO_CMoffice](TrafficLayer_SUMO_CMoffice/) | SUMO | CarMaker office | `run_sumo_cm_demo.bat` | scaffold (not yet run) |
| [TrafficLayer_SUMO_Carla](TrafficLayer_SUMO_Carla/) | SUMO | Carla | `run_sumo_carla_demo.bat` | scaffold (not yet run) |
| [../../Vissim/Probes/TrafficLayer_DSProxy_CMoffice](../../Vissim/Probes/TrafficLayer_DSProxy_CMoffice/) | VISSIM (DSProxy) | CarMaker office | `run_cm_office_demo.bat` | ✅ verified (#168) |

The point of the matrix: the **CarMaker side is identical** between the SUMO-CM
and VISSIM-CM probes (same custom exe + `VirtualEnvironment.lib`, same TestRun),
so the only variable is the traffic backend. And the SUMO-CM vs SUMO-Carla
probes share the **same traffic source**, so the only variable is the virtual
environment. That 2-D differential is what catches divergence between the two
bridge implementations the consolidation merges.

## Why these are "pipeline-first" scaffolds

They were authored without a live run (no Carla on the dev box; CM/SUMO run is a
follow-up). Each probe README has an **"Open items before first run"** section.
The cross-cutting ones:

- **Shared geometry.** SimpleLoop exists as a VISSIM authoring + `simple_loop.xodr`
  (CarMaker road) **and** a separately-authored SUMO net. The co-sim exchanges
  absolute X/Y, so these must be **coordinate-matched**. The clean fix (the #174
  PR-1 unification) is to derive *all* representations from one
  `simple_loop.xodr` — SUMO via `netconvert --opendrive-files`, CarMaker via
  `osc2cm`. Until then, positions are approximate.
- **Ego ownership per backend.** `simple_loop.rou.xml` ships a single self-driven
  `ego` and no background traffic. CarMaker (mode A) wants to **own** the ego
  (interface injects `egoCm`); Carla visualization wants SUMO to keep driving it.
  Add a background flow and reconcile the ego convention per backend.
- **VirCarlaEnv aborts with no TLS table.** `mainVirCarla.cpp` returns -1 when the
  traffic-light table is empty — but SimpleLoop has no signals. The bridge must
  learn to run a signal-free scenario (see the SUMO-Carla README, open item 1).

## The real green baseline (next PR-1 step)

These probes need real simulators installed to run. The baseline that **doesn't**
is a **mock/null `IVirEnvBackend`** that records verb calls, fed a recorded FIXS
traffic trace — a core test that asserts the bridge's *decisions* (active-id set,
spawn/despawn events, poses at the verb boundary) with **no CarMaker, SUMO, or
Carla installed**. That is the regression guard that protects the extract-class
refactor; these on-machine probes are the higher-level integration confirmation.
