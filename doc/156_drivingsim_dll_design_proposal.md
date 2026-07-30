# #156 — VISSIM `DrivingSimulatorProxy.dll` design proposal

> Status: draft, evidence-backed by probes under
> [`tests/Vissim/Probes/`](../tests/Vissim/Probes/). Targets Milestone 0.9.0,
> closes the design half of [#156](https://github.com/ORNL-Real-Sim/FIXS/issues/156),
> unblocks [#101](https://github.com/ORNL-Real-Sim/FIXS/issues/101).
>
> Scope of empirical work so far: **VISSIM 2022 only**. 2026 parity for the
> basic DSProxy flow was checked once at the header / smoke-test level (Stage 1)
> and then deprioritized until the pipeline is locked.

## Decision summary

| Axis | Choice | Rationale |
| --- | --- | --- |
| Vehicle-state path | **B′** — DSProxy in TrafficLayer for ego + all readback; DriverModel demoted to behavior-modifier-only on flagged types | Scenario 3a (Python CAV controller modulating VISSIM-internal Wiedemann) loses its natural home if we migrate fully to DSProxy (Option A). B′ preserves it. Empirically confirmed both DLLs can co-load (see Stage 2 below). |
| Signal-state path | **Source: DSProxy `VISSIM_GetSignalStates`** ; Transport: existing TrafficLayer SignalSubscription socket | DSProxy delivers signals on the same tick boundary as vehicle data, globally keyed by SC/SG. Stage 1 verified emission. `VirtualEnvironment.lib` ABI stays unchanged. |
| DriverModel scope under B′ | **Behavior-write-only** (per-vehicle desired-speed, lane-change desire). State reporting up to TrafficLayer is retired — DSProxy already provides it. | Removes redundant code paths and the #129 signal-table TODOs become moot. |
| TrafficLayer ↔ VISSIM coupling | **In-process linkage via DSProxy DLL** (TrafficLayer.exe links DrivingSimulatorProxy.lib, spawns VISSIM via `VISSIM_Connect`) | Architecturally symmetric with the libsumo path. Inverts today's "VISSIM spawns the FIXS DLL" topology. |

## Evidence pointers

| Question answered | Probe | Result |
| --- | --- | --- |
| License has DS Interface feature? | [`DSProxy_smoke/`](../tests/Vissim/Probes/DSProxy_smoke/) | `VISSIM_Connect` succeeds in 13–14 s |
| 2022 ↔ 2026 ABI parity? | [`DSProxy_smoke/`](../tests/Vissim/Probes/DSProxy_smoke/) | Same Python wrapper runs against both, only `versionNo` differs |
| Can DSProxy give XIL-style ego control? | [`DSProxy_egoctrl/`](../tests/Vissim/Probes/DSProxy_egoctrl/) | PASS — hold / reverse / lateral teleport / recovery all work; 0.46 m max steady-state X-drift; off-link excursions drop ego from snapshots but never delete it |
| Can DSProxy + DriverModel coexist in one VISSIM? | [`DSProxy_DriverModel_coexist/`](../tests/Vissim/Probes/DSProxy_DriverModel_coexist/) | YES with PTV's stock DriverModel sample (13.5 s); FIXS-specific block identified, fix scoped (see Stage C) |

---

## Staged implementation plan

Five stages. Each is independently shippable and adds bounded value; later
stages presuppose earlier ones land. Stages A–D are 0.9.0 scope; Stage E
is a follow-up milestone.

### Stage A — TrafficLayer ↔ DSProxy plumbing (no XIL, no DriverModel changes)

**Goal:** TrafficLayer can drive a VISSIM scenario via DSProxy. No CarMaker
yet, no DriverModel interaction, no behavior change for current scenarios.

**What changes**

- `CommonLib/VissimDSProxyHelper.{h,cpp}` — C++ wrapper around
  `DrivingSimulatorProxy.dll`. Same surface as the Python wrapper in
  [`DSProxy_smoke/dsproxy_wrapper.py`](../tests/Vissim/Probes/DSProxy_smoke/dsproxy_wrapper.py),
  ported to C++.
- `CommonLib/ConfigHelper`: parse new `VissimDSProxy:` block.
- `CommonLib/TrafficHelper` (or a sibling sub-module —
  `TrafficHelper.cpp` is ~1743 lines and on the 0.8.0 refactor list, so
  prefer adding alongside): new "DSProxy mode" code path. When
  `VissimDSProxy.Enable: true`, TrafficLayer spawns VISSIM via
  `VISSIM_Connect`, drives the tick from there, and bypasses the existing
  COM-based VISSIM setup.
- Per-tick loop: `VISSIM_SetDriverVehicles(empty)` → `VISSIM_DataReady`
  poll → `VISSIM_GetTrafficVehicles` + `VISSIM_GetSignalStates` →
  publish on existing application sockets.

**Test scenario:** `tests/Vissim/Probes/TrafficLayer_DSProxy/` — VISSIM
spawned by TrafficLayer via DSProxy, Python client subscribes to the
publish bus and asserts ≥1 vehicle and ≥1 signal arrive within N ticks.

**Acceptance**

- TrafficLayer+DSProxy starts VISSIM and runs N ticks
- Application sockets emit vehicle + signal data in the existing format
- Existing DriverModel-based tests still pass (no regression — different code path)

**Effort estimate:** ~1 week (most of it is the C++ port of the ctypes wrapper +
ConfigHelper schema + TrafficHelper integration; the architectural questions
are already settled empirically).

**ProprietaryFiles change required:** no.

### Stage B — XIL ego coupling (closes #101, CarMaker-VISSIM working scenario)

**Goal:** CarMaker (or Carla / Simulink) dyno controls the VISSIM ego via
DSProxy through TrafficLayer.

**What changes**

- `VirtualEnvironment.lib`: no ABI change (already simulator-agnostic per
  #157's scope clarification).
- TrafficLayer DSProxy path: accept incoming ego pose messages from
  CarMaker (existing `VehFullData_t` message shape), translate to
  `Simulator_Veh_Data`, push via `VISSIM_SetDriverVehicles` with the
  `Create=true` → remember `VehicleID` → `Create=false` lifecycle.
- Per-ego `CreateID → VehicleID` map kept in TrafficLayer state.
- [`tests/Vissim/Ipg/`](../tests/Vissim/Ipg/): refurbish to runnable.
  Today it's a stale SUMO port (see the README from PR #157). Target CM
  13.1.2. Update `cmproject.txt`, rewrite `.bat` and `.m` launchers,
  build a `config.yaml` that enables `VissimDSProxy:` + the existing
  `CarMakerSetup:` block.
- Per Stage 1.5's findings, document the link-width tolerance and
  PDF §1.2 readback caveat in [doc/CarMakerDoc.md](CarMakerDoc.md)'s
  VISSIM section (stubbed by #157).

**Test scenario:** `tests/Vissim/Ipg/` becomes the canonical
CarMaker-VISSIM scenario with end-to-end CM 13.1.2 verification.

**Acceptance**

- CarMaker dyno owns the ego pose; VISSIM honors it within link-width
- VISSIM-internal background traffic reacts to the dyno (visible in
  `LeadingVehicleID` of nearby vehicles)
- Signals delivered to CarMaker via `VirtualEnvironment.lib` (unchanged ABI)
- Reference Parquet trace alongside the scenario for regression
  (mirrors the pattern in [`tests/Vissim/SpeedLimitLite/`](../tests/Vissim/SpeedLimitLite/))

**Effort estimate:** ~1–2 weeks. Most of the risk is on the CM side
configuration, not the DSProxy side.

**ProprietaryFiles change required:** likely yes for the CM project file
updates (`ProprietaryFiles/CM11_proj` → CM13.1.2 / `CM13_proj` path).

### Stage C — DriverModel coexistence (B′ doctrine, unlocks scenario 3a)

**Goal:** DSProxy ego + DriverModel-flagged background traffic coexist
in one VISSIM. Python CAV controller scenario (3a) becomes runnable.

**What changes**

- **ProprietaryFiles patch** (`DriverModel_FIXS_Common.h`, ~line 1280):
  move `Sock_c.socketSetup(serverAddr, ...)` and
  `Sock_c.disableServerTrigger()` inside the `if (ENABLE_REALSIM)`
  block. This is the **Stage 2 root cause fix**: today those run
  unconditionally and block VISSIM long enough for DSProxy's handshake
  to abort.
- New config flag, parsed by both `ConfigHelper.cpp` and the DriverModel
  par-file path:

  ```yaml
  SimulationSetup:
      DriverModelMode: behavior_only   # behavior_only | full (default: full)
  ```

  Under `behavior_only`:
  - DriverModel skips per-vehicle state reporting up to TrafficLayer
    (DSProxy already provides it)
  - DriverModel opens only the behavior-cmd receive channel
  - Many `// TODO #129:` stubs in `DriverModel_FIXS_Common.h` become
    moot under this mode (signal-table read, connection-failed paths)
- TrafficLayer: handle behavior-only DriverModel registration —
  simplified handshake, no per-vehicle state ingest from the 1337 socket.

**Test scenario:** re-purpose
[`tests/Vissim/Probes/DSProxy_DriverModel_coexist/`](../tests/Vissim/Probes/DSProxy_DriverModel_coexist/)
with `EnableRealSim: true` + `DriverModelMode: behavior_only`. Add
a Python CAV controller stub that sends a desired-speed override on
selected DriverModel-flagged vehicles and verifies VISSIM applies it.

**Acceptance**

- Existing `DSProxy_DriverModel_coexist` matrix gains a fifth mode
  (`real_drivermodel_behavior_only`) that returns `VISSIM_Connect: True`
  in ~14 s instead of failing at 20 s
- Python CAV controller's desired-speed override visibly applied to
  flagged vehicles in the next tick's readback
- One-tick latency on behavior cmds documented (acceptable for
  desired-speed / lane-change; not for tight closed-loop control)

**Effort estimate:** ~1 week (ProprietaryFiles patch is small; the
TrafficLayer-side simplified-handshake handling is the bulk of it).

**ProprietaryFiles change required:** yes — small, well-scoped, fits the
dual-PR workflow.

### Stage D — Signal source migration (low-risk drop-in)

**Goal:** TrafficLayer sources signals from `VISSIM_GetSignalStates`
instead of COM enumeration when DSProxy is active. Tick-aligned with
vehicle data.

**What changes**

- TrafficLayer DSProxy path: replace the existing COM
  `SignalControllers.GetAll`-style enumeration with
  `proxy.get_signal_states()` (the Stage A wrapper already exposes it).
- COM signal path retained for legacy non-DSProxy scenarios — no
  regression there.
- Wire format unchanged. CarMaker (`VirtualEnvironment.lib`), Carla
  (`VirCarlaEnv`), and Python consumers see no behavioral change.

**Test scenario:** the existing
[`tests/SignalIpg/`](../tests/SignalIpg/) scenario, run with the new
DSProxy-sourced signal path, should produce structurally identical
signal sequences (allowing for tick-alignment improvements).

**Acceptance**

- Signal-bus consumers see no behavioral change beyond improved tick alignment
- Per-tick signal data is now synchronized with the vehicle frame
  (today there's a small cadence skew because COM enumeration runs on
  TrafficLayer's wall clock, not VISSIM's tick)

**Effort estimate:** ~2–3 days.

**ProprietaryFiles change required:** no.

### Stage E — Cleanup / deprecation (later milestone — explicitly NOT 0.9.0)

**Goal:** Once B′ is proven across all scenarios in 0.9.0, retire the
duplicate code paths that DSProxy replaced.

**What might change** (deferred — depends on 0.9.0 retro):

- Retire DriverModel signal-table path entirely (the #129 TODOs go away)
- Retire COM signal enumeration in TrafficLayer
- Optionally rename `DriverModelMode: full` → mark deprecated; new
  scenarios default to `behavior_only` when DSProxy is enabled
- Update [doc/VISSIMdoc.md](VISSIMdoc.md) and
  [doc/CarMakerDoc.md](CarMakerDoc.md) to recommend DSProxy as the
  canonical VISSIM coupling path

**Explicitly NOT in 0.9.0 scope.** Per #156's body, "Removing the
DriverModel path even if option 2 is chosen — staged deprecation in a
later milestone."

---

## Socket pipeline shape under B′

```
                                       ┌─── DrivingSimulatorProxy.dll
                                       │      (linked in-proc; shared memory
                                       │       with VISSIM, not a socket)
TrafficLayer  ◄────────────────────────┤
   ▲                                   │
   │ TCP 1337                          ▼
   │  (behavior cmds DOWN only,        VISSIM
   │   when DriverModelMode=             ▲
   │   behavior_only)                    │
   │                                     │ (DLL load on flagged
   │                                     │  vehicle types)
   ▼                                     │
DriverModel_FIXS.dll  ──────────────────┘
   (thin behavior write-back layer;
    no state-up reporting under B′)

TrafficLayer  ──TCP application ports──▶  CarMaker (VirtualEnvironment.lib)
                                          Carla (VirCarlaEnv)
                                          Python clients
                                          Simulink
   (ego poses + behavior cmds in,
    everyone's state + signals out)
```

**Tick sequence inside TrafficLayer** (DSProxy mode):

1. Drain inbound queues (CarMaker ego poses, Python behavior cmds)
2. `VISSIM_SetDriverVehicles(ego_array)` — inject CarMaker dyno + any DS egos
3. VISSIM advances internally:
   - Wiedemann for unhooked vehicle types
   - DriverModel callback fires on flagged types — reads queued behavior
     cmd from TrafficLayer's in-memory queue, returns it to VISSIM
4. `VISSIM_GetTrafficVehicles` + `VISSIM_GetSignalStates`
5. Publish on consumer application sockets (vehicles + signals)

**Port allocation:** no new ports. 1337 stays (now for DriverModel
behavior-cmd-down only). Application sockets (2444 etc.) unchanged.

**Latencies:**

- Ego pose: tick-locked (CarMaker → TrafficLayer → DSProxy → VISSIM)
- Behavior cmd from Python: **one tick of latency** (Python → TrafficLayer
  → queue → DriverModel returns it on the next VISSIM tick). Acceptable
  for desired-speed / lane-change modulation; not for tight closed-loop
  vehicle dynamics.
- Vehicle state out / signals out: same tick as the read (no skew between
  vehicle and signal data; that's an improvement over today's COM path).

## YAML schema additions (non-breaking)

```yaml
SimulationSetup:
    EnableFIXS: true                # existing
    SelectedTrafficSimulator: VISSIM
    TrafficSimulatorIP: "127.0.0.1"
    TrafficSimulatorPort: 1337       # DriverModel ↔ TrafficLayer (unchanged role)

    # NEW (Stage A) — only consulted when SelectedTrafficSimulator == VISSIM
    # AND Enable: true. Switches the VISSIM coupling from today's COM-driven
    # path to DSProxy. TrafficLayer spawns VISSIM via DSProxy in this mode.
    VissimDSProxy:
        Enable: false                # default off; flip to true for CarMaker-VISSIM and 0.9.0 scenarios
        NetworkFile: scenario.inpx   # required when Enable: true
        VissimVersion: 2022          # 2022 | 2026 ; selects versionNo (2200/2600) + DLL path
        SimulatorFrequency: 10       # Hz; sub-frame interpolation if > VISSIM internal step
        VisibilityRadius: -1.0       # meters; -1 = unlimited
        MaxSimulatorVeh: 10          # ceiling on simultaneous egos
        MaxTotalVeh: 50000
        MaxVissimSigGrp: 1000

    # NEW (Stage C) — narrows DriverModel's role to behavior-cmd-down-only,
    # avoiding the unconditional Sock_c.socketSetup hang and removing
    # state-up redundancy (DSProxy already provides it).
    DriverModelMode: full            # behavior_only | full (default: full — today's behavior)
```

Defaults preserve current scenario behavior. Flipping `VissimDSProxy.Enable: true`
selects the new path; adding `DriverModelMode: behavior_only` enables
coexistence under B′.

## What stays open

These are known unknowns that don't block the staged plan but should be
sized once Stages A–C land:

1. **Multi-ego scaling under B′.** Stage 1.5 validated single-ego control;
   scenario 1's "multiple egos for high-fidelity dynamics" case is
   supported by DSProxy's `Simulator_Veh_Data` array but unexercised by
   the probes. Worth a Stage B add-on probe with N=2 or N=3 egos before
   declaring it shipped.
2. **2026 port.** Header parity validated; runtime parity assumed by
   transitive reasoning. Re-run Stage 1 + Stage B against VISSIM 2026
   after the pipeline lands.
3. **VISSIM's DS-handshake watchdog timeout.** The Stage 2 failure showed
   VISSIM cancels DSProxy after ~7 s of DriverModel init blocking. If
   the FIXS DriverModel patch (Stage C) reduces socket-setup to a few
   hundred milliseconds, this is moot. But if any *legitimate*
   DriverModel init takes >5 s on heavier scenarios, we may need to
   investigate increasing the watchdog (PTV-side) or pre-loading state.
