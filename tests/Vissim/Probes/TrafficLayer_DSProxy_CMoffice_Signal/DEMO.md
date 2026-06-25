# #172 — CarMaker ↔ VISSIM co-simulation with traffic-signal sync

The complete FIXS #172 demo: CarMaker (office) drives an ego through a **signalized**
VISSIM corridor via the DSProxy pipeline, with **both** halves live:

- **vehicles transmitted** — the CM ego pose → VISSIM; VISSIM background traffic →
  CM `RS_Cxxx` objects (the #168 vehicle co-sim), and
- **signals synced** — VISSIM signal-controller state → CM traffic lights, so the
  CM ego **brakes at the VISSIM-driven red** (this issue, #172).

It builds on [../TrafficLayer_DSProxy_CMoffice/](../TrafficLayer_DSProxy_CMoffice/)
(#168, vehicles only, no signals) and the scene-only signal work in this directory
(`add_signal_stops.py`, `build_signal_table.py` — the ego stops at red in CarMaker
alone, no VISSIM).

```
CarMaker.win64.exe ──[ego VehFullData @ 2444]───────────▶ TrafficLayer.exe (DSProxy)
   User.c → VirtualEnvironment.lib                              │  VISSIM_SetDriverVehicles
        ▲      ▲                                                ▼
        │      │                                          VISSIM (DrivingSimulatorProxy.dll)
        │      └─[signal-group state @ 2445]──────────────  GetTrafficVehicles + GetSignalStates
        └────────[background traffic @ 2444]──────────────  simple_traffic_light, 3 signalized ints
```

## The two-socket topology (Plan A)

When `SynchronizeTrafficSignal: true`, CarMaker's `VirtualEnvironment.lib` opens a
**second** client socket to `TrafficSignalPort` (2445) in addition to the vehicle
socket (2444), and every tick recvs signal data on it + echoes its ego back. The
legacy DSProxyMode served only the vehicle port → the second connect had no
listener → deadlock. **Plan A** (this issue) makes `DSProxyMode` bind + serve the
signal port: it relays per-`(controller, signal-group)` `TlsData` there each tick
and drains the redundant ego. The `.lib` (CommonLib) is unchanged.

CarMaker maps the VISSIM signal-group state to its traffic-light objects via the
`RSsignalTable.csv` passed with `-s` (see `build_signal_table.py` / the table's
README section). The ego brakes because the in-road `DrvStop` references a
controller whose state is now driven by VISSIM.

## Run it

**Headless, self-checking (recommended proof / CI):**
```cmd
run_signal_demo.bat
```
Builds the assets (`add_signal_stops.py` → `build_signal_table.py` →
`build_cosim_testrun.py`), then `verify_signal_demo.py` stages the network,
launches TrafficLayer (→ VISSIM) and the headless CarMaker exe (with `-f config` +
`-s RSsignalTable.csv`), and asserts the round-trip: `.lib` connected, **signal
client connected (2445)**, per-tick **vehicles>0 AND signals>0**, ego=1, SIM_END.

**GUI (3D view), one click:**
```cmd
run_cm_office_signal_demo.bat
```
Builds the assets, stages the network, registers the custom exe + `-f config -s
RSsignalTable.csv` into the CM GUI config (via `setup_gui.py`), launches
TrafficLayer (→ VISSIM, serving ports 2444 + 2445), and opens CarMaker Office +
IPGMovie on `SimpleTL_Cosim`. Then, once the TrafficLayer window prints
`VISSIM_Connect OK` **and** `signal listener bound on port 2445`: load TestRun
`SimpleTL_Cosim` and press **Start**. The ego drives the corridor and brakes at
the VISSIM-driven red; VISSIM background traffic renders as `RS_C` cars. Stop
order: **Stop in CarMaker → close VISSIM → Ctrl-C TrafficLayer** (never `taskkill`
VISSIM while it holds the `.inpx` — leaks CodeMeter sessions).

## Files (added for #172 co-sim)

| File | Role |
|---|---|
| `config.yaml` | DSProxy + CarMaker config; `SynchronizeTrafficSignal: true`, `TrafficSignalPort: 2445` |
| `build_cosim_testrun.py` | `SimpleTL_Cosim` = ego (Car_Normal + DrvStops) + 50 `RS_C` traffic slots on Route 3 |
| `verify_signal_demo.py` | headless end-to-end check (vehicles + signals), adapted from #168 `verify_demo.py` |
| `run_signal_demo.bat` | one-click: build assets + headless verify |
| `simple_traffic_light_ds.inpx` | DS-enabled signalized network (3 controllers) |

Reuses the scene-only assets (`add_signal_stops.py`, `build_signal_table.py`,
`simple_traffic_light_signalstop.rd5`, `..._RSsignalTable.csv`) — the SAME rd5
serves both the scene-only and the co-sim demos.

## Prereqs

- VISSIM 2022 licensed + healthy (CodeMeter) — see the #168 README for the
  CodeMeter / VISSIM-dispatch recovery notes.
- `TrafficLayer.exe` built Release x64 (includes the Plan A signal-port serving).
- The custom CarMaker exe; `verify_signal_demo.py` builds the headless harness exe
  (`CarMaker_headless.win64.exe`) via the #168 `build_headless_exe.bat`.
