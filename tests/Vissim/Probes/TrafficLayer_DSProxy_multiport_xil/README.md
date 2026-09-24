# Stage B multi-port routing probe

End-to-end test for the multi-port subscription routing introduced in
PR #165 (issue #158 Stage B). Two Python clients connect to TrafficLayer
in DSProxy mode on different ports and validate that the per-port
filter in `DSProxyMode.cpp::publishesVehicle()` actually selects the
right vehicles per socket.

## Setup

- VISSIM 2022 installed at the default location (the .bat uses
  `C:\Program Files\PTV Vision\PTV Vissim 2022\API\DrivingSimulator_DLL\example\...`).
- realsim_dev conda env at `%USERPROFILE%\miniconda3\envs\realsim_dev\`.
- TrafficLayer.exe Release build at `TrafficLayer\x64\Release\`.

## Run

```
run_multiport_xil.bat
```

Stages the PTV-shipped `driving_simulator_test.inpx` into `stage_network\`,
writes `config.runtime.yaml` with `VissimSetup.NetworkFile` resolved to
this checkout (the same generated-runtime-config convention the CMoffice
demos use — the committed `config.yaml` path is a placeholder because
NetworkFile must be absolute), launches TrafficLayer with the two-port
config, then spawns `multiport_clients.py`. The latter opens both client
sockets in threads and prints per-tick + invariant summary.

## Invariants

| Name | What it checks |
| --- | --- |
| `both_clients_connected` | Both port-2444 (ego) and port-2445 (observer) clients connected to TL |
| `both_clients_no_error` | Neither client threw an exception during the run |
| `tick_aligned` | Both clients made the same number of ticks (TL drives in lockstep) |
| `observer_filtered_to_Car` | Port 2445 only received vehicles whose `type=='100'` (Car) |
| `ego_port_unfiltered` | Port 2444 received a mix of vehicle types (HGV/Bus also visible) |
| `signals_count_aligned_across_ports` | Both ports received an identical TLS count every tick (20 signal groups on the shipped DS example) |

Output: `out/summary.json` + PASS/FAIL per invariant.

Last verified run (all six PASS): observer saw only `['100']`, ego port
saw `['100', '300', '400']`, 20 TLS/tick on both ports for 80/80 ticks.
The per-tick vehicle-count delta between the ports (e.g. 58 vs 53 at
tick 60) is the non-Car traffic on the network — direct evidence the
filter selects rather than passes everything through.

## Config layout

```yaml
ApplicationSetup:
    VehicleSubscription:
    -   type: ego           # port 2444 — ego control + observe all
        attribute: { id: ['ego'], radius: [0] }
        ip: ['127.0.0.1']
        port: [2444]
    -   type: vehicleType   # port 2445 — observe only type 100 (Car)
        attribute: { id: ['100'], radius: [0] }
        ip: ['127.0.0.1']
        port: [2445]
```

`type: ego` does NOT act as a publish filter (the VISSIM-assigned ego
VehicleID is numeric and wouldn't match the literal id `'ego'`); it's a
recv-side identifier for the canonical DSProxy ego inject. `type:
vehicleType` is the real per-vehicle filter, matching on
`VehFullData_t.type` against the YAML `attribute.id` list.

## Out-of-scope

- CarMaker — this probe is Python-only; the CarMaker XIL wiring is in
  `tests/Vissim/Ipg/` (separate PR).
- Per-port signal filtering — signals are global to all sockets in the
  current implementation; per-port TLS subscription can come later if a
  scenario needs it.
