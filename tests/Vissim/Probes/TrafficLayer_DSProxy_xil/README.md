# TrafficLayer_DSProxy_xil — Stage B probe for #158

End-to-end XIL pipeline: fake-CarMaker ↔ TrafficLayer (DSProxy mode) ↔
VISSIM 2022. Same shape as the real CarMaker integration but with no
CarMaker dependency.

```
fake_carmaker.py  --[VehFullData_t @ socket 2444]-->  TrafficLayer.exe
                                                          |
                                                          |  VISSIM_SetDriverVehicles(egos)
                                                          v
                                                       VISSIM 2022
                                                          |
                                                          |  GetTrafficVehicles + GetSignalStates
                                                          v
TrafficLayer  --[VehFullData_t + TrafficLightData_t]-->  fake_carmaker.py
```

## What Stage B exercises (on top of Stage A's plumbing)

1. ConfigHelper parses `ApplicationSetup.VehicleSubscription`.
2. `runDSProxyMode` opens a FIXS-protocol server socket on the
   subscription port, waits for the client to connect.
3. Per tick:
   - `VISSIM_SetDriverVehicles(egos)` — egos populated from the *previous*
     tick's `recvData` (ego `Create` flag managed via `CreateID` →
     `VehicleID` round-trip).
   - `VISSIM_GetTrafficVehicles` + `VISSIM_GetSignalStates` from VISSIM.
   - Translate each `VISSIM_Veh_Data` to `VehFullData_t` and each
     `VISSIM_Sig_Data` to `TrafficLightData_t`, populate `MsgHelper`
     send maps.
   - `Sock_c.sendData(clientSock, …)` — publishes to consumer.
   - `Sock_c.recvData(clientSock, …)` — pulls back the ego pose for the
     next tick.
4. Translate `VehFullData_t` (whose `id == configured EgoId`) →
   `Simulator_Veh_Data` for the next `SetDriverVehicles`.
5. On shutdown, send `simState=0` to the client and disconnect VISSIM.

## Run

```cmd
scripts\dispatch\2_core_components.bat           REM build TrafficLayer.exe
cd tests\Vissim\Probes\TrafficLayer_DSProxy_xil
run_stageB_xil.bat                                REM stages .inpx, launches TL + fake_carmaker
```

`run_stageB_xil.bat` orchestrates everything: stages PTV's shipped DS
example into `stage_network\`, launches `TrafficLayer.exe`, waits, then
launches `fake_carmaker.py`. VISSIM 2022 GUI opens.

## Empirical baseline (last run: 2026-06-06)

| Check | Result |
| --- | --- |
| TrafficLayer config parse | OK |
| `VISSIM_Connect` | OK |
| Application socket bound on port 2444 | OK |
| `fake_carmaker.py` connects and handshakes | OK |
| Ego `Create=true` round-trips to `VehicleID` | OK — registered as VehicleID 7 in this run |
| Ticks executed | 300 / 300 |
| Vehicles received by client (last frame) | 138 |
| Peak vehicles | 139 |
| Signals received by C++ TrafficLayer | 20 per frame |
| Signals received by Python client | 0 — see "Known gap" below |
| Ego longitudinal motion | `ego_x_sent` 397.88 → 696.88 m over 300 ticks @ 10 m/s ✓ |
| Clean shutdown (`simState=0`, `VISSIM_Disconnect`) | OK |

## Known gap — Python TLS parsing

`CommonLib/SocketHelper.py::recv_data` recognizes the
`MessageType.traffic_light_data` identifier (msg_type 2) but its handler
is a placeholder (`aa = 1`) — bytes are read off the wire but never
parsed into `traffic_light_data_receive_list`. So `recv_tls=0` in the
fake-CarMaker summary even though the C++ TrafficLayer side is publishing
the 20 signals per tick correctly (verified by the C++ side's `signals=20`
summary).

This is a pre-existing limitation in the Python CommonLib unrelated to
Stage B's pipeline. C++ consumers (CarMaker via `VirtualEnvironment.lib`)
will receive the TLS data normally because `CommonLib/SocketHelper.cpp`
has the full parsing path.

## What's still deferred to follow-up Stage B work

- **`tests/Vissim/Ipg/` refurbishment** — bump `cmproject.txt` from CM11
  to CM13.1.2, rewrite `runCoordMergeVissim.bat` and `.m` launchers,
  produce a Parquet reference trace alongside. Requires a CM 13.1.2
  install on the dev box to validate.
- **VISSIM section of `doc/CarMakerDoc.md`** — replace #157's stub with
  the real CarMaker-VISSIM runbook.
- **Multi-port subscription routing** — Stage B honors only
  `VehicleSubscription[0].port[0]`. Real production scenarios may have
  multiple subscribers; the existing TrafficLayer publish loop does
  per-port subscription filtering and Stage B's simplification will need
  the same shape once a multi-port scenario appears.
- **Multi-ego support** — Stage B accepts one ego per tick. Multi-ego is
  trivial structurally (push more `Simulator_Veh_Data` entries) but the
  `egoCreateId` → `VehicleID` map needs to be per-ego; defer until
  scenario 1's high-fidelity multi-ego case is real.

## Files

| File | Purpose |
| --- | --- |
| `config.yaml` | TrafficLayer config with `VissimDSProxySetup` + `ApplicationSetup.VehicleSubscription` |
| `fake_carmaker.py` | Python client modeled on `tests/Python/SimpleEchoClient/simple_echo_client.py` — sends one ego per tick, receives vehicles + signals |
| `run_stageB_xil.bat` | Stage .inpx + launch TrafficLayer + launch fake-CarMaker |
| `stage_network/` | Writable mirror of PTV's shipped DS example (gitignored) |
