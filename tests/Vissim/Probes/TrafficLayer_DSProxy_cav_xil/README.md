# TrafficLayer_DSProxy_cav_xil — Stage B+ probe for #158 (WIP)

Targets the full B′ doctrine end-to-end:

```
python_ego_cav.py  --[VehFullData @ port 2444]-->  TrafficLayer (DSProxy mode + DM relay)
                                                       │
                                       ┌───────────────┴───────────────┐
                                       ▼                               ▼
                                  DSProxy.dll                  DriverModel socket (1337)
                                       │                               │
                                       ▼                               ▼
                                  VISSIM 2022                  FIXS DriverModel_RealSim.dll
                                  (ego = HGV)                  (hooked on Car type 100,
                                                                EnableRealSim: true)
```

A single Python client sends BOTH the ego pose (`id='ego'`) and CAV behavior commands (`id=<VISSIM VehicleID>`, `speedDesired=5 m/s` to slow down Cars from VISSIM-default ~14 m/s). TrafficLayer splits — ego goes to DSProxy, CAV cmds go to DriverModel via 1337.

## Status — work in progress

**What works** (verified end-to-end on this branch):
- ✅ Multi-port socket setup: bind 1337 + bind app port BEFORE `VISSIM_Connect` so DM's connect during the DSProxy handshake doesn't fail
- ✅ `VISSIM_Connect` succeeds with `EnableRealSim: true` in the par-file (PF#6 patch from Stage C carries through)
- ✅ DM connects on 1337 immediately (TCP backlog accepted from queue)
- ✅ App client connects on 2444
- ✅ "Running 300 ticks" reached

**What hangs at tick 0**:
- The per-tick loop sends DM commands (empty on tick 0) → calls `proxy.setDriverVehicles(empty)` → hangs.
- VISSIM ticks, DM callback fires, DM does `recvData(VissimSock, ...)` to read TL's commands. The 9-byte header we sent should be there.
- Either DM's `recvData` finds something it doesn't expect, OR DM sends back state in a format TL's later `recvData` can't parse, OR there's another handshake step in the FIXS DriverModel protocol we haven't accounted for.

**Most likely culprit**: there's a FIXS DriverModel handshake protocol detail (probably around `disableServerTrigger` / `disableWaitClientTrigger` state and what bytes flow during initConnection) that the existing TrafficLayer main loop handles implicitly via `SocketHelper::initConnection`'s state machine but my Stage B+ bypass replicates incorrectly.

## Where to pick up

1. Read `CommonLib/SocketHelper.cpp::initConnection` from line 720 onwards. The wait-client-trigger + server-trigger blocks are gated on different ENABLE_* flags. The FIXS DriverModel uses the 2-arg `socketSetup(serverAddr, serverPort)` variant which sets `ENABLE_SERVER=true` and `ENABLE_CLIENT=false`, and explicitly calls `disableServerTrigger()` before `initConnection`. Trace what's actually sent/received in that case.
2. Use Wireshark on `lo` port 1337 to see exactly what bytes flow between DM and TL during the existing working `tests/Vissim/SimpleEcho/` scenario. Then compare to what my Stage B+ TL does — the wire diff is the protocol gap.
3. The architectural shape (multi-port routing, send-before-set, recv-after-set) is right; only the byte-level handshake detail is wrong.

## Files

| File | Purpose |
| --- | --- |
| `config.yaml` | TrafficLayer config with `VissimDSProxySetup.EnableDriverModelRelay: true` (new Stage B+ flag) |
| `coexist_par.yaml` | DriverModel par-file — uses **`EnableRealSim: true`** (vs the Stage C probe's `false`) so DM actively connects and exchanges data |
| `patch_inpx.py` | Stages PTV's `.inpx` + attaches FIXS DriverModel on Car type 100 |
| `python_ego_cav.py` | Single Python client; sends ego + CAV cmds for each visible Car |
| `run_cav_xil.bat` | Orchestration |
| `out/`, `stage_network/`, `tl.log` | Generated; gitignored |

## What the C++ side does

In `TrafficLayer/.../DSProxyMode.cpp`:

1. **`bindListener()` lambda** — raw Winsock `socket() + bind() + listen()` for each port. Called BEFORE `VISSIM_Connect` so DM's connect (which happens during the DSProxy handshake on the VISSIM side) lands in the TCP backlog.
2. **`accept()` after `VISSIM_Connect`** — picks up DM (already queued) immediately, then waits for app client. Wires `dmSock` and `clientSock` for `SocketHelper::sendData`/`recvData`.
3. **Tick loop reordered**: `send-to-DM → setDriverVehicles → recv-from-DM → getTrafficVehicles+Signals → send-to-app → recv-from-app → split egos vs CAV cmds for next tick`.

## Carryover (not done)

- The actual handshake / first-recv bytes the FIXS DriverModel sends — needs Wireshark or instrumented `printf` in the patched `DriverModel_FIXS_Common.h` to confirm exactly what DM puts on the wire after initConnection.
- The eco-speed-validator assertion check — depends on the run actually completing N ticks.
