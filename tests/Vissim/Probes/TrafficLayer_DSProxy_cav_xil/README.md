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

**What hangs (updated 2026-06-06 after deeper trace)**:

Two issues found by reading `DriverModel_FIXS_Common.h::DRIVER_DATA_TIME` (line 525-631)
and the per-vehicle MOVE_DRIVER path (line 1605-1742):

1. **Tick 0 — fixed**: on the first VISSIM tick the FIXS DriverModel takes the
   `isVeryFirstStep` branch (line 545) which does neither send nor recv. The
   TrafficLayer-side loop was previously trying to `recvData` on tick 0 and
   blocked forever. This commit guards DM I/O with `tick >= 1`.

2. **Tick 1+ — open**: the FIXS DriverModel has `SUB_EGO_ONLY` hardcoded to
   `true` (line 54). That flag:
   - **Gates the once-per-tick send/recv at line 613 on `!SUB_EGO_ONLY`** — so the
     per-tick path is *always skipped* in current FIXS builds.
   - **Routes send/recv through the per-vehicle MOVE_DRIVER path at line 1735** —
     which only fires when `VehDataSend_v.size() > 0`, i.e., at least one
     subscribed vehicle is present in the callback.

   The probe's `coexist_par.yaml` has `EnableApplicationLayer: false`, so the
   DriverModel's subscription list is empty → no vehicle is ever subscribed →
   `VehDataSend_v` stays empty → DriverModel does **zero socket I/O for the
   entire simulation**. TrafficLayer's first `recvData(dmSock)` on tick 1 then
   blocks waiting for bytes that will never arrive.

**Two clean fixes**, each ~½ day of additional work:

A. **Easier**: Give the par-file a non-empty `ApplicationSetup.VehicleSubscription`
   targeting a vehicle type (e.g., type 100 Car). DM then exercises the
   per-vehicle send/recv path. TrafficLayer DSProxyMode needs to do `recv/send`
   pairs **per Car per tick** instead of one pair per tick — a meaningful
   restructure of the inner loop.

B. **Cleaner**: Add a `SubEgoOnly: false` config knob to FIXS DriverModel
   (small ProprietaryFiles patch, ~5 lines). When false, DM does the once-per-
   tick send/recv at line 614 — matching TrafficLayer DSProxyMode's existing
   single-pair-per-tick loop without any TL-side restructure. This is the
   right long-term answer because the per-vehicle pattern is for the old non-
   DSProxy CarMaker-side ego subscription, which DSProxy obsoletes.

I'd land **B** as a follow-up dual PR (FIXS + ProprietaryFiles) after the
main #158 stack merges. The architecture in this PR is correct for path B;
only the FIXS DM patch is missing.

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
