# FIXS canonical tick flow

This document captures the **canonical per-tick orchestration pattern** that
TrafficLayer's legacy main loop already implements. Every Mode file
(`DSProxyMode.cpp`, future `SumoMode.cpp`, etc.) and every PR touching the
per-tick pipeline should follow this seven-phase shape so that the
future XIL orchestrator refactor ([#117](https://github.com/ORNL-Real-Sim/FIXS/issues/117))
is a code-move rather than a rewrite.

The pattern is descriptive of code that already runs in production. It is
not new. Naming the phases is the point.

## Reference: `mainTrafficLayer.cpp`'s while loop

The legacy `while (!g_shutdown.shutdownRequested) { ... }` block in
`TrafficLayer/TrafficLayer/mainTrafficLayer.cpp` (approx. lines 920–1380)
implements this sequence. The phase numbers below correspond to that
ordering.

```
┌──────────────────────────────────────────────────────────────────────┐
│  Per-tick orchestration                                              │
├──────────────────────────────────────────────────────────────────────┤
│                                                                      │
│  PHASE 1 — Advance source                                            │
│    Traffic_c.runOneStepSimulation()                                  │
│    (or: VissimDSProxy::setDriverVehicles(egos) for DSProxy mode)     │
│                                                                      │
│  PHASE 2 — Collect state into MsgServer_c                            │
│    Traffic_c.recvFromTrafficSimulator(&simTime, MsgServer_c)         │
│    (or: VissimDSProxy::getTrafficVehicles + getSignalStates)         │
│                                                                      │
│  PHASE 3 — Distribute MsgServer_c → MsgClient_c per subscription     │
│    Per-port filter using ApplicationSetup.VehicleSubscription /      │
│    XilSetup.VehicleSubscription / SignalSubscription /               │
│    DetectorSubscription. Today this is inline filtering inside the   │
│    `for (iC : actualClientSock)` loop; tomorrow this becomes the     │
│    orchestrator's routing-table lookup.                              │
│                                                                      │
│  PHASE 4 — Publish to each connected client                          │
│    Sock_c.sendData(clientSock[iC], iC, simTime, simState, MsgClient_c)│
│    (one call per subscribed client)                                  │
│                                                                      │
│  PHASE 5 — Collect responses from each connected client              │
│    Sock_c.recvData(clientSock[iC], &simState, &simTime, MsgClient_c) │
│    (one call per subscribed client; blocks until client replies      │
│    — future enhancement: per-recv deadline + stale-state fallback,   │
│    see #117 Stage C)                                                 │
│                                                                      │
│  PHASE 6 — Merge client responses → commands for source              │
│    Traffic_c.parseSendMsg(MsgClient_c, MsgServer_c)                  │
│    (or: extract ego pose / behavior cmds from MsgClient_c into       │
│    the next tick's setDriverVehicles input)                          │
│                                                                      │
│  PHASE 7 — Push commands back to source                              │
│    Traffic_c.sendToTrafficSimulator(simTime, MsgServer_c)            │
│    (or: stored ego pose becomes PHASE 1 of the next tick for         │
│    DSProxy mode — DSProxy folds 7→1 into a single setDriverVehicles  │
│    call rather than a separate push)                                 │
│                                                                      │
└──────────────────────────────────────────────────────────────────────┘
```

## What's invariant across modes

PHASES 3, 4, 5 are **identical** across every Mode — they're the FIXS
socket protocol layer. Both the legacy main loop and `DSProxyMode.cpp`
call `SocketHelper::sendData` / `SocketHelper::recvData` with the same
`MsgHelper` data structures.

PHASES 1, 2, 6, 7 are **source-specific** — they touch the underlying
traffic simulator's API (SUMO `libsumo`, VISSIM COM, VISSIM DSProxy.dll).
This is where adapters live.

## What absorption looks like (#117)

When the XIL orchestrator lands:

1. PHASES 3–6 stay in `mainTrafficLayer.cpp`'s loop (now the
   orchestrator), unchanged or with the routing table extracted.
2. PHASES 1, 2, 7 become methods on an `ISimulator`-like adapter
   interface. Each Mode file becomes a thin adapter implementing those
   phases.
3. `DSProxyMode.cpp`'s current standalone tick loop disappears — its
   per-tick body becomes the body of "the orchestrator calls
   PHASE 1/2/7 hooks on the DSProxy adapter."

If a PR writes its tick loop following the seven-phase structure (and
uses `MsgHelper` / `SocketHelper` for PHASES 3–5 the same way the
legacy main loop does), absorption is a refactor pattern, not a
rewrite. That is the bar for "absorption-ready."

## Conventions for PRs touching the per-tick pipeline

If you're writing a Mode file or adding per-tick code:

1. **Label the phases** in code with `// PHASE N: <description>` comments
   at the start of each phase block.
2. **Use `SocketHelper::sendData` / `recvData`** as-is for PHASES 4–5,
   matching the legacy loop's call shape. Don't invent parallel socket
   handling.
3. **Use `MsgHelper`'s send/recv maps** (`VehDataSend_um`,
   `VehDataRecv_um`, `TlsDataSend_um`, etc.) for PHASES 3–6. Same
   structures the legacy loop uses.
4. **Put source-specific code** (PHASES 1, 2, 7) behind a clean adapter
   class (e.g., `VissimDSProxy`, `TrafficHelper`). Keep it free of
   `MsgHelper` / `SocketHelper` coupling.
5. **Cross-reference this doc** in your Mode file's top-of-file comment
   and in your PR body. Future absorption PRs grep for the reference.

## What this doc is NOT

- Not a design for the orchestrator. That's [#117](https://github.com/ORNL-Real-Sim/FIXS/issues/117).
- Not a contract. Phase numbering is descriptive, not prescriptive.
  Some modes may collapse phases (DSProxy folds 7 into the next tick's
  1; modes with no app socket skip 3–5).
- Not a deadline. PRs without phase labels still merge; this is a
  forward-looking convention to make absorption easier.

## Related

- [#117](https://github.com/ORNL-Real-Sim/FIXS/issues/117) — XIL co-simulation orchestrator (the absorption target)
- [#113](https://github.com/ORNL-Real-Sim/FIXS/issues/113) — `mainTrafficLayer.cpp` refactor target
- [#158](https://github.com/ORNL-Real-Sim/FIXS/issues/158) — VISSIM DSProxy adoption (Stages A–D use this tick flow)
- [doc/156_drivingsim_dll_design_proposal.md](156_drivingsim_dll_design_proposal.md) — DSProxy adoption design
