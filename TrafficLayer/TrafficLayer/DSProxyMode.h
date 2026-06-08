#pragma once

// Stage B + multi-port routing entry point for the VISSIM
// DrivingSimulatorProxy.dll coupling (issue #158). When
// `VissimSetup.EnableDSProxy` is true in the config, mainTrafficLayer
// dispatches to runDSProxyMode and exits when it returns.
//
// This branch adds: multi-port subscription routing. The legacy single-
// app-socket path becomes a list of AppSocketConfig entries built from
// ApplicationSetup.VehicleSubscription[].port[]; each gets its own
// publish filter (via publishesVehicle()).
//
// Per-tick tick flow follows the seven-phase canonical pattern documented
// in doc/fixs_tick_flow.md — PHASES 1/2/7 are DSProxy adapter calls,
// PHASES 3/4/5/6 use SocketHelper + MsgHelper. The per-port filter in
// PHASE 3 is *exactly* the routing-table pattern the XIL orchestrator
// refactor (#117) will formalize, so this branch's per-port loop body
// lifts into the orchestrator as a code-move.
//
// Scope boundary: this file is the orchestrator for the DSProxy mode only.
// If you're adding a different VISSIM integration mode (e.g., the legacy
// DriverModel-socket path that mainTrafficLayer's while loop drives, or
// a future SUMO-only mode), prefer a sibling `<Mode>Mode.{h,cpp}` rather
// than extending this one. mainTrafficLayer.cpp dispatches by flag, so
// each Mode file stays focused on one orchestration path. (Note:
// TrafficLayer never talks to VISSIM over COM — COM is only used by
// external bootstrap scripts to start VISSIM; the per-tick orchestration
// loop uses the DriverModel socket or DSProxy DLL.)

#include "ConfigHelper.h"

namespace FIXS {
namespace DSProxy {

// Runs the DSProxy tick loop against the VISSIM instance the DLL spawns,
// for the configured simulation time. Writes a per-tick summary line so the
// probe scenario can verify end-to-end behavior. Returns 0 on success, non-
// zero on failure (DLL load, VISSIM_Connect, etc.).
int runDSProxyMode(const ConfigHelper& config);

} // namespace DSProxy
} // namespace FIXS
