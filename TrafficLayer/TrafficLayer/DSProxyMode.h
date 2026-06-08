#pragma once

// Stage B entry point for the VISSIM DrivingSimulatorProxy.dll coupling
// (issue #158). When `VissimSetup.EnableDSProxy` is true in the config,
// mainTrafficLayer dispatches to runDSProxyMode and exits when it returns.
//
// Stage B scope: TrafficLayer drives VISSIM via DSProxy AND publishes the
// per-tick traffic state to one app socket, receives ego pose back. No
// CarMaker, no DriverModel interaction yet — those land in Stages B+/C/D.
//
// Per-tick tick flow follows the seven-phase canonical pattern documented
// in doc/fixs_tick_flow.md — PHASES 1/2/7 are DSProxy adapter calls,
// PHASES 3/4/5/6 use SocketHelper + MsgHelper the same way the legacy
// mainTrafficLayer while loop does. The intent is that the future XIL
// orchestrator refactor (#117) absorbs this loop as a code-move: PHASES
// 3-6 lift into the orchestrator, PHASES 1/2/7 stay as adapter hooks.
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
