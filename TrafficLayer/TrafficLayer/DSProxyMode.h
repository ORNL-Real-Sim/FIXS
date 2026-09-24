#pragma once

// Stage B+ entry point for the VISSIM DrivingSimulatorProxy.dll coupling
// (issue #158). When `VissimSetup.EnableDSProxy` is true in the config,
// mainTrafficLayer dispatches to runDSProxyMode and exits when it returns.
//
// Stage B+ scope: TrafficLayer drives VISSIM via DSProxy, publishes to
// the app clients (CAV controller, CarMaker, observers), AND relays CAV
// behavior commands down to the FIXS DriverModel via a second socket.
// CAV controller sends per-vehicle speedDesired/accelerationDesired; TL
// routes ego.Pose to DSProxy and non-ego.Intent to DriverModel.
//
// App-socket routing is multi-port: every (subscription, port) tuple in
// ApplicationSetup.VehicleSubscription becomes its own AppSocketConfig
// with its own server socket and its own outbound filter
// (publishesVehicle); the legacy single-client path is the N=1 case.
// That per-port filter in PHASE 3 is exactly the routing-table pattern
// the XIL orchestrator refactor (#117) will formalize, so the per-port
// loop body lifts into the orchestrator as a code-move.
//
// Per-tick tick flow follows the seven-phase canonical pattern documented
// in doc/fixs_tick_flow.md. The DriverModel is treated as a second FIXS-
// protocol endpoint — its PHASE 4 (TL→DM publish) and PHASE 5 (TL←DM
// drain) use SocketHelper / MsgHelper identically to the app client.
// From the future XIL orchestrator's pub/sub matrix view (#117), DM is
// just another node with subscribes=[VehicleIntent] and publishes=[]
// (we discard its state upload since DSProxy is canonical).
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
