#pragma once

// Stage A entry point for the VISSIM DrivingSimulatorProxy.dll coupling
// (issue #158). When `VissimSetup.EnableDSProxy` is true in the config,
// mainTrafficLayer dispatches to runDSProxyMode and exits when it returns.
//
// Stage A scope: TrafficLayer drives VISSIM via DSProxy. No CarMaker, no
// DriverModel interaction, no consumer-side publishing yet — those land in
// Stages B–D. This entry point exists so Stage A can be shipped, tested,
// and merged independently.
//
// Scope boundary: this file is the orchestrator for the DSProxy mode only.
// If you're adding a different VISSIM integration mode (e.g., DriverModel-
// only, COM-only), prefer a sibling `<Mode>Mode.{h,cpp}` rather than
// extending this one. mainTrafficLayer.cpp dispatches by flag, so each
// Mode file stays focused on one orchestration path.
//
// Forward-looking note (issue #117): TrafficLayer's per-tick orchestration
// will eventually be unified — main loop's pub/sub matrix becomes the
// router; per-Mode files (this one included) become adapter entry points
// that hand off to the unified loop rather than running their own ticks.
// Stage A keeps its own tick loop because the unified orchestrator does
// not exist yet; absorption later is straightforward because this file
// already uses CommonLib's MsgHelper / SocketHelper (when Stages B+ wire
// in publishing/recv) the same way mainTrafficLayer does. See #117 for
// the explicit XIL orchestration design.

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
