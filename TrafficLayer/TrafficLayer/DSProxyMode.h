#pragma once

// Stage A entry point for the VISSIM DrivingSimulatorProxy.dll coupling
// (issue #158). When `VissimDSProxySetup.Enable` is true in the config,
// mainTrafficLayer dispatches to runDSProxyMode and exits when it returns.
//
// Stage A scope: TrafficLayer drives VISSIM via DSProxy. No CarMaker, no
// DriverModel interaction, no consumer-side publishing yet — those land in
// Stages B–D. This entry point exists so Stage A can be shipped, tested,
// and merged independently.

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
