#pragma once
//============================================================================
//  XilGuard  --  FIXS XIL health guard  (PLACEHOLDER for issue #193)
//----------------------------------------------------------------------------
//  A generic hook any FIXS component (TrafficLayer, VirCarlaEnv, the
//  CarMaker/dSPACE bridges) can call each tick to report an XIL-health
//  condition (a measured value against a tolerance). The guard decides what to
//  do about a violation according to a policy.
//
//  STATUS: placeholder. Only `Warn` (rate-limited log) is implemented today so
//  that #174 can flag SUMO<->CARLA z-axis misalignment without blocking. The
//  real per-condition / per-test-type policy AND the graceful co-sim shutdown
//  (reap CARLA + SUMO/libsumo + TrafficLayer + dSPACE/CarMaker cleanly, with no
//  zombie processes / leaked FlexNet|CodeMeter tokens / stale locks) are the
//  deliverable of issue #193 -- see the TODO #193 markers below and in the .cpp.
//============================================================================

#include <string>

namespace fixs {

// What to do when a guarded condition is violated. PLACEHOLDER: today every mode
// behaves as Warn. #193 implements FlagInvalid (mark the run's outputs invalid)
// and Abort (graceful co-sim teardown), and selects the mode per test type
// (dyno/HIL -> Abort by default; visualization -> Warn).
enum class XilGuardMode {
    LogOnly,      // record only, no user-visible message
    Warn,         // log a warning, keep running   (the ONLY behavior wired today)
    FlagInvalid,  // TODO #193: mark run outputs invalid, keep running
    Abort         // TODO #193: trigger graceful co-sim shutdown
};

// Report one XIL-health sample. If |measured| exceeds |tolerance| the guard acts
// per `mode`. `condition` is a short stable name (e.g. "sumo_carla_z_mismatch",
// "cosim_loop_delay"). Warnings are rate-limited per condition so a per-tick
// violation does not flood the log.
//
// TODO #193: honor FlagInvalid / Abort; load thresholds + per-condition policy
// from config; route Abort through the graceful-shutdown procedure.
void RS_XIL_GUARD(const std::string& condition,
                  double measured,
                  double tolerance,
                  XilGuardMode mode = XilGuardMode::Warn);

} // namespace fixs
