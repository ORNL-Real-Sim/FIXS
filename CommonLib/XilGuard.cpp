//============================================================================
//  XilGuard  --  FIXS XIL health guard  (PLACEHOLDER for issue #193)
//  Only rate-limited `Warn` is implemented; FlagInvalid / Abort + the graceful
//  co-sim shutdown are #193. See XilGuard.h.
//============================================================================
#include "XilGuard.h"

#include <cmath>
#include <cstdio>
#include <unordered_map>

namespace fixs {

namespace {
// per-condition warning counter, to rate-limit floods (a violation can recur
// every tick). Not thread-safe by design -- guard calls come from the co-sim
// loop thread. TODO #193: replace with a proper policy/telemetry sink.
std::unordered_map<std::string, long>& warnCounts() {
    static std::unordered_map<std::string, long> m;
    return m;
}
constexpr long kWarnEvery = 100;   // log 1st, then every 100th recurrence
}

void RS_XIL_GUARD(const std::string& condition, double measured, double tolerance,
                  XilGuardMode mode) {
    if (std::fabs(measured) <= std::fabs(tolerance)) return;   // within tolerance: ok

    // TODO #193: FlagInvalid should mark the run's outputs invalid; Abort should
    // invoke the graceful co-sim teardown. For #174 all modes currently warn.
    if (mode == XilGuardMode::LogOnly) return;

    long& n = warnCounts()[condition];
    if (n == 0 || (n % kWarnEvery) == 0) {
        const char* tag = (mode == XilGuardMode::Abort)       ? "[XIL-GUARD abort(TODO#193)]"
                        : (mode == XilGuardMode::FlagInvalid) ? "[XIL-GUARD invalid(TODO#193)]"
                                                              : "[XIL-GUARD warn]";
        std::fprintf(stderr, "%s %s = %.3f exceeds tolerance %.3f (n=%ld)\n",
                     tag, condition.c_str(), measured, tolerance, n + 1);
    }
    ++n;
}

} // namespace fixs
