//============================================================================
//  EgoSpeedAdvisor -- piecewise-linear looping desired-speed profile (SDK-free).
//  The artificial L2 controller: turns sim time into a desired ego speed a real
//  external CAV controller would otherwise stream over FIXS.
//============================================================================
#include "EgoSpeedAdvisor.h"

#include <algorithm>
#include <cmath>

namespace virenv {

void EgoSpeedAdvisor::setProfile(const std::vector<std::pair<double, double>>& knots) {
    knots_ = knots;
    std::sort(knots_.begin(), knots_.end(),
              [](const std::pair<double, double>& a, const std::pair<double, double>& b) {
                  return a.first < b.first;
              });
    period_ = knots_.empty() ? 0.0 : knots_.back().first;
}

double EgoSpeedAdvisor::desiredSpeed(double simTime, double fallback) const {
    if (knots_.empty()) return fallback;
    if (knots_.size() == 1) return knots_.front().second;

    // wrap sim time into one profile period so the schedule repeats endlessly
    double t = simTime;
    if (period_ > 1e-9) {
        t = std::fmod(simTime, period_);
        if (t < 0.0) t += period_;
    }

    // locate the segment [i, i+1] bracketing t and linearly interpolate
    for (std::size_t i = 0; i + 1 < knots_.size(); ++i) {
        const double t0 = knots_[i].first;
        const double t1 = knots_[i + 1].first;
        if (t >= t0 && t <= t1) {
            const double f = (t1 > t0) ? (t - t0) / (t1 - t0) : 0.0;
            return knots_[i].second + (knots_[i + 1].second - knots_[i].second) * f;
        }
    }
    return knots_.back().second;   // t past the last knot (only if period == 0)
}

} // namespace virenv
