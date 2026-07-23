#pragma once
//============================================================================
//  EgoSpeedAdvisor  (#174 -- artificial external speed controller, L2 stand-in)
//----------------------------------------------------------------------------
//  Emits a DESIRED ego speed (m/s) as a function of sim time from a piecewise-
//  linear, looping profile of (time_s, speed_mps) knots. It is the L2 counterpart
//  of EgoDriver: EgoDriver ACTUATES (steer/pedal); this produces the ADVISORY a
//  real external CAV controller would send over FIXS. Deliberately SDK-free and
//  simulator-agnostic so it can stand in for -- and later be replaced by -- a
//  separate advisory client (the #173 external-controller path) without touching
//  the backend actuation seam (CarlaBackend::applyEgoControl).
//============================================================================

#include <vector>
#include <utility>
#include <cstddef>

namespace virenv {

class EgoSpeedAdvisor {
public:
    // knots: (time_s, speed_mps). Sorted internally by time. The profile LOOPS
    // with period == the last knot's time, so for a smooth wrap the last knot's
    // speed should equal the first knot's speed. 0 knots -> desiredSpeed() returns
    // the caller's fallback (degenerates to a constant-target L0).
    void setProfile(const std::vector<std::pair<double, double>>& knots);
    bool hasProfile() const { return !knots_.empty(); }

    // Desired speed at simTime (m/s), looping over the profile period. Returns
    // fallback when no profile is set.
    double desiredSpeed(double simTime, double fallback) const;

    double period() const { return period_; }
    std::size_t knotCount() const { return knots_.size(); }

private:
    std::vector<std::pair<double, double>> knots_;   // (time_s, speed_mps), time-sorted
    double period_ = 0.0;                            // loop period = last knot time
};

} // namespace virenv
