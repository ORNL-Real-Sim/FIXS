#pragma once
//============================================================================
//  FIXS wire constants shared by every component on the co-simulation link.
//----------------------------------------------------------------------------
//  The exchange period is a PROPERTY OF THE PROTOCOL, not a tuning knob. Every
//  VirEnvCore host (CarMaker, VirCarlaEnv) tests its FIXS send/recv boundary
//  against this grid on its own sim clock, and stages the interpolation target
//  at the next boundary; TrafficLayer steps the traffic simulator exactly once
//  per exchange. So the traffic simulator's step length must equal it, or the
//  two clocks run at different rates -- e.g. a 0.05 s SUMO step against this
//  0.1 s exchange makes the host advance two ticks per SUMO step, rendering
//  every sample twice and playing the scene back at half speed.
//
//  The HOST's own step is a separate, finer knob (CarMaker's solver dt,
//  CarlaSetup.CarlaTimeStep) and the core interpolates the feed across it.
//
//  Kept here, in one place, because it used to be spelled as a bare `* 10` in
//  VirEnvCore, again in mainVirCarla, and again as a literal "0.1" in
//  TrafficHelper's SUMO auto-launch -- four copies of one contract.
//  Header-only and dependency-free so the dSPACE real-time image can include it.
//============================================================================

namespace fixs {

// Seconds of host sim time between two FIXS vehicle-data exchanges.
static constexpr double kFeedPeriodS = 0.1;

// The same contract as a rate (1 / kFeedPeriodS). Boundary tests multiply the
// host clock by this and check the result is a whole number, so it is the form
// the grid arithmetic actually wants.
static constexpr double kFeedHz = 10.0;

// Index of the exchange interval `simTime` is in. A host whose clock is a SUM of
// steps (CarMaker's SimCore.Time) must gate on this changing, not on
// onFeedBoundary: the sum drifts, and once the drift outgrows a fixed tolerance
// the boundary test never passes again (#169 -- about 7160 s at a 1 ms step).
// An edge on the slot is only ever late by the drift, never silent. The 1e-3
// slot of slack puts the edge on the boundary tick itself, not one tick after.
inline long long feedSlot(double simTime) {
    return (long long)(simTime * kFeedHz + 1e-3);   // floor: simTime >= 0
}

// True when `simTime` sits on an exchange boundary. `tol` is applied to the
// scaled clock (simTime * kFeedHz). Only safe on a clock computed as
// stepCount * step (the CARLA hosts), whose error does not grow with run time;
// a summed clock uses feedSlot.
inline bool onFeedBoundary(double simTime, double tol) {
    const double slots = simTime * kFeedHz;
    // llround without <cmath>: slots >= 0 here (sim time never runs backwards).
    const double nearest = (double)(long long)(slots + 0.5);
    const double d = slots - nearest;
    return (d < 0.0 ? -d : d) < tol;
}

}  // namespace fixs
