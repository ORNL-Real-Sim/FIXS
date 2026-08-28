"""FIXS wire constants shared by every component on the co-simulation link.

Python peer of ``CommonLib/FixsProtocol.h`` -- same names, same values, same
semantics. See that header for the full rationale; the short version is that the
exchange period is a PROPERTY OF THE PROTOCOL, not a tuning knob: every
VirEnvCore host tests its FIXS send/recv boundary against this grid on its own
sim clock, and TrafficLayer steps the traffic simulator exactly once per
exchange. The HOST's own step (CarMaker's solver dt, CarlaSetup.CarlaTimeStep)
is a separate, finer knob and the core interpolates the feed across it.
"""

__all__ = ['kFeedPeriodS', 'kFeedHz', 'onFeedBoundary']

#: Seconds of host sim time between two FIXS vehicle-data exchanges.
kFeedPeriodS = 0.1

#: The same contract as a rate (1 / kFeedPeriodS). Boundary tests multiply the
#: host clock by this and check the result is a whole number, so it is the form
#: the grid arithmetic actually wants.
kFeedHz = 10.0


def onFeedBoundary(simTime, tol):
    """(float, float) -> bool -- is ``simTime`` on an exchange boundary?

    ``tol`` is applied to the SCALED clock (``simTime * kFeedHz``), which is how
    every host has always done it: the host clock accumulates from repeated
    ``+= step`` and lands a few ulp off a whole multiple rather than exactly on
    it. Mirrors ``fixs::onFeedBoundary`` in FixsProtocol.h, including its
    truncation-based round (sim time never runs backwards, so slots >= 0).
    """
    slots = simTime * kFeedHz
    nearest = float(int(slots + 0.5))
    d = slots - nearest
    return (-d if d < 0.0 else d) < tol
