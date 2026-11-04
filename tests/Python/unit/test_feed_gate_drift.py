"""#169: the FIXS exchange gate must keep firing on a host clock that drifts.

CarMaker hands VirEnvCore its SimCore.Time, a running sum of 1 ms steps. The
sum drifts off the 0.1 s grid, and the old gate -- |t*10 - round(t*10)| < 1e-5
-- stopped passing for good at t = 7160.4 s, silently freezing recv and send.
The same construct at rate 1000 is what froze CarMaker traffic at ~715 s (#168).
"""
from CommonLib.VirEnv import VirEnvCore, onFeedBoundary

DT = 1e-3            # CarMaker solver step
T_END = 8000.0       # past the old gate's 7160.4 s failure
N_BOUNDARIES = int(round(T_END * 10))


def summed_clock():
    """Yield (tick, t) with t accumulated as t += DT, like SimCore.Time."""
    t = 0.0
    for k in range(int(round(T_END / DT)) + 1):
        yield k, t
        t += DT


def core_updates():
    """Ticks on which VirEnvCore.runStep decides to exchange."""
    core = VirEnvCore()
    core.ENABLE_REALSIM = False                    # no transport: gate only
    fired = []
    core.processStep = lambda simTime, onUpdate, *a: (
        fired.append(simTime) if onUpdate else None, (0, None))[1]
    ticks = []
    for k, t in summed_clock():
        before = len(fired)
        core.runStep(t)
        if len(fired) > before:
            ticks.append(k)
    return ticks


def test_core_exchanges_once_per_boundary_on_a_drifting_clock():
    ticks = core_updates()
    assert len(ticks) == N_BOUNDARIES              # none missed, none doubled
    assert ticks[:3] == [100, 200, 300]            # on the boundary, not at t=0 / 0.05 s
    # every exchange on the boundary tick itself -- at 8000 s the drift is
    # still far below one tick, so the edge is not even late yet
    assert all(k % 100 == 0 for k in ticks)


def test_old_tolerance_gate_freezes_on_the_same_clock():
    """Negative control: this is the failure the core no longer has."""
    missed_after = [t for k, t in summed_clock()
                    if k % 100 == 0 and k and not onFeedBoundary(t, 1e-5)]
    assert missed_after, 'the drifting clock no longer reproduces #169'
    assert 7100.0 < missed_after[0] < 7200.0
