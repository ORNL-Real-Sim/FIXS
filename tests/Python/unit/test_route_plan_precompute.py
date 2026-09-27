"""The ego's route plan can be built before there is an ego.

`bind` needs the agent and the agent needs the actor, which the traffic simulator
does not insert until its depart time -- so the route snap happens on the first
controlled tick, 2561 waypoints measured at 0.16-0.20 s, inside the same tick
where the whole warm-up network arrives. The PLAN needs neither: it is
EgoRoutePoints snapped to the map.

`precompute_route()` lets a controller build it in its constructor, at bridge
start-up. `bind` then finds it already there and does not snap again -- which is
the behaviour these tests pin, because a cache that is silently rebuilt buys
nothing and a cache that is never refilled would hand the agent an empty plan.

Safe without an ego, and measured rather than assumed: seeded at the ego's z
(205.29) against the no-ego fallback (210.21 on MLK), the two plans agreed on
road and lane at 2560 of 2560 points. The seed feeds only the FIRST query; the
elevation is then carried forward off each returned waypoint.

    python -m pytest tests/Python/unit/test_route_plan_precompute.py
"""
from __future__ import annotations

import pytest

from CommonLib.fixs import carla as relay


@pytest.fixture(autouse=True)
def _clear_cache():
    """Each test starts with no plan, and leaves none behind."""
    relay._planCache = None
    yield
    relay._planCache = None


def test_precompute_builds_once_and_reports_its_size(monkeypatch):
    built = []

    def fake_plan():
        built.append(1)
        return ['wp'] * 7

    monkeypatch.setattr(relay, '_routePlan', fake_plan)
    assert relay.precompute_route() == 7
    assert relay.precompute_route() == 7
    assert len(built) == 1, 'the second call must not rebuild the plan'


def test_a_built_plan_short_circuits_the_snap(monkeypatch):
    """_routePlan returns the cache WITHOUT asking the map anything.

    Guarded by making the first thing it reads raise: if the snap still runs,
    this test errors rather than quietly passing.
    """
    def explode(*_a, **_k):
        raise AssertionError('_routePlan read the config; it should have used the cache')

    relay._planCache = ['cached']
    monkeypatch.setattr(relay, '_configured', explode)
    assert relay._routePlan() == ['cached']


def test_an_unbuilt_plan_is_still_built(monkeypatch):
    """The cache must not turn 'not yet built' into 'empty'."""
    monkeypatch.setattr(relay, '_configured',
                        lambda key, default=None: [] if key == 'EgoRoutePoints' else default)
    assert relay._routePlan() == []
    assert relay._planCache is None, 'a route-less scenario must not cache an empty plan'


def test_precompute_is_a_no_op_without_a_route(monkeypatch):
    monkeypatch.setattr(relay, '_configured',
                        lambda key, default=None: [] if key == 'EgoRoutePoints' else default)
    assert relay.precompute_route() == 0
    # and it did not record "no route" as "plan built and empty": the two must
    # agree, or an agent in a scenario that gains a route gets an empty plan.
    assert relay._planCache is None


def test_reset_drops_the_plan():
    """_reset clears everything cached per run; a stale plan outliving a backend
    swap would hand the next run the previous scenario's corridor."""
    relay._planCache = ['stale']
    relay._reset()
    assert relay._planCache is None


def test_it_is_exported():
    """A controller calls it as carla.precompute_route(); keep it public."""
    assert 'precompute_route' in relay.__all__
    assert callable(relay.precompute_route)
