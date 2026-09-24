"""What a mirrored vehicle LOOKS LIKE must depend only on which vehicle it is.

The blueprint is not cosmetic. It fixes `bounding_box.extent.x`, and the bridge
anchors a vehicle by stepping back that half-length from the nose the wire gives
it -- so two models put the same car in two different places, permanently, in
front of an agent that reads it as a leader.

The old draw took the next value from one shared seeded generator, so a vehicle's
model depended on how many vehicles had drawn BEFORE it. A spawn CARLA refuses
still consumes a draw and then draws again on the retry, so anything that changes
the refusal pattern re-deals every later vehicle. Measured on the MLK warm-up
burst: nine refusals shifted the sequence and 46 vehicles changed model, moving
up to 1.58 m, which walked the ego 150 m off its baseline.

    python -m pytest tests/Python/unit/test_blueprint_is_per_vehicle.py
"""
from __future__ import annotations

import pytest

pytest.importorskip("carla", reason="needs the CARLA PythonAPI")
from Carla.VirEnv.BridgeHelper import BridgeHelper              # noqa: E402


def bp(vehId, vClass='passenger'):
    return BridgeHelper.map_Sumo_vClass_to_Carla_blueprintId(vClass, vehId)


IDS = ['1.47', '11.32', '18.30', '23.52', '23.53', '24.34', '24.58', '4.7', '7.27']


def test_the_same_vehicle_always_gets_the_same_model():
    BridgeHelper.setBlueprintSeed(20260913)
    first = {v: bp(v) for v in IDS}
    for _ in range(5):
        assert {v: bp(v) for v in IDS} == first


def test_order_cannot_change_it():
    """Drawing in reverse, or with strangers interleaved, changes nothing."""
    BridgeHelper.setBlueprintSeed(20260913)
    want = {v: bp(v) for v in IDS}

    got = {}
    for v in reversed(IDS):
        bp('noise.%s' % v)                  # other vehicles drawing in between
        got[v] = bp(v)
    assert got == want


def test_a_refused_spawn_does_not_re_deal_the_others():
    """The regression this exists for.

    A refused vehicle is retried next exchange and asks for a blueprint again.
    Under the old sequential draw that second ask consumed another value and
    shifted every later vehicle. Keyed on the id, a retry returns the same model
    and nobody else is touched.
    """
    BridgeHelper.setBlueprintSeed(20260913)
    clean = {v: bp(v) for v in IDS}

    # now replay with every third vehicle "refused" and retried twice
    BridgeHelper.setBlueprintSeed(20260913)
    retried = {}
    for i, v in enumerate(IDS):
        model = bp(v)
        if i % 3 == 0:
            assert bp(v) == model           # the retry is the same vehicle
            assert bp(v) == model
        retried[v] = model
    assert retried == clean


def test_the_seed_still_decides():
    BridgeHelper.setBlueprintSeed(20260913)
    a = {v: bp(v) for v in IDS}
    BridgeHelper.setBlueprintSeed(1)
    b = {v: bp(v) for v in IDS}
    BridgeHelper.setBlueprintSeed(20260913)
    assert {v: bp(v) for v in IDS} == a
    assert a != b, 'a different seed must give a different assignment'


def test_it_is_stable_across_processes():
    """hash() is randomised per process unless PYTHONHASHSEED is set, so the
    draw must not use it. These are the values this seed produces; if they move,
    every existing baseline moves with them."""
    BridgeHelper.setBlueprintSeed(20260913)
    assert bp('23.52') == 'vehicle.audi.etron'
    assert bp('1.47') == 'vehicle.citroen.c3'
    assert bp('11.32') == 'vehicle.nissan.micra'


def test_no_id_still_works():
    """An older caller that passes no id keeps the sequential draw."""
    BridgeHelper.setBlueprintSeed(20260913)
    got = [BridgeHelper.map_Sumo_vClass_to_Carla_blueprintId('passenger')
           for _ in range(5)]
    assert all(g.startswith('vehicle.') for g in got)


def test_every_model_in_the_pool_gets_used():
    """A hash that clumps would quietly shrink the fleet to a few models."""
    BridgeHelper.setBlueprintSeed(20260913)
    pool = BridgeHelper._BY_VCLASS['passenger']
    seen = {bp('%d.%d' % (i // 8, i % 8)) for i in range(2300)}
    assert seen == set(pool)
