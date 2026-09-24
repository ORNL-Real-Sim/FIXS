"""The spare pool hands out actors it made earlier, and changes nothing else.

With `WarmUpUntilEgoEntry` the CARLA boundary is closed for the whole warm-up, so
the entire network arrives in ONE exchange. `try_spawn_actor` measured 1.6-3.8 ms
each, so ~200 arrivals freeze the co-simulation for about half a second at the
instant the ego enters -- the worst moment for a bench in the loop.
`CarlaSetup.SpareVehiclePool` pays for those actors at start-up instead, where the
bridge is about to block in recv anyway.

What must NOT change is which blueprint each vehicle gets. The draw is seeded
(FIXS#355) and the blueprint fixes `bounding_box.extent.x`, which is the pose
anchor -- so a spare is used only when its blueprint id matches what the vehicle
drew, and the pool is stocked round-robin rather than by drawing from the same
generator. `test_pool_does_not_disturb_the_blueprint_draw` is the test that
guards it.

    python -m pytest tests/Python/unit/test_spare_vehicle_pool.py
"""
from __future__ import annotations

import pytest

from CommonLib.VirEnv.IVirEnvBackend import kNoHandle, Pose

pytest.importorskip("carla", reason="needs the CARLA PythonAPI")
from Carla.VirEnv.BridgeHelper import BridgeHelper              # noqa: E402
from Carla.VirEnv.CarlaBackend import CarlaBackend              # noqa: E402


# --------------------------------------------------------------------------
# the smallest CARLA that answers what the backend asks
# --------------------------------------------------------------------------
class _Blueprint:
    def __init__(self, bpId):
        self.id = bpId


class _Actor:
    _next = 1000

    def __init__(self, bp):
        import carla
        _Actor._next += 1
        self.id = _Actor._next
        self.blueprint = bp.id
        self.destroyed = False
        self.physics = True
        self.bounding_box = carla.BoundingBox(
            carla.Location(0, 0, 0), carla.Vector3D(2.4, 1.0, 0.75))

    def set_simulate_physics(self, on):
        self.physics = on

    def destroy(self):
        self.destroyed = True


class _BpLib:
    def find(self, bpId):
        return _Blueprint(bpId)


class _Map:
    def get_spawn_points(self):
        import carla
        return [carla.Transform(carla.Location(10.0, 20.0, 205.0))]


class _World:
    """Counts what the backend asks of CARLA; that is what the tests assert on."""

    def __init__(self, spawnLimit=None):
        self.spawned = []           # blueprint ids, in the order asked for
        self.mapCalls = 0
        self.spawnLimit = spawnLimit

    def get_blueprint_library(self):
        return _BpLib()

    def get_map(self):
        self.mapCalls += 1
        return _Map()

    def try_spawn_actor(self, bp, _tf):
        if self.spawnLimit is not None and len(self.spawned) >= self.spawnLimit:
            return None
        self.spawned.append(bp.id)
        return _Actor(bp)


class _Client:
    def __init__(self):
        self.destroyedIds = []

    def apply_batch_sync(self, cmds, _flag):
        self.destroyedIds.extend(getattr(c, 'actor_id', None) for c in cmds)
        return []


def _backend(world, client, pool):
    return CarlaBackend(world, client, False, False, sparePoolSize=pool)


def _pose():
    return Pose(x=1.0, y=2.0, z=205.0, headingDeg=90.0, gradeRad=0.0)


# --------------------------------------------------------------------------
def test_pool_off_spawns_exactly_as_before():
    """The default is 0, and 0 must be indistinguishable from today."""
    world, client = _World(), _Client()
    be = _backend(world, client, 0)
    be.initTrafficPool()
    assert world.spawned == []                  # nothing parked up front

    for _ in range(5):
        assert be.spawnVehicle('car', 'passenger', _pose()) != kNoHandle
    assert len(world.spawned) == 5              # one try_spawn_actor per vehicle


def test_pool_hands_out_instead_of_spawning():
    world, client = _World(), _Client()
    be = _backend(world, client, 46)            # 2 of each of the 23 car blueprints
    be.initTrafficPool()
    parked = len(world.spawned)
    assert parked == 46

    for _ in range(20):
        assert be.spawnVehicle('car', 'passenger', _pose()) != kNoHandle
    # Every vehicle is accounted for, and CARLA was asked only for the ones whose
    # blueprint the pool did not happen to hold -- the draw is random, so which
    # ones those are is not fixed, but the arithmetic is.
    assert be._spareHits + be._spareMisses == 20
    assert be._spareHits > 0
    assert len(world.spawned) == parked + be._spareMisses


def test_pool_does_not_disturb_the_blueprint_draw():
    """Same seed, same blueprint per vehicle, pool or no pool.

    The pool is stocked round-robin precisely so it does not consume the seeded
    generator. Stocking it with `map_Sumo_vClass_to_Carla_blueprintId` instead
    fails this.
    """
    def draw(pool):
        BridgeHelper.setBlueprintSeed(4242)
        world, client = _World(), _Client()
        be = _backend(world, client, pool)
        be.initTrafficPool()
        got = []
        for _ in range(30):
            h = be.spawnVehicle('car', 'passenger', _pose())
            got.append(be.actorOf(h).blueprint)
        return got

    assert draw(0) == draw(46)


def test_an_unstocked_blueprint_still_spawns():
    """A vClass the pool does not carry falls through, and says so."""
    world, client = _World(), _Client()
    be = _backend(world, client, 46)
    be.initTrafficPool()
    parked = len(world.spawned)

    assert be.spawnVehicle('bus', 'bus', _pose()) != kNoHandle
    assert len(world.spawned) == parked + 1     # spawned, not handed a car
    assert be._spareMisses == 1


def test_trim_destroys_the_leftovers_once():
    """A parked actor costs every tick, so the unused ones go after the burst."""
    world, client = _World(), _Client()
    be = _backend(world, client, 46)
    be.initTrafficPool()
    for _ in range(6):
        be.spawnVehicle('car', 'passenger', _pose())

    assert be.trimSpares() == 40                # 46 parked, 6 handed out
    assert len(client.destroyedIds) == 40
    assert be.trimSpares() == 0                 # idempotent
    assert len(client.destroyedIds) == 40


def test_trim_keeps_the_vehicles_that_were_handed_out():
    world, client = _World(), _Client()
    be = _backend(world, client, 46)
    be.initTrafficPool()
    handles = [be.spawnVehicle('car', 'passenger', _pose()) for _ in range(6)]
    be.trimSpares()

    live = {be.actorOf(h).id for h in handles}
    assert live.isdisjoint(set(client.destroyedIds))
    assert all(not be.actorOf(h).destroyed for h in handles)


def test_spare_taken_is_the_cue_to_trim():
    world, client = _World(), _Client()
    be = _backend(world, client, 46)
    be.initTrafficPool()
    assert be.spareTaken() is False
    be.spawnVehicle('car', 'passenger', _pose())
    assert be.spareTaken() is True


def test_init_fetches_the_map_before_any_exchange():
    """The z audit's world.get_map() is ~0.6 s; it must not land in the burst."""
    world, client = _World(), _Client()
    be = _backend(world, client, 0)
    be.initTrafficPool()
    assert world.mapCalls == 1
    be.auditZAlignment()                        # no second parse
    assert world.mapCalls == 1


def test_a_short_pool_is_a_warning_not_a_failure():
    """CARLA refusing some parks leaves a smaller pool, not a broken run."""
    world, client = _World(spawnLimit=10), _Client()
    be = _backend(world, client, 46)
    be.initTrafficPool()
    assert be._sparesLeft == 10

    # This world refuses every further spawn, so a vehicle whose blueprint the
    # short pool does not hold gets the same kNoHandle it gets today when a spawn
    # point is blocked. What must not happen is a crash, or a spare going unused.
    ok = sum(be.spawnVehicle('car', 'passenger', _pose()) != kNoHandle
             for _ in range(30))
    assert ok == be._spareHits
    assert be._sparesLeft == 10 - be._spareHits


def test_report_names_the_number_to_set():
    world, client = _World(), _Client()
    be = _backend(world, client, 4)
    be.initTrafficPool()
    for _ in range(9):
        be.spawnVehicle('car', 'passenger', _pose())
    msg = be.spareReport()
    assert 'peak 9' in msg
    assert 'SpareVehiclePool: 9' in msg          # what would have covered it
