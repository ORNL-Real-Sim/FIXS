"""Everything a tick renders is applied by ONE call the server acknowledged.

`apply_batch` hands the commands to the socket and returns, and the very next
thing the bridge does is `world.tick()` -- so it races its own message. When the
tick wins, the frame renders every mirrored vehicle at its PREVIOUS pose while
the camera, placed from the pose just commanded, has moved on; the next tick
applies both and the vehicle jumps twice as far. Measured on the C++ bridge at
CarlaTimeStep 0.1: 29% of frames lagging, the followed actor alternating
0.000 / 0.582 m instead of a steady 0.291. FIXS#266/#267.

This is the wiring test for that fix. Reverting `flushBatch` to `apply_batch`
was invisible to the whole suite before it existed: nothing constructed the
flush path, so the call that carries the guarantee was never observed.

    python -m pytest tests/Python/unit/test_batch_applied_before_tick.py
"""
from __future__ import annotations

import pytest

pytest.importorskip("carla", reason="needs the CARLA PythonAPI")
import carla                                                     # noqa: E402
from Carla.VirEnv.CarlaBackend import CarlaBackend               # noqa: E402


class _Client:
    """Answers both calls and records which one the backend chose.

    Deliberately NOT a fake that only implements the synchronous call: one that
    raises AttributeError on `apply_batch` would pass for the wrong reason, and
    would keep passing if the backend grew a third way to send a batch.
    """

    def __init__(self):
        self.calls = []             # ('sync'|'async', number of commands)

    def apply_batch_sync(self, cmds, _flag):
        self.calls.append(('sync', len(cmds)))
        return []

    def apply_batch(self, cmds, _flag=False):
        self.calls.append(('async', len(cmds)))


def _backend(client):
    return CarlaBackend(None, client, False, False)


def _tf(x):
    return carla.Transform(carla.Location(x, 0.0, 0.0), carla.Rotation())


def test_the_flush_is_the_acknowledged_call():
    client = _Client()
    be = _backend(client)
    be.queueTransform(11, _tf(1.0))
    be.queueTransform(22, _tf(2.0))
    be.flushBatch()
    assert client.calls == [('sync', 2)], (
        "flushBatch must use the call that has been applied server-side when it "
        "returns; world.tick() is the next statement in the bridge loop.")


def test_the_flush_empties_the_batch():
    """A command left behind is applied again a tick later, which puts a vehicle
    back where it was -- the same symptom the sync call exists to prevent."""
    client = _Client()
    be = _backend(client)
    be.queueTransform(11, _tf(1.0))
    be.flushBatch()
    be.flushBatch()
    # The second flush sends nothing at all: an empty batch is not an empty RPC.
    assert client.calls == [('sync', 1)]
    assert be._batch == []
