"""The ego's reported pose never steps backward.

A physics ego leaving rest moves backward a millimetre or two. The wire
carries a speed, so the sign is lost, and SUMO answers a backward target by
throwing the ego off its lane. Mechanism and measurements: FIXS#358.

    python -m pytest tests/Python/unit/test_ego_backstep_guard.py
"""
from __future__ import annotations

import pytest

from CommonLib.VirEnv.IVirEnvBackend import EgoState

pytest.importorskip("carla", reason="needs the CARLA PythonAPI")
from Carla.VirEnv.CarlaBackend import CarlaBackend            # noqa: E402


class _Ego:
    """A CARLA ego the test drives along +x, heading due east.

    Heading 90 in SUMO's navigational frame (0 = north, clockwise) is +x, so
    the forward vector is (1, 0) and `vx` is the signed longitudinal speed.
    """

    id = 7

    def __init__(self):
        import carla
        self._carla = carla
        self.x = 100.0
        self.vx = 0.0
        self.bounding_box = carla.BoundingBox(
            carla.Location(0, 0, 0), carla.Vector3D(2.4, 1.0, 0.75))

    def get_transform(self):
        c = self._carla
        return c.Transform(c.Location(self.x, 0.0, 0.0), c.Rotation(yaw=0.0))

    def get_velocity(self):
        return self._carla.Vector3D(self.vx, 0.0, 0.0)


def _backend(guard=True, release=0.5):
    b = CarlaBackend.__new__(CarlaBackend)
    b._egoActor = _Ego()
    b._egoAwaitingSnapshot = False
    b._lastEgoPose = None
    b._egoBackstepHolds = 0
    b._egoBackstepGuard = guard
    b._egoBackstepRelease = release
    return b


#: The wire carries the FRONT-of-vehicle position, so a reported x is the
#: actor's pivot plus its half-length. Pointing east, that is a flat +extent.x.
_kAnchorX = 2.4


def _read(b):
    out = EgoState()
    assert b.readEgoState('ego', out) is True
    return out


def _anchorOf(b):
    """Where the wire would put this actor if nothing were held."""
    return b._egoActor.x + _kAnchorX


def _wired(monkeypatch, **env):
    """A backend built through __init__, so the env wiring is under test too.

    Every other test here sets `_egoBackstepGuard` on the object. That covers
    the guard's arithmetic and never the line that decides whether it runs at
    all -- measured: flipping that default to off left all of them green while
    the guard was dead, which is the same defect as having no test.
    """
    for k in ('FIXS_EGO_BACKSTEP_GUARD', 'FIXS_EGO_BACKSTEP_RELEASE'):
        monkeypatch.delenv(k, raising=False)
    for k, v in env.items():
        monkeypatch.setenv(k, v)
    b = CarlaBackend(None, None, False, False)
    b._egoActor = _Ego()
    return b


def test_the_guard_is_wired_on_by_default(monkeypatch):
    """No env var set: a backward millimetre is held. Guards the default."""
    b = _wired(monkeypatch)
    b._egoActor.vx = 5.0
    settled = _read(b).x
    b._egoActor.x -= 0.003
    b._egoActor.vx = -0.02
    assert _read(b).x == pytest.approx(settled)
    assert b._egoBackstepHolds == 1


def test_the_env_var_is_the_way_off(monkeypatch):
    """FIXS_EGO_BACKSTEP_GUARD=0 is how FIXS#358's 'before' arm was run, so it
    has to keep working -- and it has to be the ONLY thing that turns it off."""
    b = _wired(monkeypatch, FIXS_EGO_BACKSTEP_GUARD='0')
    b._egoActor.vx = 5.0
    _read(b)
    b._egoActor.x -= 0.003
    b._egoActor.vx = -0.02
    assert _read(b).x == pytest.approx(_anchorOf(b))
    assert b._egoBackstepHolds == 0


def test_forward_motion_is_reported_unchanged():
    b = _backend()
    b._egoActor.vx = 5.0
    first = _read(b).x
    b._egoActor.x += 0.5
    assert _read(b).x == pytest.approx(first + 0.5)
    assert b._egoBackstepHolds == 0


def test_a_millimetre_backward_reports_the_previous_pose():
    b = _backend()
    b._egoActor.vx = 5.0
    settled = _read(b).x
    # what the physics does coming off rest: a few mm backward, reported by
    # CARLA as a NEGATIVE longitudinal velocity while `speed` stays positive.
    b._egoActor.x -= 0.003
    b._egoActor.vx = -0.02
    held = _read(b)
    assert held.x == pytest.approx(settled)
    assert b._egoBackstepHolds == 1
    assert held.speed == pytest.approx(0.02)   # magnitude still reported


def test_standing_still_holds_rather_than_repeating_a_stale_compare():
    """A pose that has not moved at all is not ahead either, so it is held.

    Reporting it again is harmless -- an identical position cannot make SUMO
    walk the route -- and it keeps the comparison anchored on the pose the
    traffic simulator actually holds rather than on the last CARLA sample.
    """
    b = _backend()
    b._egoActor.vx = 0.0
    start = _read(b).x
    for _ in range(5):
        assert _read(b).x == pytest.approx(start)
    assert b._egoBackstepHolds == 5


def test_the_guard_releases_once_the_ego_is_really_reversing():
    """Millimetres are noise; half a metre is a manoeuvre.

    Holding forever would freeze the ego in the traffic simulator while the
    real one backs away, which is a worse failure than the one being prevented.
    """
    b = _backend(release=0.5)
    b._egoActor.vx = 5.0
    start = _read(b).x
    b._egoActor.vx = -1.0
    b._egoActor.x -= 0.2
    assert _read(b).x == pytest.approx(start)          # still noise-sized
    b._egoActor.x -= 0.6                                # now 0.8 m behind
    assert _read(b).x == pytest.approx(_anchorOf(b))    # released


def test_the_guard_can_be_switched_off():
    b = _backend(guard=False)
    b._egoActor.vx = 5.0
    _read(b)
    b._egoActor.x -= 0.003
    b._egoActor.vx = -0.02
    assert _read(b).x == pytest.approx(_anchorOf(b))
    assert b._egoBackstepHolds == 0
