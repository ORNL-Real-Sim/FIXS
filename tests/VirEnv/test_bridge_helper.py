"""The Python BridgeHelper puts a vehicle where the C++ one does.

The bridge exchanges ABSOLUTE positions, so a sign or an operation order wrong in
this transform does not degrade gracefully -- it puts every vehicle off the road,
and it does so identically every tick, which reads like a map problem rather than
a frame problem. That is worth an independent check rather than a re-reading of my
own arithmetic.

The independent reference is CARLA's own
``Carla/sumo/run_synchronization/sumo_integration/bridge_helper.py``, whose
``get_carla_transform`` / ``get_sumo_transform`` are the upstream code the C++
BridgeHelper was ported FROM, and are still identical in arithmetic (yaw ->
-yaw + 90, step back cos/sin * extent.x, Y flip, rotation yaw - 90). So agreeing
with it is evidence about the port, not a restatement of it.

What that reference does NOT cover is checked separately here: the vClass ->
blueprint sets, the SUMO <-> CARLA signal-state mapping, and the FIXS
traffic-light table reader, none of which exist upstream in this form.

Needs the ``carla`` package (for its geometry types); skipped without it.
"""

import math
import os
import sys

import pytest

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path.insert(0, REPO_ROOT)

carla = pytest.importorskip('carla', reason='the carla client package is not installed')

sys.path.insert(0, os.path.join(REPO_ROOT, 'Carla', 'sumo', 'run_synchronization'))

from Carla.VirEnv.BridgeHelper import (BridgeHelper, SumoTrafficLightState,  # noqa: E402
                                       TrafficLight)
from sumo_integration.bridge_helper import BridgeHelper as UpstreamBridgeHelper  # noqa: E402

#: A spread that exercises every sign: all four quadrants of heading, the 0/360
#: seam, a climb and a descent, negative coordinates, and z above and below zero.
POSES = [
    (0.0, 0.0, 0.0, 0.0, 0.0),
    (100.0, 200.0, 5.0, 90.0, 0.0),
    (-50.0, 75.5, -2.5, 180.0, 3.0),
    (1234.5, -987.25, 210.0, 270.0, -4.5),
    (10.0, 10.0, 0.0, 359.9, 0.0),
    (10.0, 10.0, 0.0, 0.1, 0.0),
    (300.0, -12.0, 18.0, 45.0, 8.25),
    (-300.0, 12.0, -18.0, 225.0, -8.25),
]

#: A passenger-car half-size and a truck-sized one: extent.x is the anchor step,
#: so a wrong sign shows up as double the half-length, not as a rounding error.
EXTENTS = [(2.3, 1.0, 0.75), (4.2, 1.3, 1.6)]


def _tf(x, y, z, yaw, pitch):
    return carla.Transform(carla.Location(x, y, z), carla.Rotation(pitch, yaw, 0.0))


def _same(a, b, tol=1e-6):
    """Compare two transforms, with yaw compared modulo 360."""
    assert a.location.x == pytest.approx(b.location.x, abs=tol)
    assert a.location.y == pytest.approx(b.location.y, abs=tol)
    assert a.location.z == pytest.approx(b.location.z, abs=tol)
    assert a.rotation.pitch == pytest.approx(b.rotation.pitch, abs=tol)
    # signed shortest arc: a yaw of 0.1 deg coming back as 0.099999 is a match,
    # not a 359.99999 miss.
    assert abs((a.rotation.yaw - b.rotation.yaw + 180.0) % 360.0 - 180.0) < tol


@pytest.fixture(autouse=True)
def _zeroOffset():
    """Both helpers read a module-level offset; pin it so neither leaks into the other."""
    saved = BridgeHelper.offset
    BridgeHelper.offset = carla.Location(0.0, 0.0, 0.0)
    UpstreamBridgeHelper.offset = (0.0, 0.0)
    yield
    BridgeHelper.offset = saved


@pytest.mark.parametrize('pose', POSES)
@pytest.mark.parametrize('ext', EXTENTS)
def test_sumo_to_carla_matches_upstream(pose, ext):
    extent = carla.Vector3D(*ext)
    sumoTf = _tf(*pose)
    _same(UpstreamBridgeHelper.get_carla_transform(sumoTf, extent),
          BridgeHelper.map_transfrom_Sumo_to_Carla(sumoTf, extent))


@pytest.mark.parametrize('pose', POSES)
@pytest.mark.parametrize('ext', EXTENTS)
def test_carla_to_sumo_matches_upstream(pose, ext):
    extent = carla.Vector3D(*ext)
    carlaTf = _tf(*pose)
    _same(UpstreamBridgeHelper.get_sumo_transform(carlaTf, extent),
          BridgeHelper.map_transfrom_Carla_to_Sumo(carlaTf, extent))


@pytest.mark.parametrize('pose', POSES)
def test_the_two_directions_invert_in_x_y_and_yaw(pose):
    """Forward then back returns the planar pose, which a wrong anchor sign would not.

    The anchor step is what makes this a real check: getting ``+ extent.x`` where
    ``- extent.x`` belongs still round-trips through a naive implementation that
    applies the same error twice, but not through these two, which derive the step
    from DIFFERENT yaw expressions (``-yaw + 90`` forward, ``-yaw`` back).

    z is checked separately below -- it does NOT invert, and that is inherited.
    """
    extent = carla.Vector3D(2.3, 1.0, 0.75)
    sumoTf = _tf(*pose)
    back = BridgeHelper.map_transfrom_Carla_to_Sumo(
        BridgeHelper.map_transfrom_Sumo_to_Carla(sumoTf, extent), extent)
    assert back.location.x == pytest.approx(sumoTf.location.x, abs=1e-4)
    assert back.location.y == pytest.approx(sumoTf.location.y, abs=1e-4)
    assert back.rotation.pitch == pytest.approx(sumoTf.rotation.pitch, abs=1e-4)
    assert abs((back.rotation.yaw - sumoTf.rotation.yaw + 180.0) % 360.0 - 180.0) < 1e-4


def test_z_does_not_invert_and_that_is_inherited_not_introduced_here():
    """ORNL-Real-Sim/FIXS#326: the reverse transform subtracts the pitch component
    instead of adding it, so a round trip moves z by 2 * sin(pitch) * extent.x
    rather than returning it.

    x and y invert correctly because the reverse mirrors their sign (``- cos``
    forward, ``+ cos`` back); z uses ``- sin(pitch) * extent.x`` in BOTH
    directions. The defect is in CARLA's own sumo_integration/bridge_helper.py and
    was carried into VirCarlaEnv/BridgeHelper.cpp, and this port reproduces it
    deliberately -- a Python bridge that quietly corrected it would place the ego's
    elevation differently from the C++ bridge, which is exactly the divergence
    #325 exists to prevent.

    Pinned here so the day it IS fixed, it is fixed in both and this test says so.
    """
    extent = carla.Vector3D(2.3, 1.0, 0.75)
    pitchDeg = 6.0
    sumoTf = _tf(0.0, 0.0, 10.0, 0.0, pitchDeg)
    back = BridgeHelper.map_transfrom_Carla_to_Sumo(
        BridgeHelper.map_transfrom_Sumo_to_Carla(sumoTf, extent), extent)
    step = math.sin(math.radians(pitchDeg)) * extent.x
    assert back.location.z == pytest.approx(10.0 - 2.0 * step, abs=1e-5)
    # ... and upstream CARLA does exactly the same, which is where it comes from.
    upstreamBack = UpstreamBridgeHelper.get_sumo_transform(
        UpstreamBridgeHelper.get_carla_transform(sumoTf, extent), extent)
    assert back.location.z == pytest.approx(upstreamBack.location.z, abs=1e-5)


def test_the_anchor_step_is_half_a_vehicle_not_a_whole_one():
    """A vehicle heading east lands its FRONT on the SUMO x, pivot half-length back.

    Pinned as an absolute distance because this is the exact error #174 fixed: the
    old code passed a hard-coded extent of 4.8 as a HALF-length, implying a 9.6 m
    car, so every actor was anchored ~2.4 m too far back -- a full car-length of
    lateral error on a curve, which looks like a map offset.
    """
    extent = carla.Vector3D(2.3, 1.0, 0.75)
    # FIXS heading 90 = due east; the CARLA pivot must sit 2.3 m behind it in -x.
    out = BridgeHelper.map_transfrom_Sumo_to_Carla(_tf(100.0, 0.0, 0.0, 90.0, 0.0), extent)
    assert out.location.x == pytest.approx(100.0 - 2.3, abs=1e-4)
    assert out.location.y == pytest.approx(0.0, abs=1e-4)


def test_location_only_mapping_applies_no_anchor():
    """map_location_Carla_to_Sumo is the Y flip alone -- signal actors have no heading."""
    out = BridgeHelper.map_location_Carla_to_Sumo(carla.Location(12.0, -34.0, 5.0))
    assert (out.x, out.y, out.z) == pytest.approx((12.0, 34.0, 5.0))


def test_grade_moves_z_by_the_pitch_component():
    """A climbing vehicle's pivot sits below its front by extent.x * sin(pitch)."""
    extent = carla.Vector3D(2.3, 1.0, 0.75)
    out = BridgeHelper.map_transfrom_Sumo_to_Carla(_tf(0.0, 0.0, 10.0, 0.0, 6.0), extent)
    assert out.location.z == pytest.approx(10.0 - math.sin(math.radians(6.0)) * 2.3, abs=1e-5)


# ---------------------------------------------------------------------------
# What upstream does not cover
# ---------------------------------------------------------------------------

@pytest.mark.parametrize('vClass,expected', [
    ('passenger', 'vehicle.'), ('truck', 'vehicle.'), ('van', 'vehicle.'),
    ('bus', 'vehicle.'), ('motorcycle', 'vehicle.'), ('bicycle', 'vehicle.'),
    ('pedestrian', 'walker.'), ('emergency', 'vehicle.'),
])
def test_every_supported_vclass_maps_into_its_own_family(vClass, expected):
    for _ in range(20):          # the pick is random; every draw must stay in family
        assert BridgeHelper.map_Sumo_vClass_to_Carla_blueprintId(vClass).startswith(expected)


def test_an_unknown_vclass_falls_back_rather_than_failing():
    """One unmapped class must not stop the run: SUMO nets carry classes CARLA lacks."""
    assert BridgeHelper.map_Sumo_vClass_to_Carla_blueprintId('rail_urban') == 'vehicle.tesla.model3'


@pytest.mark.parametrize('ch,expected', [
    ('r', carla.TrafficLightState.Red),
    ('u', carla.TrafficLightState.Red),        # red+yellow: a driver may not go
    ('y', carla.TrafficLightState.Yellow),
    ('G', carla.TrafficLightState.Green),
    ('g', carla.TrafficLightState.Green),      # green without priority is still green
    ('O', carla.TrafficLightState.Off),
    ('s', carla.TrafficLightState.Unknown),    # green right turn: no CARLA equivalent
    ('o', carla.TrafficLightState.Unknown),    # off-blinking: no CARLA equivalent
])
def test_sumo_signal_characters_map_to_carla_states(ch, expected):
    assert BridgeHelper.map_Sumo_traffic_light_state_to_Carla(ch) == expected


def test_an_unrecognised_state_character_is_refused():
    """Silently treating it as OFF would show a green light on a red phase."""
    with pytest.raises(ValueError):
        BridgeHelper.get_Sumo_traffic_light_state_from_char('X')


def test_traffic_light_table_is_read_by_junction_and_link(tmp_path):
    table = tmp_path / 'tl.csv'
    table.write_text(
        'junctionId,linkId,x,y,z,heading\n'
        'J1,0,10.0,20.0,1.0,90.0\n'
        'J1,1,10.5,20.0,1.0,90.0\n'
        'J2,0,500.0,600.0,2.0,180.0\n', encoding='utf-8')
    m = BridgeHelper.readTrafficLightTable(str(table))
    assert sorted(m) == ['J1', 'J2']
    assert sorted(m['J1']) == [0, 1]
    assert m['J2'][0].x == pytest.approx(500.0)
    assert m['J1'][1].state == SumoTrafficLightState.OFF   # until the feed says otherwise


def test_a_missing_table_is_reported_not_raised():
    """A signal-free scenario passes no table; that must not stop the bridge."""
    assert BridgeHelper.readTrafficLightTable('does_not_exist.csv') == {}


def test_closest_traffic_light_is_planar():
    """Elevation is excluded on purpose: a CARLA light actor sits at head height
    while the table row is at the stop line, so including z would bias every match
    toward whichever junction happened to be lowest."""
    m = {'J1': {0: TrafficLight('J1', 0, 0.0, 0.0, 0.0, 0.0)},
         'J2': {0: TrafficLight('J2', 0, 100.0, 0.0, 900.0, 0.0)}}
    assert BridgeHelper.find_closest_trafficLight_id(m, 99.0, 0.0) == ('J2', 0)
    assert BridgeHelper.find_closest_trafficLight_id({}, 0.0, 0.0) == ('', -1)
