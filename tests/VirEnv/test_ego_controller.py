"""The controller hook (#325): loading, both command shapes, and the guards.

Runs with no CARLA, no TrafficLayer and no SUMO -- the point of keeping
IEgoController SDK-free is that this is possible at all.

    python -m pytest tests/VirEnv/test_ego_controller.py
"""

import os
import sys
import textwrap

import pytest

_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from CommonLib import fixs                                          # noqa: E402
from CommonLib.VirEnv.IEgoController import (                       # noqa: E402
    ControllerError, loadController)


# --------------------------------------------------------------------------
# a stand-in for the ego record the bridge hands a controller
# --------------------------------------------------------------------------

def makeEgo(**fields):
    """A Vehicle with the read-only guard bypassed, as the bridge builds one."""
    ego = object.__new__(fixs.Vehicle)
    object.__setattr__(ego, 'id', 'ego')
    object.__setattr__(ego, '_written', frozenset())
    defaults = dict(positionX=0.0, positionY=0.0, positionZ=0.0, heading=90.0,
                    speed=6.0, speedDesired=8.0, signalLightColor=-1,
                    signalLightDistance=0.0, precedingVehicleDistance=0.0,
                    hasPrecedingVehicle=0, feedAge=0.0,
                    acceleratorPedalDesired=0.0, brakePedalDesired=0.0,
                    steerAngleDesired=0.0, accelerationDesired=0.0)
    defaults.update(fields)
    for k, v in defaults.items():
        object.__setattr__(ego, k, v)
    return ego


@pytest.fixture(autouse=True)
def noWireCheck():
    """Vehicle.set refuses a field absent from VehicleMessageField, which is a
    property of a connection these tests deliberately do not have. Clearing it
    exercises the real set() -- validation, _written bookkeeping and all --
    rather than a stand-in that could drift from it."""
    saved = fixs._declaredFields
    fixs._declaredFields = None
    yield
    fixs._declaredFields = saved


def write(ego, **fields):
    ego.set(**fields)


def controllerFile(tmp_path, body, name='ctl.py'):
    p = tmp_path / name
    p.write_text(textwrap.dedent(body), encoding='utf-8')
    return str(p)


# --------------------------------------------------------------------------
# the two command shapes
# --------------------------------------------------------------------------

def test_pedals_and_steer_is_an_actuation_command():
    ego = makeEgo()
    write(ego, acceleratorPedalDesired=0.3, brakePedalDesired=0.0,
          steerAngleDesired=0.1)
    assert fixs.commandKind(ego) == 'actuation'
    fixs._validateCommand(ego)


def test_speed_and_steer_is_a_speedsteer_command():
    """The shape that used to be unrepresentable: steerAngleDesired lived inside
    ACTUATION_FIELDS, so this raised 'missing acceleratorPedalDesired'."""
    ego = makeEgo()
    write(ego, speedDesired=8.3, steerAngleDesired=0.1)
    assert fixs.commandKind(ego) == 'speedsteer'
    fixs._validateCommand(ego)


def test_speed_alone_is_still_a_command():
    ego = makeEgo()
    write(ego, speedDesired=8.3)
    assert fixs.commandKind(ego) == 'speedsteer'
    fixs._validateCommand(ego)


def test_partial_pedals_still_rejected():
    """applyEgoActuation reads all three every tick, so a partial write would
    ship whatever arrived last for the rest."""
    ego = makeEgo()
    write(ego, acceleratorPedalDesired=0.3, brakePedalDesired=0.0)
    with pytest.raises(fixs.ProtocolError, match='steerAngleDesired'):
        fixs._validateCommand(ego)


def test_steer_alone_rejected():
    """Newly representable once steer left the pedal set, and meaningless:
    nothing downstream knows what to do with a steer angle alone."""
    ego = makeEgo()
    write(ego, steerAngleDesired=0.1)
    assert fixs.commandKind(ego) is None
    with pytest.raises(fixs.ProtocolError, match='alone is not a command'):
        fixs._validateCommand(ego)


def test_both_longitudinal_rejected():
    ego = makeEgo()
    write(ego, speedDesired=8.3, accelerationDesired=1.0)
    with pytest.raises(fixs.ProtocolError, match='not both'):
        fixs._validateCommand(ego)


def test_nothing_written_is_not_an_error():
    """A controller with nothing to say this step is normal: the last command
    persists in the plant. Substituting a zero would brake the car."""
    ego = makeEgo()
    assert fixs.commandKind(ego) is None
    fixs._validateCommand(ego)


# --------------------------------------------------------------------------
# loading
# --------------------------------------------------------------------------

def test_bare_function(tmp_path):
    f = controllerFile(tmp_path, '''
        def control(ego, dt):
            ego.set(speedDesired=7.5, steerAngleDesired=0.0)
    ''')
    c = loadController(f)
    c.setup({}, 'ego')
    ego = makeEgo()
    c.control(ego, 0.05)
    assert fixs.commandKind(ego) == 'speedsteer'


def test_setup_state_reaches_control(tmp_path):
    f = controllerFile(tmp_path, '''
        def setup(config, egoId):
            return {"gain": 0.5}

        def control(ego, dt, state=None):
            ego.set(speedDesired=state["gain"] * 10.0, steerAngleDesired=0.0)
    ''')
    c = loadController(f)
    c.setup({}, 'ego')
    ego = makeEgo()
    c.control(ego, 0.05)
    assert ego.speedDesired == pytest.approx(5.0)


def test_class_form(tmp_path):
    f = controllerFile(tmp_path, '''
        class Controller:
            def __init__(self, config, egoId):
                self.egoId = egoId
            def control(self, ego, dt):
                ego.set(speedDesired=3.0, steerAngleDesired=0.0)
    ''')
    c = loadController(f)
    c.setup({}, 'ego')
    ego = makeEgo()
    c.control(ego, 0.05)
    assert ego.speedDesired == pytest.approx(3.0)


def test_dt_is_given_not_guessed(tmp_path):
    """The caller is the only thing that knows the interval. A controller that
    has to assume one is how CARLA's own LocalPlanner ends up running its PIDs
    at 20 Hz inside a 10 Hz loop."""
    f = controllerFile(tmp_path, '''
        seen = []
        def control(ego, dt):
            seen.append(dt)
            ego.set(speedDesired=1.0, steerAngleDesired=0.0)
    ''')
    c = loadController(f)
    ego = makeEgo()
    c.control(ego, 0.05)
    c.control(ego, 0.05)
    assert sys.modules['ctl'].seen == [0.05, 0.05]


# --------------------------------------------------------------------------
# the guards -- each one turns a silent failure into a startup error
# --------------------------------------------------------------------------

def test_missing_file_names_the_path():
    with pytest.raises(ControllerError, match='no such file'):
        loadController('does/not/exist.py')


def test_no_entry_point_says_what_to_define(tmp_path):
    f = controllerFile(tmp_path, '''
        def drive(ego, dt):        # wrong name
            pass
    ''', name='noentry.py')
    with pytest.raises(ControllerError, match='defines none of'):
        loadController(f)


def test_named_attribute_that_is_missing(tmp_path):
    f = controllerFile(tmp_path, 'def control(ego, dt): pass\n', name='named.py')
    with pytest.raises(ControllerError, match='has no'):
        loadController(f + ':nosuch')


def test_not_callable(tmp_path):
    f = controllerFile(tmp_path, 'control = 42\n', name='notcallable.py')
    with pytest.raises(ControllerError, match='not callable'):
        loadController(f)


def test_class_without_control(tmp_path):
    f = controllerFile(tmp_path, '''
        class Controller:
            def __init__(self, config, egoId): pass
    ''', name='noctl.py')
    with pytest.raises(ControllerError, match='no control'):
        loadController(f)


def test_shipped_template_loads_and_commands(tmp_path):
    """The template is documentation people copy; if it stops working, the first
    thing anyone writes is broken."""
    template = os.path.join(_ROOT, 'Carla', 'VirEnv', 'templates',
                            'controller_template.py')
    c = loadController(template)
    c.setup({'EgoRoutePoints': [(0.0, 0.0), (10.0, 0.0)]}, 'ego')
    ego = makeEgo(speed=6.0, speedDesired=8.0)
    c.control(ego, 0.05)
    assert fixs.commandKind(ego) == 'actuation'
    fixs._validateCommand(ego)
    c.shutdown()
