"""fixs.driver -- the ready-made controller, and the one seam a rig owner uses.

The seam is a function: a speed in, the speed the cell reached out. These tests
hold it to the three cases the driver has to answer -- your function, the
simulated cell, or nobody -- and to the rule that a tick the cell could not
answer passes the reference through untouched rather than inventing motion.
"""
import io
import os
import sys
import tempfile

import pytest

sys.path.insert(0, os.path.abspath(
    os.path.join(os.path.dirname(__file__), '..', '..', '..')))

import CommonLib.fixs as fixs                                    # noqa: E402
from CommonLib.fixs._driver import (Controller, Limits, Tuning,  # noqa: E402
                                    driver, _options)  # noqa: E402


_SCENARIO = """
SimulationSetup: {EnableRealSim: true, SelectedTrafficSimulator: SUMO}
EgoSetup: {Dynamics: virenv, ActuationSource: user, Controller: x.py}
XilSetup: {EnableXil: %s, Transport: inprocess}
CarlaSetup: {EnableCosimulation: true, EnablePythonBackend: true}
"""


@pytest.fixture
def scenario(request):
    """A yaml on disk, with or without the simulated dyno declared."""
    with tempfile.NamedTemporaryFile('w', suffix='.yaml', delete=False) as f:
        f.write(_SCENARIO % ('true' if request.param else 'false'))
        path = f.name
    old = os.environ.get('FIXS_CONFIG_YAML')
    os.environ['FIXS_CONFIG_YAML'] = path
    yield path
    os.environ.pop('FIXS_CONFIG_YAML', None)
    if old is not None:
        os.environ['FIXS_CONFIG_YAML'] = old
    os.unlink(path)


def _build(cls):
    """The driver without its CARLA half: __init__ up to the agent, which is
    built on the first controlled tick and needs a live simulator."""
    obj = cls.__new__(cls)
    Controller.__init__(obj, {'EgoControllerLog': ''}, 'ego')
    return obj


# -- what fixs.driver() hands back -----------------------------------------

def test_it_returns_a_class_because_that_is_what_the_loader_expects():
    cls = driver()
    assert isinstance(cls, type)
    assert callable(getattr(cls, 'control', None))


def test_a_non_callable_exchange_is_refused_at_the_call_not_at_the_tick():
    with pytest.raises(TypeError):
        driver(42)


# -- the three cases the seam has to answer ---------------------------------

@pytest.mark.parametrize('scenario', [False], indirect=True)
def test_no_exchange_means_nothing_in_the_loop(scenario):
    d = _build(driver())
    assert d.benchInLoop is False


@pytest.mark.parametrize('scenario', [True], indirect=True)
def test_the_scenario_alone_brings_nothing(scenario):
    """EnableXil declares the SIMULATED cell exists; it does not put one in
    this driver's loop. Only passing a function does that."""
    d = _build(driver())
    assert d.benchInLoop is False


@pytest.mark.parametrize('scenario', [False], indirect=True)
def test_your_function_needs_no_scenario_flag(scenario):
    """Passing one IS the declaration."""
    d = _build(driver(lambda v, dt: v))
    assert d.benchInLoop is True


@pytest.mark.parametrize('scenario', [True], indirect=True)
def test_the_simulated_dyno_is_passed_in_like_any_other(scenario):
    """The bench is built with its parameters, and the exchange is a function
    the caller writes -- the same two lines a rig owner replaces."""
    from CommonLib.fixs import xil
    bench = xil.dyno(vehicle={'mass_kg': 900.0})

    def exchange(vref, dt):
        return bench.exchange(vref, dt)

    d = _build(driver(exchange))
    assert d.benchInLoop is True
    assert d.exchange(10.0) > 0.0        # it answered
    assert bench.sim.vehicle.mass_kg == 900.0   # stated in code, not the yaml


def test_the_driver_does_not_import_xil():
    """The coupling this PR removes: a driver that reaches for a bench is
    deciding something that belongs to whoever built it."""
    src = io.open(os.path.join(os.path.dirname(__file__), '..', '..', '..',
                               'CommonLib', 'fixs', '_driver.py'),
                  encoding='utf-8').read()
    code = [ln for ln in src.splitlines()
            if ln.startswith(('import ', 'from ')) and 'xil' in ln]
    assert code == [], code


# -- the rule that matters when hardware misbehaves -------------------------

@pytest.mark.parametrize('scenario', [False], indirect=True)
def test_a_tick_the_dyno_cannot_answer_passes_the_reference_through(scenario):
    """None back must not become 0 m/s: that would invent a stop."""
    answers = [7.5, None, 7.6]
    d = _build(driver(lambda v, dt: answers.pop(0)))
    assert d.exchange(9.0) == 7.5           # the cell answered
    assert d.exchange(9.0) == 9.0           # it did not -- the reference stands
    assert d.misses == 1
    assert d.exchange(9.0) == 7.6
    assert d.misses == 1


# -- options, nearest the run wins ------------------------------------------

def test_the_scenario_beats_the_factory_beats_the_default():
    assert _options({}).command_shape == 'speed'
    assert _options({}, {'shape': 'pedals'}).command_shape == 'pedals'
    assert _options({'EgoControllerArgs': ['--command-shape', 'speed']},
                    {'shape': 'pedals'}).command_shape == 'speed'


def test_an_unknown_option_fails_at_construction():
    with pytest.raises(TypeError):
        _options({}, {'nonsense': 1})


# -- the gains, as one value ------------------------------------------------

def test_a_gain_ladder_is_one_token_per_rung():
    assert str(Tuning()) == 'kv=0.25,ki=0.5,kp=5,k=0.01,maxAccel=1.5,fullStop=0.1'
    assert Tuning.parse('kv=0.3,ki=0.6').kv == 0.3
    assert Tuning.parse('kv=0.3,ki=0.6').ki == 0.6
    assert Tuning.parse('kv=0.3').ki == 0.5          # the rest stand


def test_a_typo_in_a_gain_name_is_refused_not_ignored():
    """The failure that matters: a run that silently used the defaults while
    its command line claimed otherwise."""
    with pytest.raises(TypeError):
        Tuning().replace(nope=1)
    with pytest.raises(TypeError):
        Tuning.parse('nope=1')
    with pytest.raises(ValueError):
        Tuning.parse('kv')
    with pytest.raises(ValueError):
        Tuning.parse('kv=fast')


def test_the_scenario_beats_the_factory_for_gains_too():
    assert _options({}, {'tuning': Tuning(kv=0.9)}).tuning.kv == 0.9
    assert _options({'EgoControllerArgs': ['--tune', 'kv=0.1']},
                    {'tuning': Tuning(kv=0.9)}).tuning.kv == 0.1


@pytest.mark.parametrize('scenario', [False], indirect=True)
def test_the_run_records_what_drove_it(scenario, tmp_path):
    """A batch once reported gains it had not used. The log says."""
    path = tmp_path / 'agent.csv'
    cls = driver(lambda v, dt: v, shape='pedals', tuning=Tuning(kv=0.33))
    obj = cls.__new__(cls)
    Controller.__init__(obj, {'EgoControllerLog': str(path)}, 'ego')
    obj.shutdown()
    first = path.read_text().splitlines()[0]
    assert first.startswith('#')
    assert 'shape=pedals' in first
    assert 'kv=0.33' in first
    assert 'exchange=<lambda>' in first
    assert 'comfortDecel=2' in first   # the limits are recorded too


# -- the law itself, which is pure arithmetic and needs no simulator ---------

def _pedalLaw(**tune):
    cls = driver(shape='pedals', tuning=Tuning(**tune))
    obj = cls.__new__(cls)
    Controller.__init__(obj, {'EgoControllerLog': '', 'CarlaTimeStep': 0.05}, 'ego')
    return obj


def test_a_stopped_car_asked_for_nothing_holds_the_brake():
    """The full-stop rule -- and the line that broke rung 4 in review, because
    it reads a gain before the law had bound it."""
    d = _pedalLaw()
    assert d._speedToPedal(0.0, 0.0) == (0.0, 1.0)


def test_zero_speed_error_still_holds_a_pedal():
    """The whole reason the integral exists: road load has to be paid."""
    d = _pedalLaw()
    for _ in range(40):
        thr, brk = d._speedToPedal(8.0, 8.0 - 0.2)     # a standing shortfall
    assert thr > 0.0
    thr, brk = d._speedToPedal(8.0, 8.0)               # error now zero
    assert thr > 0.0, 'the pedal collapsed when the error did'


def test_the_gains_reach_the_law():
    """A tuning that cannot move the pedal is a tuning nobody is applying."""
    slow, fast = _pedalLaw(kv=0.01), _pedalLaw(kv=0.9)
    assert fast._speedToPedal(9.0, 4.0)[0] > slow._speedToPedal(9.0, 4.0)[0]


# -- the limits, and that they reach the ceilings ---------------------------

def test_limits_reach_the_stopping_ceiling():
    """A limit that cannot change how early the ego slows is a limit nobody is
    applying -- the same failure the gains had, caught the same way."""
    from CommonLib.fixs._driver import Limits, _stopBy
    gentle, firm = Limits(comfortDecel=1.0), Limits(comfortDecel=4.0)
    assert _stopBy(50.0, firm.comfortDecel) > _stopBy(50.0, gentle.comfortDecel)


def test_limits_are_carried_and_recorded():
    cls = driver(limits=Limits(comfortDecel=1.5, stopMargin=3.0))
    obj = cls.__new__(cls)
    Controller.__init__(obj, {'EgoControllerLog': ''}, 'ego')
    assert obj.limits.comfortDecel == 1.5
    assert obj.limits.stopMargin == 3.0
    assert 'comfortDecel=1.5' in str(obj.limits)


def test_a_wrong_type_for_limits_is_refused():
    with pytest.raises(TypeError):
        _options({}, {'limits': {'comfortDecel': 1.5}})


def test_ideal_speed_tracking_is_settable_like_the_shape_it_pairs_with():
    """It only bites under shape='speed'; leaving it a bare global while the
    shape was settable was the inconsistency."""
    assert _options({}).idealSpeedTracking is True
    assert _options({}, {'idealSpeedTracking': False}).idealSpeedTracking is False


# -- the name is the user's ------------------------------------------------

_ANY_NAME = '''
import sys
sys.path.insert(0, %r)
import CommonLib.fixs as fixs
%s
'''


def _controllerFile(tmp_path, body, name='c.py'):
    root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
    p = tmp_path / name
    p.write_text(_ANY_NAME % (root, body))
    return str(p)


def test_the_name_you_give_the_driver_is_yours(tmp_path):
    """No `Controller =` anywhere: fixs.driver() says what it built."""
    from CommonLib.VirEnv.EgoControllerHost import loadController
    for body in ('Driver = fixs.driver()',       # any name
                 'fixs.driver()',                # or none
                 'WhateverIWant = fixs.driver()'):
        lc = loadController(_controllerFile(tmp_path, body, 'c%d.py' % hash(body)))
        assert callable(getattr(lc._obj, 'control', None))


def test_a_hand_written_controller_still_works_by_name(tmp_path):
    """The name scan stays, for anyone not using fixs.driver()."""
    from CommonLib.VirEnv.EgoControllerHost import loadController
    p = _controllerFile(tmp_path, '''
class Controller:
    def __init__(self, config, egoId): pass
    def control(self, ego, dt): pass
''', 'hand.py')
    assert loadController(p)._obj.__name__ == 'Controller'


def test_two_drivers_in_one_file_is_refused(tmp_path):
    """Last-wins would run one of two, chosen by statement order."""
    from CommonLib.VirEnv.EgoControllerHost import loadController, ControllerError
    p = _controllerFile(tmp_path, 'A = fixs.driver()\nB = fixs.driver()', 'two.py')
    with pytest.raises(ControllerError) as e:
        loadController(p)
    assert 'twice' in str(e.value) or '2 times' in str(e.value)
