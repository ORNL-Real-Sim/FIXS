"""fixs.driver -- the ready-made controller, and the one seam a rig owner uses.

The seam is a function: a speed in, the speed the cell reached out. These tests
hold it to the three cases the driver has to answer -- your function, the
simulated cell, or nobody -- and to the rule that a tick the cell could not
answer passes the reference through untouched rather than inventing motion.
"""
import os
import sys
import tempfile

import pytest

sys.path.insert(0, os.path.abspath(
    os.path.join(os.path.dirname(__file__), '..', '..', '..')))

import CommonLib.fixs as fixs                                    # noqa: E402
from CommonLib.fixs._driver import Controller, driver, _options  # noqa: E402


_SCENARIO = """
SimulationSetup: {EnableRealSim: true, SelectedTrafficSimulator: SUMO}
EgoSetup: {Dynamics: virenv, ActuationSource: user, Controller: x.py}
XilSetup: {EnableXil: %s, Transport: inprocess}
CarlaSetup: {EnableCosimulation: true, EnablePythonBackend: true}
"""


@pytest.fixture
def scenario(request):
    """A yaml on disk, with or without the simulated cell declared."""
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
def test_no_cell_declared_and_none_given_leaves_the_loop_open(scenario):
    d = _build(driver())
    assert d.benchInLoop is False


@pytest.mark.parametrize('scenario', [True], indirect=True)
def test_the_scenario_alone_brings_the_simulated_cell(scenario):
    d = _build(driver())
    assert d.benchInLoop is True
    assert d._bench is not None


@pytest.mark.parametrize('scenario', [False], indirect=True)
def test_your_function_needs_no_scenario_flag(scenario):
    """Passing one IS the declaration -- EnableXil is about the simulated cell."""
    d = _build(driver(lambda v, dt: v))
    assert d.benchInLoop is True
    assert d._bench is None


@pytest.mark.parametrize('scenario', [True], indirect=True)
def test_your_function_wins_over_the_simulated_cell(scenario):
    d = _build(driver(lambda v, dt: 1.0))
    assert d._bench is None
    assert d._exchange(9.0, 0.05) == 1.0


# -- the rule that matters when hardware misbehaves -------------------------

@pytest.mark.parametrize('scenario', [False], indirect=True)
def test_a_tick_the_cell_cannot_answer_passes_the_reference_through(scenario):
    """None back must not become 0 m/s: that would invent a stop."""
    answers = [7.5, None, 7.6]
    d = _build(driver(lambda v, dt: answers.pop(0)))
    assert d.throughCell(9.0) == 7.5           # the cell answered
    assert d.throughCell(9.0) == 9.0           # it did not -- the reference stands
    assert d.misses == 1
    assert d.throughCell(9.0) == 7.6
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
