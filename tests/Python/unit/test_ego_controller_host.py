"""Does the in-process controller hook close its loop on the plant? (#305, #325)

That is the one question this file exists to answer, because getting it wrong is
invisible in every log a run produces.

An ego controller can occupy the driver slot in two places. Served as a FIXS
client it sees the ego record only at the 0.1 s feed -- and on that path `speed`
is whatever the traffic simulator left there, which under an L2 scenario is the
eco controller's ADVISORY rather than the vehicle's measured speed. A controller
closing a speed loop on it compares its target against a delayed copy of that
same target, sees an error of ~0, and never corrects. Its own log then shows
textbook tracking while the car does something else entirely.

Loaded in-process, runController refreshes the record from the backend before
every call, so the loop closes on the plant. These tests drive the REAL host
against a stub backend -- no CARLA, no SUMO -- and assert the property directly:
hold the advisory fixed, change only what the plant reports, and what the
controller sees must change.

    python -m pytest tests/Python/unit/test_ego_controller_host.py
"""
from __future__ import annotations

import pytest

from CommonLib import fixs
from CommonLib.VirEnv.EgoControllerHost import loadController, runController
from CommonLib.VirEnv.IVirEnvBackend import EgoState

# The three shapes loadController accepts. Normalising them is what the host is
# for, so all three are exercised; each records what it was handed.
#
# A controller commands by WRITING to the ego record. control()'s return value
# is ignored -- see test_a_returned_command_is_ignored below.
CLASS_FORM = """
class Controller:
    def __init__(self, config, egoId):
        self.seen = []

    def control(self, ego, dt):
        self.seen.append((ego.speed, ego.speedDesired, dt))
        ego.set(acceleratorPedalDesired=0.4, brakePedalDesired=0.0,
                steerAngleDesired=0.0)
"""

FUNC_WITH_SETUP = """
def setup(config, egoId):
    return {"seen": []}


def control(ego, dt, state):
    state["seen"].append((ego.speed, ego.speedDesired, dt))
    ego.set(acceleratorPedalDesired=0.4, brakePedalDesired=0.0,
            steerAngleDesired=0.0)
"""

FUNC_BARE = """
def control(ego, dt):
    ego.set(acceleratorPedalDesired=0.4, brakePedalDesired=0.0,
            steerAngleDesired=0.0)
"""

RETURNS_A_DICT = """
def control(ego, dt):
    return {"throttle": 0.4, "brake": 0.0, "steer": 0.0}
"""


class StubBackend:
    """The three verbs runController touches, and nothing else."""

    def __init__(self, speed):
        self.state = EgoState()
        self.state.speed, self.state.x, self.state.y = speed, 0.0, 0.0
        self.state.heading = 90.0
        self.applied = []

    def readEgoState(self, egoId, out):
        for name in ("speed", "x", "y", "z", "heading", "grade", "brake",
                     "indL", "indR"):
            setattr(out, name, getattr(self.state, name, 0.0))
        return True

    def applyEgoActuation(self, throttle, brake, steerNorm):
        self.applied.append(("actuation", throttle, brake, steerNorm))

    def applyEgoSpeedSteer(self, speed, steerNorm, accel=None, jerk=None):
        self.applied.append(("speedsteer", speed, steerNorm))


def makeEgo(**fields):
    """A Vehicle built the way the bridge builds one, guard bypassed."""
    ego = object.__new__(fixs.Vehicle)
    object.__setattr__(ego, "id", "ego")
    object.__setattr__(ego, "_written", frozenset())
    defaults = dict(positionX=0.0, positionY=0.0, positionZ=0.0, heading=90.0,
                    speed=0.0, speedDesired=8.0, feedAge=0.0,
                    acceleratorPedalDesired=0.0, brakePedalDesired=0.0,
                    steerAngleDesired=0.0)
    defaults.update(fields)
    for k, v in defaults.items():
        object.__setattr__(ego, k, v)
    return ego


@pytest.fixture(autouse=True)
def noWireCheck():
    """Vehicle.set refuses fields absent from VehicleMessageField, which is a
    property of a connection these tests do not have."""
    saved = fixs._declaredFields
    fixs._declaredFields = None
    yield
    fixs._declaredFields = saved


@pytest.fixture
def controllerPath(tmp_path):
    p = tmp_path / "probe_controller.py"
    p.write_text(CLASS_FORM, encoding="utf-8")
    return str(p)


def seenBy(ctl):
    """What the loaded controller recorded, whichever shape it was written in."""
    if ctl._instance is not None:
        return ctl._instance.seen
    return ctl._state["seen"]


def drive(controllerPath, plantSpeed, advisory, steps=1, dt=0.05):
    backend = StubBackend(plantSpeed)
    ctl = loadController(controllerPath)
    ctl.setup({}, "ego")
    ego = makeEgo(speedDesired=advisory)
    kind = None
    for _ in range(steps):
        kind = runController(backend, ctl, ego, dt, True, maxSteerRad=0.7)
    return kind, ego, backend, ctl


def test_a_controller_loads_from_a_path_and_its_command_is_applied(controllerPath):
    kind, _, backend, _ = drive(controllerPath, plantSpeed=5.0, advisory=8.0)
    assert kind == "actuation", kind
    assert backend.applied and backend.applied[-1][0] == "actuation"


def test_the_speed_handed_over_is_the_PLANT_not_the_advisory(controllerPath):
    """The bug this file exists for, asserted directly."""
    _, ego, _, ctl = drive(controllerPath, plantSpeed=7.25, advisory=3.0)
    assert ego.speed == pytest.approx(7.25)
    assert ego.speed != pytest.approx(ego.speedDesired)
    seenSpeed, seenAdvisory, _ = ctl._instance.seen[-1] \
        if hasattr(ctl, "_instance") else (ego.speed, ego.speedDesired, 0.05)
    assert seenSpeed == pytest.approx(7.25)
    assert seenAdvisory == pytest.approx(3.0)


def test_what_the_controller_sees_follows_the_plant_with_the_advisory_held(controllerPath):
    """Hold the advisory fixed, change only what the backend reports. On the
    feed path both cases read the same number; here they must separate."""
    advisory = 8.0
    _, slowEgo, _, _ = drive(controllerPath, plantSpeed=1.0, advisory=advisory)
    _, fastEgo, _, _ = drive(controllerPath, plantSpeed=15.0, advisory=advisory)

    assert slowEgo.speed == pytest.approx(1.0)
    assert fastEgo.speed == pytest.approx(15.0)
    assert slowEgo.speedDesired == fastEgo.speedDesired == pytest.approx(advisory)


def test_the_step_the_controller_is_given_is_the_one_it_is_called_at(controllerPath):
    """dt is the CARLA step, not the feed period. The agent's PID gains are
    per-step, so handing it the feed period detunes it by the sub-step ratio."""
    _, _, _, ctl = drive(controllerPath, plantSpeed=5.0, advisory=8.0, dt=0.025)
    assert seenBy(ctl)[-1][2] == pytest.approx(0.025)


def test_a_controller_that_names_no_control_is_refused_at_load(tmp_path):
    p = tmp_path / "bad_controller.py"
    p.write_text("def setup(config, egoId):\n    pass\n", encoding="utf-8")
    with pytest.raises(Exception):
        loadController(str(p))


@pytest.mark.parametrize("shape,source", [
    ("class", CLASS_FORM),
    ("function with setup", FUNC_WITH_SETUP),
    ("bare function", FUNC_BARE),
])
def test_every_accepted_shape_is_called_the_same_way(shape, source, tmp_path):
    """The bridge has one thing to call and does not branch on how the user
    chose to write it."""
    p = tmp_path / "ctl.py"
    p.write_text(source, encoding="utf-8")
    kind, ego, backend, _ = drive(str(p), plantSpeed=6.0, advisory=8.0)
    assert kind == "actuation", (shape, kind)
    assert backend.applied[-1][1] == pytest.approx(0.4), shape


def test_a_returned_command_is_ignored(tmp_path):
    """A controller commands by WRITING to the record, not by returning. A
    controller that returns a dict commands nothing at all, and the host says so
    by returning None rather than guessing -- the last command persists in the
    plant, which is honest; substituting a zero would brake a car whose
    controller simply had nothing new to say."""
    p = tmp_path / "returns.py"
    p.write_text(RETURNS_A_DICT, encoding="utf-8")
    kind, _, backend, _ = drive(str(p), plantSpeed=6.0, advisory=8.0)
    assert kind is None
    assert backend.applied == []


# ---------------------------------------------------------------------------
# the advisory is an INPUT, and the controller's command must not eat it
# ---------------------------------------------------------------------------
#
# speedDesired is the only field that travels both ways: the traffic simulator
# writes the eco advisory into it at the feed, and a speed-commanding controller
# writes its command into the same slot every sub-step. Read back on the next
# sub-step, that command looks exactly like a fresh advisory -- and a controller
# whose target comes from it is then closing a loop on itself, on every second
# step, with nothing in any log to say so. Measured on a 300 s co-simulation
# before this was fixed: 1299 of 2554 controller steps read a speedDesired equal
# to the previous step's own command to within 1e-3, which is every sub-step.

COMMANDS_A_SPEED = """
class Controller:
    def __init__(self, config, egoId):
        self.seen = []

    def control(self, ego, dt):
        self.seen.append(ego.speedDesired)
        # Deliberately NOT the advisory, so a readback is unmistakable.
        ego.set(speedDesired=99.0, steerAngleDesired=0.0)
"""


def speedController(tmp_path):
    p = tmp_path / "speed_controller.py"
    p.write_text(COMMANDS_A_SPEED, encoding="utf-8")
    ctl = loadController(str(p))
    ctl.setup({}, "ego")
    return ctl


def test_the_advisory_survives_the_substeps_of_a_feed(tmp_path):
    """One feed, two CARLA steps. Both steps must read the feed's advisory."""
    backend, ctl = StubBackend(5.0), speedController(tmp_path)
    ego = makeEgo(speedDesired=8.0)

    runController(backend, ctl, ego, 0.05, True, maxSteerRad=0.7)    # the feed
    runController(backend, ctl, ego, 0.05, False, maxSteerRad=0.7)   # the sub-step

    assert ctl._instance.seen == [pytest.approx(8.0), pytest.approx(8.0)]


def test_the_command_still_reaches_the_plant_on_every_substep(tmp_path):
    """Restoring the input must not cost the output: the value applied is the
    controller's command, on the sub-step as much as on the feed."""
    backend, ctl = StubBackend(5.0), speedController(tmp_path)
    ego = makeEgo(speedDesired=8.0)

    runController(backend, ctl, ego, 0.05, True, maxSteerRad=0.7)
    runController(backend, ctl, ego, 0.05, False, maxSteerRad=0.7)

    assert [a[0] for a in backend.applied] == ["speedsteer", "speedsteer"]
    assert [a[1] for a in backend.applied] == [pytest.approx(99.0),
                                               pytest.approx(99.0)]


def test_a_new_feed_replaces_the_held_advisory(tmp_path):
    """Held, not frozen. The next feed's value is what the sub-steps after it
    see -- otherwise this would trade a readback for a stale target."""
    backend, ctl = StubBackend(5.0), speedController(tmp_path)
    ego = makeEgo(speedDesired=8.0)

    runController(backend, ctl, ego, 0.05, True, maxSteerRad=0.7)
    runController(backend, ctl, ego, 0.05, False, maxSteerRad=0.7)
    # the next feed lands, carrying a different advisory
    object.__setattr__(ego, "speedDesired", 4.0)
    runController(backend, ctl, ego, 0.05, True, maxSteerRad=0.7)
    runController(backend, ctl, ego, 0.05, False, maxSteerRad=0.7)

    assert ctl._instance.seen == [pytest.approx(8.0), pytest.approx(8.0),
                                  pytest.approx(4.0), pytest.approx(4.0)]


def test_a_pedal_controller_is_untouched(tmp_path):
    """Only speedDesired is dual-use. A pedal command is an output the traffic
    simulator never fills in, so nothing about it is restored."""
    backend, ctl = StubBackend(5.0), None
    p = tmp_path / "pedal_controller.py"
    p.write_text(CLASS_FORM, encoding="utf-8")
    ctl = loadController(str(p))
    ctl.setup({}, "ego")
    ego = makeEgo(speedDesired=8.0)

    runController(backend, ctl, ego, 0.05, True, maxSteerRad=0.7)
    runController(backend, ctl, ego, 0.05, False, maxSteerRad=0.7)

    assert [a[0] for a in backend.applied] == ["actuation", "actuation"]
    assert ego.acceleratorPedalDesired == pytest.approx(0.4)


# ---------------------------------------------------------------------------
# the CARLA-shaped actuation call
# ---------------------------------------------------------------------------
#
# A user bringing a CARLA script should be able to keep `apply_control(control)`.
# It writes the same record `ego.set` writes, so there is still ONE writer on the
# actuator, the command still reaches DataLogSetup and every other subscriber,
# and the controller still runs against a non-CARLA backend. The conversion from
# CARLA's normalised steer to the record's radians happens there rather than in
# every controller by hand.

RELAYED_PEDALS = """
import fixs.carla as carla


def control(ego, dt):
    carla.apply_control(carla.VehicleControl(throttle=0.4, brake=0.0, steer=0.5))
"""

RELAYED_SPEED = """
import fixs.carla as carla


def control(ego, dt):
    carla.apply_ackermann_control(
        carla.VehicleAckermannControl(speed=7.5, steer=-0.25))
"""


def loadRelayed(tmp_path, source, name):
    p = tmp_path / name
    p.write_text(source, encoding="utf-8")
    ctl = loadController(str(p))
    ctl.setup({}, "ego")
    return ctl


def test_apply_control_lands_on_the_record_as_pedals(tmp_path):
    backend = StubBackend(5.0)
    ctl = loadRelayed(tmp_path, RELAYED_PEDALS, "relayed_pedals.py")
    ego = makeEgo()
    kind = runController(backend, ctl, ego, 0.05, True, maxSteerRad=fixs.MAX_STEER_RAD)

    assert kind == "actuation", kind
    assert ego.acceleratorPedalDesired == pytest.approx(0.4)
    assert ego.brakePedalDesired == pytest.approx(0.0)
    # normalised steer -> radians, once, here
    assert ego.steerAngleDesired == pytest.approx(0.5 * fixs.MAX_STEER_RAD)
    # and the backend was handed the normalised value back
    assert backend.applied[-1][0] == "actuation"
    assert backend.applied[-1][3] == pytest.approx(0.5)


def test_apply_ackermann_control_lands_on_the_record_as_a_speed(tmp_path):
    backend = StubBackend(5.0)
    ctl = loadRelayed(tmp_path, RELAYED_SPEED, "relayed_speed.py")
    ego = makeEgo()
    kind = runController(backend, ctl, ego, 0.05, True, maxSteerRad=fixs.MAX_STEER_RAD)

    assert kind == "speedsteer", kind
    assert ego.speedDesired == pytest.approx(7.5)
    assert ego.steerAngleDesired == pytest.approx(-0.25 * fixs.MAX_STEER_RAD)
    assert backend.applied[-1][0] == "speedsteer"
    assert backend.applied[-1][1] == pytest.approx(7.5)


def test_the_relayed_call_is_refused_outside_a_controller_step():
    """It writes the record for the step in progress, so outside one there is
    nothing to write and saying so beats writing somewhere harmless."""
    import fixs.carla as fixscarla
    with pytest.raises(Exception):
        fixscarla.apply_control(fixscarla.VehicleControl(throttle=0.1))
