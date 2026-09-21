"""Write your own ego controller. The smallest form that works.

Copy this next to your application and point the scenario at it::

    EgoSetup:
      Dynamics: virenv            # the virtual environment moves the ego
      ActuationSource: user       # a controller produces the pedals and steer
      Controller: apps/<your_app>/my_controller.py

FIXS imports that file, looks for a module-level ``Controller`` (or a
``control`` function), builds it once with ``(config, egoId)``, and calls
``control(ego, dt)`` every step. Those two signatures are the whole contract.
The scenario can name the attribute instead -- ``my_controller.py:Whatever`` --
and then the name here does not matter either.

DO YOU NEED TO WRITE ONE? FIXS ships a working driver: the eco advisory, the
signal and leader ceilings, a speed-to-pedal law and both command shapes,
with one function for your dynamometer. See ``driver_template.py``. Write your
own when you want control logic FIXS does not have -- not to get an ego
moving.
"""


def setup(config, egoId):
    """Optional. Called once, before the run. Whatever you return comes back as
    the third argument to control()."""
    return {"route": config.get("EgoRoutePoints", [])}


def control(ego, dt, state=None):
    """Called once per step. Read `ego`, write a command onto it.

    dt is the interval FIXS will actually call you at -- use it rather than
    assuming one, so the same file behaves correctly embedded (CarlaTimeStep)
    and external (the 0.1 s feed).

    LIVE every call : positionX, positionY, heading, speed, acceleration
    HELD since the last feed, ego.feedAge seconds ago:
                      speedDesired, signalLightColor
    """
    target = ego.speedDesired if ego.speedDesired > 0.01 else 8.33

    # --- shape 1: pedals + steer. You close the speed loop. -----------------
    error = target - ego.speed
    ego.set(acceleratorPedalDesired=max(0.0, min(0.75, 0.25 * error + 0.15)),
            brakePedalDesired=max(0.0, min(0.30, -0.30 * error)),
            steerAngleDesired=0.0)

    # --- shape 2: speed + steer. The plant closes it, at its own rate. -------
    # Swap for the block above to hand longitudinal tracking to CARLA's own
    # Ackermann controller. Its gains are then in play instead of yours.
    #
    # ego.set(speedDesired=target, steerAngleDesired=0.0)

    # A DYNAMOMETER, if you have one, goes between the two: decide a speed,
    # ask the cell what a real vehicle reached, drive to THAT. How you reach it
    # is yours -- FIXS has no interface for it. driver_template.py shows the
    # placement and the three things that bite.


def shutdown(state=None):
    """Optional. Called once, after the run."""
