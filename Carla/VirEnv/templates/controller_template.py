"""A FIXS ego controller, in the smallest form that works.

Copy this next to your application, edit control(), and point the scenario at it:

    EgoActuationSource: embedded
    EgoController:      apps/<your_app>/my_controller.py

Run it as an ordinary FIXS client instead, with no code change, by setting
EgoActuationSource: external -- the difference is only where it runs, and so how
often it is called. See CommonLib/VirEnv/IEgoController.py for the full contract.
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

    # --- shape 1: pedals + steer. You close the loop. -----------------------
    error = target - ego.speed
    ego.set(acceleratorPedalDesired=max(0.0, min(0.75, 0.25 * error + 0.15)),
            brakePedalDesired=max(0.0, min(0.30, -0.30 * error)),
            steerAngleDesired=0.0)

    # --- shape 2: speed + steer. The plant closes it, at its own rate. -------
    # Swap for the block above to hand longitudinal tracking to CARLA's own
    # Ackermann controller. Its gains are then in play instead of yours.
    #
    # ego.set(speedDesired=target, steerAngleDesired=0.0)


def shutdown(state=None):
    """Optional. Called once, after the run."""
