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
    import fixs.xil
    return {"route": config.get("EgoRoutePoints", []),
            # None unless the SCENARIO says a bench is in the loop. The
            # simulated cell is real working code, not a stub -- a run with it
            # is a run with a vehicle that has mass and a torque delay.
            "dyno": fixs.xil.dyno() if fixs.xil.enabled() else None}


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

    # --- a dynamometer, when the scenario says so --------------------------
    # AFTER every decision you make, BEFORE you command anything: you decide a
    # speed, the cell says what a real vehicle reached, and THAT is what you
    # then drive to. Next step you read ego.speed back, so the cell is in the
    # loop rather than beside it.
    dyno = (state or {}).get("dyno")
    if dyno is not None:
        target = dyno.exchange(target, dt)

    # FOR A REAL CELL, replace the object -- nothing else changes:
    #
    #     state["dyno"] = MyCell("192.168.1.50")    # instead of fixs.xil.dyno()
    #
    # It needs one method, exchange(speed, dt) -> speed. No subclass, no
    # registration, no key in the FIXS schema: how your cell is spoken to is
    # yours, and FIXS's simulated one carries no more authority than it.
    # Three things bite. It is called every step (20 Hz at CarlaTimeStep 0.05),
    # so it MUST NOT BLOCK -- send, then take the newest answer that already
    # arrived. When none has, return the reference you were given: it is the
    # only answer that cannot invent motion. And count that, because a run
    # ending with a large miss count did not test what it claims to have.

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
    dyno = (state or {}).get("dyno")
    if dyno is not None:
        print("dyno: %d exchanges went unanswered" % dyno.misses)
        dyno.close()
