"""Drive the FIXS ego. Copy this next to your application and run it.

    EgoSetup:
      Dynamics: virenv            # the virtual environment moves the ego
      ActuationSource: user       # a controller produces the pedals and steer
      Controller: apps/<your_app>/my_driver.py --command-shape pedals

EMBEDDED ONLY: FIXS imports this once and calls it every CARLA step. A control
law served at the 0.1 s feed would read the advisory back as its own measured
speed, so FIXS refuses that combination. For a client on the feed instead, see
the application repo's apps/_template.

As written this is a working driver -- the eco advisory read off the wire, the
signal and leader ceilings, a speed-to-pedal law and both command shapes. The
two blocks below add a dynamometer, or replace the driving with your own.
"""
import fixs


# --------------------------------------------------------------------------- #
#  1. A DYNAMOMETER.  Uncomment, and the driver asks it after every decision
#     and before it commands anything -- so the cell is in the loop rather
#     than beside it. Next step the driver reads ego.speed back.
#
#     For REAL hardware, replace the body: the packet, the port and the rate
#     are yours, and FIXS has no interface for them. Three things bite.
#     NEVER BLOCK -- send, then take an answer that has ALREADY arrived. When
#     none has, return vref: it is the only value that cannot invent motion.
#     COUNT those, because nothing else will, and a run ending with many of
#     them did not test what it claims to have tested.
# --------------------------------------------------------------------------- #
# import fixs.xil
#
# dyno = fixs.xil.dyno(vehicle={'mass_kg': 2100.0})   # or your own rig
#
# def exchange(vref, dt):
#     return dyno.exchange(vref, dt)                  # a speed in, reached out


# --------------------------------------------------------------------------- #
#  2. YOUR OWN DRIVING.  Uncomment, and none of the driver's logic runs -- no
#     ceilings, no pedal law, no agent. FIXS still builds the class and calls
#     it every step, so you write a function, never __init__ or a method
#     named control. Ask your own cell inside it, wherever you want it.
#
#     ego.speed / positionX / positionY / heading / acceleration are LIVE.
#     speedDesired, signalLightColor, precedingVehicleDistance were HELD since
#     the last feed, ego.feedAge seconds ago.
#
#     speedDesired IS DUAL-USE: the advisory arrives in it, and your command
#     goes out through it. Read it only when ego.feedAge is 0 -- between feeds
#     you are reading back your own last command. (Measured before that was
#     understood: 1299 of 2554 ticks matched the previous tick's own target.)
#
#     steerAngleDesired IS AN ANGLE, in radians. A CARLA agent's control.steer
#     is normalised [-1, 1]; command through fixs.carla.apply_control, which
#     converts in the one place that belongs, or multiply by fixs.MAX_STEER_RAD.
# --------------------------------------------------------------------------- #
# def my_control(ego, dt):
#     target = ego.speedDesired if ego.feedAge == 0 else 8.33
#     ego.set(speedDesired=target, steerAngleDesired=0.0)


#: fixs.driver() tells FIXS what it built, so this name is yours to pick.
#: Add exchange=... from block 1, or usercontrol=... from block 2 -- not both:
#: your own control decides when to ask a cell, so ours would never call it.
Driver = fixs.driver()
