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
#  A DYNAMOMETER.  You decide a speed, the cell says what a real vehicle
#  reached, and THAT is what gets commanded -- so the cell is in the loop
#  rather than beside it. Next step you read ego.speed back.
#
#  For REAL hardware, replace the body of exchange(). The packet, the port and
#  the rate are yours, and FIXS has no interface for them. Three things bite:
#  NEVER BLOCK -- send, then take an answer that has ALREADY arrived; when
#  none has, return vref, the only value that cannot invent motion; and COUNT
#  those, because nothing else will and a run ending with many of them did not
#  test what it claims to have tested.
# --------------------------------------------------------------------------- #
# import fixs.xil
#
# dyno = fixs.xil.dyno(vehicle={'mass_kg': 2100.0})   # or your own rig
#
# def exchange(vref, dt):
#     return dyno.exchange(vref, dt)                  # a speed in, reached out
#
# Driver = fixs.driver(exchange)      # the driver calls it at the right point


# --------------------------------------------------------------------------- #
#  YOUR OWN DRIVING.  None of the driver's logic runs -- no ceilings, no pedal
#  law, no agent. FIXS still builds the class and calls it every step, so you
#  write a function, never __init__ or a method named control.
#
#  A DYNAMOMETER STILL WORKS HERE: you call it yourself, wherever you want it
#  in your own logic. What you cannot do is hand fixs.driver() an exchange as
#  well -- there is no point left in the tick for it to be called from, so it
#  is refused rather than silently ignored.
#
#  ego.speed / positionX / positionY / heading / acceleration are LIVE.
#  speedDesired, signalLightColor, precedingVehicleDistance were HELD since
#  the last feed, ego.feedAge seconds ago.
#
#  speedDesired IS DUAL-USE: the advisory arrives in it, and a speed command
#  goes back out through it. Read it only when ego.feedAge is 0 -- between
#  feeds you are reading back your own last command. (Measured before that was
#  understood: 1299 of 2554 ticks matched the previous tick's own target.)
#
#  steerAngleDesired IS AN ANGLE, in radians, where a CARLA agent's
#  control.steer is normalised [-1, 1]. Commanding through fixs.carla does the
#  conversion in the one place it belongs; ego.set(...) is the FIXS-native
#  form and expects radians.
# --------------------------------------------------------------------------- #
# import fixs.carla as carla
# import fixs.xil
#
# dyno = fixs.xil.dyno(vehicle={'mass_kg': 2100.0})
#
# def my_control(ego, dt):
#     target = ego.speedDesired if ego.feedAge == 0 else 8.33
#     target = dyno.exchange(target, dt)        # your cell, called by YOU
#     carla.apply_ackermann_control(
#         carla.VehicleAckermannControl(speed=max(0.0, target), steer=0.0))
#
# Driver = fixs.driver(usercontrol=my_control)


#: fixs.driver() tells FIXS what it built, so this name is yours to pick.
Driver = fixs.driver()
