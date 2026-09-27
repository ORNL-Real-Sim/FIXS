"""Drive the FIXS ego. Copy this next to your application; it runs as it is.

    EgoSetup:
      Dynamics: virenv            # the virtual environment moves the ego
      ActuationSource: user       # a controller produces the pedals and steer
      Controller: apps/<your_app>/my_driver.py --command-shape pedals

As written you get the driver FIXS ships: the eco advisory read off the wire,
the signal and leader ceilings, a speed-to-pedal law and both command shapes.

TO CHANGE IT, uncomment one of the blocks below, then move the '#' among the
three Driver lines at the end so the matching one is live. Only ever one:
two drivers and FIXS refuses to guess which you meant.

EMBEDDED ONLY: FIXS imports this once and calls it every CARLA step. A control
law served at the 0.1 s feed would read the advisory back as its own measured
speed, so FIXS refuses that combination. For a client on the feed, see the
application repo's apps/_template.
"""
import fixs


# --------------------------------------------------------------------------- #
#  A DYNAMOMETER.  You decide a speed, the cell says what a real vehicle
#  reached, and THAT is what gets commanded -- so the cell is in the loop
#  rather than beside it. Next step you read ego.speed back.
#
#  Uncomment it, then switch the Driver line at the end of the file to
#  the one that names it.
#
#  For REAL hardware, replace the body. The packet, the port and the rate are
#  yours, and FIXS has no interface for them. Three things bite: NEVER BLOCK
#  -- send, then take an answer that has ALREADY arrived; when none has,
#  return vref, the only value that cannot invent motion; and COUNT those,
#  because nothing else will.
# --------------------------------------------------------------------------- #
# import fixs.xil
#
# dyno = fixs.xil.dyno(vehicle={'mass_kg': 2100.0})   # or your own rig
#
# def exchange(vref, dt):
#     return dyno.exchange(vref, dt)                  # a speed in, reached out


# --------------------------------------------------------------------------- #
#  YOUR OWN DRIVING.  None of the driver's logic runs -- no ceilings, no pedal
#  law, no agent. FIXS still builds the class and calls it every step, so you
#  write a function, never __init__ or a method named control.
#
#  Uncomment it, then switch the Driver line at the end of the file to
#  the one that names it.
#
#  A DYNAMOMETER STILL WORKS: uncomment the block above too and call it inside
#  my_control, wherever you want it -- `target = dyno.exchange(target, dt)`.
#  What you cannot do is pass BOTH to fixs.driver(): replacing the driving
#  leaves no point in the tick for an exchange to be called from, so that is
#  refused rather than silently ignored.
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
#  control.steer is normalised [-1, 1]. Commanding through fixs.carla does
#  that conversion in the one place it belongs; ego.set(...) is the
#  FIXS-native form and expects radians.
# --------------------------------------------------------------------------- #
# import fixs.carla as carla
#
# def my_control(ego, dt):
#     target = ego.speedDesired if ego.feedAge == 0 else 8.33
#     carla.apply_ackermann_control(
#         carla.VehicleAckermannControl(speed=max(0.0, target), steer=0.0))


#: PICK ONE -- exactly one of these three is live at a time. Uncomment the
#: block above that defines what you name here, then swap which line below
#: carries the '#'. The name on the left is yours: fixs.driver() tells FIXS
#: what it built, so nothing here has to be spelled a particular way.
Driver = fixs.driver()                          # the driver as it is
# Driver = fixs.driver(exchange)                # ... with a dynamometer
# Driver = fixs.driver(usercontrol=my_control)  # your own driving
