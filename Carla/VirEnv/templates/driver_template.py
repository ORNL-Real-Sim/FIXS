"""Drive the FIXS ego. Copy this next to your application.

    EgoSetup:
      Dynamics: virenv            # the virtual environment moves the ego
      ActuationSource: user       # a controller produces the pedals and steer
      Controller: apps/<your_app>/my_driver.py --command-shape pedals

EMBEDDED ONLY. FIXS imports that file once and calls it every CARLA step. A
control law served at the 0.1 s feed instead would read the advisory back as
its own measured speed, so FIXS refuses that combination -- if you want a
client on the feed, that is a different shape entirely; see the application
repo's apps/_template.

Three ways to use this, in the order you should reach for them.


1. THE DRIVER AS IT IS
----------------------
::

    import fixs
    Driver = fixs.driver()

You get the eco advisory read off the wire, the signal and leader ceilings, a
speed-to-pedal law and both command shapes. Nothing else to write.


2. WITH YOUR DYNAMOMETER
------------------------
::

    import fixs
    import fixs.xil

    dyno = fixs.xil.dyno(vehicle={'mass_kg': 2100.0})   # or your own rig

    def exchange(vref, dt):
        return dyno.exchange(vref, dt)     # a speed in, the speed reached out

    Driver = fixs.driver(exchange)

The driver calls it after every decision and before it commands anything, so
the cell is in the loop rather than beside it. For REAL hardware, replace that
body -- the packet, the port and the rate are yours, and FIXS has no interface
for them. Three things bite: never block (send, then take an answer that has
already arrived); when none has, return `vref`, the only value that cannot
invent motion; and count those, because nothing else will.


3. YOUR OWN DRIVING
-------------------
::

    import fixs

    def my_control(ego, dt):
        ...
        ego.set(speedDesired=target, steerAngleDesired=0.0)

    Driver = fixs.driver(usercontrol=my_control)

Nothing of the driver's runs -- no ceilings, no pedal law, no agent. FIXS
still builds the class and calls it every step, so you never write __init__ or
a method named control, and the name `Driver` is yours to pick: fixs.driver()
tells FIXS what it built.

Ask your own cell inside that function, wherever you want it.


WHAT `ego` GIVES YOU, AND THE TWO THINGS THAT CATCH PEOPLE
----------------------------------------------------------
LIVE every call : positionX, positionY, heading, speed, acceleration
HELD since the last feed, `ego.feedAge` seconds ago:
                  speedDesired, signalLightColor, precedingVehicleDistance

`speedDesired` IS DUAL-USE. The traffic simulator's advisory arrives in it on
the feed, and your command goes out through it every step. The bridge restores
the feed's value before each call for exactly that reason -- but read it only
when `ego.feedAge` is 0, or on the ticks between feeds you are reading back
your own last command. Measured on a 300 s run before that was understood:
1299 of 2554 ticks matched the previous tick's own target to within 1e-3.

`steerAngleDesired` IS AN ANGLE, in radians. A CARLA agent's `control.steer`
is normalised [-1, 1]; multiply by fixs.MAX_STEER_RAD, or command through
`fixs.carla.apply_control`, which converts in the one place that belongs.
"""
