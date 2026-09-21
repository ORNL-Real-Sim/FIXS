"""Use the driver FIXS ships, and put your dynamometer in its loop.

Copy this next to your application and point the scenario at it::

    EgoSetup:
      Dynamics: virenv
      ActuationSource: user
      Controller: apps/<your_app>/my_controller.py --command-shape pedals

``fixs.driver()`` gives you the eco advisory read off the wire, the signal and
leader ceilings, a speed-to-pedal law and both command shapes. You supply one
thing FIXS cannot: how to reach your cell.

To write the control logic yourself instead, see ``controller_template.py``.
"""
import fixs
import fixs.xil

#: THE BENCH, stated here rather than left to the yaml, so this file says what
#: is on it. Omit an argument and XilSetup.Vehicle / XilSetup.Dyno supplies it.
#: This one is simulated -- real physics, not a stub: mass, torque bandwidth
#: and road load, so it answers 0.12 m/s to a first request of 10.
dyno = fixs.xil.dyno(vehicle={'mass_kg': 2100.0},
                     dyno={'road_A_N': 111.0, 'roller_inertia_kgm2': 40.0})

#: TRUE when the cell is real hardware rather than the simulation above. The
#: two exchanges below are the only difference.
USE_RIG = False


if USE_RIG:
    import socket
    import struct

    RIG_ADDR = ('192.168.1.50', 5555)

    _sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    _sock.setblocking(False)
    _sock.bind(('', 5556))

    def exchange(vref, dt):
        """Ask the rig for a speed; answer with what it reached.

        This is the whole of what the driver needs, and none of it is FIXS's
        business -- the packet, the port and the rate are yours. Three things
        bite, whatever you write here:

        NEVER BLOCK. Send, then take an answer that has ALREADY arrived. A
        blocking read turns a co-simulation into one paced by your network.

        NO ANSWER THIS TICK -> return vref. It is the only value that cannot
        invent motion; the run then behaves as though no cell were attached.

        COUNT those, because nothing else will, and a run that ends with many
        of them did not test what it claims to have tested.
        """
        _sock.sendto(struct.pack('<2f', vref, 0.0), RIG_ADDR)
        newest = None
        while True:                   # drain; the newest answer is the true one
            try:
                data, _ = _sock.recvfrom(64)
            except BlockingIOError:
                break
            newest = data
        if newest is None:
            exchange.misses += 1
            return vref
        return struct.unpack('<2f', newest)[0]

    exchange.misses = 0

else:

    def exchange(vref, dt):
        """Ask the simulated bench for a speed; answer with what it reached."""
        return dyno.exchange(vref, dt)


#: WHERE IT SITS IN THE LOOP: after every decision the driver makes, before it
#: commands anything. Next step the driver reads ego.speed back, so the cell is
#: in the loop rather than beside it.
#:
#: fixs.driver() tells FIXS what it built, so this name is yours to pick. The
#: scenario names the FILE; the loader takes the driver from here and calls its
#: control(ego, dt) every step.
Driver = fixs.driver(exchange)
