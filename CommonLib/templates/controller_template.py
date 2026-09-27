"""A FIXS application controller, in the smallest form that works.

THIS IS NOT THE EGO DRIVER. The two templates are different jobs:

    CommonLib/templates/controller_template.py   <- this one. Your APPLICATION.
        A TrafficLayer client in its own process. Each tick it is handed the
        world, decides something -- an eco-driving advisory, a ramp-meter rate,
        a platoon's spacing -- and writes the decision back. Needs no CARLA.

    Carla/templates/driver_template.py           the EGO DRIVER.
        Loaded in-process by the CARLA bridge and asked for pedals and steer
        every CARLA step, when CARLA's physics move the ego. See fixs.driver().

An application can have both, one, or -- most often -- only this one.

Copy this next to your application, edit decide(), and name it in the app
manifest your repo keeps (apps/apps.json in an app repo laid out for run_cosim)::

    { "id": "my_app", "launch": "controller.py", ... }

What FIXS requires of a controller is only what main() does below: connect, then
one recv/send per tick until FIXS says the run is over. TrafficLayer is the
traffic simulator's only client, not you; it steps the simulator once per
exchange and does not advance until every subscriber has answered.
"""
import sys
from pathlib import Path

# `import fixs` comes from the FIXS bundle, which is fetched rather than
# pip-installed. This assumes the layout run_cosim expects -- your controller at
# <repo>/apps/<your_app>/, the bundle at <repo>/FIXS/ -- so adjust the two roots
# if yours differs. CommonLib so `fixs` resolves; FIXS/ so the `CommonLib.*`
# imports that fixs itself makes resolve too.
REPO_ROOT = Path(__file__).resolve().parents[2]
FIXS_ROOT = REPO_ROOT / "FIXS"
APP_DIR = Path(__file__).resolve().parent
for _p in (REPO_ROOT, FIXS_ROOT, FIXS_ROOT / "CommonLib", APP_DIR):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

import fixs  # noqa: E402


# --------------------------------------------------------------------------- #
#  YOUR DECISION.  Everything outside this block is the FIXS contract.
# --------------------------------------------------------------------------- #

#: What the placeholder below holds [m/s] -- about 30 mph. A constant, so it is
#: obvious that nothing here is deciding anything yet.
PLACEHOLDER_SPEED = 13.4


def decide(ego):
    """Return the speed [m/s] to ask the ego to hold this tick.

    THIS IS A PLACEHOLDER. It holds one constant and ignores the world; replace
    it. It is safe to leave in place while you write something real only because
    the traffic simulator still applies its own safety checks under the scenario
    this ships with (SumoSetup.SpeedMode 31).

    `ego` carries the fields SimulationSetup.VehicleMessageField declares in the
    scenario yaml, and nothing else. Typically available:

        ego.speed                     m/s, now
        ego.acceleration              m/s^2
        ego.signalLightColor          next signal: 1 red, 2 yellow, 4 red-yellow
        ego.signalLightDistance       m to its stop bar
        ego.hasPrecedingVehicle       is there a leader
        ego.precedingVehicleDistance  m to it, or -1.0 for none in range
        ego.precedingVehicleSpeed     m/s

    A field not listed in that yaml is never decoded and reads back a default,
    with no error anywhere -- so if a value looks stuck at zero, check the
    spelling there before you debug this function.
    """
    return PLACEHOLDER_SPEED
# --------------------------------------------------------------------------- #


def main():
    # Not required -- but if run_cosim did not start us, nothing started the
    # traffic simulator or TrafficLayer either and there is nothing to connect
    # to. Saying so here beats a connect that retries forever against a port no
    # one is listening on.
    #
    # fixs.launch carries what run_cosim told this process: .supervised,
    # .sumocfg (--sumocfg, or None), .configPath (the scenario yaml) and .carla.
    # Read them from there rather than from os.environ -- the variable names are
    # FIXS's, and an application that spells them out is a second place they are
    # written down.
    if not fixs.launch.supervised:
        raise SystemExit(
            "Nothing started this run, so there is no TrafficLayer to talk to.\n"
            "  Start the co-simulation instead, and pick this app:\n"
            "      run_cosim.bat        (run_cosim.sh on Linux)")

    # No path: connect() takes the scenario yaml run_cosim gave TrafficLayer, so
    # the two cannot disagree about the wire format. No port either: it picks
    # this client's subscription out of that yaml.
    host, port = fixs.connect(connectTimeout=None, recvTimeout=None)
    print(f"[app] connected to FIXS at {host}:{port}")

    try:
        while fixs.running:
            fixs.recv()                      # one tick in

            ego = fixs.vehicle.get("ego")
            if ego is not None:
                ego.set(speedDesired=decide(ego))

            fixs.send()                      # one tick out -- EVERY tick
    except fixs.Shutdown:
        print(f"[app] run ended at t={fixs.sim.time:g}")


if __name__ == "__main__":
    main()
