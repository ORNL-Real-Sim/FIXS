"""fixs.sumo -- this run's SUMO scenario, as one call.

An application that runs against a Digital-Twin-Library map has to turn the
scenario run_cosim handed it into one SUMO can be started on, before anything
else exists. That means inserting an ego the map deliberately does not ship,
applying whatever this experiment changes about the vehicle types, pointing the
run's outputs at the run's own directory, and telling run_cosim what came out.

Every part of that was being re-implemented per application. mlk_eco_driving
carried about 550 lines of it, including a fourth copy of "read net-file out of
a sumocfg" and a subprocess call that scraped sumo_ego's stdout for its answer.

    import fixs

    cfg = fixs.sumo.scenario(
        source_cfg, run_dir,
        ego=APP_DIR / "ego.rou.xml",
        inject=demand_copy,
        vtypes={"CAV": {"probability": mpr}, "HDV": {"probability": 1 - mpr}},
        outputs=("summary-output", "tripinfo-output", "fcd-output", "log"))

    # cfg.path is what SUMO runs; it has already been reported to FIXS_HANDOFF.

`scenario()` works out whether it was handed a map bundle with no ego or a
scenario already prepared for this app -- run_cosim passes whatever the user
chose and cannot tell them apart, the scenario itself can -- and an application
should not have to ask.

WHAT AN APPLICATION SAYS, and nothing more: where its ego is (a SUMO file, read
and injected, never handed to SUMO as a route file), what this run changes about
the map's vehicle types, and which outputs it wants. The run's end time comes
from SimulationSetup.SimulationEndTime, which TrafficLayer already owns. SUMO
options a run deviates with belong in apps.json `sumo_args`, which run_cosim
passes on the command line and prints with its origin -- and the command line
beats a generated config, so an option written into the scenario by an app is
the weaker of the two places for it.

The other names here answer questions about a .sumocfg. Nothing in tree calls
them now that scenario() exists; they are public because an application driving
the pieces itself should not have to re-derive them:

    has_ego(cfg)        does this scenario already define that vehicle
    net_file(cfg)       which network it opens
    route_files(cfg)    which route files it names, resolved
    ego_from_file(path) an ego .rou.xml as build options
    build_ego_scenario  insert an ego; what scenario() calls for a bundle
    set_run_settings    write end / step-length / seed / teleport into a config

The command line is still there too. Carla/utils/sumo_ego.py drives ego.py
exactly as before, prints what it always printed, and is what a shell script
should keep using.

`fixs.sumo` is a package rather than a module so ORNL-Real-Sim/FIXS#356 can add
`import fixs.sumo.traci as traci` beside this without moving anything.
"""
from __future__ import annotations

from .build import (
    Scenario,
    build_ego_scenario,
    ego_from_file,
    has_ego,
    net_file,
    route_files,
    scenario,
    set_run_settings,
)

__all__ = ["scenario", "Scenario", "ego_from_file", "build_ego_scenario",
           "has_ego", "net_file", "route_files", "set_run_settings"]
