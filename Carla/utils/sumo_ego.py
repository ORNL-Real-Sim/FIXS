"""sumo_ego.py - the command line for fixs.sumo.ego.

The implementation moved to CommonLib/fixs/sumo/ego.py so applications can call
it directly:

    import fixs
    cfg = fixs.sumo.build_ego_scenario(bundle_cfg, run_dir, route_from="route1",
                                       depart=29100)

rather than locate this file, shell out to it, and read the generated config off
the last line of its stdout - which is what every caller was doing.

This stays, at this path, doing exactly what it did: same options, same output,
same last line. Scripts and applications that shell out keep working unchanged,
including apps/mlk_eco_driving's preserved reference controller, which hardcodes
this path on purpose. The names other tools import from here - sumo_route_points
takes route_from_bundle - are re-exported below.

Run `python sumo_ego.py --help` for the options; they are declared in ego.py.
"""
import sys
from pathlib import Path

# The repo ROOT, not CommonLib: `fixs` imports CommonLib.ConfigHelper by that
# name, so CommonLib has to be importable as a package. This file is
# Carla/utils/sumo_ego.py, so the root is two levels up.
#
# Going through the package means this command now needs PyYAML, which
# fixs/__init__ pulls in via ConfigHelper -- ego.py itself is still stdlib only.
# FIXS requires PyYAML everywhere else already, and one import path for one
# module is worth more than the CLI being independently installable.
# BOTH: `fixs` is a package inside CommonLib, so CommonLib must be on the path
# to import it -- and fixs/__init__ imports CommonLib.ConfigHelper by that name,
# so the repo root must be there too. The same pair every application inserts.
_ROOT = Path(__file__).resolve().parents[2]
for _p in (_ROOT, _ROOT / "CommonLib"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from fixs.sumo import ego  # noqa: E402  (after the path insert, necessarily)
from fixs.sumo.ego import (  # noqa: E402,F401  re-exported for importers
    build_ego_routes,
    build_sumocfg,
    demand_file_with,
    generate,
    inject_ego,
    route_from_bundle,
    vtype_from_bundle,
)

main = ego.main

if __name__ == "__main__":
    sys.exit(main())
