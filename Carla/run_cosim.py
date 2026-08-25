"""Deprecated location. The co-sim orchestrator moved to cosim/run_cosim.py (#313).

Kept because this path is the published entry point for everything released
before the move: FIXS_Applications' run_cosim.bat/.sh exec
`FIXS/Carla/run_cosim.py`, and apps/atlanta/README.md tells users to run it by
hand. Those front doors are committed in repos we do not control, so the path has
to keep working until they have all moved to FIXS.bat / FIXS.sh.

Forwards two ways on purpose. `python FIXS/Carla/run_cosim.py ...` has to run the
real module as __main__, while `import run_cosim` has to yield the real module
OBJECT - a runpy-only shim satisfies the first and hands the second an empty
module. Rebinding sys.modules keeps one object per file, so module-level state is
shared rather than silently duplicated.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _cosim_path                                     # noqa: E402

if __name__ == "__main__":
    import runpy
    _cosim_path.ensure()
    # alter_sys so the real module sees its own __file__: it re-execs itself under
    # the configured interpreter and would otherwise relaunch this shim forever.
    runpy.run_module("run_cosim", run_name="__main__", alter_sys=True)
else:
    sys.modules[__name__] = _cosim_path.load("run_cosim")
