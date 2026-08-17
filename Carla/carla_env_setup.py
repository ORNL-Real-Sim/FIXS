"""Deprecated location AND name. This is now cosim/env_setup.py (#313).

Two changes in one file: it moved out of Carla/, and it lost the `carla_` prefix,
because it owns the python environment as much as it owns the CARLA paths - the
env half is what every entry point re-execs through, on machines with no CARLA at
all.

Kept because FIXS_Applications' run_cosim.bat/.sh run `FIXS/Carla/carla_env_setup.py`
for `--setup` and `--update-python`, and those front doors are committed in repos
we do not control. See Carla/run_cosim.py for why this forwards both as a script
and as a module.

Nothing inside FIXS routes through here: the CARLA scripts that used to import it
as a sibling now import env_setup directly, so deleting this file costs nothing
internally.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _cosim_path                                     # noqa: E402

if __name__ == "__main__":
    import runpy
    _cosim_path.ensure()
    runpy.run_module("env_setup", run_name="__main__", alter_sys=True)
else:
    sys.modules[__name__] = _cosim_path.load("env_setup")
