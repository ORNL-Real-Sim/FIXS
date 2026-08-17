"""Deprecated location. The env/CARLA setup moved to cosim/carla_env_setup.py (#313).

Kept because FIXS_Applications' run_cosim.bat/.sh run `FIXS/Carla/carla_env_setup.py`
for `--setup` and `--update-python`, and those front doors are committed in repos
we do not control. See Carla/run_cosim.py for why this forwards both as a script
and as a module.

Nothing inside FIXS should route through here: the CARLA scripts that used to
import this as a sibling call _cosim_path.ensure() and get the real module
directly, so deleting this file later costs nothing internally.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _cosim_path                                     # noqa: E402

if __name__ == "__main__":
    import runpy
    _cosim_path.ensure()
    runpy.run_module("carla_env_setup", run_name="__main__", alter_sys=True)
else:
    sys.modules[__name__] = _cosim_path.load("carla_env_setup")
