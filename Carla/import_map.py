"""Deprecated location. The map importer moved to cosim/import_map.py (#313).

Kept because FIXS_Applications' run_cosim.bat/.sh run `FIXS/Carla/import_map.py`
for `--import-map`, and those front doors are committed in repos we do not
control. See Carla/run_cosim.py for why this forwards both as a script and as a
module.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _cosim_path                                     # noqa: E402

if __name__ == "__main__":
    import runpy
    _cosim_path.ensure()
    runpy.run_module("import_map", run_name="__main__", alter_sys=True)
else:
    sys.modules[__name__] = _cosim_path.load("import_map")
