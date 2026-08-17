"""Put the co-sim modules on sys.path for the CARLA component.

`carla_env_setup`, `import_map` and friends used to sit in this directory, so the
CARLA scripts imported them as plain siblings. #313 moved them out - they are not
CARLA code - and a flat `import carla_env_setup` no longer resolves.

Rather than teach five scripts the same three lines, they call ensure() once at
import time. That also keeps them independent of the deprecation shims next door:
Carla/carla_env_setup.py and Carla/import_map.py exist for front doors and docs
committed before the move and are meant to be deleted, so nothing inside FIXS
should quietly depend on them still being there.

The lookup deliberately does NOT use fixs_paths - that module is one of the things
being located. It does not need to: Carla/ did not move, so the FIXS root is its
parent in both layouts (<fixs>/Carla and <app>/FIXS/Carla). Only the co-sim folder
sits at a different place in the two, which is what the candidate list covers.
"""
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_FIXS_ROOT = os.path.dirname(_HERE)

# Unpacked release first, then the source checkout - 8_create_zip.ps1 hoists
# scripts/cosim/ to the zip root as cosim/.
_CANDIDATES = (os.path.join(_FIXS_ROOT, "cosim"),
               os.path.join(_FIXS_ROOT, "scripts", "cosim"))


def cosim_dir():
    """The co-sim module folder, or None if this tree has neither layout."""
    for d in _CANDIDATES:
        if os.path.isdir(d):
            return d
    return None


def ensure():
    """Make the co-sim modules importable. Returns the folder used, or None.

    Inserted at the FRONT so it wins over this directory: while the deprecation
    shims live here, `import carla_env_setup` would otherwise resolve to the shim
    rather than to the module it forwards to.

    Silent when it finds nothing: the import that follows will raise, and its
    ImportError names the module the caller actually wanted."""
    d = cosim_dir()
    if d and d not in sys.path:
        sys.path.insert(0, d)
    return d


def load(name):
    """Return the real co-sim module `name`, importing it from cosim/ if needed.

    Loaded from an explicit file location rather than with `import name`, because
    the caller is typically the shim of the same name: it is already registered in
    sys.modules and still initializing, so a plain import would hand it back to
    itself. Registering before exec_module keeps that key pointing at the real
    module for anything the module imports on the way up."""
    import importlib.util

    d = cosim_dir()
    if d is None:
        raise ImportError(
            f"FIXS co-sim modules not found; looked in {' and '.join(_CANDIDATES)}. "
            f"The bundle looks incomplete - refetch it with --update-fixs.")

    mod = sys.modules.get(name)
    got = getattr(mod, "__file__", None) if mod is not None else None
    if got and os.path.dirname(os.path.abspath(got)) == d:
        return mod                       # already the real one

    ensure()                             # its siblings import each other flat
    spec = importlib.util.spec_from_file_location(name, os.path.join(d, name + ".py"))
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod
