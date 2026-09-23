"""The FIXS window (run_cosim --gui, #78): a front end over run_cosim.

core.py is the toolkit-free half - which flags a form becomes, how a run is
started, read and stopped. app.py is the Qt (PySide6) half on top of it.
"""
import os


def prefer_system_icu():
    """Make Qt find Windows' own ICU, not a conda env's.

    Qt6Core.dll in the PySide6 wheels imports ICU by its UNVERSIONED names
    (ucnv_open) from icuuc.dll - the ICU Windows ships in System32. A conda env
    that has the `icu` package (libxml2 pulls it in) puts ICU 75 in
    Library\\bin, which conda's python adds to the DLL search path, and that
    icuuc.dll exports only VERSIONED names (ucnv_open_75). The loader finds it
    first, and every `from PySide6 import QtCore` dies with "DLL load failed ...
    The specified procedure could not be found" - measured on both FIXS envs
    here (realsim_dev, fixs_applications).

    Loading the System32 copies first settles it: the loader resolves a DLL name
    that is already loaded to that module. Scoped to the window's own process,
    which imports nothing that wants ICU 75; every run it starts is a separate
    process and loads what it always has.

    Called by app.py right before its first PySide6 import - not at package
    import, so importing gui.core (the engine side, the tests) loads nothing
    native - and only when PySide6 is installed at all."""
    import importlib.util
    if os.name != "nt" or importlib.util.find_spec("PySide6") is None:
        return
    import ctypes
    system32 = os.path.join(os.environ.get("SystemRoot", r"C:\Windows"), "System32")
    for name in ("icuuc.dll", "icuin.dll"):
        path = os.path.join(system32, name)
        if os.path.isfile(path):
            try:
                ctypes.WinDLL(path)
            except OSError:
                pass

