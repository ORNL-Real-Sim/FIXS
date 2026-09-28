"""Shared by every test under tests/.

No test opens a window. A test whose code path falls through to a real file
picker (import_map._select_package) put a Tk dialog on the desktop of whoever ran
the suite, and the run sat on it until they found it and cancelled. Nothing here
needs a real Tk: code that asks a person is tested by stubbing the asking, so any
Tk() is a test escaping its stubs - refuse it loudly instead.
"""
import pytest


@pytest.fixture(autouse=True)
def _no_windows(monkeypatch):
    try:
        import tkinter
    except ImportError:
        return

    def _refuse(*_a, **_k):
        raise RuntimeError("a test tried to open a Tk window; stub the prompt instead")

    monkeypatch.setattr(tkinter, "Tk", _refuse)
