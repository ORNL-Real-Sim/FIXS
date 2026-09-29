"""The package picker never opens a window for a session nobody is at.

_select_package used to open its Tk file dialog first and check for a terminal
only after it closed. A caller with nobody at the keyboard - a script, or a test
whose ensure_map fell through to a real import - put the dialog on the desktop
and hung on it until someone found the window and cancelled.
"""
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "..", "Carla"))
import import_map  # noqa: E402


@pytest.fixture
def tk_calls(monkeypatch):
    """Records every Tk() instead of opening one."""
    import tkinter
    calls = []

    def _record(*a, **k):
        calls.append(a)
        raise RuntimeError("recorded, not opened")

    monkeypatch.setattr(tkinter, "Tk", _record)
    return calls


@pytest.mark.parametrize("precooked", [False, True])
def test_no_terminal_exits_before_any_window(tk_calls, monkeypatch, precooked):
    monkeypatch.setattr(sys.stdin, "isatty", lambda: False, raising=False)
    with pytest.raises(SystemExit) as e:
        import_map._select_package("RP_Ver0529", None, precooked=precooked)
    assert "--package-dir" in str(e.value.code)
    assert tk_calls == []


def test_a_terminal_still_gets_the_picker(tk_calls, monkeypatch):
    """The guard moved; it did not take the dialog away from a person."""
    monkeypatch.setattr(sys.stdin, "isatty", lambda: True, raising=False)
    monkeypatch.setattr(import_map, "_prompt", lambda *a, **k: "")
    with pytest.raises(SystemExit):          # picker refused, empty typed path
        import_map._select_package("RP_Ver0529", None)
    assert len(tk_calls) == 1


def test_suite_refuses_real_windows():
    """tests/conftest.py: a Tk() that escapes a test's stubs fails the test
    instead of reaching the desktop."""
    import tkinter
    with pytest.raises(RuntimeError, match="stub the prompt"):
        tkinter.Tk()
