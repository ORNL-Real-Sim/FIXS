"""run_cosim's 'e) edit in your editor': which program actually gets the yaml (#405).

On one user's Ubuntu desktop xdg-open handed the yaml to GNOME's autorun helper for
removable media - the MIME default for application/yaml - which exited at once and
left the prompt waiting for an editor that never opened. The launcher now asks for
the text/plain default and a known editor before it ever falls back to xdg-open.

Nothing is launched: every process call is recorded instead, so these run anywhere.
"""
import os
import subprocess
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "..", "Carla"))
import run_cosim  # noqa: E402

YAML = "/home/u/.fixs/apps/demo/config.yaml"


class _Proc:
    def __init__(self, code):
        self.code = code

    def wait(self, timeout=None):
        if self.code is None:
            raise subprocess.TimeoutExpired("gtk-launch", timeout)
        return self.code


@pytest.fixture
def linux(monkeypatch):
    """A Linux desktop with nothing installed and no editor variables. Returns the
    call log and the knobs a test turns: which programs exist, what xdg-mime says
    the text/plain default is, and how gtk-launch exits (None = still running)."""
    state = {"calls": [], "on_path": set(), "text_plain": "", "gtk_exit": 0}

    monkeypatch.setattr(run_cosim.platform, "system", lambda: "Linux")
    for var in ("VISUAL", "EDITOR", "WAYLAND_DISPLAY"):
        monkeypatch.delenv(var, raising=False)
    monkeypatch.setenv("DISPLAY", ":0")
    monkeypatch.setattr(run_cosim.shutil, "which",
                        lambda p: f"/usr/bin/{p}" if p in state["on_path"] else None)

    def run(argv, **kw):
        state["calls"].append(("run", argv))
        return subprocess.CompletedProcess(argv, 0, stdout=state["text_plain"] + "\n")

    def popen(argv, **kw):
        state["calls"].append(("popen", argv))
        return _Proc(state["gtk_exit"] if argv[0] == "gtk-launch" else None)

    def call(argv, **kw):
        state["calls"].append(("call", argv))
        return 0

    monkeypatch.setattr(run_cosim.subprocess, "run", run)
    monkeypatch.setattr(run_cosim.subprocess, "Popen", popen)
    monkeypatch.setattr(run_cosim.subprocess, "call", call)
    return state


def _launched(state):
    """Every program started on the yaml, in order, as (how, argv)."""
    return [(how, argv) for how, argv in state["calls"] if how != "run"]


def test_the_yaml_mime_default_is_not_consulted(linux):
    """The reported failure: a desktop whose text/plain default is an editor gets
    that editor, and xdg-open - which resolves the yaml's own MIME type - is never
    run."""
    linux["on_path"] |= {"xdg-mime", "gtk-launch", "xdg-open"}
    linux["text_plain"] = "org.gnome.TextEditor.desktop"
    assert run_cosim._open_in_editor(YAML) == ("org.gnome.TextEditor", False)
    assert _launched(linux) == [("popen", ["gtk-launch", "org.gnome.TextEditor.desktop", YAML])]


def test_a_known_editor_when_text_plain_has_no_default(linux):
    linux["on_path"] |= {"xdg-mime", "gtk-launch", "gedit", "code", "xdg-open"}
    assert run_cosim._open_in_editor(YAML) == ("gedit", False)
    assert _launched(linux) == [("popen", ["gedit", YAML])]


def test_a_known_editor_when_gtk_launch_fails(linux):
    linux["on_path"] |= {"xdg-mime", "gtk-launch", "mousepad"}
    linux["text_plain"] = "gone.desktop"
    linux["gtk_exit"] = 1
    assert run_cosim._open_in_editor(YAML) == ("mousepad", False)
    assert _launched(linux)[-1] == ("popen", ["mousepad", YAML])


def test_gtk_launch_still_running_counts_as_launched(linux):
    linux["on_path"] |= {"xdg-mime", "gtk-launch", "gedit"}
    linux["text_plain"] = "org.gnome.gedit.desktop"
    linux["gtk_exit"] = None
    assert run_cosim._open_in_editor(YAML) == ("org.gnome.gedit", False)


def test_a_desktop_with_no_gui_editor_gets_a_terminal_one_before_xdg_open(linux):
    linux["on_path"] |= {"nano", "xdg-open"}
    assert run_cosim._open_in_editor(YAML) == ("nano", True)
    assert _launched(linux) == [("call", ["nano", YAML])]


def test_xdg_open_is_the_last_resort(linux):
    linux["on_path"] |= {"xdg-open"}
    assert run_cosim._open_in_editor(YAML) == ("xdg-open", False)


def test_no_display_goes_straight_to_the_terminal(linux, monkeypatch):
    monkeypatch.delenv("DISPLAY")
    linux["on_path"] |= {"xdg-mime", "gtk-launch", "gedit", "vi"}
    linux["text_plain"] = "org.gnome.gedit.desktop"
    assert run_cosim._open_in_editor(YAML) == ("vi", True)


def test_editor_variable_is_split_and_waited_on(linux, monkeypatch):
    """`code -w` is a command line. And $EDITOR is waited on: a terminal editor
    left in the background reads the same keystrokes as the Enter prompt."""
    monkeypatch.setenv("EDITOR", "code -w")
    linux["on_path"] |= {"xdg-mime", "gtk-launch", "gedit"}
    assert run_cosim._open_in_editor(YAML) == ("code", True)
    assert _launched(linux) == [("call", ["code", "-w", YAML])]


def test_visual_wins_over_editor(linux, monkeypatch):
    monkeypatch.setenv("VISUAL", "vim")
    monkeypatch.setenv("EDITOR", "nano")
    assert run_cosim._open_in_editor(YAML) == ("vim", True)


def test_nothing_to_launch_is_reported(linux, monkeypatch):
    monkeypatch.delenv("DISPLAY")
    assert run_cosim._open_in_editor(YAML) is None


# --------------------------------------------------------------------------- #
# The prompt around it
# --------------------------------------------------------------------------- #
def _answers(monkeypatch, *replies):
    it = iter(replies)
    monkeypatch.setattr(run_cosim, "_ask", lambda prompt, default=None: next(it))


def test_a_waited_editor_is_reopened_to_fix_a_bad_edit(tmp_path, monkeypatch):
    """A terminal editor has closed by the time the file is re-read; telling the
    user to 'fix it and press Enter' would leave them with nothing to fix it in."""
    path = tmp_path / "config.yaml"
    path.write_text("CarlaSetup:\n  CarlaTimeStep: 0.03\n", encoding="utf-8")
    opens = []
    monkeypatch.setattr(run_cosim, "_open_in_editor",
                        lambda p: opens.append(p) or ("nano", True))
    monkeypatch.setattr(run_cosim, "_read_scenario_config", lambda p: {})
    _answers(monkeypatch, "", "", "q")      # re-read -> FAIL, reopen, then leave it
    run_cosim.edit_yaml_in_editor(str(path))
    assert opens == [str(path), str(path)]


def test_a_gui_editor_is_not_reopened(tmp_path, monkeypatch, capsys):
    path = tmp_path / "config.yaml"
    path.write_text("CarlaSetup:\n  CarlaTimeStep: 0.03\n", encoding="utf-8")
    opens = []
    monkeypatch.setattr(run_cosim, "_open_in_editor",
                        lambda p: opens.append(p) or ("gedit", False))
    monkeypatch.setattr(run_cosim, "_read_scenario_config", lambda p: {})
    _answers(monkeypatch, "", "q")
    run_cosim.edit_yaml_in_editor(str(path))
    assert opens == [str(path)]
    out = capsys.readouterr().out
    assert "editor: gedit" in out and "Fix it and press Enter" in out
