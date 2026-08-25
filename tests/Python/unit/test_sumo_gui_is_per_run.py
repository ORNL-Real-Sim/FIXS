"""--sumo-gui overrides one run; it must never rewrite the saved setup.

The bug this pins down: both one-click front doors passed --sumo-gui on every
invocation, and _apply_cli persisted any explicit value into the run profile. So a
user who turned the SUMO GUI off in the menu (run_profile.edit_sumo) had that
choice written back to True at the very next launch, with nothing on screen to say
why. The flag predates the profile, which has defaulted sumo_gui to True on its own
since it existed - so there was never anything for a launcher to inject.

The fix splits the two directions: the CLI flag reaches args (it must, or
--no-sumo-gui would be inert), but it does not reach the record.
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__),
                                "..", "..", "..", "scripts", "cosim"))
import run_cosim  # noqa: E402


def _args(**kw):
    ns = argparse.Namespace(no_app=False, app=None, map=None, config=None,
                            engine=None, sumo_gui=None)
    for k, v in kw.items():
        setattr(ns, k, v)
    return ns


# --------------------------------------------------------------------------- #
# the setup must not move
# --------------------------------------------------------------------------- #
def test_sumo_gui_flag_does_not_touch_a_headless_setup():
    """The regression itself: a launcher passing --sumo-gui must not un-choose
    the headless setting the user picked in the menu."""
    rec = {"app": "mlk_eco_driving", "sumo_gui": False}
    run_cosim._apply_cli(rec, _args(sumo_gui=True))
    assert rec["sumo_gui"] is False


def test_no_sumo_gui_flag_does_not_touch_a_gui_setup():
    rec = {"app": "roosevelt", "sumo_gui": True}
    run_cosim._apply_cli(rec, _args(sumo_gui=False))
    assert rec["sumo_gui"] is True


def test_sumo_gui_is_not_added_to_a_setup_that_never_had_it():
    rec = {"app": "roosevelt"}
    run_cosim._apply_cli(rec, _args(sumo_gui=True))
    assert "sumo_gui" not in rec


def test_the_other_overlaid_flags_still_persist():
    """_apply_cli is still the thing that makes `--map atlanta` stick - only
    sumo_gui was carved out."""
    rec = {"app": "roosevelt", "map": "roosevelt", "engine": "py"}
    run_cosim._apply_cli(rec, _args(map="atlanta", engine="cpp"))
    assert rec["map"] == "atlanta"
    assert rec["engine"] == "cpp"


# --------------------------------------------------------------------------- #
# but the flag must still work for this run
# --------------------------------------------------------------------------- #
def test_flag_overrides_the_setup_for_this_run():
    args = _args(sumo_gui=False)
    run_cosim._rec_to_args({"sumo_gui": True}, args)
    assert args.sumo_gui is False


def test_flag_turns_the_gui_on_for_a_headless_setup():
    args = _args(sumo_gui=True)
    run_cosim._rec_to_args({"sumo_gui": False}, args)
    assert args.sumo_gui is True


def test_without_a_flag_the_setup_decides():
    args = _args()
    run_cosim._rec_to_args({"sumo_gui": False}, args)
    assert args.sumo_gui is False


def test_a_setup_that_never_said_defaults_to_the_gui():
    """The default that made the launchers' --sumo-gui redundant in the first place."""
    args = _args()
    run_cosim._rec_to_args({}, args)
    assert args.sumo_gui is True
