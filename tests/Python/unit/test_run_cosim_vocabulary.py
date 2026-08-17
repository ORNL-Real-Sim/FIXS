"""The co-sim vocabulary belongs to run_cosim, not to each app repo's wrapper.

--setup, --import-map and --carla-map used to be implemented in every application
repo's run_cosim.bat and run_cosim.sh, which translated them onto sibling scripts
by name. That is engine vocabulary held as a transcription, in two languages, with
no test - which is how `--setup carla` came to print "unrecognized arguments:
carla" from a script the user never named, and how the Windows and Linux halves of
the same feature drifted apart (#238).

These run the real parser in a subprocess, because what is being checked is the
command line contract: the names exist, they are documented, and a mistake is
answered in the engine's own voice. Nothing here launches a simulator - every case
either fails at parse time or asks for --help.
"""
import os
import subprocess
import sys

import pytest

RUN_COSIM = os.path.normpath(os.path.join(
    os.path.dirname(__file__), "..", "..", "..", "scripts", "cosim", "run_cosim.py"))


def run(*argv):
    """run_cosim with argv, captured. Never reaches a launch."""
    return subprocess.run([sys.executable, RUN_COSIM, *argv],
                          capture_output=True, text=True, timeout=120)


@pytest.fixture(scope="module")
def help_text():
    r = run("--help")
    assert r.returncode == 0, r.stderr
    return r.stdout


# --------------------------------------------------------------------------- #
# the names exist and are documented
# --------------------------------------------------------------------------- #
@pytest.mark.parametrize("flag", ["--setup", "--import-map", "--update-python",
                                  "--doctor", "--version", "--serve", "--peer"])
def test_the_front_door_vocabulary_is_all_here(help_text, flag):
    """Every name a front door used to own is answerable by the engine, so the
    front door has no flag list to keep in sync."""
    assert flag in help_text


def test_setup_documents_its_optional_target(help_text):
    assert "--setup [TARGET]" in help_text


def test_import_map_documents_its_optional_map(help_text):
    assert "--import-map [MAP]" in help_text


def test_carla_map_is_accepted_but_not_advertised(help_text):
    """Hidden on purpose: --map is the name to learn, --carla-map is kept so the
    spelling the old wrappers taught keeps working."""
    assert "--carla-map" not in help_text
    r = run("--carla-map", "atlanta", "--help")
    assert r.returncode == 0


# --------------------------------------------------------------------------- #
# mistakes are answered in the engine's voice
# --------------------------------------------------------------------------- #
def test_an_unknown_setup_target_is_rejected_by_name():
    r = run("--setup", "vissim")
    assert r.returncode == 2
    out = r.stdout + r.stderr
    assert "vissim" in out and "carla" in out
    # the old failure: an error naming a script the user never typed
    assert "carla_env_setup.py" not in out


def test_map_and_carla_map_may_not_disagree():
    """They are one setting. Silent precedence would mean a user who passed both
    gets a map they did not name, discovered several minutes into a launch."""
    r = run("--carla-map", "atlanta", "--map", "roosevelt")
    assert r.returncode == 2
    out = r.stdout + r.stderr
    assert "atlanta" in out and "roosevelt" in out


def test_map_and_carla_map_agreeing_is_not_an_error():
    r = run("--carla-map", "atlanta", "--map", "atlanta", "--help")
    assert r.returncode == 0
