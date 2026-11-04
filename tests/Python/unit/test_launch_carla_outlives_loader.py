"""A script that only boots CARLA for others (load_opendrive_world.py --launch)
exits right after, so the CARLA it started must outlive it; run_cosim's own
launch stays tied to its run.

The "CARLA" is a stand-in that writes a file a few seconds after its parent has
exited, so surviving is observable without probing pids.
"""
import os
import subprocess
import sys
import time

import pytest

CARLA_DIR = os.path.normpath(os.path.join(os.path.dirname(__file__), "..", "..", "..", "Carla"))

PARENT = r"""
import sys
sys.path.insert(0, sys.argv[1])
import run_cosim
marker, leash = sys.argv[2], sys.argv[3] == "1"
child = [sys.executable, "-c",
         "import sys, time; time.sleep(3); open(sys.argv[1], 'w').write('up')", marker]
run_cosim._carla_command = lambda *a, **k: child
run_cosim.launch_carla({"mode": "packaged"}, port=0, leash=leash)
"""


def _child_survived(tmp_path, leash):
    marker = tmp_path / f"survived_{int(leash)}"
    # DEVNULL, not a pipe: a child holding the pipe would make run() wait for it.
    subprocess.run([sys.executable, "-c", PARENT, CARLA_DIR, str(marker), "1" if leash else "0"],
                   stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
                   stderr=subprocess.DEVNULL, check=True, timeout=60)
    deadline = time.time() + 15
    while time.time() < deadline:
        if marker.exists():
            return True
        time.sleep(0.25)
    return False


def test_unleashed_carla_outlives_the_script_that_launched_it(tmp_path):
    assert _child_survived(tmp_path, leash=False)


@pytest.mark.skipif(not sys.platform.startswith(("win", "linux")),
                    reason="the leash is a Job Object / PR_SET_PDEATHSIG")
def test_leashed_carla_dies_with_its_launcher(tmp_path):
    assert not _child_survived(tmp_path, leash=True)
