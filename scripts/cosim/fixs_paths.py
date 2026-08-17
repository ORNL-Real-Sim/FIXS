"""Where FIXS is, and where the consuming application is.

Every entry point needs these two directories, and each one used to derive them
from its own nesting depth - `os.path.dirname(HERE)` in run_cosim, `HERE/".."` in
carla_env_setup, `dirname(dirname(here))` in import_map. Three copies of one fact,
each of which silently becomes wrong when a file moves.

They agreed only by coincidence: the co-sim scripts sat at `<fixs>/Carla/` in a
source checkout and at `<app>/FIXS/Carla/` in an unpacked release, the same
distance from the root in both. That coincidence ends as soon as the source tree
and the shipped bundle disagree about nesting (FIXS#313 puts them at
`<fixs>/scripts/cosim/` and `<app>/FIXS/cosim/` respectively), and running from a
checkout is a documented workflow - see Carla/README.md.

So anchor on a file instead of on a distance. `environment.yml` sits at the FIXS
root in BOTH layouts: it is committed at the top of this repo, and
scripts/dispatch/8_create_zip.ps1 copies it to the top of the release zip. Walking
up until we find it is correct wherever the caller happens to live, and no future
move needs to touch this file.
"""
import os

SENTINEL = "environment.yml"


def fixs_root(start=None):
    """The FIXS root: the nearest ancestor of `start` that holds environment.yml.

    `start` defaults to this module's own directory, which is what every caller
    wants - they are all inside the FIXS tree. Falls back to the parent of `start`
    (the pre-sentinel guess) when nothing is found, so a partially unpacked bundle
    degrades the way it always did instead of raising from inside an import."""
    here = os.path.dirname(os.path.abspath(__file__)) if start is None \
        else os.path.abspath(start)
    d = here
    while True:
        if os.path.isfile(os.path.join(d, SENTINEL)):
            return d
        parent = os.path.dirname(d)
        if parent == d:                      # hit the filesystem root
            return os.path.dirname(here)
        d = parent


def app_root(start=None):
    """The consuming application's repo root: the directory holding FIXS/.

    In a source checkout this is the parent of the FIXS clone and holds no
    manifest, so app awareness simply finds nothing and the run is generic - which
    is what a FIXS developer running from source has always got."""
    return os.path.dirname(fixs_root(start))


def env_yml(start=None):
    """The canonical conda spec shipped at the FIXS root."""
    return os.path.join(fixs_root(start), SENTINEL)
