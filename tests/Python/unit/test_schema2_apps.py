"""schema 2: an application is identified by the launcher it starts.

schema 1 spelled an app out three times - `id`, `dir` and `launch` - and two of
FIXS_Applications' three apps then declared no launcher at all and pointed `dir`
at a folder holding only a README. So the fields disagreed about what an app even
was: a folder, a name, or a thing that runs.

schema 2 picks one answer. `launch` is required and carries its own path; the
folder is that file's parent and the id is its basename, so neither is written
down, no directory called apps/ is needed anywhere, and two launchers in one
folder are two applications rather than a name collision.

schema 1 is untouched by all of this - see test_app_catalog_schema.py.
"""
import io
import json
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__),
                                "..", "..", "..", "scripts", "cosim"))
import app_catalog  # noqa: E402

EXT = ".bat" if os.name == "nt" else ".sh"


def _write(path, doc):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with io.open(path, "w", encoding="utf-8") as f:
        json.dump(doc, f)


def _repo(tmp_path, apps, schema=2, name="fixs.json"):
    root = str(tmp_path / "repo")
    doc = {"apps": apps}
    if schema is not None:
        doc["schema"] = schema
    _write(os.path.join(root, name), doc)
    return root


def _launcher(root, rel):
    """Create the launcher `rel` (extensionless) so launch_command can find it."""
    path = os.path.join(root, *(rel + EXT).split("/"))
    os.makedirs(os.path.dirname(path), exist_ok=True)
    io.open(path, "w").write("")
    return path


# --------------------------------------------------------------------------- #
# identity comes from the launcher
# --------------------------------------------------------------------------- #
@pytest.mark.parametrize("launch, folder, app_id", [
    ("projects/autolab/controlA/run_ctrl", ("projects", "autolab", "controlA"), "ctrl"),
    ("src/controllers/eco/run_eco",        ("src", "controllers", "eco"),       "eco"),
    ("run_here",                           (),                                  "here"),
    ("tools/drive",                        ("tools",),                          "drive"),
])
def test_identity_comes_from_the_launcher(tmp_path, launch, folder, app_id):
    root = _repo(tmp_path, [{"launch": launch}])
    app, = app_catalog.load_catalog(root)
    assert app["id"] == app_id
    assert app_catalog.app_dir(app, root) == os.path.join(root, *folder)


def test_an_entry_with_no_launcher_is_not_an_application(tmp_path, capsys):
    """Rejected rather than silently accepted as an app that starts nothing."""
    root = _repo(tmp_path, [{"id": "eco", "title": "no launcher"}])
    assert app_catalog.load_catalog(root) == []
    assert "launch" in capsys.readouterr().out


def test_explicit_id_beats_the_derivation(tmp_path):
    root = _repo(tmp_path, [{"launch": "a/run_ctrl", "id": "autolab_mpc"}])
    app, = app_catalog.load_catalog(root)
    assert app["id"] == "autolab_mpc"


def test_variants_in_one_folder_are_two_applications(tmp_path):
    """The variants case: one folder, one requirements.txt, two controllers."""
    root = _repo(tmp_path, [{"launch": "app/run_base"}, {"launch": "app/run_mpc"}])
    apps = app_catalog.load_catalog(root)
    assert [a["id"] for a in apps] == ["base", "mpc"]
    assert len({app_catalog.app_dir(a, root) for a in apps}) == 1
    # distinct ids -> distinct scenario dirs, so one variant's remembered scenario
    # choice cannot leak into the other's
    assert app_catalog.scenario_dir("base") != app_catalog.scenario_dir("mpc")


# --------------------------------------------------------------------------- #
# resolving the launcher
# --------------------------------------------------------------------------- #
def test_the_launcher_path_does_not_double(tmp_path):
    """`dir` IS the launcher's parent, so resolving the launcher against it again
    would look for <root>/projects/autolab/projects/autolab/run_ctrl."""
    root = _repo(tmp_path, [{"launch": "projects/autolab/run_ctrl"}])
    made = _launcher(root, "projects/autolab/run_ctrl")
    app, = app_catalog.load_catalog(root)
    argv, cwd = app_catalog.launch_command(app, root)
    assert argv is not None, "launcher not found - the path doubled"
    assert argv[0] == made
    assert cwd == os.path.join(root, "projects", "autolab")


def test_launch_arguments_pass_through_untouched(tmp_path):
    """run_cosim never reads what an app passes itself."""
    root = _repo(tmp_path, [{"launch": "ctl/run_x --mode mpc --steps 500"}])
    _launcher(root, "ctl/run_x")
    app, = app_catalog.load_catalog(root)
    argv, _ = app_catalog.launch_command(app, root)
    assert argv[1:] == ["--mode", "mpc", "--steps", "500"]


def test_a_missing_launcher_warns_and_starts_nothing(tmp_path, capsys):
    root = _repo(tmp_path, [{"launch": "ctl/run_x"}])       # never created
    app, = app_catalog.load_catalog(root)
    argv, cwd = app_catalog.launch_command(app, root)
    assert (argv, cwd) == (None, None)
    assert "not found" in capsys.readouterr().out


# --------------------------------------------------------------------------- #
# maps are a hint, and never invented
# --------------------------------------------------------------------------- #
def test_no_map_is_invented_from_the_id(tmp_path):
    """schema 1 falls back to maps=[id] because apps were named after locations.
    An id derived from a launcher is no kind of map name."""
    root = _repo(tmp_path, [{"launch": "a/run_ctrl"}])
    app, = app_catalog.load_catalog(root)
    assert app["maps"] == []


def test_a_declared_map_is_kept(tmp_path):
    root = _repo(tmp_path, [{"launch": "a/run_ctrl", "maps": ["RP_Ver0529"]}])
    app, = app_catalog.load_catalog(root)
    assert app["maps"] == ["RP_Ver0529"]


# --------------------------------------------------------------------------- #
# description, and what the schema still accepts
# --------------------------------------------------------------------------- #
def test_description_is_the_new_spelling_of_note(tmp_path):
    root = _repo(tmp_path, [{"launch": "a/run_ctrl",
                             "description": "eco-driving MPC variant"}])
    app, = app_catalog.load_catalog(root)
    assert app["note"] == "eco-driving MPC variant"


def test_note_still_reads_for_a_converted_manifest(tmp_path):
    root = _repo(tmp_path, [{"launch": "a/run_ctrl", "note": "old spelling"}])
    app, = app_catalog.load_catalog(root)
    assert app["note"] == "old spelling"


def test_hand_edited_fields_are_still_accepted(tmp_path):
    """The wizard writes four fields; the schema accepts more, for the app that
    outgrows them. Dropping these from the READER would strand such an app."""
    root = _repo(tmp_path, [{"launch": "a/run_ctrl",
                             "requirements": "requirements.txt",
                             "sumo_args": {"--time-to-teleport": "30"},
                             "defaults": {"engine": "cpp"}}])
    app, = app_catalog.load_catalog(root)
    assert app["requirements"] == "requirements.txt"
    assert app["sumo_args"] == {"--time-to-teleport": "30"}
    assert app["defaults"] == {"engine": "cpp"}


def test_title_defaults_to_the_id(tmp_path):
    root = _repo(tmp_path, [{"launch": "a/run_ctrl"}])
    app, = app_catalog.load_catalog(root)
    assert app["title"] == "ctrl"


def test_find_app_by_id_title_or_folder(tmp_path):
    root = _repo(tmp_path, [{"launch": "src/controllers/eco/run_eco",
                             "title": "Eco driving"}])
    apps = app_catalog.load_catalog(root)
    for ident in ("eco", "Eco driving", "src/controllers/eco"):
        assert app_catalog.find_app(apps, ident)["id"] == "eco"
