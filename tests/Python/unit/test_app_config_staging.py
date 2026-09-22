"""Keeping a staged scenario yaml current without ever eating the user's edits.

A declared config is copied into ~/.fixs so machine-specific values (CARLA IP,
dSPACE ports) never touch the repo. Keeping that copy up to date is the hard part,
because ~/.fixs is per-MACHINE while a checkout is per-branch, and a developer here
has twenty-odd worktrees of the same repo pointing at one ~/.fixs. The old scheme
recorded ONE whole-file hash of the source, which produced four distinct failures
seen in the field and reproduced below:

  - alternating between two worktrees rewrote that one slot each time, so an
    untouched copy silently ping-ponged between branch versions;
  - an editor rewriting CRLF to LF, or one stray blank line, read as a local edit,
    after which the copy could never be refreshed again;
  - a stale record produced a CONFLICT between two byte-identical files;
  - every conflict dropped a <name>.yaml.new fossil that was never cleaned up and,
    in one case, was older than the file it claimed to update.

Nothing here launches anything: a config is a text file and ~/.fixs is a tmp_path.
"""
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "..", "Carla"))
import app_catalog  # noqa: E402


APP = {"id": "demo", "dir": "demo", "title": "demo",
       "configs": [{"path": "cfg/scenario.yaml", "title": "", "engine": None}]}

BODY = ("# what this file is for\n"
        "CarlaSetup:\n"
        "  CarlaServerIP: 127.0.0.1\n"
        "  CarlaServerPort: 2000\n")

MINE = BODY + "  CarlaClientPort: 440\n"
THEIRS = BODY + "  TrafficRefreshRate: 0.1\n"
THEIR_COMMENT = BODY.replace("# what this file is for", "# why this file exists")


@pytest.fixture
def home(tmp_path, monkeypatch):
    """Redirect ~/.fixs at its single source: apps_home() hangs off CONFIG_PATH."""
    monkeypatch.setattr(app_catalog.env, "CONFIG_PATH", str(tmp_path / "carla.json"))
    app_catalog._ASKED.clear()
    return tmp_path


def _repo(tmp_path, body=BODY, name="repoA"):
    """A checkout holding the declared config, and its root for stage_configs."""
    root = tmp_path / name
    src = root / "apps" / "demo" / "cfg" / "scenario.yaml"
    src.parent.mkdir(parents=True, exist_ok=True)
    src.write_bytes(body.encode("utf-8"))
    return str(root), src


def _staged(home):
    return home / "apps" / "demo" / "scenario.yaml"


def _answers(monkeypatch, *replies):
    """Feed the prompt a fixed script; an unscripted question pops an empty list
    and fails loudly, so 'nothing was asked' is a real assertion and not a guess."""
    queue = list(replies)
    monkeypatch.setattr("builtins.input", lambda *a, **k: queue.pop(0))
    return queue


def _edited(home, src, mine=MINE, theirs=THEIRS):
    """The one case that asks: you edited your copy AND the repo moved on."""
    _staged(home).write_text(mine, encoding="utf-8")
    src.write_text(theirs, encoding="utf-8")


# --------------------------------------------------------------------------- #
# The instrument itself
# --------------------------------------------------------------------------- #
def test_the_prompt_actually_fires_for_a_real_edit(home, monkeypatch):
    """Negative control. Most tests below assert something stays QUIET, and a
    stage_configs that did nothing at all would pass every one of them. This fails
    unless a genuine edit against a genuinely moved repo really does ask."""
    root, src = _repo(home)
    app_catalog.stage_configs(APP, root=root, interactive=False)
    _edited(home, src)

    asked = _answers(monkeypatch, "")
    app_catalog.stage_configs(APP, root=root, interactive=True)
    assert asked == [], "the prompt never ran: the rest of this file proves nothing"


# --------------------------------------------------------------------------- #
# The quiet paths - nobody who has not edited a config is ever asked anything
# --------------------------------------------------------------------------- #
def test_first_use_copies_and_says_where(home, capsys):
    root, _ = _repo(home)
    staged = app_catalog.stage_configs(APP, root=root, interactive=False)
    assert _staged(home).read_text(encoding="utf-8") == BODY
    assert staged[0]["path"] == str(_staged(home))
    assert "staged scenario.yaml" in capsys.readouterr().out


def test_unchanged_repo_says_nothing(home, capsys):
    root, _ = _repo(home)
    app_catalog.stage_configs(APP, root=root, interactive=False)
    capsys.readouterr()
    app_catalog.stage_configs(APP, root=root, interactive=False)
    assert capsys.readouterr().out == ""


def test_an_untouched_copy_is_refreshed_without_a_question(home, monkeypatch, capsys):
    """Nothing of yours can be lost, so there is nothing to ask about."""
    root, src = _repo(home)
    app_catalog.stage_configs(APP, root=root, interactive=False)
    src.write_text(THEIRS, encoding="utf-8")

    _answers(monkeypatch)
    app_catalog.stage_configs(APP, root=root, interactive=True)
    assert _staged(home).read_text(encoding="utf-8") == THEIRS
    assert "updated scenario.yaml from the repo" in capsys.readouterr().out


def test_line_endings_alone_are_not_an_edit(home, monkeypatch):
    """The measured case: the repo file is CRLF, an editor saved the copy as LF,
    and the whole file hashed differently - so it read as edited forever."""
    root, _ = _repo(home, body=BODY.replace("\n", "\r\n"))
    app_catalog.stage_configs(APP, root=root, interactive=False)
    _staged(home).write_bytes(BODY.encode("utf-8"))

    _answers(monkeypatch)
    app_catalog.stage_configs(APP, root=root, interactive=True)


def test_a_stray_blank_line_is_not_an_edit(home, monkeypatch):
    root, _ = _repo(home)
    app_catalog.stage_configs(APP, root=root, interactive=False)
    _staged(home).write_text(BODY.replace("CarlaSetup:", "\nCarlaSetup:"),
                             encoding="utf-8")

    _answers(monkeypatch)
    app_catalog.stage_configs(APP, root=root, interactive=True)


def test_identical_files_never_conflict_on_a_stale_record(home, monkeypatch, capsys):
    """Two byte-identical files are not a conflict, whatever the record says. The
    old code compared both sides against the record and asked about a no-op."""
    root, src = _repo(home)
    app_catalog.stage_configs(APP, root=root, interactive=False)
    index = app_catalog._load_stage_index("demo")
    index["scenario.yaml"]["seen"] = ["a hash from some other worktree"]
    app_catalog._save_stage_index("demo", index)

    _answers(monkeypatch)
    capsys.readouterr()
    app_catalog.stage_configs(APP, root=root, interactive=True)
    assert capsys.readouterr().out == ""
    assert src.read_text(encoding="utf-8") == _staged(home).read_text(encoding="utf-8")


def test_alternating_worktrees_go_quiet(home, monkeypatch):
    """Two checkouts, different versions of the file, no local edits. Each launch
    takes that checkout's version and neither ever asks.

    This is what one recorded hash could not do: it held whichever branch wrote it
    last, so the other branch's launch either re-asked or silently ran the wrong
    version. Both versions are remembered here, which is why nothing is asked."""
    root_a, _ = _repo(home, name="repoA")
    root_b, _ = _repo(home, body=THEIRS, name="repoB")

    _answers(monkeypatch)
    for root in (root_a, root_b, root_a, root_b, root_a):
        app_catalog.stage_configs(APP, root=root, interactive=True)
    assert _staged(home).read_text(encoding="utf-8") == BODY
    seen = app_catalog._load_stage_index("demo")["scenario.yaml"]["seen"]
    assert len(seen) == 2, "both versions should be remembered, not overwritten"


def test_keep_and_stop_asking_holds_across_worktrees(home, monkeypatch):
    """One recorded hash could not express 'I have settled this version', so the
    answer was forgotten the moment another checkout ran."""
    root_a, _ = _repo(home, name="repoA")
    app_catalog.stage_configs(APP, root=root_a, interactive=False)
    _staged(home).write_text(MINE, encoding="utf-8")
    root_b, _ = _repo(home, body=THEIRS, name="repoB")

    _answers(monkeypatch, "k")
    for _ in range(3):
        app_catalog._ASKED.clear()
        app_catalog.stage_configs(APP, root=root_b, interactive=True)
    assert _staged(home).read_text(encoding="utf-8") == MINE


def test_one_launch_asks_once_even_if_staging_runs_twice(home, monkeypatch):
    """_bind_app re-stages whenever the setup's app could have changed."""
    root, src = _repo(home)
    app_catalog.stage_configs(APP, root=root, interactive=False)
    _edited(home, src)

    _answers(monkeypatch, "")
    app_catalog.stage_configs(APP, root=root, interactive=True)
    app_catalog.stage_configs(APP, root=root, interactive=True)


# --------------------------------------------------------------------------- #
# The one case that asks
# --------------------------------------------------------------------------- #
def test_enter_keeps_yours_and_asks_again_next_run(home, monkeypatch):
    """Enter is the non-answer: it records nothing, so hurrying past it can neither
    lose an edit nor silence an update."""
    root, src = _repo(home)
    app_catalog.stage_configs(APP, root=root, interactive=False)
    _edited(home, src)

    asked = _answers(monkeypatch, "", "")
    app_catalog.stage_configs(APP, root=root, interactive=True)
    app_catalog._ASKED.clear()
    app_catalog.stage_configs(APP, root=root, interactive=True)
    assert asked == [], "Enter must not silence the next run"
    assert _staged(home).read_text(encoding="utf-8") == MINE


def test_overwrite_keeps_a_timestamped_backup(home, monkeypatch, capsys):
    root, src = _repo(home)
    app_catalog.stage_configs(APP, root=root, interactive=False)
    _edited(home, src)

    _answers(monkeypatch, "o")
    app_catalog.stage_configs(APP, root=root, interactive=True)
    assert _staged(home).read_text(encoding="utf-8") == THEIRS
    backups = [p for p in os.listdir(str(_staged(home).parent)) if ".bak-" in p]
    assert len(backups) == 1
    kept = (_staged(home).parent / backups[0]).read_text(encoding="utf-8")
    assert kept == MINE, "the backup must hold exactly what was overwritten"
    assert backups[0] in capsys.readouterr().out


def test_the_diff_is_shown_on_request(home, monkeypatch, capsys):
    root, src = _repo(home)
    app_catalog.stage_configs(APP, root=root, interactive=False)
    _edited(home, src)

    _answers(monkeypatch, "d", "")
    capsys.readouterr()
    app_catalog.stage_configs(APP, root=root, interactive=True)
    out = capsys.readouterr().out
    assert "-  CarlaClientPort: 440" in out, "what you would lose"
    assert "+  TrafficRefreshRate: 0.1" in out, "what you would gain"


def test_the_label_fires_when_the_overwrite_is_safe(home, monkeypatch, capsys):
    """Your settings and the repo's agree, so taking theirs changes nothing that
    runs. Half the commits to these yamls touch only comments, and saying so makes
    those a one-keystroke yes instead of a diff to read."""
    root, src = _repo(home)
    app_catalog.stage_configs(APP, root=root, interactive=False)
    _edited(home, src, mine=BODY.replace("# what this file is for", "# my note"),
            theirs=THEIR_COMMENT)

    _answers(monkeypatch, "")
    app_catalog.stage_configs(APP, root=root, interactive=True)
    assert "only comments differ" in capsys.readouterr().out


def test_an_edit_of_yours_to_a_real_value_is_never_called_comments(home, monkeypatch,
                                                                  capsys):
    """The dangerous mislabel: upstream touched only prose, but YOUR edit is a live
    setting, so taking theirs would drop it. That must not read as safe."""
    root, src = _repo(home)
    app_catalog.stage_configs(APP, root=root, interactive=False)
    _edited(home, src, theirs=THEIR_COMMENT)

    _answers(monkeypatch, "")
    app_catalog.stage_configs(APP, root=root, interactive=True)
    assert "only comments differ" not in capsys.readouterr().out


def test_a_settings_change_is_not_called_comments_only(home, monkeypatch, capsys):
    root, src = _repo(home)
    app_catalog.stage_configs(APP, root=root, interactive=False)
    _edited(home, src)

    _answers(monkeypatch, "")
    app_catalog.stage_configs(APP, root=root, interactive=True)
    assert "only comments differ" not in capsys.readouterr().out


def test_no_tty_keeps_yours_and_says_which(home, capsys):
    root, src = _repo(home)
    app_catalog.stage_configs(APP, root=root, interactive=False)
    _edited(home, src)

    capsys.readouterr()
    app_catalog.stage_configs(APP, root=root, interactive=False)
    out = capsys.readouterr().out
    assert "scenario.yaml changed in the repo" in out
    assert "keeping yours" in out
    assert _staged(home).read_text(encoding="utf-8") == MINE


def test_nothing_is_ever_left_beside_your_file(home, monkeypatch):
    """No .new fossils, whichever way the question is answered."""
    root, src = _repo(home)
    for n, reply in enumerate(("", "k", "o")):
        app_catalog._ASKED.clear()
        app_catalog.stage_configs(APP, root=root, interactive=False)
        _edited(home, src,
                mine=BODY + "  CarlaClientPort: 44%d\n" % n,
                theirs=BODY + "  TrafficRefreshRate: 0.%d\n" % (n + 1))
        _answers(monkeypatch, reply)
        app_catalog.stage_configs(APP, root=root, interactive=True)
    left = [p for p in os.listdir(str(_staged(home).parent)) if p.endswith(".new")]
    assert left == []


# --------------------------------------------------------------------------- #
# Upgrading from the record the old scheme wrote
# --------------------------------------------------------------------------- #
def test_an_old_record_does_not_reopen_a_settled_question(home, monkeypatch, capsys):
    """The old index held one WHOLE-FILE hash. Where it names the source file we are
    looking at, the user already settled this version; asking again would make the
    upgrade itself look like an upstream change."""
    root, src = _repo(home)
    app_catalog.stage_configs(APP, root=root, interactive=False)
    _staged(home).write_text(MINE, encoding="utf-8")
    app_catalog._save_stage_index("demo", {"scenario.yaml": {
        "source": "cfg/scenario.yaml", "hash": app_catalog._sha256(str(src))}})

    _answers(monkeypatch)
    capsys.readouterr()
    app_catalog.stage_configs(APP, root=root, interactive=True)
    assert capsys.readouterr().out == ""
    assert _staged(home).read_text(encoding="utf-8") == MINE


def test_an_old_record_for_a_different_version_still_asks(home, monkeypatch):
    root, src = _repo(home)
    app_catalog.stage_configs(APP, root=root, interactive=False)
    _edited(home, src)
    app_catalog._save_stage_index("demo", {"scenario.yaml": {
        "source": "cfg/scenario.yaml", "hash": "a digest of something else"}})

    asked = _answers(monkeypatch, "")
    app_catalog.stage_configs(APP, root=root, interactive=True)
    assert asked == []
