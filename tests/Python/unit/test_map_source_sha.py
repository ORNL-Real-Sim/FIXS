"""Which bundle a cooked map came from (FIXS #224).

A cooked map is identified by a directory name, so it has no memory of the release
it was built from - invisible until the two halves of a bundle drift apart, which
is the distributed case, where they live on different machines. The sha is a string
GitHub publishes and we copy, so none of this needs CARLA, gh, or a network.
"""
import os
import sys
import types

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__),
                                "..", "..", "..", "scripts", "cosim"))
import import_map  # noqa: E402
import run_cosim  # noqa: E402

A, B = "sha256:aaaa1111", "sha256:bbbb2222"


def test_no_note_is_written_when_there_is_nothing_to_record(tmp_path):
    """An empty note would read back as a value, making two unrelated maps match."""
    import_map._write_sha(str(tmp_path), None)
    assert not os.path.exists(os.path.join(str(tmp_path), import_map.SHA_FILE))
    assert import_map._read_sha(str(tmp_path)) is None


def test_release_asset_sha_picks_the_right_asset(monkeypatch):
    """A release carries both halves; matching the wrong one would stamp a map with
    its sibling's sha. A null digest (older releases) is unknown, not a value."""
    monkeypatch.setattr(import_map.shutil, "which", lambda _: "gh")
    monkeypatch.setattr(import_map.subprocess, "check_output", lambda *a, **k:
                        f"roosevelt_cooked.tar.gz {B}\nroosevelt.zip {A}\nold.zip \n")
    assert import_map._release_asset_sha("r", "t", "*.zip") == A
    assert import_map._release_asset_sha("r", "t", "*.tar.gz") == B
    monkeypatch.setattr(import_map.shutil, "which", lambda _: None)
    assert import_map._release_asset_sha("r", "t", "*.zip") is None


#          cooked  running  peer  serve  allow   expected
CASES = [
    (A,     A,      None,  False, False, "silent"),   # agree
    (None,  A,      None,  False, False, "silent"),   # map predates the note
    (A,     None,   None,  False, False, "silent"),   # local pick: nothing to check
    (A,     B,      None,  False, False, "warn"),     # the finding
    (A,     B,      None,  True,  False, "refuse"),   # ...on the render host
    (A,     B,      None,  True,  True,  "warn"),     # ...overridden
    (A,     A,      B,     False, False, "warn"),     # peer beats the local cache
]


@pytest.mark.parametrize("cooked,running,peer,serve,allow,expected", CASES)
def test_comparison(monkeypatch, capsys, cooked, running, peer, serve, allow, expected):
    """Unrecorded on either side means 'not checked', never 'mismatch' - most maps
    in the wild have no note, and nagging about those buries the real warning.
    Under --serve it refuses rather than warns: the reader is at the other machine.
    The peer's sha wins because only the traffic host knows what its SUMO data came
    from - the render host's own cache answers a different question."""
    monkeypatch.setattr(import_map, "cooked_sha", lambda *a, **k: cooked)
    monkeypatch.setattr(import_map, "read_cached_sha", lambda *a, **k: running)
    args = types.SimpleNamespace(serve=serve, allow_map_skew=allow, peer_map_sha=peer)

    if expected == "refuse":
        with pytest.raises(SystemExit):
            run_cosim._check_map_source(args, "/carla", "mymap", "source")
        return

    run_cosim._check_map_source(args, "/carla", "mymap", "source")
    out = capsys.readouterr().out
    if expected == "silent":
        assert out == ""
    else:
        assert "WARNING" in out and "--reimport" in out    # names the fix
