"""run_cosim's front door: naming the peer CARLA, and settling the scenario yaml.

Both behaviours exist because a decision the user made was not visible where they
made it. --peer names the CARLA on the other machine, and the setup summary derives
that line from the SCENARIO YAML - so an endpoint written through only after the
review loop drew the yaml's old address while the run dialled the new one. And the
scenario slot used to open a menu of yamls before the user had seen the setup at
all, when the application had already declared which yaml it means.

Nothing here launches anything: the endpoint lives in a text file and the scenario
choice is a path, so a tmp_path and a stub app manifest cover both.
"""
import argparse
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__),
                                "..", "..", "..", "scripts", "cosim"))
import app_catalog  # noqa: E402
import run_cosim  # noqa: E402


def _args(**kw):
    """A bare stand-in for the parsed command line, with only what is read here."""
    ns = argparse.Namespace(peer=None, carla_host=None, carla_port=None)
    for k, v in kw.items():
        setattr(ns, k, v)
    return ns


def _yaml(tmp_path, host="127.0.0.1", port=2000, name="config.yaml"):
    path = tmp_path / name
    path.write_text(
        "CarlaSetup:\n"
        f"  CarlaServerIP: {host}\n"
        f"  CarlaServerPort: {port}\n"
        "  CarlaTimeStep: 0.1\n",
        encoding="utf-8")
    return str(path)


# --------------------------------------------------------------------------- #
# --peer -> the CARLA endpoint
# --------------------------------------------------------------------------- #
def test_peer_sets_the_host():
    args = run_cosim.resolve_peer(_args(peer="192.168.140.56"))
    assert args.carla_host == "192.168.140.56"
    assert args.carla_port is None          # left to the yaml / the default


def test_peer_takes_an_optional_port():
    args = run_cosim.resolve_peer(_args(peer="192.168.140.56:2001"))
    assert (args.carla_host, args.carla_port) == ("192.168.140.56", 2001)


def test_peer_leaves_an_unset_endpoint_alone():
    args = run_cosim.resolve_peer(_args())
    assert (args.carla_host, args.carla_port) == (None, None)


@pytest.mark.parametrize("spec", ["", ":2000", "1.2.3.4:99999", "1.2.3.4:carla"])
def test_peer_rejects_what_is_not_an_address(spec):
    with pytest.raises(SystemExit):
        run_cosim.resolve_peer(_args(peer=spec))


def test_peer_and_carla_host_are_the_same_setting():
    """Two spellings of one endpoint: stop, rather than pick a winner."""
    with pytest.raises(SystemExit):
        run_cosim.resolve_peer(_args(peer="192.168.140.56", carla_host="10.0.0.5"))


# --------------------------------------------------------------------------- #
# The endpoint reaches the yaml the summary reads
# --------------------------------------------------------------------------- #
def test_cli_endpoint_is_written_into_the_scenario_yaml(tmp_path):
    path = _yaml(tmp_path)
    ctx = {}
    run_cosim._push_cli_endpoint(path, _args(carla_host="192.168.140.56"), ctx)
    assert run_cosim.read_carla_endpoint(path) == ("192.168.140.56", 2000)


def test_cli_endpoint_keeps_the_yaml_port_when_only_a_host_was_given(tmp_path):
    path = _yaml(tmp_path, port=2005)
    run_cosim._push_cli_endpoint(path, _args(carla_host="192.168.140.56"), {})
    assert run_cosim.read_carla_endpoint(path) == ("192.168.140.56", 2005)


def test_localhost_is_written_as_a_literal_address(tmp_path):
    """The C++ side takes an address, not a name it would have to resolve."""
    path = _yaml(tmp_path, host="192.168.140.56")
    run_cosim._push_cli_endpoint(path, _args(carla_host="localhost"), {})
    assert run_cosim.read_carla_endpoint(path)[0] == "127.0.0.1"


def test_cli_endpoint_is_pushed_once_so_a_later_edit_wins(tmp_path):
    """The flag is where this run STARTS. Editing the CARLA row afterwards is the
    user overruling it, and a push on every redraw would silently undo that."""
    path = _yaml(tmp_path)
    ctx = {}
    args = _args(carla_host="192.168.140.56")
    run_cosim._push_cli_endpoint(path, args, ctx)
    run_cosim.write_carla_endpoint(path, "10.0.0.5", 2000)      # the row-5 editor
    run_cosim._push_cli_endpoint(path, args, ctx)               # the next redraw
    assert run_cosim.read_carla_endpoint(path) == ("10.0.0.5", 2000)


def test_the_summary_row_shows_the_peer(tmp_path):
    """The symptom this fixes, end to end: --peer, then the line the user reads.

    Row 5 is DERIVED from the yaml, never from the flag, so this only holds once
    the flag has reached the file - which is the whole point of the push."""
    import run_profile
    path = _yaml(tmp_path)
    args = _args(carla_host="192.168.140.56")
    run_cosim._push_cli_endpoint(path, args, {})
    derived = run_cosim.derived_from_yaml(path, [], args)
    row = run_profile._fmt("carla", {"config": path}, {"mode": "client"}, derived)
    assert "192.168.140.56:2000" in row
    assert "[remote]" in row


def test_no_flag_leaves_the_yaml_untouched(tmp_path):
    path = _yaml(tmp_path, host="10.0.0.9", port=2003)
    run_cosim._push_cli_endpoint(path, _args(), {})
    assert run_cosim.read_carla_endpoint(path) == ("10.0.0.9", 2003)


def test_a_yaml_that_does_not_exist_yet_is_not_created(tmp_path):
    """It is generated later, seeded from the same args - see generate_config_yaml."""
    path = str(tmp_path / "not_written_yet.yaml")
    run_cosim._push_cli_endpoint(path, _args(carla_host="192.168.140.56"), {})
    assert not os.path.exists(path)


# --------------------------------------------------------------------------- #
# The scenario yaml settles itself
# --------------------------------------------------------------------------- #
@pytest.fixture
def fixs_home(tmp_path, monkeypatch):
    """Point ~/.fixs at a tmp dir, so nothing here reads or writes a real setup."""
    home = tmp_path / "fixs"
    home.mkdir()
    monkeypatch.setattr(app_catalog.env, "CONFIG_PATH", str(home / "carla.json"))
    return home


def _staged(tmp_path, *names):
    out = []
    for i, n in enumerate(names):
        p = tmp_path / n
        p.write_text("CarlaSetup:\n  CarlaServerIP: 127.0.0.1\n", encoding="utf-8")
        out.append({"path": str(p), "title": f"declared config {i}",
                    "engine": "cpp", "source": str(p)})
    return out


def test_default_config_takes_the_app_s_own_yaml(tmp_path, fixs_home):
    """An app that declares its yamls has already answered this question, and the
    first one it lists is the one it means."""
    staged = _staged(tmp_path, "config_ecoDriving.yaml", "config_dSPACE.yaml")
    path, scope = run_cosim.default_config(staged, "mlk_no_signal", "mlk_eco_driving")
    assert os.path.basename(path) == "config_ecoDriving.yaml"
    assert scope == "app"                  # AUTHORED: the generator must not touch it


def test_default_config_falls_back_to_the_per_map_yaml(tmp_path, fixs_home):
    """No declared yaml -> the app+map path, which main() generates on first run."""
    path, scope = run_cosim.default_config([], "roosevelt_full", "roosevelt")
    assert path == app_catalog.scenario_path("roosevelt", "roosevelt_full")
    assert scope == "map"
    assert not os.path.exists(path)        # a name to create, not a file to find


def test_default_config_keeps_what_the_setup_already_runs(tmp_path, fixs_home):
    """A choice already made outranks a default re-derived over the top of it."""
    staged = _staged(tmp_path, "config_ecoDriving.yaml", "config_dSPACE.yaml")
    current = staged[1]["path"]
    path, scope = run_cosim.default_config(staged, "mlk_no_signal",
                                           "mlk_eco_driving", current)
    assert (path, scope) == (current, "app")


def test_default_config_drops_a_current_that_is_gone(tmp_path, fixs_home):
    staged = _staged(tmp_path, "config_ecoDriving.yaml")
    path, _ = run_cosim.default_config(staged, "mlk_no_signal", "mlk_eco_driving",
                                       str(tmp_path / "deleted.yaml"))
    assert os.path.basename(path) == "config_ecoDriving.yaml"


# --------------------------------------------------------------------------- #
# Which slots settle themselves
# --------------------------------------------------------------------------- #
def test_ask_returns_only_what_was_picked(monkeypatch):
    """cascade() is the caller's to apply: a row the user opened and a row that
    merely fell over with it are answered differently."""
    import run_profile
    monkeypatch.setattr(run_profile, "_input", lambda *a, **k: "1")
    rec = {"app": "mlk_eco_driving", "map": "mlk_no_signal", "config_scope": "map"}
    assert run_profile.ask("s", rec, interactive=True, can_switch=False) == {"app"}
    assert run_profile.cascade(rec, {"app"}) == {"app", "map", "config"}
