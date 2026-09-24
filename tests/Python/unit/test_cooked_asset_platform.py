"""A precooked map is per OS (FIXS_Applications#29): Windows fetches
`<map>_cooked_windows.tar.gz`, Linux `<map>_cooked.tar.gz`. Pure logic - no
archive and no CARLA needed, unlike test_import_map_packaged.py."""
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "..", "Carla"))
import carla_env_setup as env  # noqa: E402
import import_map  # noqa: E402


def test_asset_name_follows_the_os():
    assert import_map.cooked_asset_name("mlk.zip", system="Linux") == "mlk_cooked.tar.gz"
    assert import_map.cooked_asset_name("mlk.zip", system="Windows") == \
        "mlk_cooked_windows.tar.gz"


def test_catalog_override_is_per_os():
    entry = {"asset": "a.zip", "cooked_asset": "lin.tar.gz", "cooked_asset_windows": "win.tar.gz"}
    assert import_map.catalog_cooked_asset(entry, system="Linux") == "lin.tar.gz"
    assert import_map.catalog_cooked_asset(entry, system="Windows") == "win.tar.gz"
    # A Linux-only override must not be handed to Windows ...
    assert import_map.catalog_cooked_asset({"asset": "a.zip", "cooked_asset": "lin.tar.gz"},
                                           system="Windows") == "a_cooked_windows.tar.gz"
    # ... and an explicit empty value still means "none for this OS".
    assert import_map.catalog_cooked_asset({"asset": "a.zip", "cooked_asset_windows": ""},
                                           system="Windows") is None


def test_cached_linux_cook_is_not_handed_to_windows(tmp_path, monkeypatch):
    """A map's cache dir can hold both cooks. Matching the cache by suffix alone
    returned the Linux one for a Windows request - a grey map, silently."""
    cache = tmp_path / "mlk"
    cache.mkdir()
    (cache / "mlk_cooked.tar.gz").write_text("linux")
    downloaded = []

    def fake_gh(cmd):
        pattern = cmd[cmd.index("-p") + 1]
        downloaded.append(pattern)
        (cache / pattern).write_text("windows")
        return 0

    monkeypatch.setattr(import_map, "_require_gh", lambda: "gh")
    monkeypatch.setattr(import_map, "_map_cache_dir", lambda name=None: str(cache))
    monkeypatch.setattr(import_map.subprocess, "call", fake_gh)

    got = import_map.download_cooked_tar("o/r", "mlk", "mlk_cooked_windows.tar.gz",
                                         cache_name="mlk")
    assert os.path.basename(got) == "mlk_cooked_windows.tar.gz"
    assert downloaded == ["mlk_cooked_windows.tar.gz"]

    # Now cached, the same request is served from the cache without a download.
    again = import_map.download_cooked_tar("o/r", "mlk", "mlk_cooked_windows.tar.gz",
                                           cache_name="mlk")
    assert again == got and downloaded == ["mlk_cooked_windows.tar.gz"]


def test_setup_offers_packaged_on_windows(monkeypatch, capsys):
    monkeypatch.setattr(env.platform, "system", lambda: "Windows")
    monkeypatch.setattr("builtins.input", lambda prompt="": "9")
    with pytest.raises(SystemExit) as exc:
        env.run_setup()
    out = capsys.readouterr().out
    assert "[1] Packaged CARLA" in out and "EXPERIMENTAL on Windows" in out
    assert "expected 1 or 2 or 3" in str(exc.value)
