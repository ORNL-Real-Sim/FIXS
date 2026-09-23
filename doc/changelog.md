# Releases

FIXS publishes builds on three rolling channels. Each channel is a GitHub prerelease
that CI re-publishes on every push to its branch, and each has a matching version of
this documentation (pick it in the version menu at the bottom left).

| Channel | Built from | Docs version | Meant for |
|---|---|---|---|
| `stable` | `main` | `stable` (default) | users |
| `vX.Y.Z-beta` | `beta_vX.Y.Z` | `vX.Y.Z-beta` | testing a release candidate |
| `vX.Y.Z-alpha` | `dev_vX.Y.Z` | `vX.Y.Z-alpha` | development builds |

## Downloads

All builds are on the [GitHub Releases](https://github.com/ORNL-Real-Sim/FIXS/releases)
page. A channel's release carries `fixs-build-<channel>.zip` (Windows) and, on the
0.9 trains and later, `fixs-build-<channel>-linux-x86_64.zip`. Applications install a
build with their own update command (for example `run_cosim.bat --update-fixs`),
which unpacks it with [`scripts/update_fixs.ps1`](../scripts/update_fixs.ps1).

## Release notes

Release notes are on each GitHub release. The last fixed-version releases are
[v0.7.0](https://github.com/ORNL-Real-Sim/FIXS/releases/tag/v0.7.0) and
[v0.6.0](https://github.com/ORNL-Real-Sim/FIXS/releases/tag/v0.6.0); 0.9.0 is on the
beta and alpha channels.

How CI builds and publishes the channels is described in
[BUILD.md](BUILD.md#rolling-release-channels).
