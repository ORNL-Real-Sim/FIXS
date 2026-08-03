# ============================================================================
# Bundle key (issue #191)
# ----------------------------------------------------------------------------
# Emits the key that pairs a proprietary-binaries bundle to the proprietary
# source it was built from:
#
#     <ProprietaryFiles-sha[:12]>
#
# The bundle release is named  Binaries-<key>. Because the key is DERIVED from
# the submodule pointer (never hand-typed), it can't be mis-named; and because
# it's the same formula everywhere, pack_binaries.ps1 (naming), release.yml
# (lookup) and bundle-guard.yml (the PR gate) all agree by construction.
#
# SCOPE (deliberately simple): this keys ONLY on the proprietary source. It does
# NOT try to detect a FIXS-side wire-contract change - that's a separate concern
# (a component's actual wire compatibility can only be proven by running the
# licensed binaries, which no hosted runner can do). msg_contract_hash.ps1 is
# kept in the tree for that future licensed-box / golden-vector work, but is not
# part of the key or the guard. Writes ONLY the key to stdout.
# ============================================================================
param(
    [string]$RepoRoot
)

$ErrorActionPreference = 'Stop'
if (-not $RepoRoot) {
    $RepoRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
}

# ProprietaryFiles submodule pointer (the proprietary-source dimension).
$pfLine = & git -C $RepoRoot ls-tree HEAD ProprietaryFiles
if ($LASTEXITCODE -ne 0 -or -not $pfLine) {
    Write-Error "Could not read the ProprietaryFiles submodule pointer via git ls-tree."
    exit 1
}
$pfSha = ($pfLine -split '\s+')[2]

Write-Output $pfSha.Substring(0, 12)
