# ============================================================================
# Bundle key (issue #191)
# ----------------------------------------------------------------------------
# Emits the COMPOSITE key that uniquely pairs a proprietary-binaries bundle to
# the exact inputs it was built for:
#
#     <ProprietaryFiles-sha[:12]>-<message-contract-hash[:8]>
#
# Two dimensions, because the binaries depend on both:
#   - the ProprietaryFiles submodule commit (the proprietary source), and
#   - the FIXS wire contract they compile against (CommonLib/VehDataMsgDefs.h,
#     which lives in FIXS, not ProprietaryFiles).
#
# The bundle release is named  Binaries-<key>. Because the key is DERIVED from
# the tree (never hand-typed), it can't be mis-named; and because it's the same
# formula everywhere, pack_binaries.ps1 (naming), release.yml (lookup) and
# bundle-guard.yml (the PR gate) all agree by construction. Writes ONLY the key
# to stdout; diagnostics go to stderr.
# ============================================================================
param(
    [string]$RepoRoot
)

$ErrorActionPreference = 'Stop'
if (-not $RepoRoot) {
    $RepoRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
}

# --- ProprietaryFiles submodule pointer (proprietary-source dimension) -------
$pfLine = & git -C $RepoRoot ls-tree HEAD ProprietaryFiles
if ($LASTEXITCODE -ne 0 -or -not $pfLine) {
    Write-Error "Could not read the ProprietaryFiles submodule pointer via git ls-tree."
    exit 1
}
$pfSha = ($pfLine -split '\s+')[2]

# --- Message-contract hash (wire-format dimension) --------------------------
# Called with the call operator so it runs in THIS interpreter (works under both
# Windows PowerShell on the licensed box and pwsh on the hosted runners).
$msgHash = (& (Join-Path $PSScriptRoot 'msg_contract_hash.ps1') -RepoRoot $RepoRoot).Trim()

Write-Output ("{0}-{1}" -f $pfSha.Substring(0, 12), $msgHash.Substring(0, 8))
