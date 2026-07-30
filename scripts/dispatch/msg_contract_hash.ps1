# ============================================================================
# Message-contract hash (issue #191 compatibility guard)
# ----------------------------------------------------------------------------
# Emits a stable SHA-256 of the FIXS wire-format definition so the release CI can
# detect when a pinned FIXS-Binaries bundle was built against a DIFFERENT message
# contract than the fresh public parts (which would produce a latest.zip whose
# components disagree on the wire format).
#
#   - pack_binaries.ps1 records this hash in the bundle manifest at build time.
#   - release.yml recomputes it from the current tree at assembly time and fails
#     loudly on mismatch.
#
# Hashes ONLY CommonLib/VehDataMsgDefs.h (the struct/field definitions), with
# comments and whitespace normalized out, so cosmetic edits (reflow, comments)
# don't trigger false-positive proprietary rebuilds. Writes ONLY the 64-char hex
# hash to stdout; diagnostics go to stderr.
# ============================================================================
param(
    [string]$RepoRoot
)

$ErrorActionPreference = 'Stop'
if (-not $RepoRoot) {
    $RepoRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
}

$ContractFile = Join-Path $RepoRoot 'CommonLib\VehDataMsgDefs.h'
if (-not (Test-Path $ContractFile)) {
    Write-Error "Message-contract file not found: $ContractFile"
    exit 1
}

$text = Get-Content $ContractFile -Raw

# Normalize: strip /* */ and // comments, then collapse all whitespace, so the
# hash reflects only the structural wire definition.
$text = [regex]::Replace($text, '/\*.*?\*/', '', [System.Text.RegularExpressions.RegexOptions]::Singleline)
$text = [regex]::Replace($text, '//[^\r\n]*', '')
$text = [regex]::Replace($text, '\s+', '')

$bytes  = [System.Text.Encoding]::UTF8.GetBytes($text)
$sha256 = [System.Security.Cryptography.SHA256]::Create()
$hash   = ($sha256.ComputeHash($bytes) | ForEach-Object { $_.ToString('x2') }) -join ''

Write-Output $hash
