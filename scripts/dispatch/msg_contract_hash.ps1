# ============================================================================
# Message-contract hash (issue #191; recorded, NOT enforced - see #204)
# ----------------------------------------------------------------------------
# Emits a stable SHA-256 of the FIXS message-struct header. pack_binaries.ps1
# records it in the bundle manifest so a bundle says which header it was built
# against. Nothing compares it: #204 decided against gating on it, because the
# header is the wrong file for that. New fields are appended and selected by
# name, so the hash changes on edits that break nothing, while the real wire
# order lives in MsgHelper.cpp packVehData, which it does not cover.
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
