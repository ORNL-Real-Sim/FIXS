# ============================================================================
# Acquire CommonLib/libcarla (the Carla C++ client SDK) without committing it.
# ----------------------------------------------------------------------------
# libcarla is gitignored (~800 MB) and is the ONLY missing ingredient to build
# VirCarlaEnv.exe (post-#174 VirtualEnvironment.lib no longer needs it). This
# script places it, driven by the per-machine ~/.fixs/carla.json:
#
#   { "mode": "source",   "carla_root": "C:/src_ext/Carla", ... }
#     -> copy <carla_root>/PythonAPI/carla/dependencies/{lib,include} into CommonLib/libcarla
#
#   { "mode": "prebuilt", ... }
#     -> download the public fixs-deps-<ver>.zip release asset (gh-free, no token),
#        verify its SHA-256, and extract into CommonLib/ (libcarla/ [+ libsumo/ later])
#
# Idempotent: skips if libcarla is already present (unless -Force). Skips cleanly
# (exit 0) if Carla isn't configured - Carla is optional, like MATLAB/dSPACE.
#
# Usage:
#   fetch_carla_deps.ps1                 # honour ~/.fixs/carla.json
#   fetch_carla_deps.ps1 -Force          # re-acquire even if present
#   fetch_carla_deps.ps1 -Mode prebuilt  # override the mode
# ============================================================================
param(
    [string]$RepoRoot,
    [ValidateSet('', 'source', 'prebuilt')] [string]$Mode = '',
    [string]$DepsVersion,
    [string]$Repo = 'ORNL-Real-Sim/FIXS',
    [switch]$Force
)

$ErrorActionPreference = 'Stop'
if (-not $RepoRoot) { $RepoRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path }
$LibCarla   = Join-Path $RepoRoot 'CommonLib\libcarla'
$SentinelLib = Join-Path $LibCarla 'lib\carla_client.lib'

function Read-CarlaConfig {
    $p = Join-Path $env:USERPROFILE '.fixs\carla.json'
    if (-not (Test-Path $p)) { return $null }
    return (Get-Content $p -Raw | ConvertFrom-Json)
}

function Get-DepsVersion {
    # machine-independent Carla version from dependencies.yaml (e.g. "0.9.15")
    $depsYaml = Join-Path $RepoRoot 'dependencies.yaml'
    if (Test-Path $depsYaml) {
        $m = Select-String -Path $depsYaml -Pattern '^\s*(carla_version|version)\s*:\s*["'']?([0-9][0-9.]*)' |
             Where-Object { $_.Line -match 'carla' } | Select-Object -First 1
        if ($m) { return $m.Matches[0].Groups[2].Value }
    }
    return '0.9.15'
}

# --- idempotent short-circuit -----------------------------------------------
if (-not $Force -and (Test-Path $SentinelLib)) {
    Write-Host "libcarla already present ($LibCarla) - skipping (use -Force to re-acquire)."
    exit 0
}

$cfg = Read-CarlaConfig
if (-not $cfg) {
    Write-Warning "No ~/.fixs/carla.json - skipping libcarla acquisition (Carla is optional; VirCarlaEnv will not build). See doc/CARLAdoc.md."
    exit 0
}
if (-not $Mode) { $Mode = if ($cfg.mode) { $cfg.mode } else { 'source' } }
Write-Host "libcarla acquisition mode: $Mode"
New-Item -ItemType Directory -Path $LibCarla -Force | Out-Null

if ($Mode -eq 'source') {
    if (-not $cfg.carla_root) { Write-Error "carla.json mode=source but carla_root is unset."; exit 1 }
    $src = Join-Path $cfg.carla_root 'PythonAPI\carla\dependencies'
    if (-not (Test-Path $src)) {
        Write-Error "Carla deps not found: $src`nBuild LibCarla from the Carla source first (see doc/CARLAdoc.md), or switch carla.json to mode 'prebuilt'."
        exit 1
    }
    foreach ($sub in 'lib', 'include') {
        $from = Join-Path $src $sub
        if (-not (Test-Path $from)) { Write-Error "missing $from"; exit 1 }
        Write-Host "  copying $sub/ from source ..."
        Copy-Item -Path $from -Destination $LibCarla -Recurse -Force
    }
    Write-Host "Copied libcarla from source: $src"
}
elseif ($Mode -eq 'prebuilt') {
    if (-not $DepsVersion) { $DepsVersion = Get-DepsVersion }
    $tag     = "fixs-deps-$DepsVersion"
    $zipName = "fixs-deps-$DepsVersion.zip"
    $url     = "https://github.com/$Repo/releases/download/$tag/$zipName"
    $tmp     = Join-Path $env:TEMP $zipName
    Write-Host "  downloading $url ..."
    try { Invoke-WebRequest -UseBasicParsing -Uri $url -OutFile $tmp }
    catch { Write-Error "download failed ($url). Is Binaries/deps release '$tag' published? $_"; exit 1 }

    # verify SHA-256 against the published sidecar (fail loudly on mismatch)
    try {
        $expected = ((Invoke-WebRequest -UseBasicParsing -Uri "$url.sha256").Content -split '\s+')[0].Trim().ToLower()
        $actual   = (Get-FileHash $tmp -Algorithm SHA256).Hash.ToLower()
        if ($expected -ne $actual) { Write-Error "checksum mismatch for $zipName`n  expected $expected`n  actual   $actual"; exit 1 }
        Write-Host "  checksum OK ($actual)"
    } catch {
        Write-Warning "no .sha256 sidecar for $tag - proceeding WITHOUT checksum verification."
    }

    # the zip is CommonLib-relative (libcarla/... [+ libsumo/... going forward])
    $CommonLib = Join-Path $RepoRoot 'CommonLib'
    Write-Host "  extracting into $CommonLib ..."
    Expand-Archive -Path $tmp -DestinationPath $CommonLib -Force
    Remove-Item $tmp -Force -ErrorAction SilentlyContinue
    Write-Host "Extracted $zipName into CommonLib/"
}
else { Write-Error "Unknown carla.json mode: '$Mode' (expected source|prebuilt)."; exit 1 }

# --- verify the result ------------------------------------------------------
if (Test-Path $SentinelLib) {
    $n = (Get-ChildItem (Join-Path $LibCarla 'lib') -Filter *.lib).Count
    Write-Host "libcarla ready: $n libs, include/=$([bool](Test-Path (Join-Path $LibCarla 'include')))"
    exit 0
} else {
    Write-Error "libcarla acquisition did not produce lib/carla_client.lib."
    exit 1
}
