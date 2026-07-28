# ============================================================================
# Acquire the vendored native C++ deps that don't live in git - libcarla (the
# Carla client SDK, gitignored ~800 MB) and, when it's dropped from git later,
# libsumo. Driven by the per-machine ~/.fixs/carla.json:
#
#   { "mode": "source", "carla_root": "C:/src_ext/Carla" }
#     -> copy <carla_root>/PythonAPI/carla/dependencies/{lib,include} into CommonLib/libcarla
#
#   { "mode": "prebuilt" }  (or explicit -Mode prebuilt; CI uses this)
#     -> download the PUBLIC, ROLLING 'fixs-native-deps' release's per-component,
#        version-named assets - libcarla-<carla_ver>.zip (+ libsumo-<sumo_ver>.zip
#        only if libsumo isn't already vendored) - verify SHA-256, extract into CommonLib/.
#
# The release tag is version-LESS on purpose (component versions live on the asset
# names + in dependencies.yaml); multiple versions can coexist as extra assets.
#
# Idempotent (skips if libcarla present unless -Force); skips cleanly if Carla
# isn't configured (optional, like MATLAB/dSPACE).
# ============================================================================
param(
    [string]$RepoRoot,
    [ValidateSet('', 'source', 'prebuilt')] [string]$Mode = '',
    [string]$Repo = 'ORNL-Real-Sim/FIXS',
    [switch]$Force
)

$ErrorActionPreference = 'Stop'
if (-not $RepoRoot) { $RepoRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path }
$CommonLib     = Join-Path $RepoRoot 'CommonLib'
$LibCarla      = Join-Path $CommonLib 'libcarla'
$SentinelCarla = Join-Path $LibCarla 'lib\carla_client.lib'
$Tag           = 'fixs-native-deps'   # rolling release tag

function Read-CarlaConfig {
    $p = Join-Path $env:USERPROFILE '.fixs\carla.json'
    if (Test-Path $p) { return (Get-Content $p -Raw | ConvertFrom-Json) }
    return $null
}
function Get-DepVersion([string]$block) {
    $depsYaml = Join-Path $RepoRoot 'dependencies.yaml'
    if (Test-Path $depsYaml) {
        $c = Get-Content $depsYaml -Raw
        if ($c -match "(?ms)^\s*${block}:\s.*?^\s*version:\s*[`"']?([0-9][0-9.]*)") { return $Matches[1] }
    }
    return $null
}
function Get-Asset([string]$name) {
    # download <name> + verify its .sha256 sidecar, then extract into CommonLib/
    $url = "https://github.com/$Repo/releases/download/$Tag/$name"
    $tmp = Join-Path $env:TEMP $name
    Write-Host "  downloading $name ..."
    try { Invoke-WebRequest -UseBasicParsing -Uri $url -OutFile $tmp }
    catch { Write-Error "download failed ($url). Is the '$Tag' release published with '$name'? $_"; exit 1 }
    try {
        $expected = ((Invoke-WebRequest -UseBasicParsing -Uri "$url.sha256").Content -split '\s+')[0].Trim().ToLower()
        $actual   = (Get-FileHash $tmp -Algorithm SHA256).Hash.ToLower()
        if ($expected -ne $actual) { Write-Error "checksum mismatch for ${name}: expected $expected got $actual"; exit 1 }
        Write-Host "    checksum OK"
    } catch { Write-Warning "    no .sha256 sidecar for $name - proceeding WITHOUT checksum verification." }
    Expand-Archive -Path $tmp -DestinationPath $CommonLib -Force
    Remove-Item $tmp -Force -ErrorAction SilentlyContinue
}

# --- idempotent short-circuit -----------------------------------------------
if (-not $Force -and (Test-Path $SentinelCarla)) {
    Write-Host "libcarla already present ($LibCarla) - skipping (use -Force to re-acquire)."
    exit 0
}

# --- resolve mode (explicit -Mode needs no carla.json; only 'source' does) ---
if (-not $Mode) {
    $cfg = Read-CarlaConfig
    if (-not $cfg) {
        Write-Warning "No ~/.fixs/carla.json and no -Mode - skipping libcarla acquisition (Carla optional; VirCarlaEnv will not build). See doc/CARLAdoc.md."
        exit 0
    }
    $Mode = if ($cfg.mode) { $cfg.mode } else { 'source' }
} else {
    $cfg = Read-CarlaConfig
}
Write-Host "native-deps acquisition mode: $Mode"
New-Item -ItemType Directory -Path $LibCarla -Force | Out-Null

if ($Mode -eq 'source') {
    if (-not $cfg -or -not $cfg.carla_root) { Write-Error "source mode needs ~/.fixs/carla.json with carla_root."; exit 1 }
    $src = Join-Path $cfg.carla_root 'PythonAPI\carla\dependencies'
    if (-not (Test-Path $src)) { Write-Error "Carla deps not found: $src (build LibCarla first, or use mode 'prebuilt'). See doc/CARLAdoc.md."; exit 1 }
    foreach ($sub in 'lib', 'include') { Copy-Item (Join-Path $src $sub) -Destination $LibCarla -Recurse -Force }
    Write-Host "Copied libcarla from source: $src"
}
elseif ($Mode -eq 'prebuilt') {
    $cver = Get-DepVersion 'carla'; if (-not $cver) { $cver = '0.9.15' }
    Get-Asset "libcarla-$cver.zip"
    # libsumo is still committed today; fetch it only if it's absent (future-proof).
    if (-not (Test-Path (Join-Path $CommonLib 'libsumo'))) {
        $sver = Get-DepVersion 'sumo'; if ($sver) { Get-Asset "libsumo-$sver.zip" }
    }
    Write-Host "Extracted native deps into CommonLib/"
}
else { Write-Error "Unknown mode: '$Mode' (expected source|prebuilt)."; exit 1 }

# --- verify -----------------------------------------------------------------
if (Test-Path $SentinelCarla) {
    $n = (Get-ChildItem (Join-Path $LibCarla 'lib') -Filter *.lib).Count
    Write-Host "libcarla ready: $n libs, include/=$([bool](Test-Path (Join-Path $LibCarla 'include')))"
    exit 0
} else { Write-Error "acquisition did not produce lib/carla_client.lib."; exit 1 }
