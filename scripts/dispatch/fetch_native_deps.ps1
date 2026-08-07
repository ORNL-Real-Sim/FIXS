# ============================================================================
# Acquire the native C++ deps that do NOT live in git. Both are extracted into
# CommonLib/ and both are gitignored.
#
#   libsumo  - REQUIRED. The SUMO runtime (libsumo/libtraci headers + bin/).
#              TrafficLayer.vcxproj links CommonLib\libsumo\bin\libsumocpp.lib
#              directly, so nothing core builds without it. Always fetched from
#              the rolling 'fixs-native-deps' release (#238 dropped the ~430 MB
#              vendored copy from git).
#
#   libcarla - OPTIONAL. The Carla client SDK (~800 MB), needed only to build
#              VirCarlaEnv. Skips cleanly when Carla isn't configured, exactly
#              like MATLAB/dSPACE.
#
# libcarla has two acquisition modes, driven by the per-machine ~/.fixs/carla.json:
#
#   { "mode": "source", "carla_root": "C:/src_ext/Carla" }
#     -> copy <carla_root>/PythonAPI/carla/dependencies/{lib,include} into CommonLib/libcarla
#
#   { "mode": "prebuilt" }  (or explicit -Mode prebuilt; CI uses this)
#     -> download libcarla-<carla_ver>.zip from the release, verify SHA-256, extract.
#
# libsumo has no 'source' mode: rebuilding it from SUMO source is a separate,
# rarely-run utility (scripts/build_libsumo.ps1). -Mode applies to libcarla only.
#
# The release tag is version-LESS on purpose (component versions live on the asset
# names + in dependencies.yaml); multiple versions can coexist as extra assets.
#
# Idempotent: each component is skipped when already present, unless -Force.
#
# Usage:
#   fetch_native_deps.ps1                        # libsumo (required) + libcarla (if configured)
#   fetch_native_deps.ps1 -Component sumo        # only libsumo
#   fetch_native_deps.ps1 -Component carla -Mode prebuilt
#   fetch_native_deps.ps1 -Force                 # re-acquire even if present
#
# Exit codes: 0 = everything required is in place; 1 = a REQUIRED dep could not
# be acquired. A missing/failed libcarla is a warning, never a failure.
# ============================================================================
param(
    [string]$RepoRoot,
    [ValidateSet('', 'source', 'prebuilt')] [string]$Mode = '',
    [ValidateSet('both', 'carla', 'sumo')] [string]$Component = 'both',
    [string]$Repo = 'ORNL-Real-Sim/FIXS',
    [switch]$Force
)

$ErrorActionPreference = 'Stop'
if (-not $RepoRoot) { $RepoRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path }
$CommonLib     = Join-Path $RepoRoot 'CommonLib'
$LibCarla      = Join-Path $CommonLib 'libcarla'
$LibSumo       = Join-Path $CommonLib 'libsumo'
$SentinelCarla = Join-Path $LibCarla 'lib\carla_client.lib'
$SentinelSumo  = Join-Path $LibSumo  'bin\libsumocpp.lib'
$Tag           = 'fixs-native-deps'   # rolling release tag

. (Join-Path $PSScriptRoot 'libsumo_verify.ps1')

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
    catch { throw "download failed ($url). Is the '$Tag' release published with '$name'? $_" }
    try {
        # The CDN serves the tiny .sha256 as octet-stream, so .Content is a byte[]
        # (Invoke-WebRequest only string-decodes text content-types) - decode it.
        $c = (Invoke-WebRequest -UseBasicParsing -Uri "$url.sha256").Content
        if ($c -is [byte[]]) { $c = [System.Text.Encoding]::UTF8.GetString($c) }
        $expected = ($c -split '\s+')[0].Trim().ToLower()
        $actual   = (Get-FileHash $tmp -Algorithm SHA256).Hash.ToLower()
        if ($expected -ne $actual) { throw "checksum mismatch for ${name}: expected $expected got $actual" }
        Write-Host "    checksum OK ($actual)"
    } catch {
        if ("$_" -match 'checksum mismatch') { throw }
        Write-Warning "    could not verify .sha256 for ${name}: $($_.Exception.Message) - proceeding WITHOUT verification."
    }
    Expand-Archive -Path $tmp -DestinationPath $CommonLib -Force
    Remove-Item $tmp -Force -ErrorAction SilentlyContinue
}

# ============================================================================
# libsumo - REQUIRED, always from the rolling release
# ============================================================================
function Get-LibSumo {
    if (-not $Force -and (Test-Path $SentinelSumo)) {
        Write-Host "libsumo already present ($LibSumo) - skipping (use -Force to re-acquire)."
        return
    }
    $sver = Get-DepVersion 'sumo'
    if (-not $sver) { throw "could not read the sumo version from dependencies.yaml - cannot pick the libsumo asset." }
    Write-Host "Acquiring libsumo $sver (prebuilt, from the '$Tag' release)..."
    Get-Asset "libsumo-$sver.zip"

    # The zip is now the only source of the SUMO runtime, so verify it actually
    # loads rather than trusting that the file count looks right (#70 / #237).
    Test-LibsumoLoadable -BinDir (Join-Path $LibSumo 'bin') `
        -Context "The published libsumo-$sver.zip asset is incomplete - re-pack and re-publish it with scripts/dispatch/pack_native_deps.ps1 -Component sumo -Publish."
    Write-Host "libsumo ready: $LibSumo"
}

# ============================================================================
# libcarla - OPTIONAL, source copy or prebuilt fetch
# ============================================================================
function Get-LibCarla {
    if (-not $Force -and (Test-Path $SentinelCarla)) {
        Write-Host "libcarla already present ($LibCarla) - skipping (use -Force to re-acquire)."
        return $true
    }
    # resolve mode (explicit -Mode needs no carla.json; only 'source' does)
    $m = $Mode
    $cfg = Read-CarlaConfig
    if (-not $m) {
        if (-not $cfg) {
            Write-Warning "No ~/.fixs/carla.json and no -Mode - skipping libcarla (Carla is optional; VirCarlaEnv will not build). See doc/CARLAdoc.md."
            return $false
        }
        $m = if ($cfg.mode) { $cfg.mode } else { 'source' }
    }
    Write-Host "libcarla acquisition mode: $m"
    New-Item -ItemType Directory -Path $LibCarla -Force | Out-Null

    if ($m -eq 'source') {
        if (-not $cfg -or -not $cfg.carla_root) { throw "source mode needs ~/.fixs/carla.json with carla_root." }
        $src = Join-Path $cfg.carla_root 'PythonAPI\carla\dependencies'
        if (-not (Test-Path $src)) { throw "Carla deps not found: $src (build LibCarla first, or use mode 'prebuilt'). See doc/CARLAdoc.md." }
        foreach ($sub in 'lib', 'include') { Copy-Item (Join-Path $src $sub) -Destination $LibCarla -Recurse -Force }
        Write-Host "Copied libcarla from source: $src"
    }
    elseif ($m -eq 'prebuilt') {
        $cver = Get-DepVersion 'carla'; if (-not $cver) { $cver = '0.9.15' }
        Get-Asset "libcarla-$cver.zip"
    }
    else { throw "Unknown mode: '$m' (expected source|prebuilt)." }

    if (-not (Test-Path $SentinelCarla)) { throw "acquisition did not produce lib/carla_client.lib." }
    $n = (Get-ChildItem (Join-Path $LibCarla 'lib') -Filter *.lib).Count
    Write-Host "libcarla ready: $n libs, include/=$([bool](Test-Path (Join-Path $LibCarla 'include')))"
    return $true
}

# ============================================================================
# main - libsumo failures are fatal, libcarla failures are not
# ============================================================================
$failed = $false

if ($Component -in 'both', 'sumo') {
    try { Get-LibSumo }
    catch { Write-Error "libsumo acquisition failed (REQUIRED - TrafficLayer cannot link without it): $_"; $failed = $true }
}

if ($Component -in 'both', 'carla') {
    try { [void](Get-LibCarla) }
    catch { Write-Warning "libcarla acquisition failed (optional - VirCarlaEnv will be skipped): $_" }
}

if ($failed) { exit 1 }
exit 0
