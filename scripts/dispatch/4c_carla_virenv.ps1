# ============================================================================
# Build the Carla virtual-environment executable (VirCarlaEnv.exe).
# ----------------------------------------------------------------------------
# PUBLIC component (open-source Carla) - builds on CI and dev boxes alike, so it
# is NOT gated by RS_FIXS_AUTOMATION. It just needs libcarla, which is acquired
# first via fetch_native_deps.ps1 (~/.fixs/carla.json: source copy or prebuilt
# fetch). Depends on VirtualEnvironment.lib (step 4) + yaml-cpp + libsumo.
#
# Skips cleanly (exit 0) when Carla isn't configured - Carla is optional.
# Output: VirCarlaEnv\x64\Release\VirCarlaEnv.exe, copied to build\.
# ============================================================================
param([string]$RunMode = 'inline')

$ErrorActionPreference = 'Stop'
$RepoRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
Write-Host "=== Carla virtual environment (libcarla + VirCarlaEnv) ==="

# --- 1. acquire libcarla (source copy or prebuilt fetch) --------------------
& (Join-Path $PSScriptRoot 'fetch_native_deps.ps1') -RepoRoot $RepoRoot
if ($LASTEXITCODE -ne 0) { Write-Warning "libcarla acquisition failed - skipping VirCarlaEnv."; exit 0 }

$sentinel = Join-Path $RepoRoot 'CommonLib\libcarla\lib\carla_client.lib'
if (-not (Test-Path $sentinel)) {
    Write-Host "libcarla not available - skipping VirCarlaEnv build (Carla is optional)."
    exit 0
}

# --- 2. resolve msbuild (self-contained, same as pack_binaries) -------------
$vswhere = Join-Path ${env:ProgramFiles(x86)} 'Microsoft Visual Studio\Installer\vswhere.exe'
$msbuild = & $vswhere -latest -requires Microsoft.Component.MSBuild -find 'MSBuild\**\Bin\MSBuild.exe' | Select-Object -First 1
if (-not $msbuild) { Write-Error "MSBuild not found (vswhere)."; exit 1 }

# --- 3. build VirCarlaEnv.sln (Release) -------------------------------------
$sln = Join-Path $RepoRoot 'VirCarlaEnv\VirCarlaEnv.sln'
$cfg = if ($env:RS_BUILD_CONFIG) { $env:RS_BUILD_CONFIG } else { 'Release' }
Write-Host "Building VirCarlaEnv ($cfg) ..."
& $msbuild $sln /p:Configuration=$cfg /v:minimal /nologo
if ($LASTEXITCODE -ne 0) { Write-Error "VirCarlaEnv build failed."; exit 1 }

$exe = Join-Path $RepoRoot "VirCarlaEnv\x64\$cfg\VirCarlaEnv.exe"
if (-not (Test-Path $exe)) { Write-Error "VirCarlaEnv.exe not produced: $exe"; exit 1 }

# --- 4. copy to build\ ------------------------------------------------------
$buildDir = Join-Path $RepoRoot 'build'
New-Item -ItemType Directory -Path $buildDir -Force | Out-Null
Copy-Item -Path $exe -Destination $buildDir -Force
$mb = [math]::Round((Get-Item $exe).Length / 1MB, 1)
Write-Host "VirCarlaEnv.exe built + copied to build\ ($mb MB)."
exit 0
