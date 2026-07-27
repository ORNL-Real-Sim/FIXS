# ============================================================================
# Pack the FIXS native-deps bundle (libcarla + libsumo) - issue #109.
# ----------------------------------------------------------------------------
# These are large C++ client SDKs that don't belong in git (libcarla is
# gitignored ~800 MB; libsumo is committed ~413 MB but moving to fetch-only).
# This packs them, CommonLib-relative, into fixs-deps-<ver>.zip + a .sha256, and
# (with -Publish) uploads a PUBLIC prerelease fixs-deps-<ver> on the FIXS repo -
# the open-source counterpart to the proprietary Binaries-<key> bundle. CI and
# developers fetch it via scripts/dispatch/fetch_carla_deps.ps1 (prebuilt mode).
#
# libcarla ships the RELEASE SUBSET: carla_client_debug.lib (319 MB) is dropped -
# VirCarlaEnv's Release build links only carla_client.lib (proven in #109). Debug
# builds are out of scope (no debug Boost ships in the Carla deps anyway).
#
# Usage:
#   pack_carla_deps.ps1                 # pack into .\dist\
#   pack_carla_deps.ps1 -Publish        # pack + publish fixs-deps-<ver>
# ============================================================================
param(
    [switch]$Publish,
    [string]$Version,
    [string]$Repo = 'ORNL-Real-Sim/FIXS',
    [string]$OutDir
)

$ErrorActionPreference = 'Stop'
$RepoRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
$CommonLib = Join-Path $RepoRoot 'CommonLib'
if (-not $OutDir) { $OutDir = Join-Path $RepoRoot 'dist' }

if (-not $Version) {
    $depsYaml = Join-Path $RepoRoot 'dependencies.yaml'
    $c = if (Test-Path $depsYaml) { Get-Content $depsYaml -Raw } else { '' }
    $Version = if ($c -match '(?ms)^\s*carla:\s.*?^\s*version:\s*["'']?([0-9][0-9.]*)') { $Matches[1] } else { '0.9.15' }
}
Write-Host "Native-deps version: $Version"

$libcarla = Join-Path $CommonLib 'libcarla'
$libsumo  = Join-Path $CommonLib 'libsumo'
if (-not (Test-Path (Join-Path $libcarla 'lib\carla_client.lib'))) {
    Write-Error "CommonLib/libcarla not present. Acquire it first (fetch_carla_deps.ps1 -Mode source)."
    exit 1
}

# --- stage CommonLib-relative -----------------------------------------------
$Stage = Join-Path ([System.IO.Path]::GetTempPath()) "fixs-deps-stage-$(Get-Random)"
$sCarla = Join-Path $Stage 'libcarla'
New-Item -ItemType Directory -Path (Join-Path $sCarla 'lib') -Force | Out-Null

# libcarla: include/ + lib/ minus the 319 MB debug client (release subset)
Copy-Item -Path (Join-Path $libcarla 'include') -Destination $sCarla -Recurse -Force
Get-ChildItem (Join-Path $libcarla 'lib') -File |
    Where-Object { $_.Name -ne 'carla_client_debug.lib' } |
    ForEach-Object { Copy-Item $_.FullName -Destination (Join-Path $sCarla 'lib') -Force }
$dropped = Test-Path (Join-Path $libcarla 'lib\carla_client_debug.lib')
Write-Host "  libcarla: staged release subset$(if($dropped){' (dropped carla_client_debug.lib)'})"

# libsumo: everything except the out/build cmake tree (headers + bin/)
if (Test-Path $libsumo) {
    $sSumo = Join-Path $Stage 'libsumo'
    New-Item -ItemType Directory -Path $sSumo -Force | Out-Null
    Get-ChildItem $libsumo | Where-Object { $_.Name -ne 'out' } |
        ForEach-Object { Copy-Item $_.FullName -Destination $sSumo -Recurse -Force }
    Write-Host "  libsumo: staged (headers + bin/)"
}

# --- zip + checksum ---------------------------------------------------------
New-Item -ItemType Directory -Path $OutDir -Force | Out-Null
$zipName = "fixs-deps-$Version.zip"
$zipPath = Join-Path $OutDir $zipName
if (Test-Path $zipPath) { Remove-Item $zipPath -Force }
Write-Host "Compressing $zipName ..."
Compress-Archive -Path "$Stage\*" -DestinationPath $zipPath -CompressionLevel Optimal
Remove-Item $Stage -Recurse -Force

$sha = (Get-FileHash $zipPath -Algorithm SHA256).Hash.ToLower()
Set-Content -Path "$zipPath.sha256" -Value "$sha  $zipName" -Encoding ascii
$size = [math]::Round((Get-Item $zipPath).Length / 1MB, 1)
Write-Host "Packed $zipName ($size MB)"
Write-Host "  sha256: $sha"

# --- publish (optional) -----------------------------------------------------
if ($Publish) {
    $tag = "fixs-deps-$Version"
    $notes = "Prebuilt FIXS native C++ deps (libcarla release subset + libsumo) for Carla version ``$Version``.`nPublic, open-source - fetched by scripts/dispatch/fetch_carla_deps.ps1 (prebuilt mode) and the release CI.`nsha256(``$zipName``): ``$sha``"
    $prevEAP = $ErrorActionPreference; $ErrorActionPreference = 'Continue'
    & gh release view $tag --repo $Repo *> $null
    $exists = ($LASTEXITCODE -eq 0)
    $ErrorActionPreference = $prevEAP
    $target = (& git -C $RepoRoot rev-parse HEAD).Trim()
    if ($exists) {
        Write-Host "Release '$tag' exists - updating assets..."
        & gh release upload $tag $zipPath "$zipPath.sha256" --repo $Repo --clobber
    } else {
        Write-Host "Creating release '$tag'..."
        & gh release create $tag $zipPath "$zipPath.sha256" --repo $Repo --prerelease --target $target `
            --title "FIXS native deps $Version" --notes $notes
    }
    if ($LASTEXITCODE -ne 0) { Write-Error "Publish failed."; exit 1 }
    Write-Host "Published $tag to $Repo."
}
