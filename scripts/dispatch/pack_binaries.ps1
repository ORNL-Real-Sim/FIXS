# ============================================================================
# Pack FIXS proprietary-binaries bundle (issue #191)
# ----------------------------------------------------------------------------
# Run on a LICENSED workstation AFTER a full dispatch (build/ must already hold
# the CarMaker / VISSIM / dSPACE / MATLAB outputs). Zips ONLY the proprietary-
# toolchain subset of build/ - at their build/-relative paths - into
# fixs-binaries-<key>.zip, and (with -Publish) uploads it as the release
# Binaries-<key> on the FIXS repo itself.
#
# <key> is the bundle key (bundle_key.ps1): <ProprietaryFiles-sha[:12]>.
# The FIXS release CI (release.yml) computes the same key from its tree,
# downloads Binaries-<key>, unzips it INTO build/ (overlay), and packs the
# single canonical FIXS release zip. The key pairs a bundle to the proprietary
# source it was built from; a bundle-guard PR check enforces "PF pointer moved
# -> a matching Binaries-<key> must exist".
#
# Usage:
#   pack_binaries.ps1               # pack only, into .\dist\
#   pack_binaries.ps1 -Publish      # pack + publish the Binaries-<key> release
# ============================================================================
param(
    [switch]$Publish,
    [string]$Repo = 'ORNL-Real-Sim/FIXS',
    [string]$OutDir
)

$ErrorActionPreference = 'Stop'
$RepoRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
$BuildDir = Join-Path $RepoRoot 'build'
if (-not $OutDir) { $OutDir = Join-Path $RepoRoot 'dist' }

if (-not (Test-Path $BuildDir)) {
    Write-Error "build/ not found. Run a full dispatch (dispatch.bat, no RS_FIXS_AUTOMATION) first."
    exit 1
}

# --- Proprietary subset (build/-relative globs) -----------------------------
# Exactly the files dispatch step 7 copies from licensed-toolchain outputs; the
# hosted CI cannot produce these.
$Patterns = @(
    'DriverModel_RealSim.dll',              # VISSIM driver model (default, int API, 2021+)
    'DriverModel_RealSim_legacy.dll',       # VISSIM driver model (frozen, long API, <=2020)
    'CarMaker\*\CarMaker.win64.exe',        # CarMaker executable (per CM version)
    'CarMaker\*\*.mexw64',                  # libcarmaker4sl MEX (per CM version)
    'CarMaker\*\libRealSimDsLib_*.a',       # dSPACE library staged under CarMaker
    'CommonLib\libRealSimDsLib_*.a',        # dSPACE library staged under CommonLib
    'CommonLib\RealSimSocket.mexw64'        # MATLAB MEX
)

$found = New-Object System.Collections.Generic.List[System.IO.FileInfo]
foreach ($p in $Patterns) {
    Get-ChildItem -Path (Join-Path $BuildDir $p) -File -ErrorAction SilentlyContinue |
        ForEach-Object { $found.Add($_) }
}
if ($found.Count -eq 0) {
    Write-Error "No proprietary binaries found under $BuildDir. Did the full (non-automation) dispatch build VISSIM/CarMaker/dSPACE/MEX?"
    exit 1
}

# --- Composite key + its two dimensions (for the manifest) ------------------
$key     = (& (Join-Path $PSScriptRoot 'bundle_key.ps1') -RepoRoot $RepoRoot).Trim()
$pfSha   = (($( & git -C $RepoRoot ls-tree HEAD ProprietaryFiles )) -split '\s+')[2]
$msgHash = (& (Join-Path $PSScriptRoot 'msg_contract_hash.ps1') -RepoRoot $RepoRoot).Trim()
$tag     = "Binaries-$key"

# --- Stage the overlay at build/-relative paths + manifest ------------------
$Stage = Join-Path ([System.IO.Path]::GetTempPath()) "fixs-binaries-stage-$(Get-Random)"
New-Item -ItemType Directory -Path $Stage -Force | Out-Null
$relList = New-Object System.Collections.Generic.List[string]
foreach ($f in $found) {
    $rel = $f.FullName.Substring($BuildDir.Length).TrimStart('\', '/')
    $relList.Add(($rel -replace '\\', '/'))
    $dest = Join-Path $Stage $rel
    New-Item -ItemType Directory -Path (Split-Path $dest -Parent) -Force | Out-Null
    Copy-Item -Path $f.FullName -Destination $dest -Force
}

$manifest = [ordered]@{
    bundle_key            = $key
    proprietary_files_sha = $pfSha
    msg_contract_hash     = $msgHash
    file_count            = $found.Count
    files                 = $relList
    note                  = 'Overlay: unzip into build/ (excluding manifest.json) before packing the FIXS release zip.'
}
$manifest | ConvertTo-Json -Depth 4 | Set-Content -Path (Join-Path $Stage 'manifest.json') -Encoding utf8

# --- Zip --------------------------------------------------------------------
New-Item -ItemType Directory -Path $OutDir -Force | Out-Null
$BundleName = "fixs-binaries-$key.zip"
$BundlePath = Join-Path $OutDir $BundleName
if (Test-Path $BundlePath) { Remove-Item $BundlePath -Force }
Compress-Archive -Path "$Stage\*" -DestinationPath $BundlePath -CompressionLevel Optimal
Remove-Item $Stage -Recurse -Force

$size = [math]::Round((Get-Item $BundlePath).Length / 1MB, 1)
Write-Host "Packed $BundleName ($size MB, $($found.Count) files)"
Write-Host "  bundle key:           $key"
Write-Host "  ProprietaryFiles SHA: $pfSha"
Write-Host "  msg-contract hash:    $msgHash"
$relList | ForEach-Object { Write-Host "    + $_" }

# --- Publish (optional) -----------------------------------------------------
if ($Publish) {
    if (-not (Get-Command gh -ErrorAction SilentlyContinue)) {
        Write-Error "GitHub CLI (gh) required for -Publish."
        exit 1
    }
    $notes = "Prebuilt FIXS proprietary-toolchain binaries.`nBundle key: ``$key`` (ProprietaryFiles ``$pfSha``, msg-contract ``$msgHash``)."
    # Per-key prerelease: create if new, clobber the asset if the key was already
    # published (a rebuild for the same inputs).
    & gh release view $tag --repo $Repo *> $null
    if ($LASTEXITCODE -eq 0) {
        Write-Host "Release '$tag' exists - updating asset..."
        & gh release upload $tag $BundlePath --repo $Repo --clobber
    } else {
        Write-Host "Creating release '$tag'..."
        # --target the built commit so the bundle release doesn't default-anchor
        # to the repo's default branch (main).
        $target = (& git -C $RepoRoot rev-parse HEAD).Trim()
        & gh release create $tag $BundlePath --repo $Repo --prerelease --target $target `
            --title "FIXS binaries $key" --notes $notes
    }
    if ($LASTEXITCODE -eq 0) {
        Write-Host "Published $tag to $Repo."
    } else {
        Write-Error "Publish to $Repo failed."
        exit 1
    }
}
