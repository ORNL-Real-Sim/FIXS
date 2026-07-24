# ============================================================================
# Pack FIXS-Binaries bundle (issue #191)
# ----------------------------------------------------------------------------
# Run on a LICENSED workstation AFTER a full dispatch (build/ must already hold
# the CarMaker / VISSIM / dSPACE / MATLAB outputs). Zips ONLY the proprietary-
# toolchain subset of build/ - at their build/-relative paths - into
# fixs-binaries-<PF-sha>.zip, plus a manifest for the CI compatibility guard.
# The FIXS release CI downloads this, unzips it INTO build/ (overlay), and packs
# everything into the single canonical FIXS release zip.
#
# Usage:
#   pack_binaries.ps1                 # pack only, into .\dist\
#   pack_binaries.ps1 -Publish        # pack + upload to the FIXS-Binaries repo
# ============================================================================
param(
    [switch]$Publish,
    [string]$BinariesRepo = 'ORNL-Real-Sim/FIXS-Binaries',
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

# --- The proprietary subset (build/-relative globs) -------------------------
# Exactly the files dispatch step 7 copies from licensed-toolchain outputs; the
# hosted CI cannot produce these.
$Patterns = @(
    'DriverModel_RealSim.dll',              # VISSIM driver model
    'DriverModel_RealSim_v2021.dll',        # VISSIM driver model (2021)
    'CarMaker\*\CarMaker.win64.exe',        # CarMaker executable (per CM version)
    'CarMaker\*\*.mexw64',                  # libcarmaker4sl MEX (per CM version)
    'CarMaker\*\libRealSimDsLib_*.a',       # dSPACE library (per CM version)
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

# --- Keys: ProprietaryFiles submodule SHA + message-contract hash -----------
$pfLine = (& git -C $RepoRoot ls-tree HEAD ProprietaryFiles) 2>&1
if ($LASTEXITCODE -ne 0 -or -not $pfLine) {
    Write-Error "Could not read the ProprietaryFiles submodule pointer via git ls-tree."
    exit 1
}
$pfSha = ($pfLine -split '\s+')[2]
$msgHash = & powershell -NoProfile -ExecutionPolicy Bypass -File (Join-Path $PSScriptRoot 'msg_contract_hash.ps1') -RepoRoot $RepoRoot

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
    proprietary_files_sha = $pfSha
    msg_contract_hash     = $msgHash.Trim()
    file_count            = $found.Count
    files                 = $relList
    packed_from           = 'build/'
    note                  = 'Overlay: unzip into build/ (excluding manifest.json) before packing the FIXS release zip.'
}
$manifest | ConvertTo-Json -Depth 4 | Set-Content -Path (Join-Path $Stage 'manifest.json') -Encoding utf8

# --- Zip --------------------------------------------------------------------
New-Item -ItemType Directory -Path $OutDir -Force | Out-Null
$BundleName = "fixs-binaries-$pfSha.zip"
$BundlePath = Join-Path $OutDir $BundleName
if (Test-Path $BundlePath) { Remove-Item $BundlePath -Force }
Compress-Archive -Path "$Stage\*" -DestinationPath $BundlePath -CompressionLevel Optimal
Remove-Item $Stage -Recurse -Force

$size = [math]::Round((Get-Item $BundlePath).Length / 1MB, 1)
Write-Host "Packed $BundleName ($size MB, $($found.Count) files)"
Write-Host "  ProprietaryFiles SHA: $pfSha"
Write-Host "  msg-contract hash:    $($msgHash.Trim())"
$relList | ForEach-Object { Write-Host "    + $_" }

# --- Publish (optional) -----------------------------------------------------
if ($Publish) {
    if (-not (Get-Command gh -ErrorAction SilentlyContinue)) {
        Write-Error "GitHub CLI (gh) required for -Publish."
        exit 1
    }
    $notes = "Prebuilt FIXS binaries for ProprietaryFiles ``$pfSha``.`nmsg-contract: ``$($msgHash.Trim())``"
    # Rolling per-SHA release: create if new, clobber the asset if the SHA was
    # already published (a rebuild for the same source).
    & gh release view $pfSha --repo $BinariesRepo *> $null
    if ($LASTEXITCODE -eq 0) {
        Write-Host "Release '$pfSha' exists - updating asset..."
        & gh release upload $pfSha $BundlePath --repo $BinariesRepo --clobber
    } else {
        Write-Host "Creating release '$pfSha'..."
        & gh release create $pfSha $BundlePath --repo $BinariesRepo `
            --title "FIXS binaries @ ProprietaryFiles $($pfSha.Substring(0,12))" --notes $notes
    }
    if ($LASTEXITCODE -eq 0) {
        Write-Host "Published to $BinariesRepo (tag $pfSha)."
    } else {
        Write-Error "Upload to $BinariesRepo failed."
        exit 1
    }
}
