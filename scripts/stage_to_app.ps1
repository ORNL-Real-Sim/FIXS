<#
.SYNOPSIS
    Install THIS checkout into an application repo's FIXS/ bundle, as a release would.

.DESCRIPTION
    An application repo runs against a FETCHED release bundle in <app>/FIXS/, not
    against a FIXS source tree -- there is no source-checkout mode, by design (see
    fixs_sources.txt). So to try an unreleased change from an application's own
    front door, the bundle has to be replaced with what this checkout would ship.

    That is what this does. It copies the same parts 8_create_zip.ps1 packs:

        CommonLib/*.py + CommonLib/VirEnv/     the Python engine
        Carla/                                 run_cosim and the bridges
        TrafficLayer.exe, VirCarlaEnv.exe      only with -IncludeBinaries

    standalone/ is deliberately NOT copied: it is outside the packed tree (#330)
    because it speaks no FIXS, and a bundle is exactly where it should be absent.

    The first run saves the bundle's original files to FIXS/_prestage_backup, so
    -Restore puts the fetched release back without a re-download.

.PARAMETER AppRepo
    The application checkout to stage into -- the directory holding FIXS/ and
    apps/ (e.g. C:\src_git\RS_FIXS_Applications\dev).

.PARAMETER IncludeBinaries
    Also stage TrafficLayer.exe / VirCarlaEnv.exe from this checkout. OFF by
    default: a stray x64 Release build can be any age and any branch, and staging
    it silently replaces the release binary a scenario was verified against. The
    Python bridge needs neither rebuilt.

.PARAMETER Restore
    Put the fetched bundle back and remove everything this staged.

.EXAMPLE
    powershell -File scripts\stage_to_app.ps1 -AppRepo C:\src_git\RS_FIXS_Applications\dev
    powershell -File scripts\stage_to_app.ps1 -AppRepo C:\src_git\RS_FIXS_Applications\dev -Restore

.NOTES
    A development aid, not part of the release path. Once the branch ships,
    `run_cosim --update-fixs` is the supported way to get the same code -- and it
    OVERWRITES what this staged, which is the intended direction.
#>
param(
    [Parameter(Mandatory = $true)][string]$AppRepo,
    [switch]$Restore,
    [switch]$IncludeBinaries
)

$ErrorActionPreference = 'Stop'
$Src = Split-Path -Parent $PSScriptRoot
$Dst = Join-Path $AppRepo 'FIXS'
$Bak = Join-Path $Dst '_prestage_backup'

if (-not (Test-Path $Dst)) {
    throw "No FIXS bundle at $Dst. Fetch one first (run_cosim --update-fixs)."
}

# ---- restore -------------------------------------------------------------
if ($Restore) {
    if (-not (Test-Path $Bak)) { Write-Host "Nothing staged (no $Bak)."; exit 0 }
    Copy-Item (Join-Path $Bak 'CommonLib\*.py') (Join-Path $Dst 'CommonLib') -Force
    Copy-Item (Join-Path $Bak 'Carla') $Dst -Recurse -Force
    foreach ($exe in 'TrafficLayer.exe', 'VirCarlaEnv.exe') {
        $b = Join-Path $Bak $exe
        if (Test-Path $b) { Copy-Item $b $Dst -Force }
    }
    Remove-Item (Join-Path $Dst 'CommonLib\VirEnv') -Recurse -Force -ErrorAction SilentlyContinue
    Remove-Item $Bak -Recurse -Force
    Write-Host "Restored the fetched bundle at $Dst."
    exit 0
}

# ---- back up once, so the fetched release is recoverable ------------------
if (-not (Test-Path $Bak)) {
    New-Item -ItemType Directory -Path (Join-Path $Bak 'CommonLib') -Force | Out-Null
    Copy-Item (Join-Path $Dst 'CommonLib\*.py') (Join-Path $Bak 'CommonLib') -Force
    Copy-Item (Join-Path $Dst 'Carla') $Bak -Recurse -Force
    foreach ($exe in 'TrafficLayer.exe', 'VirCarlaEnv.exe') {
        $d = Join-Path $Dst $exe
        if (Test-Path $d) { Copy-Item $d $Bak -Force }
    }
    Write-Host "Backed up the fetched bundle -> $Bak"
}

# ---- the Python engine ---------------------------------------------------
Copy-Item (Join-Path $Src 'CommonLib\*.py') (Join-Path $Dst 'CommonLib') -Force
Copy-Item (Join-Path $Src 'CommonLib\VirEnv') (Join-Path $Dst 'CommonLib') -Recurse -Force

# ---- Carla/: run_cosim, the bridges, the map tooling ---------------------
# Removed first rather than merged: a file deleted in this checkout has to
# disappear from the bundle too, or the bundle keeps running the old one.
Remove-Item (Join-Path $Dst 'Carla') -Recurse -Force
Copy-Item (Join-Path $Src 'Carla') $Dst -Recurse -Force
Get-ChildItem (Join-Path $Dst 'Carla') -Recurse -Directory -Filter '__pycache__' |
    Remove-Item -Recurse -Force -ErrorAction SilentlyContinue

# ---- binaries: OPT-IN, never by default ----------------------------------
# A stray x64\Release\*.exe in a source tree can be any age and any branch, and
# staging it silently replaces the RELEASE binary that a scenario was verified
# against -- which is how a run stops meaning what you think it means. The
# Python bridge needs neither exe rebuilt, so the fetched ones stay put unless
# you say otherwise. Pass -IncludeBinaries when you have deliberately built the
# C++ side and want to run THAT.
$built = @{
    'TrafficLayer.exe' = Join-Path $Src 'TrafficLayer\x64\Release\TrafficLayer.exe'
    'VirCarlaEnv.exe'  = Join-Path $Src 'VirCarlaEnv\VirCarlaEnv\x64\Release\VirCarlaEnv.exe'
}
foreach ($name in $built.Keys) {
    if (-not $IncludeBinaries) {
        Write-Host "  kept   $name (fetched release; -IncludeBinaries to stage a local build)"
    } elseif (Test-Path $built[$name]) {
        $when = (Get-Item $built[$name]).LastWriteTime
        Copy-Item $built[$name] (Join-Path $Dst $name) -Force
        Write-Host "  staged $name (local build, $when)"
    } else {
        Write-Host "  kept   $name (not built here)"
    }
}
Write-Host ""
Write-Host "Staged $Src -> $Dst"
Write-Host "Undo with: powershell -File scripts\stage_to_app.ps1 -AppRepo $AppRepo -Restore"
