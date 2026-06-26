# ====================================
# Create Release Zip
# Packages the build/ directory into a versioned zip file
# ====================================

param(
    [ValidateSet('standalone', 'inline')]
    [string]$RunMode = 'standalone'
)

$ScriptDir = $PSScriptRoot
$RepoRoot = (Resolve-Path (Join-Path $ScriptDir '..\..')).Path
$BuildDir = Join-Path $RepoRoot 'build'

function Exit-Script {
    param([int]$ExitCode = 0)
    if ($RunMode -eq 'standalone') {
        Write-Host ''
        pause
    }
    exit $ExitCode
}

if (-not (Test-Path $BuildDir)) {
    Write-Error "Build directory not found: $BuildDir"
    Exit-Script 1
}

# Determine version label from git
$VersionLabel = 'dev'
try {
    $tag = git describe --tags --abbrev=0 2>$null
    $commit = git rev-parse --short HEAD 2>$null
    if ($tag) {
        $VersionLabel = "$tag-$commit"
    } elseif ($commit) {
        $VersionLabel = "dev-$commit"
    }
} catch {}

$ZipName = "fixs-build-$VersionLabel.zip"
$ZipPath = Join-Path $BuildDir $ZipName

# Remove existing zip with same name
if (Test-Path $ZipPath) {
    Remove-Item $ZipPath -Force
}

Write-Host "Creating $ZipName (excluding libsumo DLLs)..."
try {
    # Create a temp staging directory excluding libsumo/bin (large DLLs fetched separately)
    $StagingDir = Join-Path ([System.IO.Path]::GetTempPath()) "fixs-zip-staging-$(Get-Random)"
    New-Item -ItemType Directory -Path $StagingDir -Force | Out-Null

    # Copy everything except libsumo/bin
    Get-ChildItem -Path $BuildDir -Exclude 'fixs-build-*' | ForEach-Object {
        if ($_.Name -eq 'CommonLib') {
            $destCommonLib = Join-Path $StagingDir 'CommonLib'
            New-Item -ItemType Directory -Path $destCommonLib -Force | Out-Null
            Get-ChildItem -Path $_.FullName | Where-Object { -not ($_.Name -eq 'libsumo') } | ForEach-Object {
                Copy-Item -Path $_.FullName -Destination $destCommonLib -Recurse -Force
            }
            # Copy libsumo headers only (not bin/)
            $libsumoSrc = Join-Path $_.FullName 'libsumo'
            if (Test-Path $libsumoSrc) {
                $libsumoDest = Join-Path $destCommonLib 'libsumo'
                New-Item -ItemType Directory -Path $libsumoDest -Force | Out-Null
                Get-ChildItem -Path $libsumoSrc -File | Copy-Item -Destination $libsumoDest -Force
            }
        } else {
            Copy-Item -Path $_.FullName -Destination $StagingDir -Recurse -Force
        }
    }

    # Include the conda env spec so the fetched FIXS/ folder carries the
    # canonical 'realsim' environment definition. carla is pulled from PyPI
    # (carla==0.9.15), so no wheel needs to be bundled here.
    $EnvYml = Join-Path $RepoRoot 'environment.yml'
    if (Test-Path $EnvYml) {
        Copy-Item -Path $EnvYml -Destination $StagingDir -Force
        Write-Host "  + environment.yml"
    }

    # Include the SUMO<->CARLA co-sim runtime (Carla/, minus the carla wheel) and
    # its shared Python utils (scripts/*.py) so the fetched FIXS release carries
    # the co-sim with its import layout (Carla/ + scripts/ side by side). Test-only
    # files (tests/Sumo/Carla) and the .ps1 build tooling stay source-side.
    $CarlaSrc = Join-Path $RepoRoot 'Carla'
    if (Test-Path $CarlaSrc) {
        $carlaDest = Join-Path $StagingDir 'Carla'
        New-Item -ItemType Directory -Path $carlaDest -Force | Out-Null
        foreach ($item in @('helper_scripts', 'run_cosim.py', 'run_cosim.bat', 'run_cosim.sh', 'README.md')) {
            $p = Join-Path $CarlaSrc $item
            if (Test-Path $p) { Copy-Item -Path $p -Destination $carlaDest -Recurse -Force }
        }
        Get-ChildItem -Path $carlaDest -Recurse -Directory -Filter '__pycache__' -ErrorAction SilentlyContinue |
            Remove-Item -Recurse -Force -ErrorAction SilentlyContinue
        Write-Host "  + Carla/ co-sim runtime"
    }
    $scriptsDest = Join-Path $StagingDir 'scripts'
    New-Item -ItemType Directory -Path $scriptsDest -Force | Out-Null
    foreach ($u in @('trafficlight_helper.py', 'extract_sumo_tls_as_table.py', 'unreal_remove_tl.py')) {
        $p = Join-Path (Join-Path $RepoRoot 'scripts') $u
        if (Test-Path $p) { Copy-Item -Path $p -Destination $scriptsDest -Force }
    }
    Write-Host "  + scripts/ co-sim utils"

    Compress-Archive -Path "$StagingDir\*" -DestinationPath $ZipPath -CompressionLevel Optimal
    Remove-Item $StagingDir -Recurse -Force

    $size = [math]::Round((Get-Item $ZipPath).Length / 1MB, 1)
    Write-Host "Created: $ZipPath ($size MB)"
} catch {
    Write-Error "Failed to create zip: $_"
    if (Test-Path $StagingDir) { Remove-Item $StagingDir -Recurse -Force }
    Exit-Script 1
}

Exit-Script 0
