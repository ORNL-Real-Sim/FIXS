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

# Determine version label from git.
# Full semver-matched describe (e.g. v0.8.0-120-gce90f3c0) so the zip name is
# traceable to the exact commit. --match 'v[0-9]*' ignores the rolling
# lightweight tags (latest, alpha_v0.9.0) that would otherwise shadow the real
# semver tag (#191); --always degrades a tagless checkout to the short SHA.
$VersionLabel = 'dev'
try {
    $describe = git describe --tags --match 'v[0-9]*' --always 2>$null
    $commit = git rev-parse --short HEAD 2>$null
    if ($describe) {
        $VersionLabel = $describe
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

            # Drop the vendored YAMLMatlab unit-test fixtures from the release.
            # They are dead weight at runtime (nothing references
            # CommonLib/YAMLMatlab/Tests), and their directories extract without
            # the execute bit on Linux, which breaks a downstream `rm -rf FIXS/`
            # on re-init (see FIXS #190). Excluded here at pack time rather than
            # deleted from source, so the vendored lib stays complete for future
            # YAMLMatlab drop-in updates.
            $yamlTests = Join-Path $destCommonLib 'YAMLMatlab\Tests'
            if (Test-Path $yamlTests) {
                Remove-Item -Path $yamlTests -Recurse -Force
                Write-Host "  - CommonLib/YAMLMatlab/Tests (test fixtures excluded)"
            }
        } elseif ($_.Name -eq 'CarMaker') {
            # Lowercase the CarMaker directory in the release zip for a tidier,
            # consistent layout. Copied straight to the lowercase name (a case-only
            # rename is a no-op on Windows' case-insensitive FS but matters in the
            # zip, which Linux consumers read case-sensitively). build/ and the
            # proprietary bundle keep the original casing.
            Copy-Item -Path $_.FullName -Destination (Join-Path $StagingDir 'carmaker') -Recurse -Force
            Write-Host "  ~ CarMaker/ -> carmaker/"
        } elseif ($_.Name -like 'DriverModel_RealSim*.dll') {
            # Group the VISSIM driver-model DLLs under vissim/ instead of the zip
            # root (they land at build/ root from dispatch step 7 / the CI overlay).
            $vissimDest = Join-Path $StagingDir 'vissim'
            if (-not (Test-Path $vissimDest)) { New-Item -ItemType Directory -Path $vissimDest -Force | Out-Null }
            Copy-Item -Path $_.FullName -Destination $vissimDest -Force
            Write-Host "  ~ $($_.Name) -> vissim/"
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

    # Ship the self-contained Carla/ co-sim component (sumo/ runtime + utils/ +
    # run_cosim), minus the carla wheel. Consumers (e.g. FIXS_Applications'
    # roosevelt_sumo_carla) fetch Carla/ from this release zip. Test-only files
    # and the .ps1 build tooling under scripts/ stay source-side.
    $CarlaSrc = Join-Path $RepoRoot 'Carla'
    if (Test-Path $CarlaSrc) {
        $carlaDest = Join-Path $StagingDir 'Carla'
        New-Item -ItemType Directory -Path $carlaDest -Force | Out-Null
        Get-ChildItem -Path $CarlaSrc -Exclude '*.whl' | ForEach-Object {
            Copy-Item -Path $_.FullName -Destination $carlaDest -Recurse -Force
        }
        Get-ChildItem -Path $carlaDest -Recurse -Directory -Filter '__pycache__' -ErrorAction SilentlyContinue |
            Remove-Item -Recurse -Force -ErrorAction SilentlyContinue
        Write-Host "  + Carla/ co-sim component"
    }

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
