# ====================================
# Create Release Zip
# Packages the build/ directory into a versioned zip file
# ====================================

param(
    [ValidateSet('standalone', 'inline')]
    [string]$RunMode = 'standalone',

    # Name the zip after the release CHANNEL it is being published to ('latest',
    # 'v0.9.0-alpha') rather than deriving a name from git. CI passes the tag
    # resolved by release_channel.ps1, via this parameter or the environment
    # variable, so a channel's asset always has the same, predictable name.
    #
    # Why this exists: the git-derived fallback below names the zip after the
    # nearest reachable version tag, and tags MOVE BETWEEN BRANCHES when a
    # release branch merges. After dev_v0.9.0 merged into main, main's nearest
    # tag was the 0.9.0 train's rolling v0.9.0-alpha, so a main build published
    # to 'latest' shipped fixs-build-v0.9.0-alpha-1-g65f2970d.zip - a name that
    # both misidentified the channel and changed on every dev-side retag.
    #
    # Local/PR builds pass nothing and keep the commit-traceable git name.
    [string]$VersionLabel = $env:RS_FIXS_VERSION_LABEL
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

# Version label: the release channel when one was given, else derived from git.
if ($VersionLabel) {
    # Keep the label filename-safe - it reaches disk and a release asset name.
    $VersionLabel = ($VersionLabel.Trim() -replace '[^A-Za-z0-9._-]', '-')
    Write-Host "Version label from release channel: $VersionLabel"
} else {
    # No channel (local or PR build): full semver-matched describe
    # (e.g. v0.8.0-120-gce90f3c0) so the zip is traceable to the exact commit.
    # --match 'v[0-9]*' ignores the rolling lightweight tag 'latest' that would
    # otherwise shadow a real version tag (#191); --always degrades a tagless
    # checkout to the short SHA.
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
    Write-Host "Version label from git describe: $VersionLabel"
}

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

    # Ship the CARLA component (sumo/ runtime + utils/ + the placers), minus the
    # carla wheel. Test-only files and the .ps1 build tooling under scripts/ stay
    # source-side.
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

    # Ship the renderer-agnostic co-sim entry points, hoisted out of scripts/ so
    # the bundle reads FIXS/cosim/ rather than FIXS/scripts/cosim/ (#313). Same
    # remapping idea as CarMaker/ -> carmaker/ above.
    #
    # Named subdirectories, not a scripts/*.py glob: scripts/ also holds the build
    # and release tooling, which must NOT ship - and update_fixs.{sh,ps1} in
    # particular has to stay exactly where it is, because every front door ever
    # committed downloads it from that path on raw.githubusercontent. An explicit
    # list also means a new .py dropped into scripts/ cannot ship by accident.
    foreach ($component in @('cosim')) {
        $src = Join-Path $RepoRoot "scripts\$component"
        if (-not (Test-Path $src)) { continue }
        $dest = Join-Path $StagingDir $component
        New-Item -ItemType Directory -Path $dest -Force | Out-Null
        Get-ChildItem -Path $src | ForEach-Object {
            Copy-Item -Path $_.FullName -Destination $dest -Recurse -Force
        }
        Get-ChildItem -Path $dest -Recurse -Directory -Filter '__pycache__' -ErrorAction SilentlyContinue |
            Remove-Item -Recurse -Force -ErrorAction SilentlyContinue
        Write-Host "  ~ scripts/$component/ -> $component/"
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
