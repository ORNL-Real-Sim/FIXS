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
        } else {
            Copy-Item -Path $_.FullName -Destination $StagingDir -Recurse -Force
        }
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
