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

Write-Host "Creating $ZipName..."
try {
    Compress-Archive -Path "$BuildDir\*" -DestinationPath $ZipPath -CompressionLevel Optimal
    $size = [math]::Round((Get-Item $ZipPath).Length / 1MB, 1)
    Write-Host "Created: $ZipPath ($size MB)"
} catch {
    Write-Error "Failed to create zip: $_"
    Exit-Script 1
}

Exit-Script 0
