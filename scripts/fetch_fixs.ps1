# ====================================
# Fetch FIXS Build Artifacts
# Downloads pre-built FIXS executables from GitHub Releases.
# Place this script in your consuming project.
#
# Usage:
#   fetch_fixs.ps1                       # Fetch latest dev build
#   fetch_fixs.ps1 -Version v0.8.0      # Fetch a specific version
#   fetch_fixs.ps1 -Version latest      # Explicitly fetch latest
#   fetch_fixs.ps1 -OutputDir deps\fixs # Custom output directory
# ====================================

param(
    [string]$Version = 'latest',
    [string]$OutputDir = 'deps\fixs',
    [string]$Repo = 'ORNL-Real-Sim/FIXS'
)

# Check gh CLI
if (-not (Get-Command gh -ErrorAction SilentlyContinue)) {
    Write-Error "GitHub CLI (gh) is required. Install from https://cli.github.com/"
    exit 1
}

# Check authentication
gh auth status 2>$null
if ($LASTEXITCODE -ne 0) {
    Write-Error "Not authenticated with GitHub. Run 'gh auth login' first."
    exit 1
}

# Resolve output path
$OutputDir = [System.IO.Path]::GetFullPath($OutputDir)

# Check if we already have this version
$VersionFile = Join-Path $OutputDir 'FIXS_VERSION.txt'
if (Test-Path $VersionFile) {
    $current = Get-Content $VersionFile -Raw
    if ($current.Trim() -eq $Version -and $Version -ne 'latest') {
        Write-Host "FIXS $Version is already fetched in $OutputDir"
        Write-Host "Delete $VersionFile to force re-fetch."
        exit 0
    }
}

Write-Host "Fetching FIXS build ($Version) from $Repo..."

# Create temp directory for download
$TempDir = Join-Path ([System.IO.Path]::GetTempPath()) "fixs-fetch-$(Get-Random)"
New-Item -ItemType Directory -Path $TempDir -Force | Out-Null

try {
    # Download release asset
    Write-Host "Downloading..."
    gh release download $Version `
        -R $Repo `
        --pattern 'fixs-build-*.zip' `
        --dir $TempDir

    if ($LASTEXITCODE -ne 0) {
        Write-Error "Failed to download FIXS release '$Version'. Check that the version exists."
        exit 1
    }

    # Find the downloaded zip
    $ZipFile = Get-ChildItem -Path $TempDir -Filter 'fixs-build-*.zip' | Select-Object -First 1
    if (-not $ZipFile) {
        Write-Error "No fixs-build zip found in downloaded assets."
        exit 1
    }

    Write-Host "Extracting to $OutputDir..."

    # Clean and recreate output directory
    if (Test-Path $OutputDir) {
        Remove-Item -Path $OutputDir -Recurse -Force
    }
    New-Item -ItemType Directory -Path $OutputDir -Force | Out-Null

    # Extract
    Expand-Archive -Path $ZipFile.FullName -DestinationPath $OutputDir -Force

    # Write version marker
    # For "latest", resolve the actual version from the zip name or release info
    $ActualVersion = $Version
    if ($Version -eq 'latest') {
        $releaseInfo = gh release view latest -R $Repo --json tagName,targetCommitish,publishedAt 2>$null
        if ($releaseInfo) {
            $ActualVersion = "latest ($(($releaseInfo | ConvertFrom-Json).publishedAt))"
        }
    }

    @"
$ActualVersion
Fetched: $(Get-Date -Format 'yyyy-MM-dd HH:mm:ss')
Source: $Repo
Zip: $($ZipFile.Name)
"@ | Out-File -FilePath $VersionFile -Encoding UTF8

    Write-Host ""
    Write-Host "FIXS build fetched successfully."
    Write-Host "  Location: $OutputDir"
    Write-Host "  Version:  $ActualVersion"

    # Show what's available
    if (Test-Path (Join-Path $OutputDir 'BUILD_INFO.txt')) {
        Write-Host ""
        Write-Host "Build details:"
        Get-Content (Join-Path $OutputDir 'BUILD_INFO.txt') | Select-Object -First 15
    }

} finally {
    # Cleanup temp
    if (Test-Path $TempDir) {
        Remove-Item -Path $TempDir -Recurse -Force
    }
}
