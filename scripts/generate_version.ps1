# Auto-generate RealSimVersion.h from git tags
# This script is called by Visual Studio Pre-Build Event

$ErrorActionPreference = "Stop"

$RepoRoot = Split-Path -Parent $PSScriptRoot
$TemplateFile = Join-Path $RepoRoot "CommonLib\RealSimVersion.h.in"
$OutputFile = Join-Path $RepoRoot "CommonLib\RealSimVersion.h"

# Get git tag information
Push-Location $RepoRoot
try {
    # Get latest tag (e.g., v0.7.0)
    $GitTag = git describe --tags --abbrev=0

    # Parse version from tag (v0.7.0 → 0, 7, 0)
    if ($GitTag -match '^v?(\d+)\.(\d+)\.(\d+)') {
        $Major = [int]$Matches[1]
        $Minor = [int]$Matches[2]
        $Patch = [int]$Matches[3]
    } else {
        throw "Invalid tag format: $GitTag (expected vX.Y.Z)"
    }

    # Get current commit hash (short)
    $GitCommit = git rev-parse --short HEAD

    # Build version string and hex
    $VersionString = "$Major.$Minor.$Patch"
    $VersionHex = "{0:X2}{1:X2}{2:X2}" -f $Major, $Minor, $Patch
    $GenTime = Get-Date -Format "yyyy-MM-dd HH:mm:ss"

    # Read template and replace placeholders
    $Template = Get-Content $TemplateFile -Raw
    $Output = $Template `
        -replace '@VERSION_MAJOR@', $Major `
        -replace '@VERSION_MINOR@', $Minor `
        -replace '@VERSION_PATCH@', $Patch `
        -replace '@VERSION_STRING@', $VersionString `
        -replace '@VERSION_HEX@', $VersionHex `
        -replace '@GIT_COMMIT@', $GitCommit `
        -replace '@GIT_TAG@', $GitTag `
        -replace '@GENERATION_TIME@', $GenTime

    Set-Content -Path $OutputFile -Value $Output -NoNewline
    Write-Host "Generated $OutputFile from tag: $GitTag (commit: $GitCommit)" -ForegroundColor Green

} catch {
    Write-Error "Failed to generate version header: $_"
    exit 1
} finally {
    Pop-Location
}
