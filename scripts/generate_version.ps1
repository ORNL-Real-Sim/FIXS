# Generate CommonLib/RealSimVersion.h. Called by the TrafficLayer pre-build event and dispatch.bat.
# The version comes from the tracked VERSION file; git only supplies the commit and a describe label.

$ErrorActionPreference = "Stop"

$RepoRoot = Split-Path -Parent $PSScriptRoot
$TemplateFile = Join-Path $RepoRoot "CommonLib\RealSimVersion.h.in"
$OutputFile = Join-Path $RepoRoot "CommonLib\RealSimVersion.h"
$VersionFile = Join-Path $RepoRoot "VERSION"

Push-Location $RepoRoot
try {
    if (-not (Test-Path $TemplateFile)) { throw "Template file not found: $TemplateFile" }
    if (-not (Test-Path $VersionFile)) { throw "VERSION file not found: $VersionFile" }

    $declared = (Get-Content $VersionFile -Raw).Trim()
    if ($declared -notmatch '^(\d+)\.(\d+)\.(\d+)$') {
        throw "VERSION must be X.Y.Z, got '$declared'"
    }
    $major = [int]$Matches[1]; $minor = [int]$Matches[2]; $patch = [int]$Matches[3]

    $commit = "UNKNOWN"
    $tag = "UNKNOWN"
    if ((Get-Command git -ErrorAction SilentlyContinue) -and (Test-Path (Join-Path $RepoRoot '.git'))) {
        $c = & git rev-parse --short HEAD 2>$null
        if ($LASTEXITCODE -eq 0 -and $c) { $commit = $c.Trim() }
        # --match skips the non-semver 'stable' tag; --always degrades to the short SHA.
        $d = & git describe --tags --match 'v[0-9]*' --always 2>$null
        if ($LASTEXITCODE -eq 0 -and $d) { $tag = $d.Trim() }
    } else {
        Write-Output "WARNING: git unavailable; REALSIM_GIT_COMMIT/TAG set to UNKNOWN."
    }

    if (Test-Path $OutputFile) {
        $outputItem = Get-Item $OutputFile -ErrorAction SilentlyContinue
        if ($outputItem -and $outputItem.IsReadOnly) {
            Set-ItemProperty -Path $OutputFile -Name IsReadOnly -Value $false
        }
    }

    $rendered = (Get-Content $TemplateFile -Raw) `
        -replace '@VERSION_MAJOR@', $major.ToString() `
        -replace '@VERSION_MINOR@', $minor.ToString() `
        -replace '@VERSION_PATCH@', $patch.ToString() `
        -replace '@VERSION_STRING@', ("{0}.{1}.{2}" -f $major, $minor, $patch) `
        -replace '@VERSION_HEX@', ("{0:X2}{1:X2}{2:X2}" -f $major, $minor, $patch) `
        -replace '@GIT_COMMIT@', $commit `
        -replace '@GIT_TAG@', $tag `
        -replace '@GENERATION_TIME@', (Get-Date -Format "yyyy-MM-dd HH:mm:ss")

    [System.IO.File]::WriteAllText($OutputFile, $rendered)

    Write-Output "Generated $OutputFile`: version $declared (tag $tag, commit $commit)"
    exit 0

} catch {
    Write-Output "ERROR: Failed to generate version header: $($_.Exception.Message)"
    exit 1

} finally {
    Pop-Location
}
