# Auto-generate RealSimVersion.h from git tags
# This script is called by Visual Studio Pre-Build Event

$ErrorActionPreference = "Stop"

$RepoRoot = Split-Path -Parent $PSScriptRoot
$TemplateFile = Join-Path $RepoRoot "CommonLib\RealSimVersion.h.in"
$OutputFile = Join-Path $RepoRoot "CommonLib\RealSimVersion.h"

function New-VersionInfo {
    param(
        [int]$Major,
        [int]$Minor,
        [int]$Patch,
        [string]$Commit = "UNKNOWN",
        [string]$Tag = "UNKNOWN",
        [string]$Source = "unknown"
    )

    $versionString = "{0}.{1}.{2}" -f $Major, $Minor, $Patch
    $versionHex = "{0:X2}{1:X2}{2:X2}" -f $Major, $Minor, $Patch

    return [pscustomobject]@{
        Major = $Major
        Minor = $Minor
        Patch = $Patch
        VersionString = $versionString
        VersionHex = $versionHex
        Commit = $Commit
        Tag = $Tag
        Source = $Source
    }
}

function Get-GitVersionInfo {
    $gitPath = Get-Command git -ErrorAction SilentlyContinue
    if ($null -eq $gitPath) {
        throw "git command not found in PATH. Install git or add it to system PATH."
    }

    Write-Verbose "Git path: $($gitPath.Source)"

    if (-not (Test-Path (Join-Path $RepoRoot '.git'))) {
        throw "No .git directory present under $RepoRoot"
    }

    # Semver macros: match ONLY vX.Y.Z tags. The rolling release publishes a
    # lightweight 'latest' tag that sits on HEAD; a bare `git describe --tags`
    # returns 'latest', the semver regex below throws, and the version silently
    # falls back to 0.0.0. Restricting to 'v[0-9]*' ignores 'latest' (issue #191).
    $semverTag = & git describe --tags --match 'v[0-9]*' --abbrev=0 2>&1
    if ($LASTEXITCODE -ne 0) {
        throw "git describe (semver tag) failed: $semverTag"
    }

    if ($semverTag -match '^v?(\d+)\.(\d+)\.(\d+)') {
        $major = [int]$Matches[1]
        $minor = [int]$Matches[2]
        $patch = [int]$Matches[3]
    } else {
        throw "Invalid tag format: $semverTag (expected vX.Y.Z)"
    }

    # Traceability label -> REALSIM_GIT_TAG. Use the FULL describe (e.g.
    # v0.7.0-244-gce90f3c0) so a rolling/dev build points at its exact commit
    # instead of the bare nearest tag. Still semver-matched so 'latest' never
    # leaks in; --always degrades a tagless checkout to the short SHA rather
    # than failing the build.
    $describeLabel = & git describe --tags --match 'v[0-9]*' --always 2>&1
    if ($LASTEXITCODE -ne 0) {
        $describeLabel = $semverTag
    }

    $commitOutput = & git rev-parse --short HEAD 2>&1
    if ($LASTEXITCODE -ne 0) {
        throw "git rev-parse failed: $commitOutput"
    }

    return New-VersionInfo -Major $major -Minor $minor -Patch $patch -Commit $commitOutput.Trim() -Tag $describeLabel.Trim() -Source "git"
}

function Get-ExistingHeaderVersion {
    param([string]$Path)

    if (-not (Test-Path $Path)) {
        return $null
    }

    $content = Get-Content $Path -Raw

    $majorMatch = [regex]::Match($content, '#define\s+REALSIM_VERSION_MAJOR\s+(\d+)')
    $minorMatch = [regex]::Match($content, '#define\s+REALSIM_VERSION_MINOR\s+(\d+)')
    $patchMatch = [regex]::Match($content, '#define\s+REALSIM_VERSION_PATCH\s+(\d+)')

    if (-not ($majorMatch.Success -and $minorMatch.Success -and $patchMatch.Success)) {
        return $null
    }

    $tag = "UNKNOWN"
    $tagMatch = [regex]::Match($content, '#define\s+REALSIM_GIT_TAG\s+"([^"]*)"')
    if ($tagMatch.Success) {
        $tag = $tagMatch.Groups[1].Value
    }

    $commit = "UNKNOWN"
    $commitMatch = [regex]::Match($content, '#define\s+REALSIM_GIT_COMMIT\s+"([^"]*)"')
    if ($commitMatch.Success) {
        $commit = $commitMatch.Groups[1].Value
    }

    return New-VersionInfo -Major ([int]$majorMatch.Groups[1].Value) -Minor ([int]$minorMatch.Groups[1].Value) -Patch ([int]$patchMatch.Groups[1].Value) -Commit $commit -Tag $tag -Source "existing header"
}

Push-Location $RepoRoot
try {
    Write-Verbose "Generating version header"

    if (-not (Test-Path $TemplateFile)) {
        throw "Template file not found: $TemplateFile"
    }

    $versionInfo = $null
    $gitErrorMessage = $null

    try {
        $versionInfo = Get-GitVersionInfo
    } catch {
        $gitErrorMessage = $_.Exception.Message
    }

    if ($null -eq $versionInfo) {
        $existingInfo = Get-ExistingHeaderVersion -Path $OutputFile
        if ($null -ne $existingInfo) {
            Write-Output "WARNING: Using existing RealSimVersion.h ($($existingInfo.VersionString)) because git metadata is unavailable."
            $versionInfo = $existingInfo
        } else {
            $versionInfo = New-VersionInfo -Major 0 -Minor 0 -Patch 0 -Commit "UNKNOWN" -Tag "v0.0.0" -Source "default"
            Write-Output "WARNING: Using default version $($versionInfo.VersionString) because git metadata is unavailable."
        }

        if ($gitErrorMessage) {
            Write-Output "WARNING: $gitErrorMessage"
        }
    }

    if (Test-Path $OutputFile) {
        $outputItem = Get-Item $OutputFile -ErrorAction SilentlyContinue
        if ($outputItem -and $outputItem.IsReadOnly) {
            Write-Verbose "Clearing read-only attribute on $OutputFile"
            Set-ItemProperty -Path $OutputFile -Name IsReadOnly -Value $false
        }
    }

    $templateContent = Get-Content $TemplateFile -Raw
    $generationTime = Get-Date -Format "yyyy-MM-dd HH:mm:ss"

    $rendered = $templateContent `
        -replace '@VERSION_MAJOR@', $versionInfo.Major.ToString() `
        -replace '@VERSION_MINOR@', $versionInfo.Minor.ToString() `
        -replace '@VERSION_PATCH@', $versionInfo.Patch.ToString() `
        -replace '@VERSION_STRING@', $versionInfo.VersionString `
        -replace '@VERSION_HEX@', $versionInfo.VersionHex `
        -replace '@GIT_COMMIT@', $versionInfo.Commit `
        -replace '@GIT_TAG@', $versionInfo.Tag `
        -replace '@GENERATION_TIME@', $generationTime

    [System.IO.File]::WriteAllText($OutputFile, $rendered)

    Write-Output "Generated $OutputFile from source $($versionInfo.Source): tag $($versionInfo.Tag) (commit: $($versionInfo.Commit))"
    exit 0

} catch {
    $errorMsg = $_.Exception.Message
    if ($_.Exception.InnerException) {
        $inner = $_.Exception.InnerException.Message
        if ($inner) {
            $errorMsg = "$errorMsg`n$inner"
        }
    }
    Write-Output "ERROR: Failed to generate version header: $errorMsg"
    exit 1

} finally {
    Pop-Location
}
