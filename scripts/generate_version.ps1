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

function Get-HighestSemverTag {
    # Pick the HIGHEST semver tag reachable from HEAD, not the nearest one.
    #
    # `git describe` answers "nearest by commit distance", which is not stable
    # once more than one version tag is in play. Tags enter a branch's history
    # whenever a release branch merges, and rolling prerelease tags are deleted
    # and recreated on new commits - so the nearest tag can jump BACKWARDS
    # (v0.9.0-alpha one build, v0.7.0 the next) and the compiled version would
    # follow it down. Highest-wins is monotonic: adding tags can only raise it.
    #
    # Ordering is semver: 0.9.0 > 0.8.1, and a stable release outranks its own
    # prereleases (v0.9.0 > v0.9.0-alpha), so the final tag takes over from the
    # alpha the moment it is cut.
    $tags = & git tag --merged HEAD --list 'v[0-9]*' 2>$null
    if ($LASTEXITCODE -ne 0 -or -not $tags) { return $null }

    $parsed = foreach ($t in $tags) {
        $t = $t.Trim()
        if ($t -match '^v(\d+)\.(\d+)\.(\d+)(?:-(.+))?$') {
            [pscustomobject]@{
                Tag        = $t
                Major      = [int]$Matches[1]
                Minor      = [int]$Matches[2]
                Patch      = [int]$Matches[3]
                PreRelease = $Matches[4]
                # 1 = stable, 0 = prerelease, so stable sorts above its own alphas.
                Stability  = if ($Matches[4]) { 0 } else { 1 }
            }
        }
    }
    if (-not $parsed) { return $null }

    return ($parsed |
        Sort-Object Major, Minor, Patch, Stability, PreRelease |
        Select-Object -Last 1)
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

    # Semver macros: only vX.Y.Z tags count. The rolling release also publishes a
    # non-version tag on HEAD ('latest'); a bare `git describe --tags` returns
    # that, the semver regex throws, and the version silently falls back to
    # 0.0.0. Highest-wins rather than nearest-wins - see Get-HighestSemverTag.
    $highest = Get-HighestSemverTag
    if ($null -eq $highest) {
        throw "No semver tag (vX.Y.Z) reachable from HEAD"
    }

    $semverTag = $highest.Tag
    $major = $highest.Major
    $minor = $highest.Minor
    $patch = $highest.Patch

    # Traceability label -> REALSIM_GIT_TAG. Describe against the SAME tag the
    # macros came from (e.g. v0.9.0-alpha-12-gce90f3c0), so the label and the
    # version can never disagree about which release this build descends from.
    # --always degrades a tagless checkout to the short SHA rather than failing.
    $describeLabel = & git describe --tags --match $semverTag --always 2>&1
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
