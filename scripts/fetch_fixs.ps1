# ====================================
# Fetch FIXS Build Artifacts (gh-free; FIXS is a public repo)
# Downloads the pre-built FIXS release zip + libsumo DLLs - no GitHub CLI, no auth.
# Place this script in your consuming project (or have it fetched on demand).
#
# Usage:
#   fetch_fixs.ps1                       # latest dev build
#   fetch_fixs.ps1 -Version v0.8.0       # a specific version
#   fetch_fixs.ps1 -OutputDir deps\fixs  # custom output directory
# ====================================

param(
    [string]$Version = 'latest',
    [string]$OutputDir = 'deps\fixs',
    [string]$Repo = 'ORNL-Real-Sim/FIXS'
)

# Note: do NOT set $ErrorActionPreference='Stop' globally - git writes normal
# progress ("Cloning into...") to stderr, which PS 5.1 would turn into a
# terminating error. Web calls below use try/catch; git uses $LASTEXITCODE.
$ProgressPreference = 'SilentlyContinue'   # faster Invoke-WebRequest for large files

# Resolve output path
$OutputDir = [System.IO.Path]::GetFullPath($OutputDir)

$VersionFile = Join-Path $OutputDir 'FIXS_VERSION.txt'

Write-Host "Fetching FIXS build ($Version) from $Repo (public, no auth)..."
$Api = "https://api.github.com/repos/$Repo"
$Headers = @{ 'User-Agent' = 'fixs-fetch'; 'Accept' = 'application/vnd.github+json' }

# --- Resolve the release (by tag) via the public REST API ---
try {
    $release = Invoke-RestMethod -Uri "$Api/releases/tags/$Version" -Headers $Headers
} catch {
    Write-Error "Could not find FIXS release '$Version' at $Repo. $_"
    exit 1
}
$asset = $release.assets | Where-Object { $_.name -like 'fixs-build-*.zip' } | Select-Object -First 1
if (-not $asset) {
    Write-Error "No 'fixs-build-*.zip' asset on release '$Version'."
    exit 1
}
# Skip only when the release is immutable. Two bugs used to sit in this check, one
# masking the other: it compared the whole 4-line FIXS_VERSION.txt (-Raw) against the
# tag, so it never matched and every run re-downloaded; and the rolling prereleases
# ('latest', 'v0.9.0-alpha') republish in place under a fixed tag, so a matching tag
# does NOT mean the bundle is current. Fixing only the comparison would have pinned an
# install to the first alpha it ever fetched - so both change together. 'prerelease'
# marks exactly the rolling channels release.yml publishes, so no tag list to maintain.
if ((Test-Path $VersionFile) -and -not $release.prerelease) {
    $current = (Get-Content $VersionFile | Select-Object -First 1).Trim()
    if ($current -eq $Version) {
        Write-Host "FIXS $Version is already fetched in $OutputDir (delete $VersionFile to force)."
        exit 0
    }
}

# Ref for the libsumo clone below. The rolling prereleases are anchored to the exact
# commit built (-Target $GITHUB_SHA, #191), so target_commitish is a raw SHA - and
# `git clone --branch` accepts only branch/tag names, not SHAs. The release tag is a
# valid ref on both trains ('latest' and 'v0.9.0-alpha' are real tags), so use it.
$GitRef = $Version

$TempDir = Join-Path ([System.IO.Path]::GetTempPath()) "fixs-fetch-$(Get-Random)"
New-Item -ItemType Directory -Path $TempDir -Force | Out-Null
try {
    # --- Download + extract the release zip ---
    $ZipPath = Join-Path $TempDir $asset.name
    Write-Host "Downloading $($asset.name)..."
    Invoke-WebRequest -Uri $asset.browser_download_url -OutFile $ZipPath -Headers $Headers

    Write-Host "Extracting to $OutputDir..."
    if (Test-Path $OutputDir) { Remove-Item -Path $OutputDir -Recurse -Force }
    New-Item -ItemType Directory -Path $OutputDir -Force | Out-Null
    Expand-Archive -Path $ZipPath -DestinationPath $OutputDir -Force

    # --- libsumo DLLs: anonymous sparse clone (excluded from the zip for size) ---
    Write-Host "Fetching libsumo DLLs from $Repo (anonymous clone)..."
    $LibsumoDest = Join-Path $OutputDir 'CommonLib\libsumo\bin'
    $LibsumoTmp = Join-Path ([System.IO.Path]::GetTempPath()) "fixs-libsumo-$(Get-Random)"
    # TrafficLayer loads the libsumo runtime at startup, so a bundle without these DLLs
    # is unusable. Fail loudly instead of warning: the version marker below is written
    # only on success, otherwise an incomplete bundle would be stamped as good and the
    # skip above would keep reporting it as already fetched. Mirrors initialize.sh.
    $n = 0
    # advice.detachedHead: cloning a TAG checks out a detached HEAD, and git prints that
    # advice to stderr even under --quiet. Harmless on a console, but if a caller ever
    # redirects stderr, PS 5.1 turns it into a NativeCommandError - which under an
    # inherited $ErrorActionPreference='Stop' aborts the install on a *successful* clone.
    git -c advice.detachedHead=false clone --depth 1 --filter=blob:none --sparse --branch $GitRef --quiet "https://github.com/$Repo.git" $LibsumoTmp
    if ($LASTEXITCODE -eq 0) {
        Push-Location $LibsumoTmp; git sparse-checkout set CommonLib/libsumo/bin; Pop-Location
        $src = Join-Path $LibsumoTmp 'CommonLib\libsumo\bin'
        if (Test-Path $src) {
            New-Item -ItemType Directory -Path $LibsumoDest -Force | Out-Null
            Copy-Item -Path "$src\*" -Destination $LibsumoDest -Recurse -Force
            $n = (Get-ChildItem -Path $LibsumoDest -Filter '*.dll' | Measure-Object).Count
            Write-Host "  Fetched $n libsumo DLLs"
        }
        Remove-Item $LibsumoTmp -Recurse -Force -ErrorAction SilentlyContinue
    } else {
        Remove-Item $LibsumoTmp -Recurse -Force -ErrorAction SilentlyContinue
    }
    if ($n -eq 0) {
        Write-Error @"
Could not fetch the libsumo runtime (ref '$GitRef') from $Repo.
TrafficLayer cannot start without $LibsumoDest. Check that git is installed and
the ref exists, or download libsumo-<ver>.zip from
  https://github.com/$Repo/releases/tag/fixs-native-deps
and extract it into $OutputDir\CommonLib\.
"@
        exit 1
    }

    # --- Version marker ---
    $stamp = if ($Version -eq 'latest') { "latest ($($release.published_at))" } else { $Version }
    @"
$stamp
Fetched: $(Get-Date -Format 'yyyy-MM-dd HH:mm:ss')
Source: $Repo
Zip: $($asset.name)
"@ | Out-File -FilePath $VersionFile -Encoding UTF8

    Write-Host ""
    Write-Host "FIXS build fetched successfully."
    Write-Host "  Location: $OutputDir"
    Write-Host "  Version:  $stamp"
    if (Test-Path (Join-Path $OutputDir 'BUILD_INFO.txt')) {
        Write-Host ""; Write-Host "Build details:"
        Get-Content (Join-Path $OutputDir 'BUILD_INFO.txt') | Select-Object -First 15
    }
} finally {
    if (Test-Path $TempDir) { Remove-Item -Path $TempDir -Recurse -Force }
}

# Declare success explicitly so a caller can trust $LASTEXITCODE. Without it the
# value left over from the last native command (git) leaks out as this script's
# result, which is only accidentally 0.
exit 0
