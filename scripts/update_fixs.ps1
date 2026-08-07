# ============================================================================
# Fetch/refresh a FIXS build into a consuming application's checkout (Windows).
#
# NOT a user entry point - the application's front door owns that
# (run_cosim.bat --update-fixs). That front door resolves WHICH release the user
# wants and then downloads this script FROM THAT RELEASE'S TAG, so the unpacker
# always matches the bundle it unpacks. Keeping it here rather than in each app
# repo is #272: release-format knowledge (zip layout, where the native runtime
# comes from, the freshness rule) is engine knowledge, and a per-app copy drifts.
# scripts/update_fixs.sh is the exact counterpart; change them together.
#
# Usage:
#   update_fixs.ps1 -Root C:\path\to\app                  # newest installable release
#   update_fixs.ps1 -Root C:\path\to\app -Version v0.9.0  # a specific release
#   update_fixs.ps1 -Root . -Repo my-fork/FIXS
#
# Installs into <Root>\FIXS. Needs PowerShell 5.1+ and network access. No git, no
# GitHub CLI, no auth - FIXS is public.
#
# Exit codes: 0 = a complete bundle is in place; 1 = nothing usable was installed.
# ============================================================================
param(
    [Parameter(Mandatory = $true)] [string]$Root,
    [string]$Version,
    [string]$Repo = 'ORNL-Real-Sim/FIXS',
    # The ref this copy of the script was downloaded from. Supplied by the app's
    # bootstrap; lets the hand-off below know whether it is already the right
    # version of itself. Absent for standalone use, which disables the hand-off.
    [string]$SelfRef,
    # The consuming app's preferred channel, offered as the Enter-default in the
    # picker. App config, not engine policy - see Select-Release.
    [string]$DefaultVersion
)

# 'Stop' is safe here: this script shells out to nothing. The old fetch_fixs.ps1
# could not set it, because it ran `git` and PS 5.1 turns a native command's
# stderr into a terminating NativeCommandError - so an ordinary "Cloning into..."
# aborted the install. Dropping git (#238 moved the native deps to a release)
# removed that hazard along with the dependency.
$ErrorActionPreference = 'Stop'
$ProgressPreference = 'SilentlyContinue'   # faster Invoke-WebRequest for large files

$Api     = "https://api.github.com/repos/$Repo"
$Headers = @{ 'User-Agent' = 'fixs-fetch'; 'Accept' = 'application/vnd.github+json' }
$DepsTag = 'fixs-native-deps'   # rolling release carrying the packed native runtimes

$Root        = [System.IO.Path]::GetFullPath($Root)
$OutputDir   = Join-Path $Root 'FIXS'
$VersionFile = Join-Path $OutputDir 'FIXS_VERSION.txt'

# ---------------------------------------------------------------------------
# A release is INSTALLABLE iff it carries a fixs-build-*.zip. That one rule is
# what keeps the picker honest, with no tag list to maintain: it excludes the
# native-deps release, every Binaries-<sha> bundle (there can be arbitrarily
# many), and old tags like v0.6.0/v0.7.0 that were cut before the build zip
# existed and have no assets at all. Four of the six tags visible today are
# unusable, and the menu used to offer all six.
# ---------------------------------------------------------------------------
function Get-BuildAsset($release) {
    $release.assets | Where-Object { $_.name -like 'fixs-build-*.zip' } | Select-Object -First 1
}

function Get-InstallableReleases {
    $rels = Invoke-RestMethod -UseBasicParsing -Uri "$Api/releases?per_page=30" -Headers $Headers
    @($rels | Where-Object { -not $_.draft -and (Get-BuildAsset $_) })
}

function Get-Asset {
    # Download <url> to <dest> and verify it against the published .sha256 sidecar.
    # A missing/unreachable sidecar warns and proceeds (older assets may predate
    # them); a MISMATCH is always fatal - that is a corrupt or tampered download.
    param([string]$Url, [string]$Dest)
    Invoke-WebRequest -UseBasicParsing -Uri $Url -OutFile $Dest -Headers $Headers
    try {
        # The CDN serves the tiny .sha256 as octet-stream, so .Content comes back
        # as byte[] (Invoke-WebRequest only string-decodes text content types).
        $c = (Invoke-WebRequest -UseBasicParsing -Uri "$Url.sha256" -Headers $Headers).Content
        if ($c -is [byte[]]) { $c = [System.Text.Encoding]::UTF8.GetString($c) }
        $expected = ($c -split '\s+')[0].Trim().ToLower()
        $actual   = (Get-FileHash $Dest -Algorithm SHA256).Hash.ToLower()
        if ($expected -ne $actual) {
            throw "checksum mismatch for $(Split-Path $Url -Leaf): expected $expected, got $actual"
        }
        Write-Host "    checksum OK ($actual)"
    } catch {
        if ("$_" -match 'checksum mismatch') { throw }
        Write-Warning "    could not verify .sha256 for $(Split-Path $Url -Leaf): $($_.Exception.Message) - proceeding WITHOUT verification."
    }
}

function Select-Release($releases) {
    # Annotate every entry with what it actually resolves to. Bare tag names are
    # not enough to tell the rolling channels apart: 'latest' and 'v0.9.0-alpha'
    # both move, so the tag alone says nothing about WHICH build is behind it.
    # Commit + date make them distinguishable BEFORE the choice.
    # (Asset names are now the channel name - fixs-build-latest.zip - so they no
    # longer carry that information either; they used to carry a git-describe
    # name that was traceable but named after whichever tag happened to be
    # nearest, which was worse: it read as the wrong release entirely.)
    Write-Host "Available FIXS releases:"
    for ($i = 0; $i -lt $releases.Count; $i++) {
        $r = $releases[$i]
        $c = if ($r.target_commitish -match '^[0-9a-f]{40}$') { $r.target_commitish.Substring(0, 7) } else { $r.target_commitish }
        $kind = if ($r.prerelease) { 'rolling' } else { 'pinned ' }
        Write-Host ("   {0}) {1,-18} {2}  {3}  {4}" -f ($i + 1), $r.tag_name, $kind, $c, $r.published_at.Substring(0, 10))
    }
    # The Enter-default is the CONSUMING APP's channel preference, not ours: an
    # app built against 0.9.0 features wants v0.9.0-alpha even though 'latest' may
    # be newer. It arrives via -DefaultVersion (fixs_default_version in the app's
    # fixs_sources.txt) so the choice lives in the app's config rather than being
    # hardcoded in two scripts per app repo, which is where it used to live.
    # Falling back to the first entry means API order, NOT newest - GitHub orders
    # releases by creation, and a rolling tag keeps its original created_at when
    # it is republished, so the list is not sorted by publish time.
    $default = if ($DefaultVersion -and ($releases.tag_name -contains $DefaultVersion)) { $DefaultVersion } else { $releases[0].tag_name }
    $choice = Read-Host "Which release? [1-$($releases.Count)], Enter = $default (default)"
    if ($choice -match '^\d+$' -and [int]$choice -ge 1 -and [int]$choice -le $releases.Count) {
        return $releases[[int]$choice - 1]
    }
    return ($releases | Where-Object { $_.tag_name -eq $default } | Select-Object -First 1)
}

# ---------------------------------------------------------------------------
# Resolve the release.
#
# INVARIANT: -Version given => never prompt, never hand off. That is what bounds
# the hand-off below to a single step and makes an infinite bounce impossible.
# ---------------------------------------------------------------------------
if ($Version) {
    try {
        $release = Invoke-RestMethod -UseBasicParsing -Uri "$Api/releases/tags/$Version" -Headers $Headers
    } catch {
        Write-Error "Could not find FIXS release '$Version' at $Repo. $_"
        exit 1
    }
    $asset = Get-BuildAsset $release
    if (-not $asset) {
        Write-Error "Release '$Version' carries no 'fixs-build-*.zip' and cannot be installed."
        exit 1
    }
} else {
    $installable = Get-InstallableReleases
    if (-not $installable -or $installable.Count -eq 0) {
        Write-Error "No installable FIXS release at $Repo (none carries a 'fixs-build-*.zip')."
        exit 1
    }
    $release = if ([Environment]::UserInteractive -and -not [Console]::IsInputRedirected) {
        Select-Release $installable
    } elseif ($DefaultVersion -and ($installable.tag_name -contains $DefaultVersion)) {
        $installable | Where-Object { $_.tag_name -eq $DefaultVersion } | Select-Object -First 1
    } else {
        $installable[0]
    }
    $Version = $release.tag_name
    $asset   = Get-BuildAsset $release

    # Hand off to the chosen release's own copy of this script, so the unpacker
    # always matches the bundle it unpacks (#272). Only reachable when the
    # bootstrap told us which ref we came from and the user picked a different
    # one; -Version on the child makes it terminal by the invariant above.
    if ($SelfRef -and $SelfRef -ne $Version) {
        Write-Host "Switching to $Version's own updater (this copy came from $SelfRef) ..."
        $url = "https://raw.githubusercontent.com/$Repo/$Version/scripts/update_fixs.ps1"
        $child = Join-Path ([System.IO.Path]::GetTempPath()) ("update_fixs_" + [guid]::NewGuid().ToString() + ".ps1")
        try {
            Invoke-WebRequest -UseBasicParsing -Uri $url -OutFile $child
        } catch {
            # Not fatal: a release predating this script still has a build zip we
            # know how to unpack, so carry on with the copy already running.
            Write-Host "  $Version has no scripts/update_fixs.ps1; continuing with this one."
            $child = $null
        }
        if ($child) {
            try {
                & $child -Root $Root -Version $Version -Repo $Repo
                exit $LASTEXITCODE
            } finally {
                Remove-Item $child -Force -ErrorAction SilentlyContinue
            }
        }
    }
}

# State the resolution in full. A rolling tag's asset is named after the CHANNEL
# (fixs-build-latest.zip), which is stable and predictable but says nothing about
# which commit is inside. Printing tag + commit + published-date together is what
# pins down exactly what was installed.
$sha7 = if ($release.target_commitish -match '^[0-9a-f]{40}$') { $release.target_commitish.Substring(0, 7) } else { $release.target_commitish }
Write-Host "=== Fetching the FIXS build from $Repo (public, no auth) ==="
Write-Host "  release:   $Version$(if ($release.prerelease) { '  (rolling prerelease - always re-fetched)' })"
Write-Host "  commit:    $sha7"
Write-Host "  asset:     $($asset.name)"
Write-Host "  published: $($release.published_at)"
Write-Host "  into:      $OutputDir"

# Skip only when the release is IMMUTABLE. The rolling prereleases ('latest',
# 'v0.9.0-alpha') republish in place under a fixed tag, so a matching tag does
# NOT mean the local bundle is current - re-fetch those every time. The
# 'prerelease' flag marks exactly the rolling channels release.yml publishes,
# which is why this reads the flag instead of hardcoding a tag list: the .sh
# used to hardcode one, and the two rules agreed only by coincidence.
if ((Test-Path $VersionFile) -and -not $release.prerelease) {
    $current = (Get-Content $VersionFile | Select-Object -First 1).Trim()
    if ($current -eq $Version) {
        Write-Host "FIXS $Version is already installed (delete $VersionFile to force)."
        exit 0
    }
}

$TempDir = Join-Path ([System.IO.Path]::GetTempPath()) "fixs-fetch-$(Get-Random)"
New-Item -ItemType Directory -Path $TempDir -Force | Out-Null
try {
    # --- The build zip -----------------------------------------------------
    $ZipPath = Join-Path $TempDir $asset.name
    Write-Host "Downloading $($asset.name) ..."
    Get-Asset -Url $asset.browser_download_url -Dest $ZipPath

    Write-Host "Extracting to $OutputDir ..."
    if (Test-Path $OutputDir) { Remove-Item -Path $OutputDir -Recurse -Force }
    New-Item -ItemType Directory -Path $OutputDir -Force | Out-Null
    Expand-Archive -Path $ZipPath -DestinationPath $OutputDir -Force

    # --- The native runtime ------------------------------------------------
    # The build zip ships libsumo HEADERS only (bin/ is ~413 MB), but
    # TrafficLayer links libsumocpp.lib and loads the runtime at startup, so a
    # headers-only install dies with "Unable to locate SUMO library directory".
    #
    # This used to be an anonymous sparse clone of CommonLib/libsumo/bin. #238
    # deleted that path from git and moved the runtime to the version-named
    # assets on the '$DepsTag' release - the clone kept succeeding against an
    # empty tree, so every Windows install failed with 0 DLLs. A plain HTTPS
    # download of the packed asset needs no git and no ref resolution.
    Write-Host "Fetching the native runtime from the '$DepsTag' release ..."
    $depsRelease = Invoke-RestMethod -UseBasicParsing -Uri "$Api/releases/tags/$DepsTag" -Headers $Headers
    # Version-named: take whatever the release currently carries, so bumping the
    # SUMO version in dependencies.yaml + publishing the asset is the whole change.
    $sumoAsset = $depsRelease.assets | Where-Object { $_.name -like 'libsumo-*.zip' } | Select-Object -First 1
    if (-not $sumoAsset) { throw "no 'libsumo-*.zip' asset on the '$DepsTag' release." }
    Write-Host "  downloading $($sumoAsset.name) ..."
    $SumoZip = Join-Path $TempDir $sumoAsset.name
    Get-Asset -Url $sumoAsset.browser_download_url -Dest $SumoZip
    # Extracting into CommonLib/ reproduces the <component>/bin layout the
    # executables already search; any future runtime asset lands the same way.
    Expand-Archive -Path $SumoZip -DestinationPath (Join-Path $OutputDir 'CommonLib') -Force

    $SumoBin = Join-Path $OutputDir 'CommonLib\libsumo\bin'
    $n = @(Get-ChildItem -Path $SumoBin -Filter '*.dll' -ErrorAction SilentlyContinue).Count
    if (-not (Test-Path (Join-Path $SumoBin 'libsumocpp.lib')) -or $n -eq 0) {
        throw "$($sumoAsset.name) extracted but $SumoBin has no libsumocpp.lib / no DLLs."
    }
    Write-Host "  native runtime ready ($n DLLs)"

    # --- Version marker ----------------------------------------------------
    # Written LAST and only on success: an incomplete bundle stamped as good
    # would be reported as already-installed by the skip above, and TrafficLayer
    # would not fail until startup.
    $stamp = if ($release.prerelease) { "$Version ($($release.published_at))" } else { $Version }
    $versionText = @"
$stamp
Fetched: $(Get-Date -Format 'yyyy-MM-dd HH:mm:ss')
Source: $Repo
Commit: $sha7
Zip: $($asset.name)
"@
    # NOT `Out-File -Encoding UTF8`: under Windows PowerShell 5.1 that switch
    # writes UTF-8 *with* a BOM ('utf8NoBOM' is PowerShell 6+ only). This file is
    # read by Carla/run_cosim.py, where a BOM is a character rather than metadata
    # - it rode into the parsed tag and silently disabled the freshness check.
    # Get-Content strips a BOM on the way back in, so a re-read from PowerShell
    # looked correct and the problem stayed invisible from this side.
    [System.IO.File]::WriteAllText(
        $VersionFile, $versionText + [Environment]::NewLine,
        (New-Object System.Text.UTF8Encoding($false)))
} catch {
    Write-Host ""
    Write-Error "FIXS update failed: $_"
    Write-Host "$OutputDir is incomplete. Re-run, or download the assets by hand from"
    Write-Host "  https://github.com/$Repo/releases/tag/$Version"
    Write-Host "  https://github.com/$Repo/releases/tag/$DepsTag   (libsumo-<ver>.zip -> <Root>\FIXS\CommonLib\)"
    exit 1
} finally {
    if (Test-Path $TempDir) { Remove-Item -Path $TempDir -Recurse -Force }
}

Write-Host ""
Write-Host "FIXS $Version is ready in $OutputDir"
if (Test-Path (Join-Path $OutputDir 'BUILD_INFO.txt')) {
    Write-Host ""
    Write-Host "Build details:"
    Get-Content (Join-Path $OutputDir 'BUILD_INFO.txt') | Select-Object -First 15
}

# Declare success explicitly so the caller can trust $LASTEXITCODE rather than
# whatever the last command happened to leave behind.
exit 0
