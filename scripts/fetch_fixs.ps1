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

# Skip if this exact (non-latest) version is already present
$VersionFile = Join-Path $OutputDir 'FIXS_VERSION.txt'
if ((Test-Path $VersionFile) -and $Version -ne 'latest') {
    $current = (Get-Content $VersionFile -Raw).Trim()
    if ($current -eq $Version) {
        Write-Host "FIXS $Version is already fetched in $OutputDir (delete $VersionFile to force)."
        exit 0
    }
}

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
$GitRef = if ($release.target_commitish) { $release.target_commitish } else { 'main' }

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
    git clone --depth 1 --filter=blob:none --sparse --branch $GitRef --quiet "https://github.com/$Repo.git" $LibsumoTmp
    if ($LASTEXITCODE -eq 0) {
        Push-Location $LibsumoTmp; git sparse-checkout set CommonLib/libsumo/bin; Pop-Location
        $src = Join-Path $LibsumoTmp 'CommonLib\libsumo\bin'
        if (Test-Path $src) {
            New-Item -ItemType Directory -Path $LibsumoDest -Force | Out-Null
            Copy-Item -Path "$src\*" -Destination $LibsumoDest -Recurse -Force
            $n = (Get-ChildItem -Path $LibsumoDest -Filter '*.dll' | Measure-Object).Count
            Write-Host "  Fetched $n libsumo DLLs"
        } else {
            Write-Warning "libsumo bin not found in clone; copy CommonLib/libsumo/bin manually if needed."
        }
        Remove-Item $LibsumoTmp -Recurse -Force -ErrorAction SilentlyContinue
    } else {
        Write-Warning "Could not clone for libsumo DLLs (is git installed?). Copy CommonLib/libsumo/bin manually if needed."
        Remove-Item $LibsumoTmp -Recurse -Force -ErrorAction SilentlyContinue
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
