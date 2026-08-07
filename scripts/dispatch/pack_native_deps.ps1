# ============================================================================
# Pack the FIXS native C++ deps into per-component, version-named zips and
# publish them to ONE rolling public release: fixs-native-deps. Issue #109.
# ----------------------------------------------------------------------------
# Assets (each with a .sha256 sidecar):
#   libcarla-<carla_ver>.zip   - Carla client SDK, RELEASE subset (drops the
#                                319 MB carla_client_debug.lib; Release-only)
#   libsumo-<sumo_ver>.zip     - SUMO libsumo/libtraci headers + bin/
#
# Versions come from dependencies.yaml; the release TAG is version-less so
# multiple versions can coexist as separate assets. CI + developers fetch via
# scripts/dispatch/fetch_native_deps.ps1 (prebuilt mode). Open-source, ungated -
# the public counterpart to the proprietary Binaries-<key> bundle.
#
# Usage:
#   pack_native_deps.ps1                        # pack both into .\dist\
#   pack_native_deps.ps1 -Component carla       # only libcarla
#   pack_native_deps.ps1 -Publish               # pack + publish to fixs-native-deps
# ============================================================================
param(
    [ValidateSet('both', 'carla', 'sumo')] [string]$Component = 'both',
    [switch]$Publish,
    [string]$Repo = 'ORNL-Real-Sim/FIXS',
    [string]$OutDir
)

$ErrorActionPreference = 'Stop'
$RepoRoot  = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
$CommonLib = Join-Path $RepoRoot 'CommonLib'
$Tag       = 'fixs-native-deps'
if (-not $OutDir) { $OutDir = Join-Path $RepoRoot 'dist' }
New-Item -ItemType Directory -Path $OutDir -Force | Out-Null

function Get-DepVersion([string]$block) {
    $c = Get-Content (Join-Path $RepoRoot 'dependencies.yaml') -Raw
    if ($c -match "(?ms)^\s*${block}:\s.*?^\s*version:\s*[`"']?([0-9][0-9.]*)") { return $Matches[1] }
    throw "could not parse $block version from dependencies.yaml"
}
# The load probe #237 introduced here as Assert-LibsumoSelfContained now lives in
# libsumo_verify.ps1, because #238 needs the SAME check on the fetch side: the
# zip became the only source of the SUMO runtime, so a clone must be able to
# reject a bad asset, not just refuse to publish one. Same technique, one copy.
. (Join-Path $PSScriptRoot 'libsumo_verify.ps1')

function New-Zip([string]$stageDir, [string]$zipName) {
    $zipPath = Join-Path $OutDir $zipName
    if (Test-Path $zipPath) { Remove-Item $zipPath -Force }
    Compress-Archive -Path "$stageDir\*" -DestinationPath $zipPath -CompressionLevel Optimal
    $sha = (Get-FileHash $zipPath -Algorithm SHA256).Hash.ToLower()
    Set-Content -Path "$zipPath.sha256" -Value "$sha  $zipName" -Encoding ascii
    $mb = [math]::Round((Get-Item $zipPath).Length / 1MB, 1)
    Write-Host "Packed $zipName ($mb MB)  sha256=$sha"
    return $zipPath
}

$assets = New-Object System.Collections.Generic.List[string]

# --- libcarla (Release subset) ----------------------------------------------
if ($Component -in 'both', 'carla') {
    $libcarla = Join-Path $CommonLib 'libcarla'
    if (-not (Test-Path (Join-Path $libcarla 'lib\carla_client.lib'))) {
        Write-Error "CommonLib/libcarla not present. Acquire it first (fetch_native_deps.ps1 -Mode source)."; exit 1
    }
    $cver  = Get-DepVersion 'carla'
    $stage = Join-Path ([System.IO.Path]::GetTempPath()) "nd-carla-$(Get-Random)"
    $sc    = Join-Path $stage 'libcarla'
    New-Item -ItemType Directory -Path (Join-Path $sc 'lib') -Force | Out-Null
    Copy-Item (Join-Path $libcarla 'include') -Destination $sc -Recurse -Force
    Get-ChildItem (Join-Path $libcarla 'lib') -File |
        Where-Object { $_.Name -ne 'carla_client_debug.lib' } |
        ForEach-Object { Copy-Item $_.FullName -Destination (Join-Path $sc 'lib') -Force }
    Write-Host "libcarla ${cver}: staged Release subset (dropped carla_client_debug.lib)"
    $assets.Add((New-Zip $stage "libcarla-$cver.zip"))
    Remove-Item $stage -Recurse -Force
}

# --- libsumo ----------------------------------------------------------------
if ($Component -in 'both', 'sumo') {
    $libsumo = Join-Path $CommonLib 'libsumo'
    if (Test-Path $libsumo) {
        $sver  = Get-DepVersion 'sumo'
        $stage = Join-Path ([System.IO.Path]::GetTempPath()) "nd-sumo-$(Get-Random)"
        $ss    = Join-Path $stage 'libsumo'
        New-Item -ItemType Directory -Path $ss -Force | Out-Null
        Get-ChildItem $libsumo | Where-Object { $_.Name -ne 'out' } |
            ForEach-Object { Copy-Item $_.FullName -Destination $ss -Recurse -Force }
        Write-Host "libsumo ${sver}: staged (headers + bin/)"
        # Probe the STAGED tree, not the source tree: this is byte-for-byte what
        # gets zipped, so a staging bug cannot slip past the check either.
        Test-LibsumoLoadable -BinDir (Join-Path $ss 'bin') `
            -Context 'Re-run scripts/build_libsumo.ps1, or restore the missing DLL from the official SUMO Windows distribution, before packing.'
        $assets.Add((New-Zip $stage "libsumo-$sver.zip"))
        Remove-Item $stage -Recurse -Force
    } else { Write-Warning "CommonLib/libsumo not present - skipping libsumo." }
}

# --- publish (optional): one rolling release, clobber the built assets -------
if ($Publish) {
    $upload = @(); foreach ($z in $assets) { $upload += $z; $upload += "$z.sha256" }
    $prevEAP = $ErrorActionPreference; $ErrorActionPreference = 'Continue'
    & gh release view $Tag --repo $Repo *> $null
    $exists = ($LASTEXITCODE -eq 0)
    $ErrorActionPreference = $prevEAP
    if ($exists) {
        Write-Host "Updating rolling release '$Tag' assets..."
        & gh release upload $Tag @upload --repo $Repo --clobber
    } else {
        $target = (& git -C $RepoRoot rev-parse HEAD).Trim()
        $notes  = "Prebuilt FIXS native C++ deps (public, open-source). Per-component, version-named assets; fetched by scripts/dispatch/fetch_native_deps.ps1 (prebuilt) and the release CI. Rolling tag - add new-version assets here as needed."
        Write-Host "Creating rolling release '$Tag'..."
        & gh release create $Tag @upload --repo $Repo --prerelease --target $target --title "FIXS native deps" --notes $notes
    }
    if ($LASTEXITCODE -ne 0) { Write-Error "Publish failed."; exit 1 }
    Write-Host "Published $($assets.Count) asset(s) to $Tag on $Repo."
}
