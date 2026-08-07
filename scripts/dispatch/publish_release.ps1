# ====================================
# Publish FIXS Release to GitHub
# Uploads the build zip to a GitHub Release.
# Run after dispatch.bat when you're ready to publish.
#
# Usage:
#   publish_release.ps1                              # rolling "latest" release
#   publish_release.ps1 -Tag alpha_v0.9.0 -Rolling   # rolling named prerelease
#   publish_release.ps1 -Tag v0.8.0                  # fixed versioned release
# ====================================

param(
    [string]$Tag = 'latest',
    # Treat $Tag as a rolling prerelease (move it to HEAD each publish), the way
    # 'latest' always behaves. Used for the v0.9.0-alpha channel (issue #191).
    [switch]$Rolling,
    # Commit/branch the release + tag anchor to. WITHOUT this, `gh release create`
    # defaults a new tag to the repo's DEFAULT branch (main) - which wrongly
    # pinned the dev_v0.9.0 alpha release to main. In CI pass $env:GITHUB_SHA so
    # the release anchors to the exact commit that was built (#191).
    [string]$Target
)

$RepoRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
$BuildDir = Join-Path $RepoRoot 'build'

# Find the most recent fixs-build zip
$ZipFiles = Get-ChildItem -Path $BuildDir -Filter 'fixs-build-*.zip' | Sort-Object LastWriteTime -Descending
if ($ZipFiles.Count -eq 0) {
    Write-Error "No fixs-build-*.zip found in $BuildDir. Run dispatch.bat first."
    exit 1
}
$ZipPath = $ZipFiles[0].FullName
$ZipName = $ZipFiles[0].Name
Write-Host "Publishing: $ZipName"

# Check gh CLI is available
if (-not (Get-Command gh -ErrorAction SilentlyContinue)) {
    Write-Error "GitHub CLI (gh) is required. Install from https://cli.github.com/"
    exit 1
}

# Check authentication
$authStatus = gh auth status 2>&1
if ($LASTEXITCODE -ne 0) {
    Write-Error "Not authenticated with GitHub. Run 'gh auth login' first."
    exit 1
}

# Get commit info for release notes
$commit = git rev-parse --short HEAD 2>$null
$branch = git rev-parse --abbrev-ref HEAD 2>$null
$msg = git log -1 --pretty=%s 2>$null
$date = Get-Date -Format 'yyyy-MM-dd HH:mm'

$notes = @"
FIXS Build - $date

- Branch: $branch
- Commit: $commit ($msg)
- Zip: $ZipName
"@

# Anchor releases to the built commit; without --target a new tag defaults to
# the repo's default branch (main). Falls back to HEAD when run outside CI.
$targetArgs = @()
if ($Target) { $targetArgs = @('--target', $Target) }
elseif (-not $env:GITHUB_SHA) {
    $head = git rev-parse HEAD 2>$null
    if ($head) { $targetArgs = @('--target', $head.Trim()) }
}

if ($Rolling -or $Tag -eq 'latest') {
    Write-Host "Updating rolling '$Tag' prerelease..."

    # Delete the existing rolling release + tag if present, so the tag moves to
    # the current commit (delete+recreate is how the tag is re-pointed).
    gh release delete $Tag --yes --cleanup-tag 2>$null

    $title = if ($Tag -eq 'latest') { 'Latest Dev Build' } else { "Rolling build: $Tag" }
    gh release create $Tag $ZipPath @targetArgs `
        --prerelease `
        --title $title `
        --notes $notes

} else {
    Write-Host "Creating versioned release: $Tag"

    # Check if tag already exists
    $existingTag = git tag -l $Tag 2>$null
    if ($existingTag) {
        Write-Error "Tag '$Tag' already exists. Use a different version or delete the existing tag."
        exit 1
    }

    gh release create $Tag $ZipPath @targetArgs `
        --title "FIXS $Tag" `
        --notes $notes
}

if ($LASTEXITCODE -eq 0) {
    Write-Host "Published successfully."
    Write-Host "Consumers install this with: run_cosim.bat --update-fixs $Tag (or ./run_cosim.sh)"
} else {
    Write-Error "Failed to publish release."
    exit 1
}
