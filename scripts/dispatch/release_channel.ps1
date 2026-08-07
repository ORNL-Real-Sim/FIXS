# ====================================
# Resolve the release CHANNEL for a git ref.
#
# One place decides, for a given branch: does it publish, under which tag, with
# which release title. Everything downstream (zip name, publish_release.ps1, the
# local describe tag stamped in CI) reads the answer from here instead of
# re-deriving it, so adding the next train is a branch name, not a workflow edit.
#
# The mapping is CONVENTIONAL, not a table:
#
#   main            ->  latest        rolling, titled 'Latest Dev Build'
#   dev_vX.Y.Z      ->  vX.Y.Z-alpha  rolling, titled 'Rolling build: vX.Y.Z-alpha'
#   anything else   ->  no channel (build only, never publish)
#
# So dev_v0.10.0 gets its v0.10.0-alpha channel the day the branch is created,
# with no change to release.yml.
#
# Usage:
#   release_channel.ps1                       # channel for the current branch
#   release_channel.ps1 -Ref refs/heads/main  # channel for an explicit ref
#
# In GitHub Actions it also writes tag/title/rolling/publish to $GITHUB_OUTPUT.
# ====================================

param(
    # Branch name or full ref (refs/heads/...). Defaults to the checked-out branch.
    [string]$Ref = ''
)

if (-not $Ref) {
    $Ref = (& git rev-parse --abbrev-ref HEAD 2>$null)
}
$branch = ($Ref -replace '^refs/heads/', '').Trim()

$channel = [pscustomobject]@{
    Branch  = $branch
    Publish = $false
    Tag     = ''
    Title   = ''
    Rolling = $false
    # $true when Tag is a vX.Y.Z-shaped name that `git describe` can resolve, so
    # CI knows whether stamping it as a local tag would help the version header.
    # 'latest' is not semver-shaped; 'v0.9.0-alpha' is.
    Semver  = $false
}

switch -Regex ($branch) {
    '^main$' {
        $channel.Publish = $true
        $channel.Tag     = 'latest'
        $channel.Title   = 'Latest Dev Build'
        $channel.Rolling = $true
        break
    }
    '^dev_v(\d+)\.(\d+)\.(\d+)$' {
        $version = "v$($Matches[1]).$($Matches[2]).$($Matches[3])"
        $channel.Publish = $true
        $channel.Tag     = "$version-alpha"
        $channel.Title   = "Rolling build: $version-alpha"
        $channel.Rolling = $true
        $channel.Semver  = $true
        break
    }
}

if ($channel.Publish) {
    Write-Host "Release channel for '$branch': tag '$($channel.Tag)' ($($channel.Title))"
} else {
    Write-Host "Release channel for '$branch': none (build only, no publish)."
}

# GitHub Actions consumption: expose the fields as step outputs. Booleans are
# lowercased so `if:` expressions can compare them as strings.
if ($env:GITHUB_OUTPUT) {
    @(
        "publish=$($channel.Publish.ToString().ToLower())"
        "tag=$($channel.Tag)"
        "title=$($channel.Title)"
        "rolling=$($channel.Rolling.ToString().ToLower())"
        "semver=$($channel.Semver.ToString().ToLower())"
    ) | Add-Content -Path $env:GITHUB_OUTPUT -Encoding utf8
}

return $channel
