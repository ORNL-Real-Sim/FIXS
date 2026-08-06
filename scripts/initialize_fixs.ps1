# ============================================================================
# initialize_fixs.ps1 - bring a fresh FIXS clone to a buildable state. (#238)
# ----------------------------------------------------------------------------
# Before this script the setup steps were real but uncoordinated: you had to
# know to run fetch_native_deps.ps1 by hand, and to init the submodule, and
# dispatch.bat only ever built yaml-cpp. Once libsumo stopped being committed to
# git that guesswork became mandatory, and the failure mode for skipping it was
# an opaque link error. This is the one command that closes that gap:
#
#   1. ProprietaryFiles submodule  (OPTIONAL - private repo; external
#      contributors have no access and that is fine)
#   2. native deps                 - libsumo (REQUIRED), libcarla (OPTIONAL)
#   3. yaml-cpp                    (REQUIRED - built from the vendored source)
#
# dispatch.bat calls this as step 0, so `dispatch.bat` alone works on a fresh
# clone. Running it directly is equally fine.
#
# IDEMPOTENT: every step short-circuits when its output is already in place, so
# re-running is cheap and safe. -Force re-acquires the native deps; -SkipYamlCpp
# / -SkipSubmodules / -SkipNativeDeps opt out of individual steps.
#
# NOT to be confused with scripts/fetch_fixs.ps1, which is the CONSUMER-side
# script for downloading a published FIXS release zip. This one is for a
# developer checkout.
#
# Usage:
#   powershell -ExecutionPolicy Bypass -File scripts\initialize_fixs.ps1
#   powershell -ExecutionPolicy Bypass -File scripts\initialize_fixs.ps1 -Force
#
# Exit codes: 0 = ready to build; 1 = a REQUIRED step failed.
# ============================================================================
param(
    [string]$RepoRoot,
    [ValidateSet('', 'source', 'prebuilt')] [string]$CarlaMode = '',
    [switch]$Force,
    [switch]$SkipSubmodules,
    [switch]$SkipNativeDeps,
    [switch]$SkipYamlCpp
)

$ErrorActionPreference = 'Stop'
if (-not $RepoRoot) { $RepoRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path }
$DispatchDir = Join-Path $RepoRoot 'scripts\dispatch'

# Per-step outcome, reported as one table at the end. A developer who skips a
# step by accident should be able to see it in the summary, not infer it from
# a build error twenty minutes later.
$steps = New-Object System.Collections.Generic.List[object]
function Add-Step([string]$name, [string]$status, [string]$detail) {
    $steps.Add([pscustomobject]@{ Step = $name; Status = $status; Detail = $detail })
}

function Invoke-Native {
    <#
      Run a native command (git, cmd) and return its exit code.

      Windows PowerShell 5.1 wraps every stderr line from a native command in a
      NativeCommandError ErrorRecord. Under $ErrorActionPreference = 'Stop' that
      aborts the script - so one benign line on stderr (CMake's
      "Deprecation Warning", git's progress output) would fail initialization
      even though the command succeeded. Judge native commands by their exit
      code only, which is what this wrapper does.
    #>
    param([Parameter(Mandatory)] [scriptblock]$Command)
    $prev = $ErrorActionPreference
    $ErrorActionPreference = 'Continue'
    try {
        & $Command 2>&1 | ForEach-Object { Write-Host "  $_" }
        return $LASTEXITCODE
    } finally {
        $ErrorActionPreference = $prev
    }
}

Write-Host ""
Write-Host "=============================================="
Write-Host " FIXS clone initialization"
Write-Host "=============================================="
Write-Host " repo: $RepoRoot"
Write-Host ""

# ---------------------------------------------------------------------------
# 1. ProprietaryFiles submodule (optional - private repo)
# ---------------------------------------------------------------------------
Write-Host "[1/3] ProprietaryFiles submodule..."
if ($SkipSubmodules) {
    Write-Host "  skipped (-SkipSubmodules)."
    Add-Step 'submodule' 'SKIPPED' '-SkipSubmodules'
} elseif (-not (Test-Path (Join-Path $RepoRoot '.gitmodules'))) {
    Write-Host "  no .gitmodules - nothing to do."
    Add-Step 'submodule' 'N/A' 'no .gitmodules'
} else {
    $pfMarker = Join-Path $RepoRoot 'ProprietaryFiles\.git'
    if (-not $Force -and (Test-Path $pfMarker)) {
        Write-Host "  already initialized."
        Add-Step 'submodule' 'OK' 'already initialized'
    } else {
        # Never let a private-repo auth prompt hang an unattended build.
        $prevPrompt = $env:GIT_TERMINAL_PROMPT
        $env:GIT_TERMINAL_PROMPT = '0'
        $rc = Invoke-Native { git -C $RepoRoot submodule update --init --recursive }
        $ok = ($rc -eq 0)
        $env:GIT_TERMINAL_PROMPT = $prevPrompt
        if ($ok) {
            Write-Host "  initialized."
            Add-Step 'submodule' 'OK' 'initialized'
        } else {
            Write-Warning "  ProprietaryFiles could not be initialized - it is a PRIVATE repo. This is expected for external contributors; the public core still builds (VISSIM/CarMaker/dSPACE steps are skipped)."
            Add-Step 'submodule' 'SKIPPED' 'no access (private repo) - public core still builds'
        }
    }
}
Write-Host ""

# ---------------------------------------------------------------------------
# 2. Native deps: libsumo (required) + libcarla (optional)
# ---------------------------------------------------------------------------
Write-Host "[2/3] Native deps (libsumo required, libcarla optional)..."
$nativeOk = $true
if ($SkipNativeDeps) {
    Write-Host "  skipped (-SkipNativeDeps)."
    Add-Step 'native deps' 'SKIPPED' '-SkipNativeDeps'
} else {
    $fetch = Join-Path $DispatchDir 'fetch_native_deps.ps1'
    $fetchArgs = @{ RepoRoot = $RepoRoot }
    if ($Force)      { $fetchArgs['Force'] = $true }
    if ($CarlaMode)  { $fetchArgs['Mode']  = $CarlaMode }
    & $fetch @fetchArgs
    if ($LASTEXITCODE -eq 0) {
        Add-Step 'native deps' 'OK' 'libsumo present'
    } else {
        $nativeOk = $false
        Add-Step 'native deps' 'FAILED' 'libsumo (REQUIRED) not acquired'
    }
}
Write-Host ""

# ---------------------------------------------------------------------------
# 3. yaml-cpp (required, built from vendored source)
# ---------------------------------------------------------------------------
Write-Host "[3/3] yaml-cpp..."
$yamlOk = $true
$yamlBuild = Join-Path $RepoRoot 'CommonLib\yaml-cpp\build'
# Sentinel is the LIBRARY, not the build directory. dispatch.bat used to test
# only for the directory, which CMake creates at configure time - so a build
# that was interrupted (or failed) after configuring looked "already built" and
# the real failure surfaced later as an opaque link error in step 2. Release
# only: the Debug build is deliberately skipped under RS_FIXS_AUTOMATION, and
# requiring yaml-cppd.lib would make CI rebuild on every run.
$yamlLib = Join-Path $yamlBuild 'Release\yaml-cpp.lib'
if ($SkipYamlCpp) {
    Write-Host "  skipped (-SkipYamlCpp)."
    Add-Step 'yaml-cpp' 'SKIPPED' '-SkipYamlCpp'
} elseif ((Test-Path $yamlLib) -and -not $Force) {
    Write-Host "  already built ($yamlLib)."
    Add-Step 'yaml-cpp' 'OK' 'already built'
} else {
    $rc = Invoke-Native { cmd /c "`"$(Join-Path $DispatchDir '1_external_libraries.bat')`" inline" }
    if ($rc -eq 0 -and (Test-Path $yamlBuild)) {
        Write-Host "  built."
        Add-Step 'yaml-cpp' 'OK' 'built'
    } else {
        $yamlOk = $false
        Write-Warning "  yaml-cpp build failed - is CMake on PATH?"
        Add-Step 'yaml-cpp' 'FAILED' 'build failed (CMake on PATH?)'
    }
}
Write-Host ""

# ---------------------------------------------------------------------------
# Summary
# ---------------------------------------------------------------------------
Write-Host "=============================================="
Write-Host " Initialization summary"
Write-Host "=============================================="
$steps | Format-Table -AutoSize | Out-String | Write-Host

if (-not $nativeOk -or -not $yamlOk) {
    Write-Host "NOT ready to build - a required step failed (see above)."
    exit 1
}
Write-Host "Ready to build. Next: scripts\dispatch\dispatch.bat"
exit 0
