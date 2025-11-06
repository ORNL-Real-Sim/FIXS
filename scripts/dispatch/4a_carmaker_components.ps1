# ====================================
# Build CarMaker Components
# Builds: CarMaker desktop and Simulink variants
# ====================================

param(
    [ValidateSet('standalone', 'inline', 'window')]
    [string]$RunMode = 'standalone'
)

$ScriptDir = $PSScriptRoot
$RepoRoot = (Resolve-Path (Join-Path $ScriptDir '..\..')).Path
$DispatchDir = $ScriptDir
$DepsFile = Join-Path $RepoRoot 'dependencies.yaml'
$YamlHelper = Join-Path $ScriptDir 'yaml_helper.ps1'

# Use shared log files if set by dispatch, otherwise create local ones
$LogOutput = if ($env:RS_BUILD_LOG) { $env:RS_BUILD_LOG } else { Join-Path $DispatchDir 'build.log' }
$LogSummary = if ($env:RS_BUILD_SUMMARY) { $env:RS_BUILD_SUMMARY } else { Join-Path $DispatchDir 'build_summary.log' }

# Handle window mode
if ($RunMode -eq 'window') {
    Start-Process powershell -ArgumentList "-NoProfile -ExecutionPolicy Bypass -File `"$PSCommandPath`" -RunMode inline" -NoNewWindow -Wait
    exit 0
}

# Helper function to exit with pause in standalone mode
function Exit-Script {
    param([int]$ExitCode = 0)
    if ($RunMode -eq 'standalone') {
        Write-Host ''
        pause
    }
    exit $ExitCode
}

$BuildResult = 0
$FailedBuilds = @()
$CarMakerVersions = $null

# Configuration detection
# When double-clicked: builds Debug only (change $StandaloneConfigs to build what you need)
# When called from dispatch.bat: RS_BUILD_CONFIG will be set to Release
if (-not $env:RS_BUILD_CONFIG) {
    $StandaloneConfigs = @('Debug')
    # To build both Debug and Release when double-clicked, change above line to: $StandaloneConfigs = @('Debug', 'Release')
} else {
    $StandaloneConfigs = @($env:RS_BUILD_CONFIG)
}

if (-not (Test-Path $RepoRoot)) {
    Write-Error "Repository root not found: $RepoRoot"
    Exit-Script 1
}

Set-Location $RepoRoot

if (-not (Test-Path $DepsFile)) {
    Write-Error "dependencies.yaml not found at $DepsFile"
    Exit-Script 1
}

Write-Host 'Parsing dependencies from:' $DepsFile

# Read CarMaker versions from YAML
function Read-YamlList {
    param([string]$File, [string]$Section, [string]$ListKey)

    if (Test-Path $YamlHelper) {
        try {
            $result = & powershell -NoProfile -File $YamlHelper -File $File -Section $Section -ListKey $ListKey -ReturnList 2>$null
            return $result
        } catch {
            return $null
        }
    }
    return $null
}

$CarMakerVersions = Read-YamlList $DepsFile 'carmaker' 'versions'

if (-not $CarMakerVersions) {
    Write-Error 'CarMaker versions not found in dependencies.yaml'
    Exit-Script 1
}

Write-Host ''
Write-Host 'Using versions from dependencies.yaml:'
Write-Host "  CarMaker versions: $CarMakerVersions"
Write-Host ''

# Only initialize logs in standalone mode
if ($RunMode -eq 'standalone') {
    'CarMaker Build Results' | Out-File -FilePath $LogSummary -Encoding UTF8
    '======================' | Out-File -FilePath $LogSummary -Append -Encoding UTF8
    '' | Out-File -FilePath $LogSummary -Append -Encoding UTF8
    if (Test-Path $LogOutput) { Remove-Item $LogOutput -Force }
} else {
    '' | Out-File -FilePath $LogSummary -Append -Encoding UTF8
    'CarMaker Components Build' | Out-File -FilePath $LogSummary -Append -Encoding UTF8
    '-------------------------' | Out-File -FilePath $LogSummary -Append -Encoding UTF8
}

# Build solution helper
function Build-Solution {
    param(
        [string]$TargetName,
        [string]$SolutionPath,
        [string]$MsBuildArgs
    )

    if (-not (Test-Path $SolutionPath)) {
        "===> $TargetName skipped (solution not found)" | Out-File -FilePath $LogSummary -Append -Encoding UTF8
        return 0
    }

    Write-Host "Building $TargetName..."
    "===> $TargetName build started" | Out-File -FilePath $LogSummary -Append -Encoding UTF8

    $argList = "`"$SolutionPath`" $MsBuildArgs"
    $output = & cmd /c "msbuild $argList 2>&1"
    $exitCode = $LASTEXITCODE

    $output | Out-File -FilePath $LogOutput -Append -Encoding UTF8

    if ($exitCode -ne 0) {
        "===> $TargetName build failed" | Out-File -FilePath $LogSummary -Append -Encoding UTF8
        return 1
    }

    "===> $TargetName build success" | Out-File -FilePath $LogSummary -Append -Encoding UTF8
    return 0
}

# Build CarMaker desktop and Simulink variants
foreach ($version in $CarMakerVersions -split '\s+') {
    if ([string]::IsNullOrWhiteSpace($version)) { continue }

    # Extract major version (e.g., 13.1.3 -> 13, 11.1.2 -> 11)
    $cmMajor = $version.Split('.')[0]

    $cmProjPath = Join-Path $RepoRoot "ProprietaryFiles\CM${cmMajor}_proj"

    if (Test-Path $cmProjPath) {
        foreach ($config in $StandaloneConfigs) {
            # Build desktop CarMaker
            $solutionPath = Join-Path $cmProjPath 'src\CarMaker.sln'
            $result = Build-Solution "CarMaker$cmMajor ($config)" $solutionPath "/target:CarMaker /p:Configuration=$config"
            if ($result -ne 0) {
                $FailedBuilds += "CarMaker$cmMajor ($config)"
                $BuildResult = 1
            }

            # Build Simulink variant
            $solutionPath = Join-Path $cmProjPath 'src_cm4sl\CarMaker for Simulink.sln'
            $result = Build-Solution "CarMaker$cmMajor Simulink ($config)" $solutionPath "/p:Configuration=$config"
            if ($result -ne 0) {
                $FailedBuilds += "CarMaker$cmMajor Simulink ($config)"
                $BuildResult = 1
            }
        }
    } else {
        "CM${cmMajor}_proj folder not found, skipping CarMaker $cmMajor builds" | Out-File -FilePath $LogSummary -Append -Encoding UTF8
    }
}

Write-Host ''
Write-Host '=============================='
if ($FailedBuilds.Count -gt 0) {
    Write-Host 'CarMaker build completed with failures!'
    Write-Host "Failed builds: $($FailedBuilds -join ' ')"
} else {
    Write-Host 'All CarMaker components built successfully!'
}
Write-Host "Check $LogSummary for summary and $LogOutput for details."
Write-Host '=============================='

Exit-Script $BuildResult
