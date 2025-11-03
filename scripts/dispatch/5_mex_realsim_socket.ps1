# ====================================
# Build RealSimSocket MEX (mexw64)
# Supports standalone and inline invocation via dispatch
# ====================================

param(
    [ValidateSet('standalone', 'inline', 'window')]
    [string]$RunMode = 'standalone'
)

$ScriptDir = $PSScriptRoot
$RepoRoot = (Resolve-Path (Join-Path $ScriptDir '..\..")).Path
$DepsFile = Join-Path $RepoRoot 'dependencies.yaml'
$YamlHelper = Join-Path $ScriptDir 'yaml_helper.ps1'
$DetectToolPaths = Join-Path $ScriptDir 'detect_tool_paths.ps1'

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
$MatlabVersion = $null
$MatlabInstall = $null
$VsVersion = $null

# Use shared log files if set by dispatch
$UseLogging = $false
$LogOutput = $null
$LogSummary = $null

if ($env:RS_BUILD_LOG) {
    $LogOutput = $env:RS_BUILD_LOG
    $LogSummary = $env:RS_BUILD_SUMMARY
    $UseLogging = $true
}

Write-Host "Parsing dependencies from: $DepsFile"

if (-not (Test-Path $DepsFile)) {
    Write-Error "dependencies.yaml not found at $DepsFile"
    Exit-Script 1
}

# Helper function for YAML parsing
function Read-YamlVersion {
    param([string]$File, [string]$Section)

    if (Test-Path $YamlHelper) {
        try {
            $result = & powershell -NoProfile -File $YamlHelper -File $File -Section $Section 2>$null
            return $result
        } catch {
            return $null
        }
    }
    return $null
}

$MatlabVersion = Read-YamlVersion $DepsFile 'matlab'
$VsVersion = Read-YamlVersion $DepsFile 'visual_studio'

if (-not $MatlabVersion) {
    Write-Error 'MATLAB version not found in dependencies.yaml'
    Exit-Script 1
}

if (-not $VsVersion) {
    Write-Warning 'Visual Studio version not found in dependencies.yaml, MEX will auto-detect compiler'
}

# Try to locate MATLAB installation
if ($env:MATLAB_ROOT) {
    $MatlabInstall = $env:MATLAB_ROOT
} elseif (Test-Path "$env:ProgramFiles\MATLAB\R$MatlabVersion") {
    $MatlabInstall = "$env:ProgramFiles\MATLAB\R$MatlabVersion"
} else {
    Write-Host 'MATLAB not found at standard location, attempting auto-detection...'
    if (Test-Path $DetectToolPaths) {
        try {
            $MatlabInstall = & powershell -NoProfile -File $DetectToolPaths -Tool 'matlab' 2>$null
        } catch {}
    }
}

if (-not $MatlabInstall) {
    Write-Error 'Could not locate MATLAB installation'
    Write-Error 'Please set MATLAB_ROOT environment variable or add install_path to dependencies.yaml'
    Exit-Script 1
}

Write-Host ''
Write-Host "Building RealSimSocket MEX using MATLAB version: $MatlabVersion"
Write-Host "MATLAB root: $MatlabInstall"

$MexBat = Join-Path $MatlabInstall 'bin\mex.bat'
if (-not (Test-Path $MexBat)) {
    Write-Error "Unable to locate mex.bat at: $MexBat"
    Exit-Script 1
}

$SourceDir = Join-Path $RepoRoot 'CommonLib'
$SourceFile = Join-Path $SourceDir 'RealSimSocket.cpp'

if (-not (Test-Path $SourceFile)) {
    Write-Error "Source file not found: $SourceFile"
    Exit-Script 1
}

Set-Location $SourceDir

# Add section header to summary log if using shared logging
if ($UseLogging) {
    '' | Out-File -FilePath $LogSummary -Append -Encoding UTF8
    'RealSimSocket MEX Build' | Out-File -FilePath $LogSummary -Append -Encoding UTF8
    '-----------------------' | Out-File -FilePath $LogSummary -Append -Encoding UTF8
}

# Set up compiler if Visual Studio version is specified
if ($VsVersion) {
    Write-Host "Configuring MEX to use Visual Studio $VsVersion..."
    $setupOutput = & cmd /c "`"$MexBat`" -setup C++ -client engine COMPILER=msvc$VsVersion 2>&1"
    if ($LASTEXITCODE -ne 0) {
        Write-Warning "Failed to configure Visual Studio $VsVersion, MEX will use default compiler"
    }
}

# Invoke mex
Write-Host 'Invoking mex...'

if ($UseLogging) {
    "===> Building RealSimSocket.mexw64..." | Out-File -FilePath $LogSummary -Append -Encoding UTF8

    $output = & cmd /c "`"$MexBat`" -largeArrayDims -outdir `"$SourceDir`" `"$SourceFile`" 2>&1"
    $output | Out-File -FilePath $LogOutput -Append -Encoding UTF8

    if ($LASTEXITCODE -ne 0) {
        Write-Host ''
        Write-Host '=============================='
        Write-Host 'RealSimSocket MEX build FAILED!'
        Write-Host '=============================='
        "===> RealSimSocket.mexw64 build FAILED" | Out-File -FilePath $LogSummary -Append -Encoding UTF8
        $BuildResult = 1
    } else {
        Write-Host ''
        Write-Host '=============================='
        Write-Host 'RealSimSocket MEX built successfully.'
        Write-Host "Output: $SourceDir\RealSimSocket.mexw64"
        Write-Host '=============================='
        "===> RealSimSocket.mexw64 build SUCCESS" | Out-File -FilePath $LogSummary -Append -Encoding UTF8
        $BuildResult = 0
    }
} else {
    & cmd /c "`"$MexBat`" -largeArrayDims -outdir `"$SourceDir`" `"$SourceFile`""

    if ($LASTEXITCODE -ne 0) {
        Write-Host ''
        Write-Host '=============================='
        Write-Host 'RealSimSocket MEX build FAILED!'
        Write-Host '=============================='
        $BuildResult = 1
    } else {
        Write-Host ''
        Write-Host '=============================='
        Write-Host 'RealSimSocket MEX built successfully.'
        Write-Host "Output: $SourceDir\RealSimSocket.mexw64"
        Write-Host '=============================='
        $BuildResult = 0
    }
}

Exit-Script $BuildResult
