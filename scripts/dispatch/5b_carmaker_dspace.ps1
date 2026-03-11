# ====================================
# Build dSPACE Library for CarMaker
# Supports standalone and inline invocation via dispatch
# ====================================

param(
    [ValidateSet('standalone', 'inline', 'window')]
    [string]$RunMode = 'standalone'
)

$ScriptDir = $PSScriptRoot
$RepoRoot = (Resolve-Path (Join-Path $ScriptDir '..\..')).Path
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
$DspaceVersion = $null
$DspaceInstall = $null
$CarMakerVersions = $null
$CarMakerBase = 'C:\IPG\carmaker'

# Override with environment variables if set
if ($env:RS_FIXS_CARMAKER_BASE) { $CarMakerBase = $env:RS_FIXS_CARMAKER_BASE }
if ($env:CARMAKER_ROOT) { $CarMakerBase = $env:CARMAKER_ROOT }

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

# Helper functions for YAML parsing
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

$DspaceVersion = Read-YamlVersion $DepsFile 'dspace'
$CarMakerVersions = Read-YamlList $DepsFile 'carmaker' 'versions'

if (-not $DspaceVersion) {
    Write-Error 'dSPACE version not found in dependencies.yaml'
    Exit-Script 1
}

# Auto-detect dSPACE installation
Write-Host 'Auto-detecting dSPACE installation...'
if (Test-Path $DetectToolPaths) {
    try {
        $DspaceInstall = & powershell -NoProfile -File $DetectToolPaths -Tool 'dspace' 2>$null
    } catch {}
}

if (-not $DspaceInstall) {
    $msg = 'Could not auto-detect dSPACE installation'
    if ($RunMode -eq 'inline') {
        Write-Warning "$msg; skipping dSPACE CarMaker libraries build."
        if ($UseLogging) {
            '' | Out-File -FilePath $LogSummary -Append -Encoding UTF8
            "SKIPPED: $msg" | Out-File -FilePath $LogSummary -Append -Encoding UTF8
        }
        Exit-Script 0
    }
    Write-Error $msg
    Write-Error 'Please ensure dSPACE ConfigurationDesk is installed'
    Exit-Script 1
}

Write-Host "Auto-detected dSPACE at: $DspaceInstall"

if (-not $CarMakerVersions) {
    $msg = 'CarMaker versions not found in dependencies.yaml'
    if ($RunMode -eq 'inline') {
        Write-Warning "$msg; skipping dSPACE CarMaker libraries build."
        if ($UseLogging) {
            '' | Out-File -FilePath $LogSummary -Append -Encoding UTF8
            "SKIPPED: $msg" | Out-File -FilePath $LogSummary -Append -Encoding UTF8
        }
        Exit-Script 0
    }
    Write-Error $msg
    Exit-Script 1
}

# Sanitize version for filenames (dots to underscores)
$DspaceVersionSafe = $DspaceVersion -replace '\.', '_'

Write-Host ''
Write-Host "Building dSPACE library version: $DspaceVersion"
Write-Host "Install path: $DspaceInstall"
Write-Host "CarMaker install root: $CarMakerBase"
Write-Host "Target CarMaker versions: $CarMakerVersions"

# Check for required dSPACE files
$CfdVars = Join-Path $DspaceInstall 'CFD_vars.bat'
if (-not (Test-Path $CfdVars)) {
    Write-Error "dSPACE environment file not found: $CfdVars"
    Exit-Script 1
}

$DspaceMakefile = Join-Path $DspaceInstall 'SCALEXIO\DsBuildLibrary.mk'
if (-not (Test-Path $DspaceMakefile)) {
    Write-Error "dSPACE makefile not found: $DspaceMakefile"
    Exit-Script 1
}

# Change to CommonLib directory
$CommonLibDir = Join-Path $RepoRoot 'CommonLib'
if (-not (Test-Path $CommonLibDir)) {
    Write-Error "CommonLib directory not found: $CommonLibDir"
    Exit-Script 1
}

Set-Location $CommonLibDir

# Add section header to summary log if using shared logging
if ($UseLogging) {
    '' | Out-File -FilePath $LogSummary -Append -Encoding UTF8
    'dSPACE CarMaker Libraries Build' | Out-File -FilePath $LogSummary -Append -Encoding UTF8
    '-------------------------------' | Out-File -FilePath $LogSummary -Append -Encoding UTF8
}

$CarMakerSuccessCount = 0
$CarMakerFailCount = 0
$CarMakerSkipCount = 0

# Build for each CarMaker version
foreach ($cmVersion in $CarMakerVersions -split '\s+') {
    if ([string]::IsNullOrWhiteSpace($cmVersion)) { continue }

    $carMakerInclude = Join-Path $CarMakerBase "win64-$cmVersion\include"
    $carMakerCM4SL = Join-Path $CarMakerBase "win64-$cmVersion\CM4SL"
    $originalBuildConfig = Join-Path $carMakerCM4SL "CM_BuildConfig.py"

    if (-not (Test-Path $carMakerInclude)) {
        $skipMsg = "CarMaker include path not found for version $cmVersion at $carMakerInclude"
        Write-Warning "$skipMsg; skipping this CarMaker version."
        if ($UseLogging) {
            "===> SKIPPED: $skipMsg" | Out-File -FilePath $LogSummary -Append -Encoding UTF8
        }
        $CarMakerSkipCount++
        continue
    }

    # Sanitize CarMaker version for filename
    $cmVersionSafe = $cmVersion -replace '\.', '_'
    $outputName = "RealSimDsLib_${DspaceVersionSafe}_CM${cmVersionSafe}"
    $cppOpts = "-std=c++11 -I`"$carMakerInclude`" -DDSRTLX -DRS_DSPACE -DRS_CAVE -DRS_DEBUG"

    Write-Host ''
    Write-Host '----------------------------------'
    Write-Host "Building dSPACE library for CarMaker $cmVersion ..."
    Write-Host "Include path: $carMakerInclude"
    Write-Host "Output name : $outputName (CarMaker $cmVersion)"
    Write-Host '----------------------------------'

    # Initialize dSPACE environment and call dsmake
    if ($UseLogging) {
        "===> Building $outputName..." | Out-File -FilePath $LogSummary -Append -Encoding UTF8

        $output = & cmd /c "`"$CfdVars`" >nul 2>&1 && dsmake -f `"$DspaceMakefile`" output_filename=$outputName source_files=`"SocketHelper.cpp MsgHelper.cpp VirEnvHelper.cpp VirEnv_Wrapper.cpp`" custom_cpp_options=`"$cppOpts`" target=Dsx86_32 2>&1"
        $output | Out-File -FilePath $LogOutput -Append -Encoding UTF8
    } else {
        & cmd /c "`"$CfdVars`" && dsmake -f `"$DspaceMakefile`" output_filename=$outputName source_files=`"SocketHelper.cpp MsgHelper.cpp VirEnvHelper.cpp VirEnv_Wrapper.cpp`" custom_cpp_options=`"$cppOpts`" target=Dsx86_32"
    }

    # Check if library was actually created (dsmake returns non-zero even on success due to warnings)
    $libraryFile = "lib$outputName.a"
    if (-not (Test-Path $libraryFile)) {
        Write-Host ''
        Write-Host '=============================='
        Write-Host "dSPACE library build FAILED for CarMaker $cmVersion - output file not found"
        Write-Host '=============================='

        if ($UseLogging) {
            "===> $outputName build FAILED" | Out-File -FilePath $LogSummary -Append -Encoding UTF8
        }

        $CarMakerFailCount++
        $BuildResult = 1
    } else {
        Write-Host ''
        Write-Host '=============================='
        Write-Host "dSPACE library built successfully: $libraryFile"
        Write-Host '=============================='

        if ($UseLogging) {
            "===> $outputName build SUCCESS" | Out-File -FilePath $LogSummary -Append -Encoding UTF8
        }

        $CarMakerSuccessCount++

        # Create modified CM_BuildConfig.py for RealSim
        if (Test-Path $originalBuildConfig) {
            Write-Host "Creating RealSim BuildConfig script for CarMaker $cmVersion ..."

            $carMakerDir = Join-Path $RepoRoot 'CarMaker'
            $rsBuildConfigName = "RS_CM${cmVersionSafe}_BuildConfig_${DspaceVersionSafe}.py"
            $rsBuildConfigPath = Join-Path $carMakerDir $rsBuildConfigName

            # Read original file as UTF8 without BOM
            $content = Get-Content $originalBuildConfig -Raw -Encoding UTF8

            # Modify CARMAKER_DIR path to point to actual installation
            $content = $content -replace 'CARMAKER_DIR\s*=\s*"[^"]*"', "CARMAKER_DIR    = `"C:/IPG/carmaker/win64-$cmVersion`""

            # Add RS defines to CM_DEFINES
            # Find each CM_DEFINES = [ line and prepend RS defines to the existing content
            $content = $content -replace '(CM_DEFINES\s*=\s*\[)(\s*")', '$1"RS_CAVE", "RS_DEBUG", "RS_DSPACE", $2'

            # Add library to libs array in CM_BuildConfig.Initialize()
            # Find: libs = [ "libdscandrv.so" ]
            # Replace with: libs = [ "libdscandrv.so", "lib{outputName}.a" ]
            $libName = "lib$outputName.a"
            $content = $content -replace '(libs\s*=\s*\[\s*"libdscandrv\.so")\s*\]', "`$1, `"$libName`" ]"

            # Write modified file as UTF8 without BOM
            # Use .NET method to avoid BOM in PowerShell 5.1
            [System.IO.File]::WriteAllText($rsBuildConfigPath, $content, (New-Object System.Text.UTF8Encoding $false))

            Write-Host "Created: $rsBuildConfigName"
        } else {
            Write-Warning "Original CM_BuildConfig.py not found at $originalBuildConfig - skipping BuildConfig creation"
        }
    }
}

Write-Host ''
Write-Host "CarMaker build summary: $CarMakerSuccessCount succeeded, $CarMakerFailCount failed, $CarMakerSkipCount skipped"

if ($CarMakerFailCount -ne 0) {
    Write-Host 'ERROR: One or more CarMaker library builds failed.'
} else {
    Write-Host 'All requested CarMaker library builds completed successfully.'
}

Exit-Script $BuildResult
