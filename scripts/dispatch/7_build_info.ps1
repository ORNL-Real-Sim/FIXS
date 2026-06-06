# ====================================
# Generate BUILD_INFO.txt
# Creates comprehensive build metadata file in build/ directory
# ====================================

param(
    [ValidateSet('standalone', 'inline')]
    [string]$RunMode = 'standalone',
    [string]$BuildStartTime = '',
    [string]$BuildDuration = 'N/A'
)

# Resolve paths
$ScriptDir = $PSScriptRoot
$RepoRoot = (Resolve-Path (Join-Path $ScriptDir '..\..')).Path
$BuildDir = Join-Path $RepoRoot 'build'
$DepsFile = Join-Path $RepoRoot 'dependencies.yaml'
$OutputFile = Join-Path $BuildDir 'BUILD_INFO.txt'
$YamlHelper = Join-Path $ScriptDir 'yaml_helper.ps1'

# Helper function to exit with pause in standalone mode
function Exit-Script {
    param([int]$ExitCode = 0)
    if ($RunMode -eq 'standalone') {
        Write-Host ''
        pause
    }
    exit $ExitCode
}

# Use current time if not provided
if ([string]::IsNullOrEmpty($BuildStartTime)) {
    $BuildStartTime = Get-Date -Format 'yyyy-MM-dd HH:mm:ss'
}

# Exit if build directory doesn't exist
if (-not (Test-Path $BuildDir)) {
    Write-Error "Build directory not found: $BuildDir"
    Exit-Script 1
}

Write-Host 'Generating BUILD_INFO.txt...'

# ====================================
# Helper Functions
# ====================================

function Read-YamlVersion {
    param([string]$Section)

    if (Test-Path $YamlHelper) {
        try {
            $result = & powershell -NoProfile -File $YamlHelper -File $DepsFile -Section $Section 2>$null
            return $result
        } catch {
            return $null
        }
    }
    return $null
}

function Read-YamlKey {
    param([string]$Section, [string]$Key)

    if (Test-Path $YamlHelper) {
        try {
            $result = & powershell -NoProfile -File $YamlHelper -File $DepsFile -Section $Section -Key $Key 2>$null
            return $result
        } catch {
            return $null
        }
    }
    return $null
}

function Read-YamlList {
    param([string]$Section, [string]$ListKey)

    if (Test-Path $YamlHelper) {
        try {
            $result = & powershell -NoProfile -File $YamlHelper -File $DepsFile -Section $Section -ListKey $ListKey -ReturnList 2>$null
            return $result
        } catch {
            return $null
        }
    }
    return $null
}

function Get-FileStatusLine {
    param(
        [string]$FilePath,
        [string]$FileName,
        [string]$MissingMsg = ''
    )

    if (Test-Path $FilePath) {
        return "  $script:CheckSymbol $FileName"
    } elseif (-not [string]::IsNullOrEmpty($MissingMsg)) {
        $padding = ' ' * [Math]::Max(0, 50 - "  $script:CrossSymbol $FileName".Length)
        return "  $script:CrossSymbol $FileName$padding[$MissingMsg]"
    }
    return $null
}

# ====================================
# Gather System Information
# ====================================

$CheckSymbol = [char]0x2713  # ✓
$CrossSymbol = [char]0x2717  # ✗
$TreeBranch = [char]0x251C + [char]0x2500 + [char]0x2500  # +--
$TreeLine = [char]0x2502  # |

# OS and hostname
$OsVersion = (Get-CimInstance Win32_OperatingSystem).Caption
$Hostname = $env:COMPUTERNAME

# ====================================
# Git Information
# ====================================

$GitAvailable = $null -ne (Get-Command git -ErrorAction SilentlyContinue)

if ($GitAvailable) {
    try {
        $GitBranch = git rev-parse --abbrev-ref HEAD 2>$null
        $GitCommit = git rev-parse --short HEAD 2>$null
        $GitMsg = git log -1 --pretty=%s 2>$null
        $GitTag = git describe --tags --abbrev=0 2>$null
        if ([string]::IsNullOrEmpty($GitTag)) { $GitTag = 'No tag' }

        # Check for uncommitted changes
        git diff-index --quiet HEAD -- 2>$null
        $GitStatusClean = $LASTEXITCODE -eq 0

        # Count modified files
        $GitModified = (git status --porcelain 2>$null | Measure-Object).Count
    } catch {
        $GitBranch = 'N/A'
        $GitCommit = 'N/A'
        $GitMsg = 'N/A'
        $GitTag = 'N/A'
        $GitStatusClean = $true
        $GitModified = 0
    }
} else {
    $GitBranch = 'N/A'
    $GitCommit = 'N/A'
    $GitMsg = 'N/A'
    $GitTag = 'N/A'
    $GitStatusClean = $true
    $GitModified = 0
}

# ====================================
# Dependency Versions
# ====================================

$SumoVer = Read-YamlVersion 'sumo'
$CarlaVer = Read-YamlVersion 'carla'
$CarMakerVer = Read-YamlVersion 'carmaker'
$MatlabVer = Read-YamlVersion 'matlab'
$DspaceVer = Read-YamlVersion 'dspace'
$CarMakerVersions = Read-YamlList 'carmaker' 'versions'
$YamlMatlabDir = Read-YamlKey 'yaml_matlab' 'location'

# ====================================
# Build Tool Versions
# ====================================

$MsBuildVer = $null
$MsBuildPath = $null
try {
    $MsBuildPath = (Get-Command msbuild -ErrorAction SilentlyContinue).Source
    if ($MsBuildPath) {
        $MsBuildVerOutput = & msbuild -version 2>$null | Select-String '^\d'
        if ($MsBuildVerOutput) {
            $MsBuildVer = $MsBuildVerOutput.ToString().Trim()
        }
    }
} catch {}

$MatlabPath = $null
try {
    $MatlabPath = (Get-Command matlab -ErrorAction SilentlyContinue).Source
} catch {}

# ====================================
# Scan Build Directory
# ====================================

# Define expected components for each category
$ExpectedCore = @('TrafficLayer.exe', 'VirtualEnvironment.lib')
$ExpectedVissim = @('DriverModel_RealSim.dll', 'DriverModel_RealSim_legacy.dll')

$CountCore = 0
$CountVissim = 0
$CountCm = 0
$CountMatlab = 0
$CountDsLib = 0

# Count Core components
foreach ($file in $ExpectedCore) {
    if (Test-Path (Join-Path $BuildDir $file)) { $CountCore++ }
}

# Count VISSIM components
foreach ($file in $ExpectedVissim) {
    if (Test-Path (Join-Path $BuildDir $file)) { $CountVissim++ }
}

# Count CarMaker versions
$ExpectedCm = 0
if ($CarMakerVersions) {
    foreach ($version in $CarMakerVersions -split '\s+') {
        if ([string]::IsNullOrWhiteSpace($version)) { continue }
        $ExpectedCm++
        $major = $version.Split('.')[0]
        if (Test-Path (Join-Path $BuildDir "CarMaker\CM$major")) {
            $CountCm++
        }
    }
}

# Count dSPACE libraries
$dsLibPath = Join-Path $BuildDir 'CarMaker\libRealSimDsLib_*.a'
if (Test-Path (Split-Path $dsLibPath -Parent)) {
    $CountDsLib = (Get-ChildItem $dsLibPath -ErrorAction SilentlyContinue).Count
}

# Count MATLAB files
if (Test-Path (Join-Path $BuildDir 'CommonLib\RealSimSocket.mexw64')) { $CountMatlab++ }
$mFilesPath = Join-Path $BuildDir 'CommonLib\*.m'
if (Test-Path (Split-Path $mFilesPath -Parent)) {
    $CountMatlab += (Get-ChildItem $mFilesPath -ErrorAction SilentlyContinue).Count
}

# Build statistics - auto-calculate based on expected components
$TotalComponents = $ExpectedCore.Count + $ExpectedVissim.Count + $ExpectedCm
$BuiltComponents = $CountCore + $CountVissim + $CountCm
$SkippedComponents = $TotalComponents - $BuiltComponents

# ====================================
# Generate BUILD_INFO.txt
# ====================================

$sb = [System.Text.StringBuilder]::new()

[void]$sb.AppendLine('================================================================================')
[void]$sb.AppendLine('RealSim FIXS Build Information')
[void]$sb.AppendLine('================================================================================')
[void]$sb.AppendLine()
[void]$sb.AppendLine('BUILD METADATA')
[void]$sb.AppendLine('--------------')
[void]$sb.AppendLine("Build Date:           $BuildStartTime")
[void]$sb.AppendLine("Build Duration:       $BuildDuration")
[void]$sb.AppendLine('Build Configuration:  Release')
[void]$sb.AppendLine('Build Platform:       x64')
[void]$sb.AppendLine()
[void]$sb.AppendLine('SOURCE INFORMATION')
[void]$sb.AppendLine('------------------')
[void]$sb.AppendLine("Git Branch:           $GitBranch")
[void]$sb.AppendLine("Git Commit:           $GitCommit ($GitMsg)")
[void]$sb.AppendLine("Git Tag:              $GitTag")
if ($GitModified -gt 0) {
    [void]$sb.AppendLine("Git Status:           Modified ($GitModified file(s))")
} else {
    [void]$sb.AppendLine('Git Status:           Clean')
}
[void]$sb.AppendLine()
[void]$sb.AppendLine('DEPENDENCY VERSIONS')
[void]$sb.AppendLine('-------------------')
[void]$sb.AppendLine('Simulators:')
if ($SumoVer) { [void]$sb.AppendLine("  SUMO:               $SumoVer") }
if ($CarlaVer) { [void]$sb.AppendLine("  CARLA:              $CarlaVer") }
if ($CarMakerVer) { [void]$sb.AppendLine("  CarMaker:           $CarMakerVer") }
[void]$sb.AppendLine('  VISSIM:             2022')
[void]$sb.AppendLine()
[void]$sb.AppendLine('Development Tools:')
if ($MatlabVer) { [void]$sb.AppendLine("  MATLAB:             $MatlabVer") }
if ($DspaceVer) { [void]$sb.AppendLine("  dSPACE:             $DspaceVer") }
if ($MsBuildVer) { [void]$sb.AppendLine("  MSBuild:            $MsBuildVer") }
[void]$sb.AppendLine()
[void]$sb.AppendLine('Libraries:')
[void]$sb.AppendLine('  yaml-cpp:           (from CommonLib/yaml-cpp)')
if ($SumoVer) { [void]$sb.AppendLine("  libsumo:            (from SUMO $SumoVer)") }
[void]$sb.AppendLine()
[void]$sb.AppendLine('BUILD RESULTS')
[void]$sb.AppendLine('-------------')
[void]$sb.AppendLine('Core Components:')

# Core components
$line = Get-FileStatusLine (Join-Path $BuildDir 'TrafficLayer.exe') 'TrafficLayer.exe'
if ($line) { [void]$sb.AppendLine($line) }
$line = Get-FileStatusLine (Join-Path $BuildDir 'VirtualEnvironment.lib') 'VirtualEnvironment.lib'
if ($line) { [void]$sb.AppendLine($line) }

[void]$sb.AppendLine()
[void]$sb.AppendLine('VISSIM Interface:')

# VISSIM components
$line = Get-FileStatusLine (Join-Path $BuildDir 'DriverModel_RealSim.dll') 'DriverModel_RealSim.dll' 'Not built - VISSIMserver not found'
if ($line) { [void]$sb.AppendLine($line) }
$line = Get-FileStatusLine (Join-Path $BuildDir 'DriverModel_RealSim_legacy.dll') 'DriverModel_RealSim_legacy.dll' 'Not built - VISSIMserver not found'
if ($line) { [void]$sb.AppendLine($line) }

[void]$sb.AppendLine()
[void]$sb.AppendLine('CarMaker Integration:')

# CarMaker versions
if ($CarMakerVersions) {
    foreach ($version in $CarMakerVersions -split '\s+') {
        if ([string]::IsNullOrWhiteSpace($version)) { continue }
        $major = $version.Split('.')[0]
        $label = "CarMaker $version (CM$major)"
        $skipMsg = "Skipped - CM${major}_proj not found"
        $line = Get-FileStatusLine (Join-Path $BuildDir "CarMaker\CM$major") $label $skipMsg
        if ($line) { [void]$sb.AppendLine($line) }
    }
} else {
    [void]$sb.AppendLine('  [No CarMaker versions configured in dependencies.yaml]')
}

# dSPACE libraries
$dsLibPath = Join-Path $BuildDir 'CarMaker\libRealSimDsLib_*.a'
if (Test-Path (Split-Path $dsLibPath -Parent)) {
    Get-ChildItem $dsLibPath -ErrorAction SilentlyContinue | ForEach-Object {
        [void]$sb.AppendLine("  $CheckSymbol $($_.Name)")
    }
}

# CarMaker Python utilities
$pyPath = Join-Path $BuildDir 'CarMaker\*.py'
if (Test-Path (Split-Path $pyPath -Parent)) {
    Get-ChildItem $pyPath -ErrorAction SilentlyContinue | ForEach-Object {
        [void]$sb.AppendLine("  $CheckSymbol $($_.Name)")
    }
}

[void]$sb.AppendLine()
[void]$sb.AppendLine('MATLAB/Simulink:')

# MATLAB components
$line = Get-FileStatusLine (Join-Path $BuildDir 'CommonLib\RealSimSocket.mexw64') 'RealSimSocket.mexw64'
if ($line) { [void]$sb.AppendLine($line) }

$mFilesPath = Join-Path $BuildDir 'CommonLib\*.m'
if (Test-Path (Split-Path $mFilesPath -Parent)) {
    if ((Get-ChildItem $mFilesPath -ErrorAction SilentlyContinue).Count -gt 0) {
        $padding = ' ' * [Math]::Max(0, 50 - "  $CheckSymbol CommonLib MATLAB files".Length)
        [void]$sb.AppendLine("  $CheckSymbol CommonLib MATLAB files$padding[$CountMatlab files]")
    }
}

if ($YamlMatlabDir) {
    $yamlMatlabFolder = Split-Path $YamlMatlabDir -Leaf
    if (Test-Path (Join-Path $BuildDir "CommonLib\$yamlMatlabFolder")) {
        $padding = ' ' * [Math]::Max(0, 50 - "  $CheckSymbol $yamlMatlabFolder/".Length)
        [void]$sb.AppendLine("  $CheckSymbol $yamlMatlabFolder/$padding[Library]")
    }
}

if (Test-Path (Join-Path $BuildDir 'CommonLib\libsumo')) {
    $padding = ' ' * [Math]::Max(0, 50 - "  $CheckSymbol libsumo/".Length)
    [void]$sb.AppendLine("  $CheckSymbol libsumo/$padding[Library]")
}

[void]$sb.AppendLine()
[void]$sb.AppendLine('Python TrafficLayer Protocol:')
$pyFilesPath = Join-Path $BuildDir 'CommonLib\*.py'
$CountPython = 0
if (Test-Path (Split-Path $pyFilesPath -Parent)) {
    $CountPython = (Get-ChildItem $pyFilesPath -ErrorAction SilentlyContinue).Count
}
if ($CountPython -gt 0) {
    $padding = ' ' * [Math]::Max(0, 50 - "  $CheckSymbol CommonLib Python package".Length)
    [void]$sb.AppendLine("  $CheckSymbol CommonLib Python package$padding[$CountPython files]")
} else {
    [void]$sb.AppendLine("  $CrossSymbol CommonLib Python package          [No .py files staged]")
}

[void]$sb.AppendLine()
[void]$sb.AppendLine('BUILD ENVIRONMENT')
[void]$sb.AppendLine('-----------------')
[void]$sb.AppendLine("Host OS:              $OsVersion")
[void]$sb.AppendLine('Build Tool Paths:')
if ($MsBuildPath) { [void]$sb.AppendLine("  msbuild:            $MsBuildPath") }
if ($MatlabPath) { [void]$sb.AppendLine("  MATLAB:             $MatlabPath") }
if ($DspaceVer) { [void]$sb.AppendLine("  dSPACE:             C:\Program Files\dSPACE ConfigurationDesk $DspaceVer") }
[void]$sb.AppendLine()
[void]$sb.AppendLine('PACKAGE CONTENTS')
[void]$sb.AppendLine('----------------')
[void]$sb.AppendLine('build/')

# Build directory tree
if (Test-Path (Join-Path $BuildDir 'TrafficLayer.exe')) { [void]$sb.AppendLine('  +-- TrafficLayer.exe') }
if (Test-Path (Join-Path $BuildDir 'VirtualEnvironment.lib')) { [void]$sb.AppendLine('  +-- VirtualEnvironment.lib') }
if (Test-Path (Join-Path $BuildDir 'DriverModel_RealSim.dll')) { [void]$sb.AppendLine('  +-- DriverModel_RealSim.dll') }
if (Test-Path (Join-Path $BuildDir 'DriverModel_RealSim_legacy.dll')) { [void]$sb.AppendLine('  +-- DriverModel_RealSim_legacy.dll') }

if (Test-Path (Join-Path $BuildDir 'CommonLib')) {
    [void]$sb.AppendLine('  +-- CommonLib/')
    if (Test-Path (Join-Path $BuildDir 'CommonLib\RealSimSocket.mexw64')) { [void]$sb.AppendLine('  |   +-- RealSimSocket.mexw64') }

    $mFilesPath = Join-Path $BuildDir 'CommonLib\*.m'
    if (Test-Path (Split-Path $mFilesPath -Parent)) {
        if ((Get-ChildItem $mFilesPath -ErrorAction SilentlyContinue).Count -gt 0) {
            [void]$sb.AppendLine("  |   +-- *.m files ($CountMatlab files)")
        }
    }

    if ($CountPython -gt 0) {
        [void]$sb.AppendLine("  |   +-- *.py files ($CountPython files)")
    }

    if (Test-Path (Join-Path $BuildDir 'CommonLib\libsumo')) { [void]$sb.AppendLine('  |   +-- libsumo/') }

    if ($YamlMatlabDir) {
        $yamlMatlabFolder = Split-Path $YamlMatlabDir -Leaf
        if (Test-Path (Join-Path $BuildDir "CommonLib\$yamlMatlabFolder")) {
            [void]$sb.AppendLine("  |   +-- $yamlMatlabFolder/")
        }
    }
}

if (Test-Path (Join-Path $BuildDir 'CarMaker')) {
    [void]$sb.AppendLine('  +-- CarMaker/')

    if ($CarMakerVersions) {
        foreach ($version in $CarMakerVersions -split '\s+') {
            if ([string]::IsNullOrWhiteSpace($version)) { continue }
            $major = $version.Split('.')[0]
            if (Test-Path (Join-Path $BuildDir "CarMaker\CM$major")) {
                [void]$sb.AppendLine("      +-- CM$major/")
            }
        }
    }

    $dsLibPath = Join-Path $BuildDir 'CarMaker\libRealSimDsLib_*.a'
    if (Test-Path (Split-Path $dsLibPath -Parent)) {
        Get-ChildItem $dsLibPath -ErrorAction SilentlyContinue | ForEach-Object {
            [void]$sb.AppendLine("      +-- $($_.Name)")
        }
    }

    $pyPath = Join-Path $BuildDir 'CarMaker\*.py'
    if (Test-Path (Split-Path $pyPath -Parent)) {
        Get-ChildItem $pyPath -ErrorAction SilentlyContinue | ForEach-Object {
            [void]$sb.AppendLine("      +-- $($_.Name)")
        }
    }
}

[void]$sb.AppendLine()
[void]$sb.AppendLine('NOTES')
[void]$sb.AppendLine('-----')

if ($BuiltComponents -lt $TotalComponents) {
    [void]$sb.AppendLine("- Partial build: $BuiltComponents of $TotalComponents components built successfully")
    [void]$sb.AppendLine("- $SkippedComponents components skipped due to missing source directories")
} else {
    [void]$sb.AppendLine("- Full build: All $TotalComponents components built successfully")
}

if ($CountDsLib -gt 0) {
    [void]$sb.AppendLine('- dSPACE libraries available for CarMaker 13.1.3 (11.1.2 also supported)')
}

if ($GitModified -gt 0) {
    [void]$sb.AppendLine('- Build contains uncommitted changes')
}

[void]$sb.AppendLine('- See dependencies.yaml for required external tool versions')
[void]$sb.AppendLine()
[void]$sb.AppendLine('================================================================================')
[void]$sb.AppendLine("Generated by dispatch.bat on $(Get-Date -Format 'yyyy-MM-dd HH:mm:ss')")
[void]$sb.AppendLine('================================================================================')

# Write to file
$sb.ToString() | Out-File -FilePath $OutputFile -Encoding UTF8

Write-Host "BUILD_INFO.txt generated: $OutputFile"

Exit-Script 0
