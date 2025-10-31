param(
    [string]$BuildDir = "..\..\build",
    [string]$DepsFile = "..\..\dependencies.yaml",
    [string]$BuildStartTime = (Get-Date -Format "yyyy-MM-dd HH:mm:ss"),
    [string]$BuildDuration = "N/A"
)

$ErrorActionPreference = "SilentlyContinue"

# Resolve paths
$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$BuildDir = Join-Path $ScriptDir $BuildDir | Resolve-Path
$DepsFile = Join-Path $ScriptDir $DepsFile | Resolve-Path
$OutputFile = Join-Path $BuildDir "BUILD_INFO.txt"

if (-not (Test-Path $BuildDir)) {
    Write-Error "Build directory not found: $BuildDir"
    exit 1
}

Write-Host "Generating BUILD_INFO.txt..."

# Get Git information
Push-Location $ScriptDir
$GitBranch = git rev-parse --abbrev-ref HEAD 2>$null
$GitCommit = git rev-parse --short HEAD 2>$null
$GitMsg = git log -1 --pretty=%s 2>$null
$GitTag = git describe --tags --abbrev=0 2>$null
if (-not $GitTag) { $GitTag = "No tag" }

# Count modified files
$GitModified = (git status --porcelain 2>$null | Measure-Object).Count
$GitStatus = if ($GitModified -gt 0) { "Modified ($GitModified file(s))" } else { "Clean" }
Pop-Location

# Parse dependencies.yaml
function Get-YamlValue {
    param([string]$Section)
    if (Test-Path $DepsFile) {
        $content = Get-Content $DepsFile -Raw
        if ($content -match "$Section\s*:\s*version\s*:\s*`"([^`"]+)`"") {
            return $matches[1]
        }
        if ($content -match "$Section\s*:\s*\r?\n\s+version\s*:\s*`"([^`"]+)`"") {
            return $matches[1]
        }
    }
    return "N/A"
}

$SumoVer = Get-YamlValue "sumo"
$CarlaVer = Get-YamlValue "carla"
$CarMakerVer = Get-YamlValue "carmaker"
$MatlabVer = Get-YamlValue "matlab"
$DSpaceVer = Get-YamlValue "dspace"

# Get system info
$OSVersion = (Get-WmiObject Win32_OperatingSystem).Caption
$MSBuildVer = (msbuild -version 2>$null | Select-String "^\d+\.\d+").Matches.Value
$MSBuildPath = (Get-Command msbuild -ErrorAction SilentlyContinue).Source
$MatlabPath = (Get-Command matlab -ErrorAction SilentlyContinue).Source

# Scan build directory
$HasTrafficLayer = Test-Path "$BuildDir\TrafficLayer.exe"
$HasVirtualEnv = Test-Path "$BuildDir\VirtualEnvironment.lib"
$HasVissimDll = Test-Path "$BuildDir\DriverModel_RealSim.dll"
$HasVissim2021Dll = Test-Path "$BuildDir\DriverModel_RealSim_v2021.dll"
$HasMex = Test-Path "$BuildDir\CommonLib\RealSimSocket.mexw64"

$CM9Exists = Test-Path "$BuildDir\CarMaker\CM9"
$CM10Exists = Test-Path "$BuildDir\CarMaker\CM10"
$CM11Exists = Test-Path "$BuildDir\CarMaker\CM11"
$CM13Exists = Test-Path "$BuildDir\CarMaker\CM13"

$DSpaceLibs = Get-ChildItem "$BuildDir\CarMaker\libRealSimDsLib_*.a" -ErrorAction SilentlyContinue
$CarMakerPyFiles = Get-ChildItem "$BuildDir\CarMaker\*.py" -ErrorAction SilentlyContinue
$MatlabFiles = Get-ChildItem "$BuildDir\CommonLib\*.m" -ErrorAction SilentlyContinue

$BuiltCount = ($HasTrafficLayer -as [int]) + ($HasVirtualEnv -as [int]) + ($HasVissimDll -as [int]) + ($HasVissim2021Dll -as [int]) + ($CM9Exists -as [int]) + ($CM10Exists -as [int]) + ($CM11Exists -as [int]) + ($CM13Exists -as [int])
$TotalCount = 11
$SkippedCount = $TotalCount - $BuiltCount

# Generate BUILD_INFO.txt
$output = @"
================================================================================
RealSim FIXS Build Information
================================================================================

BUILD METADATA
--------------
Build Date:           $BuildStartTime
Build Duration:       $BuildDuration
Build Configuration:  Release
Build Platform:       x64

SOURCE INFORMATION
------------------
Git Branch:           $GitBranch
Git Commit:           $GitCommit ($GitMsg)
Git Tag:              $GitTag
Git Status:           $GitStatus

DEPENDENCY VERSIONS
-------------------
Simulators:
  SUMO:               $SumoVer
  CARLA:              $CarlaVer
  CarMaker:           $CarMakerVer
  VISSIM:             2022

Development Tools:
  MATLAB:             $MatlabVer
  dSPACE:             $DSpaceVer
  MSBuild:            $MSBuildVer

Libraries:
  yaml-cpp:           (from CommonLib/yaml-cpp)
  libsumo:            (from SUMO $SumoVer)

BUILD RESULTS
-------------
Core Components:
"@

if ($HasTrafficLayer) { $output += "`n  [OK] TrafficLayer.exe" }
if ($HasVirtualEnv) { $output += "`n  [OK] VirtualEnvironment.lib" }
if (-not $HasTrafficLayer -and -not $HasVirtualEnv) { $output += "`n  (none built)" }

$output += @"

VISSIM Interface:
"@

if ($HasVissimDll) {
    $output += "`n  [OK] DriverModel_RealSim.dll"
} else {
    $output += "`n  [--] DriverModel_RealSim.dll                 [Not built - VISSIMserver not found]"
}

if ($HasVissim2021Dll) {
    $output += "`n  [OK] DriverModel_RealSim_v2021.dll"
} else {
    $output += "`n  [--] DriverModel_RealSim_v2021.dll           [Not built - VISSIMserver not found]"
}

$output += @"

CarMaker Integration:
"@

if (-not $CM9Exists) { $output += "`n  [--] CarMaker 9 (CM9)                        [Skipped - CM9_proj not found]" } else { $output += "`n  [OK] CarMaker 9 (CM9)" }
if (-not $CM10Exists) { $output += "`n  [--] CarMaker 10 (CM10)                      [Skipped - CM10_proj not found]" } else { $output += "`n  [OK] CarMaker 10 (CM10)" }
if (-not $CM11Exists) { $output += "`n  [--] CarMaker 11 (CM11)                      [Skipped - CM11_proj not found]" } else { $output += "`n  [OK] CarMaker 11 (CM11)" }
if (-not $CM13Exists) { $output += "`n  [--] CarMaker 13 (CM13)                      [Skipped - CM13_proj not found]" } else { $output += "`n  [OK] CarMaker 13 (CM13)" }

foreach ($lib in $DSpaceLibs) {
    $output += "`n  [OK] $($lib.Name)"
}

foreach ($py in $CarMakerPyFiles) {
    $output += "`n  [OK] $($py.Name)"
}

$output += @"

MATLAB/Simulink:
"@

if ($HasMex) { $output += "`n  [OK] RealSimSocket.mexw64" }
if ($MatlabFiles.Count -gt 0) { $output += "`n  [OK] CommonLib MATLAB files                  [$($MatlabFiles.Count) files]" }
if (Test-Path "$BuildDir\CommonLib\YAMLMatlab_0.4.3") { $output += "`n  [OK] YAMLMatlab_0.4.3/                       [Library]" }
if (Test-Path "$BuildDir\CommonLib\libsumo") { $output += "`n  [OK] libsumo/                                [Library]" }

$output += @"


BUILD ENVIRONMENT
-----------------
Host OS:              $OSVersion
Build Tool Paths:
"@

if ($MSBuildPath) { $output += "`n  msbuild:            $MSBuildPath" }
if ($MatlabPath) { $output += "`n  MATLAB:             $MatlabPath" }
if ($DSpaceVer -ne "N/A") { $output += "`n  dSPACE:             C:\Program Files\dSPACE ConfigurationDesk $DSpaceVer" }

$output += @"


PACKAGE CONTENTS
----------------
build/
"@

if ($HasTrafficLayer) { $output += "`n  |-- TrafficLayer.exe" }
if ($HasVirtualEnv) { $output += "`n  |-- VirtualEnvironment.lib" }
if ($HasVissimDll) { $output += "`n  |-- DriverModel_RealSim.dll" }
if ($HasVissim2021Dll) { $output += "`n  |-- DriverModel_RealSim_v2021.dll" }
if (Test-Path "$BuildDir\CommonLib") {
    $output += "`n  |-- CommonLib/"
    if ($HasMex) { $output += "`n  |   |-- RealSimSocket.mexw64" }
    if ($MatlabFiles.Count -gt 0) { $output += "`n  |   |-- *.m files ($($MatlabFiles.Count) files)" }
    if (Test-Path "$BuildDir\CommonLib\libsumo") { $output += "`n  |   |-- libsumo/" }
    if (Test-Path "$BuildDir\CommonLib\YAMLMatlab_0.4.3") { $output += "`n  |   +-- YAMLMatlab_0.4.3/" }
}
if (Test-Path "$BuildDir\CarMaker") {
    $output += "`n  +-- CarMaker/"
    if ($CM9Exists) { $output += "`n      |-- CM9/" }
    if ($CM10Exists) { $output += "`n      |-- CM10/" }
    if ($CM11Exists) { $output += "`n      |-- CM11/" }
    if ($CM13Exists) { $output += "`n      |-- CM13/" }
    foreach ($lib in $DSpaceLibs) { $output += "`n      |-- $($lib.Name)" }
    foreach ($py in $CarMakerPyFiles) { $output += "`n      +-- $($py.Name)" }
}

$output += @"


NOTES
-----
"@

if ($BuiltCount -lt $TotalCount) {
    $output += "`n- Partial build: $BuiltCount of $TotalCount components built successfully"
    $output += "`n- $SkippedCount components skipped due to missing source directories"
} else {
    $output += "`n- Full build: All $TotalCount components built successfully"
}

if ($DSpaceLibs.Count -gt 0) {
    $output += "`n- dSPACE libraries available for CarMaker 13.1.3 (11.1.2 also supported)"
}

if ($GitModified -gt 0) {
    $output += "`n- Build contains uncommitted changes"
}

$output += "`n- See dependencies.yaml for required external tool versions"

$output += @"


================================================================================
Generated on $(Get-Date -Format "yyyy-MM-dd HH:mm:ss")
================================================================================
"@

# Write to file
$output | Out-File -FilePath $OutputFile -Encoding UTF8
Write-Host "BUILD_INFO.txt generated: $OutputFile"
