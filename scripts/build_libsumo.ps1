# Script to build SUMO Release and Debug DLLs from source
# Reads version from dependencies.yaml, clones SUMO, builds Release and Debug DLLs, and copies them along with headers

param(
    [switch]$KeepBuildDir = $false,
    [switch]$DryRun = $false,
    [string]$Generator = "Visual Studio 17 2022",
    [string]$VcpkgRoot = "C:\vcpkg",
    [switch]$UseSumoLibraries = $true  # Use SUMOLibraries by default (faster)
)

# Read version from dependencies.yaml
$depsFile = Join-Path $PSScriptRoot "..\dependencies.yaml"
if (-not (Test-Path $depsFile)) {
    Write-Error "dependencies.yaml not found at $depsFile"
    exit 1
}

$yamlContent = Get-Content $depsFile -Raw
if ($yamlContent -match 'sumo:\s+version:\s+"([^"]+)"') {
    $version = $matches[1]
} else {
    Write-Error "Could not parse SUMO version from dependencies.yaml"
    exit 1
}

Write-Host "Building SUMO Release and Debug DLLs for version $version" -ForegroundColor Cyan

# Check for required tools
Write-Host "`nChecking prerequisites..." -ForegroundColor Cyan

# Check Windows long path support
Write-Host "  Checking Windows long path support..." -NoNewline
$longPathEnabled = (Get-ItemProperty -Path "HKLM:\SYSTEM\CurrentControlSet\Control\FileSystem" -Name "LongPathsEnabled" -ErrorAction SilentlyContinue).LongPathsEnabled
if ($longPathEnabled -eq 1) {
    Write-Host " OK" -ForegroundColor Green
} else {
    Write-Host " NOT ENABLED" -ForegroundColor Yellow
    Write-Host "`nWarning: Windows long path support is not enabled." -ForegroundColor Yellow
    Write-Host "This may cause issues with files that have long names." -ForegroundColor Yellow
    Write-Host "`nTo enable long paths, run PowerShell as Administrator and execute:" -ForegroundColor Cyan
    Write-Host '  New-ItemProperty -Path "HKLM:\SYSTEM\CurrentControlSet\Control\FileSystem" -Name "LongPathsEnabled" -Value 1 -PropertyType DWORD -Force' -ForegroundColor Gray
    Write-Host "`nOr enable via Group Policy: Computer Configuration > Administrative Templates > System > Filesystem > Enable Win32 long paths" -ForegroundColor Cyan
    Write-Host "`nContinuing anyway (may fail for files with long names)..." -ForegroundColor Yellow
}

# Check for git
try {
    $null = git --version
    Write-Host "  Git: OK" -ForegroundColor Green

    # Enable long paths for Git (Windows has 260 char limit by default)
    Write-Host "  Enabling Git long path support..." -NoNewline
    git config --global core.longpaths true
    Write-Host " OK" -ForegroundColor Green
} catch {
    Write-Error "Git not found. Please install Git and add it to PATH."
    exit 1
}

# Check for cmake
try {
    $null = cmake --version
    Write-Host "  CMake: OK" -ForegroundColor Green
} catch {
    Write-Error "CMake not found. Please install CMake and add it to PATH."
    exit 1
}

# Setup temporary directory (in repo, will be gitignored)
$repoRoot = Join-Path $PSScriptRoot ".."
$tempDir = Join-Path $repoRoot "tmp"
$sumoBuildDir = Join-Path $tempDir "sumo_build"
$sumoDir = Join-Path $sumoBuildDir "sumo"
$buildDir = Join-Path $sumoDir "build"
$sumoLibrariesDir = Join-Path $sumoBuildDir "SUMOLibraries"

Write-Host "`nBuild directory: $sumoBuildDir" -ForegroundColor Cyan
Write-Host "(This directory is gitignored; source clones will be kept for faster future builds)" -ForegroundColor Gray

if ($DryRun) {
    Write-Host "[DRY RUN] Would build in $sumoBuildDir" -ForegroundColor Yellow
    exit 0
}

# Check if SUMO build directory already exists
$sumoExists = $false
$sumoLibrariesExists = $false
$reuseClones = $false
$skipBuild = $false

# Check if SUMO is a valid git repository
if ((Test-Path $sumoDir) -and (Test-Path (Join-Path $sumoDir ".git"))) {
    Push-Location $sumoDir
    try {
        $null = git rev-parse --git-dir 2>&1
        if ($LASTEXITCODE -eq 0) {
            $sumoExists = $true
        }
    } catch {
        # Not a valid git repo
    }
    Pop-Location
}

# Check if SUMOLibraries is a valid git repository
if ((Test-Path $sumoLibrariesDir) -and (Test-Path (Join-Path $sumoLibrariesDir ".git"))) {
    Push-Location $sumoLibrariesDir
    try {
        $null = git rev-parse --git-dir 2>&1
        if ($LASTEXITCODE -eq 0) {
            $sumoLibrariesExists = $true
        }
    } catch {
        # Not a valid git repo
    }
    Pop-Location
}

if (Test-Path $sumoBuildDir) {
    Write-Host "`nSUMO build directory found: $sumoBuildDir" -ForegroundColor Cyan

    # Check what's inside
    $existingItems = @()
    if ($sumoExists) { $existingItems += "sumo" }
    if ($sumoLibrariesExists) { $existingItems += "SUMOLibraries" }

    # Check for invalid repositories
    $hasInvalidRepos = $false
    if ((Test-Path $sumoDir) -and -not $sumoExists) {
        Write-Host "  Warning: sumo directory exists but is not a valid git repository" -ForegroundColor Yellow
        $hasInvalidRepos = $true
    }
    if ((Test-Path $sumoLibrariesDir) -and -not $sumoLibrariesExists) {
        Write-Host "  Warning: SUMOLibraries directory exists but is not a valid git repository" -ForegroundColor Yellow
        $hasInvalidRepos = $true
    }

    if ($hasInvalidRepos) {
        $response = Read-Host "Invalid repositories found. Re-clone? (Y=yes/N=quit)"
        if ($response -match '^[Yy]') {
            Write-Host "Removing invalid directories to re-clone..." -ForegroundColor Yellow
            if ((Test-Path $sumoDir) -and -not $sumoExists) {
                Remove-Item $sumoDir -Recurse -Force -ErrorAction Stop
            }
            if ((Test-Path $sumoLibrariesDir) -and -not $sumoLibrariesExists) {
                Remove-Item $sumoLibrariesDir -Recurse -Force -ErrorAction Stop
            }
            $sumoExists = $false
            $sumoLibrariesExists = $false
        } else {
            Write-Host "Exiting..." -ForegroundColor Gray
            exit 0
        }
    }

    if ($existingItems.Count -gt 0) {
        Write-Host "Found valid repositories: $($existingItems -join ', ')" -ForegroundColor Green

        # Check if DLLs already exist in bin
        $binDir = Join-Path $sumoDir "bin"
        $requiredDlls = @("libsumocpp.dll", "libsumocppD.dll", "libtracicpp.dll", "libtracicppD.dll")
        $allDllsExist = $true

        foreach ($dll in $requiredDlls) {
            if (-not (Test-Path (Join-Path $binDir $dll))) {
                $allDllsExist = $false
                break
            }
        }

        if ($allDllsExist) {
            Write-Host "`nAll required DLLs already exist in the build!" -ForegroundColor Green
            $response = Read-Host "What would you like to do? (C=copy existing/R=rebuild/N=re-clone/Q=quit)"

            if ($response -match '^[Cc]') {
                Write-Host "Will copy existing DLLs and headers (fastest)" -ForegroundColor Green
                $skipBuild = $true
                $reuseClones = $true
            } elseif ($response -match '^[Rr]') {
                Write-Host "Will rebuild from source" -ForegroundColor Yellow
                $reuseClones = $true
                if (Test-Path $buildDir) {
                    Write-Host "Removing old build artifacts..." -ForegroundColor Yellow
                    Remove-Item $buildDir -Recurse -Force -ErrorAction Stop
                }
            } elseif ($response -match '^[Nn]') {
                Write-Host "Removing old SUMO build directory to re-clone..." -ForegroundColor Yellow
                Remove-Item $sumoBuildDir -Recurse -Force -ErrorAction Stop
                $sumoExists = $false
                $sumoLibrariesExists = $false
            } else {
                Write-Host "Exiting..." -ForegroundColor Gray
                exit 0
            }
        } else {
            $response = Read-Host "Do you want to reuse existing clones or re-clone? (Y=reuse/N=re-clone/Q=quit)"

            if ($response -match '^[Yy]') {
                Write-Host "Reusing existing clones (faster build)" -ForegroundColor Green
                $reuseClones = $true

                # Only delete build directory if it exists
                if (Test-Path $buildDir) {
                    Write-Host "Removing old build artifacts..." -ForegroundColor Yellow
                    Remove-Item $buildDir -Recurse -Force -ErrorAction Stop
                }
            } elseif ($response -match '^[Nn]') {
                Write-Host "Removing old SUMO build directory to re-clone..." -ForegroundColor Yellow
                Remove-Item $sumoBuildDir -Recurse -Force -ErrorAction Stop
                $sumoExists = $false
                $sumoLibrariesExists = $false
            } else {
                Write-Host "Exiting..." -ForegroundColor Gray
                exit 0
            }
        }
    } else {
        # Directory exists but is empty or has invalid repos, will clone fresh
        Write-Host "Will clone fresh." -ForegroundColor Cyan
    }
}

# Create temp directory structure if it doesn't exist
if (-not (Test-Path $sumoBuildDir)) {
    New-Item -ItemType Directory -Path $sumoBuildDir -Force | Out-Null
}

try {
    Push-Location $sumoBuildDir

    if (-not $skipBuild) {
        # Clone SUMOLibraries if requested
        if ($UseSumoLibraries) {
            if ($sumoLibrariesExists -and $reuseClones) {
                Write-Host "`nReusing existing SUMOLibraries clone" -ForegroundColor Green
            } else {
                Write-Host "`nCloning SUMOLibraries (pre-compiled dependencies)..." -ForegroundColor Cyan
                Write-Host "  git clone --depth 1 https://github.com/DLR-TS/SUMOLibraries.git" -ForegroundColor Gray
                git clone --depth 1 https://github.com/DLR-TS/SUMOLibraries.git

                if ($LASTEXITCODE -ne 0) {
                    throw "Git clone of SUMOLibraries failed"
                }

                Write-Host "  SUMOLibraries clone complete" -ForegroundColor Green
            }
        }

        # Clone SUMO repository
        if ($sumoExists -and $reuseClones) {
            Write-Host "`nReusing existing SUMO clone" -ForegroundColor Green
        } else {
            Write-Host "`nCloning SUMO repository..." -ForegroundColor Cyan
            $tag = "v$($version.Replace('.', '_'))"

            Write-Host "  git clone --depth 1 --branch $tag https://github.com/eclipse/sumo.git" -ForegroundColor Gray
            git clone --depth 1 --branch $tag https://github.com/eclipse/sumo.git

            if ($LASTEXITCODE -ne 0) {
                throw "Git clone failed"
            }

            Write-Host "  Clone complete" -ForegroundColor Green
        }

        # Create build directory
        New-Item -ItemType Directory -Path $buildDir -Force | Out-Null

        # Configure with CMake
        Write-Host "`nConfiguring with CMake..." -ForegroundColor Cyan
        Push-Location $buildDir

        # Determine which dependency method to use
        if ($UseSumoLibraries -and (Test-Path $sumoLibrariesDir)) {
            Write-Host "  Using SUMOLibraries from: $sumoLibrariesDir" -ForegroundColor Green
            Write-Host "  cmake .. -G `"$Generator`" -A x64 -DSUMO_LIBRARIES=`"$sumoLibrariesDir`"" -ForegroundColor Gray
            cmake .. -G $Generator -A x64 -DSUMO_LIBRARIES="$sumoLibrariesDir"
        } else {
            # Check if vcpkg exists
            $vcpkgToolchain = Join-Path $VcpkgRoot "scripts\buildsystems\vcpkg.cmake"

            if (Test-Path $vcpkgToolchain) {
                Write-Host "  Using vcpkg toolchain from: $VcpkgRoot" -ForegroundColor Green
                Write-Host "  cmake .. -G `"$Generator`" -A x64 -DCMAKE_TOOLCHAIN_FILE=`"$vcpkgToolchain`"" -ForegroundColor Gray
                cmake .. -G $Generator -A x64 -DCMAKE_TOOLCHAIN_FILE="$vcpkgToolchain"
            } else {
                Write-Host "  No dependency manager found (vcpkg or SUMOLibraries)" -ForegroundColor Yellow
                Write-Host "  Attempting build without dependencies (may fail)" -ForegroundColor Yellow
                Write-Host "  cmake .. -G `"$Generator`" -A x64" -ForegroundColor Gray
                cmake .. -G $Generator -A x64
            }
        }

        if ($LASTEXITCODE -ne 0) {
            throw "CMake configuration failed"
        }

        Write-Host "  Configuration complete" -ForegroundColor Green

        # Build Release configuration first
        Write-Host "`nBuilding Release libraries (this may take 10-30 minutes)..." -ForegroundColor Cyan
        Write-Host "  Building libsumocpp (Release)..." -ForegroundColor Gray
        cmake --build . --config Release --target libsumocpp

        if ($LASTEXITCODE -ne 0) {
            throw "Building libsumocpp (Release) failed"
        }

        Write-Host "  Building libtracicpp (Release)..." -ForegroundColor Gray
        cmake --build . --config Release --target libtracicpp

        if ($LASTEXITCODE -ne 0) {
            throw "Building libtracicpp (Release) failed"
        }

        Write-Host "  Release build complete" -ForegroundColor Green

        # Build Debug configuration
        Write-Host "`nBuilding Debug libraries (this may take 10-30 minutes)..." -ForegroundColor Cyan
        Write-Host "  Building libsumocpp (Debug)..." -ForegroundColor Gray
        cmake --build . --config Debug --target libsumocpp

        if ($LASTEXITCODE -ne 0) {
            throw "Building libsumocpp (Debug) failed"
        }

        Write-Host "  Building libtracicpp (Debug)..." -ForegroundColor Gray
        cmake --build . --config Debug --target libtracicpp

        if ($LASTEXITCODE -ne 0) {
            throw "Building libtracicpp (Debug) failed"
        }

        Write-Host "  Debug build complete" -ForegroundColor Green

        Pop-Location
        Pop-Location
    } else {
        Write-Host "`nSkipping build, using existing DLLs" -ForegroundColor Green
    }

    # Find and copy DLLs and headers
    Write-Host "`nPreparing CommonLib\libsumo directory..." -ForegroundColor Cyan

    $destDir = Join-Path $PSScriptRoot "..\CommonLib\libsumo"

    # Clean up existing CommonLib/libsumo to avoid stale files
    if (Test-Path $destDir) {
        Write-Host "  Cleaning up old files in CommonLib\libsumo..." -ForegroundColor Yellow
        Remove-Item -Path $destDir -Recurse -Force -ErrorAction Stop
        Write-Host "  Old files removed" -ForegroundColor Green
    }

    # Create fresh directory
    New-Item -ItemType Directory -Path $destDir -Force | Out-Null
    Write-Host "  Created fresh CommonLib\libsumo directory" -ForegroundColor Green

    Write-Host "`nCopying Release and Debug DLLs..." -ForegroundColor Cyan

    # DLLs are built directly in sumo/bin
    $binDir = Join-Path $sumoDir "bin"

    # List of .lib files to copy to bin subdirectory (for linking)
    $requiredLibs = @(
        "libsumocpp.lib",
        "libtracicpp.lib",
        "libsumocppD.lib",
        "libtracicppD.lib"
    )

    # Create bin subdirectory for both .lib and .dll files
    $destBinDir = Join-Path $destDir "bin"
    if (-not (Test-Path $destBinDir)) {
        New-Item -ItemType Directory -Path $destBinDir -Force | Out-Null
    }

    $successCount = 0
    $failCount = 0

    # Copy .lib files to bin subdirectory
    Write-Host "`nCopying .lib files to bin/..." -ForegroundColor Cyan
    foreach ($file in $requiredLibs) {
        $sourceFile = Join-Path $binDir $file
        $destFile = Join-Path $destBinDir $file

        Write-Host "  Copying $file..." -NoNewline

        if (Test-Path $sourceFile) {
            Copy-Item -Path $sourceFile -Destination $destFile -Force
            Write-Host " OK" -ForegroundColor Green
            $successCount++
        } else {
            Write-Host " NOT FOUND" -ForegroundColor Red
            Write-Host "    Expected at: $sourceFile" -ForegroundColor Yellow
            $failCount++
        }
    }

    # Copy all SUMO runtime DLLs to bin subdirectory
    Write-Host "`nCopying all SUMO runtime DLLs to bin/..." -ForegroundColor Cyan
    $sumoBinDir = Join-Path $sumoDir "bin"

    if (Test-Path $sumoBinDir) {
        $allDlls = Get-ChildItem -Path $sumoBinDir -Filter "*.dll"
        $dllCount = 0

        foreach ($dll in $allDlls) {
            Copy-Item -Path $dll.FullName -Destination $destBinDir -Force
            $dllCount++
        }

        Write-Host "  Copied $dllCount DLL files to bin/" -ForegroundColor Green
    } else {
        Write-Host "  Warning: SUMO bin directory not found at $sumoBinDir" -ForegroundColor Yellow
    }

    # Copy header files from source
    Write-Host "`nCopying header files from SUMO source..." -ForegroundColor Cyan
    $sumoSrcLibsumoDir = Join-Path $sumoDir "src\libsumo"

    $headerFiles = @(
        "libsumo.h",
        "libtraci.h",
        "Edge.h",
        "GUI.h",
        "InductionLoop.h",
        "Junction.h",
        "LaneArea.h",
        "Lane.h",
        "MultiEntryExit.h",
        "POI.h",
        "Polygon.h",
        "Route.h",
        "Simulation.h",
        "TrafficLight.h",
        "VehicleType.h",
        "Vehicle.h",
        "Person.h",
        "Calibrator.h",
        "BusStop.h",
        "ParkingArea.h",
        "ChargingStation.h",
        "OverheadWire.h",
        "Rerouter.h",
        "MeanData.h",
        "VariableSpeedSign.h",
        "RouteProbe.h",
        "Helper.h",
        "StorageHelper.h",
        "Subscription.h",
        "TraCIConstants.h",
        "TraCIDefs.h"
    )

    foreach ($file in $headerFiles) {
        $sourceFile = Join-Path $sumoSrcLibsumoDir $file
        $destFile = Join-Path $destDir $file

        Write-Host "  Copying $file..." -NoNewline

        if (Test-Path $sourceFile) {
            Copy-Item -Path $sourceFile -Destination $destFile -Force
            Write-Host " OK" -ForegroundColor Green
            $successCount++
        } else {
            Write-Host " NOT FOUND" -ForegroundColor Red
            Write-Host "    Expected at: $sourceFile" -ForegroundColor Yellow
            $failCount++
        }
    }

    $totalFiles = $requiredFiles.Count + $headerFiles.Count
    Write-Host "`nCopy Summary:" -ForegroundColor Cyan
    Write-Host "  Success: $successCount / $totalFiles" -ForegroundColor Green
    Write-Host "  Failed: $failCount / $totalFiles" -ForegroundColor $(if ($failCount -eq 0) { "Green" } else { "Red" })

    if ($failCount -gt 0) {
        throw "Failed to copy all required files"
    }

    # Cleanup only the build artifacts, keep source clones for future builds
    Write-Host "`nCleaning up build artifacts (keeping source clones for reuse)..." -ForegroundColor Cyan
    if (Test-Path $buildDir) {
        Remove-Item $buildDir -Recurse -Force -ErrorAction Stop
        Write-Host "  Removed build directory" -ForegroundColor Green
    }

    if (-not $KeepBuildDir) {
        Write-Host "  SUMO source preserved at: $sumoDir" -ForegroundColor Green
        if (Test-Path $sumoLibrariesDir) {
            Write-Host "  SUMOLibraries preserved at: $sumoLibrariesDir" -ForegroundColor Green
        }
        Write-Host "`nNote: Source clones are kept in tmp/ for faster future builds" -ForegroundColor Cyan
        Write-Host "      To force re-clone, delete the tmp/ directory manually" -ForegroundColor Gray
    } else {
        Write-Host "`nBuild directory preserved at: $sumoBuildDir" -ForegroundColor Yellow
    }

    Write-Host "`n========================================" -ForegroundColor Green
    Write-Host "SUCCESS! Release and Debug libraries are ready." -ForegroundColor Green
    Write-Host "========================================" -ForegroundColor Green
    Write-Host "`nCopied import libraries (.lib) to bin/:" -ForegroundColor Cyan
    foreach ($file in $requiredLibs) {
        $destFile = Join-Path $destBinDir $file
        if (Test-Path $destFile) {
            $fileSize = (Get-Item $destFile).Length / 1MB
            Write-Host "  $file ($("{0:N2}" -f $fileSize) MB)" -ForegroundColor Green
        }
    }
    Write-Host "`nCopied runtime dependency DLLs to bin/:" -ForegroundColor Cyan
    if (Test-Path $destBinDir) {
        $dllFiles = Get-ChildItem -Path $destBinDir -Filter "*.dll"
        $totalDllSize = ($dllFiles | Measure-Object -Property Length -Sum).Sum / 1MB
        Write-Host "  $($dllFiles.Count) DLL files ($("{0:N2}" -f $totalDllSize) MB total)" -ForegroundColor Green
    }
    Write-Host "`nCopied headers:" -ForegroundColor Cyan
    foreach ($file in $headerFiles) {
        Write-Host "  $file" -ForegroundColor Green
    }
    Write-Host "`nNext step: Build TrafficLayer in Visual Studio (Release or Debug)" -ForegroundColor Yellow
    Write-Host "`nPress any key to exit..." -ForegroundColor Gray
    $null = $Host.UI.RawUI.ReadKey("NoEcho,IncludeKeyDown")

    exit 0

} catch {
    Write-Host "`n========================================" -ForegroundColor Red
    Write-Host "ERROR OCCURRED" -ForegroundColor Red
    Write-Host "========================================" -ForegroundColor Red
    Write-Host "`nError: $($_.Exception.Message)" -ForegroundColor Red
    Write-Host "`nStack trace:" -ForegroundColor Yellow
    Write-Host $_.ScriptStackTrace -ForegroundColor Gray

    # Cleanup on error (only remove build artifacts, keep source clones)
    Pop-Location -ErrorAction SilentlyContinue
    Pop-Location -ErrorAction SilentlyContinue

    if (Test-Path $buildDir) {
        Write-Host "`nCleaning up build artifacts..." -ForegroundColor Yellow
        Remove-Item $buildDir -Recurse -Force -ErrorAction SilentlyContinue
    }

    Write-Host "`nSource clones preserved at: $sumoBuildDir" -ForegroundColor Yellow
    Write-Host "You can inspect it for debugging or delete tmp/ to start fresh." -ForegroundColor Yellow

    Write-Host "`nPress any key to exit..." -ForegroundColor Gray
    $null = $Host.UI.RawUI.ReadKey("NoEcho,IncludeKeyDown")

    exit 1
}
