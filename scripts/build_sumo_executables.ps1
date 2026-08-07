# ============================================================
# build_sumo_executables.ps1 - Standalone Utility Script
# ============================================================
# PURPOSE: Build SUMO executables (sumo.exe, sumo-gui.exe, etc.) from source.
#          Reads the SUMO version from dependencies.yaml, clones the SUMO
#          repository if needed, and builds the full SUMO application suite.
#
# NOTE: This script is NOT called by dispatch.bat. It is a standalone utility
#       used when you need to build or update the SUMO executables themselves
#       (as opposed to libsumo DLLs). Most developers will use pre-built SUMO
#       releases; this script is only needed when building SUMO from source.
#
# USAGE:
#   powershell -ExecutionPolicy Bypass -File scripts\build_sumo_executables.ps1
#   powershell -ExecutionPolicy Bypass -File scripts\build_sumo_executables.ps1 -DryRun
#   powershell -ExecutionPolicy Bypass -File scripts\build_sumo_executables.ps1 -Configuration Debug
#
# PREREQUISITES: Visual Studio 2022 with C++ workload, CMake, vcpkg
# ============================================================

# Script to build SUMO executables (sumo.exe, sumo-gui.exe, etc.) from source
# Reads version from dependencies.yaml, clones SUMO if needed, and builds executables

param(
    [switch]$KeepBuildDir = $false,
    [switch]$DryRun = $false,
    [string]$Generator = "Visual Studio 17 2022",
    [string]$VcpkgRoot = "C:\vcpkg",
    [switch]$UseSumoLibraries = $true,  # Use SUMOLibraries by default (faster)
    [string]$Configuration = "Release"   # Build configuration (Release or Debug)
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

Write-Host "Building SUMO executables for version $version ($Configuration)" -ForegroundColor Cyan

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
$templatesPath = Join-Path $sumoDir "tools\build_config\templates.py"
$templatesPatched = $false
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

        # Check if executables already exist in bin
        $binDir = Join-Path $sumoDir "bin"
        $requiredExes = @("sumo.exe", "sumo-gui.exe")
        $allExesExist = $true

        foreach ($exe in $requiredExes) {
            if (-not (Test-Path (Join-Path $binDir $exe))) {
                $allExesExist = $false
                break
            }
        }

        if ($allExesExist) {
            Write-Host "`nSUMO executables already exist in the build!" -ForegroundColor Green
            $response = Read-Host "What would you like to do? (C=copy existing/R=rebuild/N=re-clone/Q=quit)"

            if ($response -match '^[Cc]') {
                Write-Host "Will copy existing executables (fastest)" -ForegroundColor Green
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
            $response = Read-Host "Do you want to reuse existing SUMO source or re-clone? (Y=reuse/N=re-clone/Q=quit)"

            if ($response -match '^[Yy]') {
                Write-Host "Reusing existing source (faster build)" -ForegroundColor Green
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

        # Apply escape fix to SUMO templates generator (temporary for build)
        if (Test-Path $templatesPath) {
            $templatesContent = Get-Content $templatesPath -Raw
            if ($templatesContent -notmatch "formatted_lines\.append") {
                Write-Host "`nApplying local templates.py escape fix..." -ForegroundColor Cyan
                $fixedFunction = @'

def formatToolTemplate(templateStr):
    """
    @brief format python tool template
    """
    if not templateStr or templateStr[0] != "<":
        return '""'
    # replace all current directory values (src/netedit)
    templateStr = re.sub("(?<=value)(.*)(?=netedit)", "", templateStr)
    templateStr = templateStr.replace('netedit', '="')
    templateStr = templateStr.replace("\"", "\\\"")
    lines = templateStr.splitlines()
    formatted_lines = []
    for idx, line in enumerate(lines):
        prefix = '' if idx == 0 else '    '
        formatted_lines.append(f'{prefix}"{line}"')
    return "\n".join(formatted_lines)

'@
                $regexOptions = [System.Text.RegularExpressions.RegexOptions]::Singleline -bor [System.Text.RegularExpressions.RegexOptions]::Multiline
                $regex = New-Object System.Text.RegularExpressions.Regex('def formatToolTemplate\(templateStr\):.*?(?=^def generateTemplate\(app, appBin\):)', $regexOptions)
                $updatedContent = $regex.Replace($templatesContent, $fixedFunction)
                if ($templatesContent -ne $updatedContent) {
                    Set-Content -Path $templatesPath -Value $updatedContent -Encoding UTF8
                    $templatesPatched = $true
                    Write-Host "  Patched templates.py for build" -ForegroundColor Green
                } else {
                    Write-Host "  templates.py escape fix pattern not found (skipping)" -ForegroundColor Yellow
                }
            } else {
                Write-Host "  templates.py already contains escape fix" -ForegroundColor Gray
            }
        } else {
            Write-Host "  WARNING: templates.py not found; skipping escape fix" -ForegroundColor Yellow
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

        # Build SUMO executables
        Write-Host "`nBuilding SUMO executables (this may take 30-60 minutes)..." -ForegroundColor Cyan
        Write-Host "  Configuration: $Configuration" -ForegroundColor Gray
        Write-Host "  cmake --build . --config $Configuration" -ForegroundColor Gray
        cmake --build . --config $Configuration

        if ($LASTEXITCODE -ne 0) {
            throw "Building SUMO executables failed"
        }

        Write-Host "  Build complete" -ForegroundColor Green

        if ($templatesPatched -and (Test-Path $templatesPath)) {
            Write-Host "`nRestoring templates.py to upstream state..." -ForegroundColor Cyan
            Push-Location $sumoDir
            git checkout -- tools/build_config/templates.py | Out-Null
            if ($LASTEXITCODE -eq 0) {
                Write-Host "  Restored templates.py" -ForegroundColor Green
                $templatesPatched = $false
            } else {
                Write-Host "  WARNING: Failed to restore templates.py (please check manually)" -ForegroundColor Yellow
            }
            Pop-Location
        }

        Pop-Location
        Pop-Location
    } else {
        Write-Host "`nSkipping build, using existing executables" -ForegroundColor Green
    }

    # Display build summary and request confirmation before copying
    Write-Host "`n========================================" -ForegroundColor Cyan
    Write-Host "BUILD COMPLETE - READY TO COPY FILES" -ForegroundColor Cyan
    Write-Host "========================================" -ForegroundColor Cyan

    Write-Host "`nBuild Information:" -ForegroundColor Yellow
    Write-Host "  SUMO Version: $version" -ForegroundColor Green
    Write-Host "  Configuration: $Configuration" -ForegroundColor Green
    Write-Host "  Source Directory: $sumoDir" -ForegroundColor Gray

    # Detect generator/compiler used
    Write-Host "  CMake Generator: $Generator" -ForegroundColor Gray

    # Detect dependency method used
    if ($UseSumoLibraries -and (Test-Path $sumoLibrariesDir)) {
        Write-Host "  Dependencies: SUMOLibraries (pre-compiled)" -ForegroundColor Gray
    } elseif (Test-Path (Join-Path $VcpkgRoot "scripts\buildsystems\vcpkg.cmake")) {
        Write-Host "  Dependencies: vcpkg" -ForegroundColor Gray
    } else {
        Write-Host "  Dependencies: None (built without external deps)" -ForegroundColor Yellow
    }

    Write-Host "`nExecutables built:" -ForegroundColor Yellow
    $binDir = Join-Path $sumoDir "bin"
    if (Test-Path $binDir) {
        $exeFiles = Get-ChildItem -Path $binDir -Filter "*.exe"
        $totalExeCount = $exeFiles.Count
        $totalExeSize = ($exeFiles | Measure-Object -Property Length -Sum).Sum / 1MB
        Write-Host "  $totalExeCount executables ($("{0:N2}" -f $totalExeSize) MB total)" -ForegroundColor Gray

        # Show key executables
        $keyExes = @("sumo.exe", "sumo-gui.exe", "netconvert.exe", "netgenerate.exe", "polyconvert.exe")
        foreach ($exe in $keyExes) {
            $exePath = Join-Path $binDir $exe
            if (Test-Path $exePath) {
                $exeSize = (Get-Item $exePath).Length / 1MB
                Write-Host "    - $exe ($("{0:N2}" -f $exeSize) MB)" -ForegroundColor Green
            }
        }
    }

    Write-Host "`nDestination:" -ForegroundColor Yellow
    $destDir = Join-Path $PSScriptRoot "..\build\sumo"
    Write-Host "  $destDir" -ForegroundColor Gray

    if (Test-Path $destDir) {
        Write-Host "`n  WARNING: This will DELETE existing files in build\sumo" -ForegroundColor Red
    }

    Write-Host "`nDo you want to proceed with copying executables to the repository?" -ForegroundColor Cyan
    $response = Read-Host "(Y=yes and copy/N=no and exit)"

    if (-not ($response -match '^[Yy]')) {
        Write-Host "`nCopy cancelled by user. Executables remain in build directory:" -ForegroundColor Yellow
        Write-Host "  $binDir" -ForegroundColor Gray
        Write-Host "`nYou can run SUMO directly from:" -ForegroundColor Cyan
        Write-Host "  $binDir\sumo.exe" -ForegroundColor Gray
        Write-Host "  $binDir\sumo-gui.exe" -ForegroundColor Gray
        Write-Host "`nPress any key to exit..." -ForegroundColor Gray
        $null = $Host.UI.RawUI.ReadKey("NoEcho,IncludeKeyDown")
        exit 0
    }

    # User confirmed, proceed with copy
    Write-Host "`nProceeding with file copy..." -ForegroundColor Green

    # Clean up existing build/sumo directory to avoid stale files
    if (Test-Path $destDir) {
        Write-Host "  Cleaning up old files in build\sumo..." -ForegroundColor Yellow
        Remove-Item -Path $destDir -Recurse -Force -ErrorAction Stop
        Write-Host "  Old files removed" -ForegroundColor Green
    }

    # Create fresh directory
    New-Item -ItemType Directory -Path $destDir -Force | Out-Null
    Write-Host "  Created fresh build\sumo directory" -ForegroundColor Green

    Write-Host "`nCopying executables and DLLs..." -ForegroundColor Cyan

    # Copy all files from bin directory
    if (Test-Path $binDir) {
        $allFiles = Get-ChildItem -Path $binDir -File
        $copiedCount = 0

        foreach ($file in $allFiles) {
            Copy-Item -Path $file.FullName -Destination $destDir -Force
            $copiedCount++
        }

        Write-Host "  Copied $copiedCount files" -ForegroundColor Green
    } else {
        throw "SUMO bin directory not found at $binDir"
    }

    # Copy data directory (contains configuration files and schemas)
    $dataSourceDir = Join-Path $sumoDir "data"
    $dataDestDir = Join-Path $destDir "data"

    if (Test-Path $dataSourceDir) {
        Write-Host "`nCopying data directory (configs, schemas, etc.)..." -ForegroundColor Cyan
        Copy-Item -Path $dataSourceDir -Destination $dataDestDir -Recurse -Force
        Write-Host "  Data directory copied" -ForegroundColor Green
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
    Write-Host "SUCCESS! SUMO executables are ready." -ForegroundColor Green
    Write-Host "========================================" -ForegroundColor Green
    Write-Host "`nCopied to: $destDir" -ForegroundColor Cyan
    Write-Host "`nYou can now run:" -ForegroundColor Yellow
    Write-Host "  $destDir\sumo.exe" -ForegroundColor Green
    Write-Host "  $destDir\sumo-gui.exe" -ForegroundColor Green
    Write-Host "`nAdd to PATH to run from anywhere:" -ForegroundColor Cyan
    Write-Host "  `$env:PATH += `";$destDir`"" -ForegroundColor Gray
    Write-Host "`nPress any key to exit..." -ForegroundColor Gray
    $null = $Host.UI.RawUI.ReadKey("NoEcho,IncludeKeyDown")

    exit 0

} catch {
    if ($templatesPatched -and (Test-Path $templatesPath)) {
        Push-Location $sumoDir
        git checkout -- tools/build_config/templates.py | Out-Null
        $templatesPatched = $false
        Pop-Location
    }
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
