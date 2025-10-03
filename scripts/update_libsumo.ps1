# Script to update libsumo files from SUMO GitHub repository
# Reads version from dependencies.yaml and downloads source files and DLLs

param(
    [switch]$DryRun = $false,
    [switch]$SkipDLLs = $false
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

Write-Host "Updating libsumo to SUMO version $version" -ForegroundColor Cyan

$tag = "v$($version.Replace('.', '_'))"
$baseUrl = "https://raw.githubusercontent.com/eclipse/sumo/$tag/src/libsumo"
$destDir = Join-Path $PSScriptRoot "..\CommonLib\libsumo"

# List of files to download (based on current libsumo directory)
$files = @(
    "BusStop.cpp", "BusStop.h",
    "Calibrator.cpp", "Calibrator.h",
    "ChargingStation.cpp", "ChargingStation.h",
    "Edge.cpp", "Edge.h",
    "GUI.cpp", "GUI.h",
    "Helper.cpp", "Helper.h",
    "InductionLoop.cpp", "InductionLoop.h",
    "Junction.cpp", "Junction.h",
    "Lane.cpp", "Lane.h",
    "LaneArea.cpp", "LaneArea.h",
    "libsumo.h",
    "libsumo.i",
    "libsumo_typemap.i",
    "libtraci.h",
    "MeanData.cpp", "MeanData.h",
    "MultiEntryExit.cpp", "MultiEntryExit.h",
    "OverheadWire.cpp", "OverheadWire.h",
    "ParkingArea.cpp", "ParkingArea.h",
    "Person.cpp", "Person.h",
    "POI.cpp", "POI.h",
    "Polygon.cpp", "Polygon.h",
    "Rerouter.cpp", "Rerouter.h",
    "Route.cpp", "Route.h",
    "RouteProbe.cpp", "RouteProbe.h",
    "Simulation.cpp", "Simulation.h",
    "StorageHelper.h",
    "Subscription.h",
    "TraCIConstants.h",
    "TraCIDefs.h",
    "TrafficLight.cpp", "TrafficLight.h",
    "VariableSpeedSign.cpp", "VariableSpeedSign.h",
    "Vehicle.cpp", "Vehicle.h",
    "VehicleType.cpp", "VehicleType.h"
)

$successCount = 0
$failCount = 0

foreach ($file in $files) {
    $url = "$baseUrl/$file"
    $outFile = Join-Path $destDir $file

    Write-Host "Downloading $file..." -NoNewline

    if ($DryRun) {
        Write-Host " [DRY RUN] Would download from $url" -ForegroundColor Yellow
        continue
    }

    try {
        Invoke-WebRequest -Uri $url -OutFile $outFile -ErrorAction Stop
        Write-Host " OK" -ForegroundColor Green
        $successCount++
    }
    catch {
        Write-Host " FAILED" -ForegroundColor Red
        Write-Host "  Error: $($_.Exception.Message)" -ForegroundColor Red
        $failCount++
    }
}

Write-Host "`nSource Files Summary:" -ForegroundColor Cyan
Write-Host "  Success: $successCount" -ForegroundColor Green
Write-Host "  Failed: $failCount" -ForegroundColor $(if ($failCount -eq 0) { "Green" } else { "Red" })

# Download DLLs from SUMO release
if (-not $SkipDLLs -and $failCount -eq 0) {
    Write-Host "`nDownloading SUMO Windows binaries (DLLs)..." -ForegroundColor Cyan

    $releaseVersion = $version
    $zipUrl = "https://sumo.dlr.de/releases/$releaseVersion/sumo-win64-$releaseVersion.zip"
    $tempZip = Join-Path $env:TEMP "sumo-$releaseVersion.zip"
    $tempExtract = Join-Path $env:TEMP "sumo-$releaseVersion-extract"

    Write-Host "Downloading SUMO release from $zipUrl..." -NoNewline

    if ($DryRun) {
        Write-Host " [DRY RUN]" -ForegroundColor Yellow
    } else {
        try {
            Invoke-WebRequest -Uri $zipUrl -OutFile $tempZip -ErrorAction Stop
            Write-Host " OK" -ForegroundColor Green

            Write-Host "Extracting archive..." -NoNewline
            Expand-Archive -Path $tempZip -DestinationPath $tempExtract -Force
            Write-Host " OK" -ForegroundColor Green

            # Find and copy DLLs and import libraries
            $binaryFiles = @(
                "libsumocpp.dll",
                "libsumocpp.lib",
                "libsumocppD.dll",
                "libsumocppD.lib",
                "libtracicpp.dll",
                "libtracicpp.lib",
                "libtracicppD.dll",
                "libtracicppD.lib"
            )

            $dllSuccessCount = 0
            $dllFailCount = 0

            foreach ($file in $binaryFiles) {
                Write-Host "Looking for $file..." -NoNewline

                # Search for file in extracted files (usually in bin/ directory)
                $foundFile = Get-ChildItem -Path $tempExtract -Filter $file -Recurse -ErrorAction SilentlyContinue | Select-Object -First 1

                if ($foundFile) {
                    $fileDest = Join-Path $destDir $file
                    Copy-Item -Path $foundFile.FullName -Destination $fileDest -Force
                    Write-Host " OK" -ForegroundColor Green
                    $dllSuccessCount++
                } else {
                    Write-Host " NOT FOUND (may not be in this release)" -ForegroundColor Yellow
                    $dllFailCount++
                }
            }

            # Cleanup
            Remove-Item $tempZip -Force -ErrorAction SilentlyContinue
            Remove-Item $tempExtract -Recurse -Force -ErrorAction SilentlyContinue

            Write-Host "`nBinary Files Summary:" -ForegroundColor Cyan
            Write-Host "  Found: $dllSuccessCount" -ForegroundColor Green
            Write-Host "  Not found: $dllFailCount" -ForegroundColor Yellow
            if ($dllFailCount -gt 0) {
                Write-Host "  Note: Debug libraries (*D.dll/*D.lib) are not in official releases" -ForegroundColor Yellow
            }

        }
        catch {
            Write-Host " FAILED" -ForegroundColor Red
            Write-Host "  Error: $($_.Exception.Message)" -ForegroundColor Red
            Write-Host "  Note: You may need to manually download DLLs from:" -ForegroundColor Yellow
            Write-Host "  https://sumo.dlr.de/releases/$releaseVersion/" -ForegroundColor Yellow
            $failCount++
        }
    }
}

if ($failCount -eq 0 -and -not $DryRun) {
    Write-Host "`nUpdate complete! Next steps:" -ForegroundColor Green
    Write-Host "  1. Rebuild the project to ensure compatibility" -ForegroundColor Yellow
    Write-Host "  2. Run tests to verify everything works" -ForegroundColor Yellow
} elseif ($SkipDLLs) {
    Write-Host "`nSource files updated. DLLs were skipped (use without -SkipDLLs to update them)" -ForegroundColor Yellow
}

exit $failCount
