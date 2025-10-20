#!/usr/bin/env pwsh
# Download large simulation assets from OneDrive

$ASSETS = @{
    "tests/Applications/SUMO_CARLA_EcoDriving/MLK_Carla_Scenario/CARLAFiles/MLK_noped1002_final_debug.fbx" = "https://outlookuga-my.sharepoint.com/:u:/g/personal/ys04893_uga_edu/EU8vuZV8DvtOlJ6HhpL7Z8MBkhCtVcloma0qxJXXMsPP7A?e=0KgETK&download=1"
}

$repoRoot = Split-Path -Parent $PSScriptRoot
Set-Location $repoRoot

foreach ($file in $ASSETS.Keys) {
    $dest = Join-Path $repoRoot $file

    if (Test-Path $dest) {
        $sizeMB = [math]::Round((Get-Item $dest).Length / 1MB, 2)
        Write-Host "File $file already exists ($sizeMB MB)" -ForegroundColor Yellow
        $response = Read-Host "Redownload? (y/N)"
        if ($response -notmatch '^[yY]') {
            Write-Host "Skipped $file" -ForegroundColor Green
            continue
        }
    }

    Write-Host "Downloading $file..." -ForegroundColor Yellow
    $destDir = Split-Path -Parent $dest
    New-Item -ItemType Directory -Path $destDir -Force | Out-Null

    try {
        $ProgressPreference = 'SilentlyContinue'
        Invoke-WebRequest -Uri $ASSETS[$file] -OutFile $dest
        $ProgressPreference = 'Continue'

        $sizeMB = [math]::Round((Get-Item $dest).Length / 1MB, 2)
        Write-Host "OK Downloaded $file ($sizeMB MB)" -ForegroundColor Green
    }
    catch {
        Write-Host "`nERROR Failed to download $file" -ForegroundColor Red
        Write-Host $_.Exception.Message -ForegroundColor Red
        Write-Host "`nTroubleshooting:" -ForegroundColor Yellow
        Write-Host "1. Make sure you have access to the OneDrive link" -ForegroundColor Yellow
        Write-Host "2. Check your internet connection" -ForegroundColor Yellow
        Write-Host "3. Try downloading manually from: $($ASSETS[$file])" -ForegroundColor Yellow
        Read-Host "`nPress Enter to exit"
        exit 1
    }
}

Write-Host "`nAll assets ready!" -ForegroundColor Green
Read-Host "`nPress Enter to exit"
