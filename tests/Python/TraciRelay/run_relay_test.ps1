# #356 end-to-end: SUMO + TrafficLayer + a FIXS client that also speaks TraCI.
#
#   powershell -ExecutionPolicy Bypass -File run_relay_test.ps1
#
# Headless and self-checking: exits 0 only if every assertion in relay_client.py
# passed. SUMO is started here (EnableAutoLaunch is false in config.yaml) so nothing
# opens a GUI and nothing is left running when the test ends.
param(
    [string]$Python = "$env:USERPROFILE\miniconda3\envs\realsim_dev\python.exe",
    [int]$Port = 1337
)
$ErrorActionPreference = 'Stop'
$here = $PSScriptRoot
$root = (Resolve-Path (Join-Path $here '..\..\..')).Path
$exe = Join-Path $root 'TrafficLayer\x64\Release\TrafficLayer.exe'
$cfg = Join-Path $root 'tests\Sumo\Probes\TraciRelay\net\probe.sumocfg'

if (-not (Test-Path $exe)) { throw "TrafficLayer.exe not found -- run scripts\dispatch\2_core_components.bat" }
if (-not (Test-Path $Python)) { $Python = 'python' }
$sumo = Join-Path $env:SUMO_HOME 'bin\sumo.exe'
if (-not (Test-Path $sumo)) { throw 'sumo.exe not found -- set SUMO_HOME' }

$sumoProc = $null
$tlProc = $null
try {
    Write-Host "`n=== starting SUMO (headless, port $Port) ===" -ForegroundColor Cyan
    $sumoProc = Start-Process -FilePath $sumo -PassThru -WindowStyle Hidden -ArgumentList @(
        '-c', $cfg, '--remote-port', $Port, '--num-clients', '1', '--step-length', '0.1', '--start')
    Start-Sleep -Milliseconds 800

    Write-Host "=== starting TrafficLayer ===" -ForegroundColor Cyan
    $tlLog = Join-Path $here 'trafficlayer.log'
    $tlProc = Start-Process -FilePath $exe -PassThru -WindowStyle Hidden `
        -WorkingDirectory $here -ArgumentList @('-f', (Join-Path $here 'config.yaml')) `
        -RedirectStandardOutput $tlLog -RedirectStandardError (Join-Path $here 'trafficlayer.err')
    Start-Sleep -Seconds 2

    Write-Host "=== running the client ===" -ForegroundColor Cyan
    & $Python (Join-Path $here 'relay_client.py')
    $code = $LASTEXITCODE
}
finally {
    foreach ($p in @($tlProc, $sumoProc)) {
        if ($p -and -not $p.HasExited) { $p.Kill(); $p.WaitForExit(2000) }
    }
}

if ($code -ne 0) {
    Write-Host "`nclient failed -- TrafficLayer output:" -ForegroundColor Yellow
    if (Test-Path (Join-Path $here 'trafficlayer.log')) {
        Get-Content (Join-Path $here 'trafficlayer.log') -Tail 30
    }
}
exit $code
