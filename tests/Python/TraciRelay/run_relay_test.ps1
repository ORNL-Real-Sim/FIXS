# Live check of the TraCI relay: headless SUMO, TrafficLayer, relay_client.py. Exit 0 only on PASS.
param(
    [string]$Python = "$env:USERPROFILE\miniconda3\envs\realsim_dev\python.exe",
    [int]$Port = 1337
)
$ErrorActionPreference = 'Stop'
$here = $PSScriptRoot
$root = (Resolve-Path (Join-Path $here '..\..\..')).Path
$exe  = Join-Path $root 'TrafficLayer\x64\Release\TrafficLayer.exe'
$cfg  = Join-Path $root 'tests\Sumo\Probes\TraciRelay\net\probe.sumocfg'
$sumo = Join-Path $env:SUMO_HOME 'bin\sumo.exe'
if (-not (Test-Path $exe))  { throw "TrafficLayer.exe not found - run scripts\dispatch\2_core_components.bat" }
if (-not (Test-Path $sumo)) { throw 'sumo.exe not found - set SUMO_HOME' }
if (-not (Test-Path $Python)) { $Python = 'python' }

$procs = @()
$code = 1
try {
    $procs += Start-Process -FilePath $sumo -PassThru -WindowStyle Hidden -ArgumentList @(
        '-c', $cfg, '--remote-port', $Port, '--num-clients', '1', '--step-length', '0.1', '--start')
    Start-Sleep -Milliseconds 800
    $procs += Start-Process -FilePath $exe -PassThru -WindowStyle Hidden -WorkingDirectory $here `
        -ArgumentList @('-f', (Join-Path $here 'config.yaml')) `
        -RedirectStandardOutput (Join-Path $here 'trafficlayer.log') -RedirectStandardError (Join-Path $here 'trafficlayer.err')
    Start-Sleep -Seconds 2
    & $Python (Join-Path $here 'relay_client.py')
    $code = $LASTEXITCODE
}
finally {
    foreach ($p in $procs) { if ($p -and -not $p.HasExited) { $p.Kill(); $p.WaitForExit(2000) } }
}
if ($code -ne 0 -and (Test-Path (Join-Path $here 'trafficlayer.log'))) {
    Write-Host "`nTrafficLayer output:" -ForegroundColor Yellow
    Get-Content (Join-Path $here 'trafficlayer.log') -Tail 25
}
exit $code
