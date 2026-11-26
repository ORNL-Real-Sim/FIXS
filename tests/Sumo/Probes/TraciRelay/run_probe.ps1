# #356 probe driver. Three phases, two throwaway SUMO runs on the same
# deterministic net:
#   A  capture.py   - real traci, taps Connection._sendCmd, records request bytes
#                     and the response bytes traci sees
#   B  probe_relay  - libtraci, replays those request bytes through
#                     Connection::doCommand on the connection it already owns
#   C  replay.py    - stock traci parsers, fed the bytes phase B returned
param(
    [int]$PortA = 8823,
    [int]$PortB = 8824,
    [int]$Steps = 50,
    [string]$LibSumo = (Join-Path $PSScriptRoot "..\..\..\..\CommonLib\libsumo"),
    [string]$Python = "python"
)
$ErrorActionPreference = "Stop"
$here = $PSScriptRoot
$out = Join-Path $here "out"
New-Item -ItemType Directory -Force -Path $out | Out-Null
$sumo = Join-Path $env:SUMO_HOME "bin\sumo.exe"
if (-not (Test-Path $sumo)) { throw "sumo.exe not found - set SUMO_HOME" }
$cfg = Join-Path $here "net\probe.sumocfg"

function Start-Sumo([int]$port) {
    $p = Start-Process -FilePath $sumo -PassThru -WindowStyle Hidden `
        -ArgumentList @("-c", $cfg, "--remote-port", $port, "--num-clients", "1", "--start")
    Start-Sleep -Milliseconds 800
    return $p
}

Write-Host "`n=== phase A: capture (port $PortA) ===" -ForegroundColor Cyan
$pa = Start-Sumo $PortA
try { & $Python (Join-Path $here "capture.py") $PortA $Steps $out; if ($LASTEXITCODE) { throw "capture failed" } }
finally { if (-not $pa.HasExited) { $pa.Kill() } }

Write-Host "`n=== phase B: doCommand relay (port $PortB) ===" -ForegroundColor Cyan
$env:PATH = (Join-Path (Resolve-Path $LibSumo) "bin") + ";" + $env:PATH
$pb = Start-Sumo $PortB
try {
    & (Join-Path $here "probe_relay.exe") $PortB $Steps (Join-Path $out "requests.txt") (Join-Path $out "responses.txt")
    if ($LASTEXITCODE) { throw "probe_relay failed" }
} finally { if (-not $pb.HasExited) { $pb.Kill() } }

Write-Host "`n=== phase C: replay through stock traci parsers ===" -ForegroundColor Cyan
& $Python (Join-Path $here "replay.py") $out
exit $LASTEXITCODE
