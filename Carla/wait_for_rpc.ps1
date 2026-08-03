<#
  Poll a CARLA RPC endpoint until it accepts TCP connections (or timeout).
  Generalized from the inline :waitloop in the ANL run_cosim_roosevelt_full.bat.

  Usage:
    powershell -NoProfile -ExecutionPolicy Bypass -File wait_for_rpc.ps1 [-CarlaHost 127.0.0.1] [-Port 2000] [-TimeoutSec 180]

  Exit code 0 when the port is up, 1 on timeout -- so a .bat can gate on it:
    powershell ... -File wait_for_rpc.ps1 -Port 2000 || ( echo CARLA never came up & exit /b 1 )
#>
param(
    [string]$CarlaHost = "127.0.0.1",
    [int]$Port = 2000,
    [int]$TimeoutSec = 180
)

$deadline = (Get-Date).AddSeconds($TimeoutSec)
Write-Host "Waiting for CARLA RPC on ${CarlaHost}:${Port} (timeout ${TimeoutSec}s)..."
while ((Get-Date) -lt $deadline) {
    try {
        $client = New-Object System.Net.Sockets.TcpClient
        $iar = $client.BeginConnect($CarlaHost, $Port, $null, $null)
        if ($iar.AsyncWaitHandle.WaitOne(2000) -and $client.Connected) {
            $client.Close()
            Write-Host "CARLA RPC is up."
            exit 0
        }
        $client.Close()
    } catch { }
    Start-Sleep -Seconds 3
}
Write-Error "Timed out waiting for CARLA RPC on ${CarlaHost}:${Port}."
exit 1
