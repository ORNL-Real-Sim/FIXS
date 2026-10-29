# Fails if libtracicpp.dll does not export what CommonLib/TraciRelay.cpp declares locally.
# Exit 0 also when no libsumo is fetched yet; that is not this check's to report.
param(
    [string]$RepoRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
)
$ErrorActionPreference = 'Stop'

$dll = Join-Path $RepoRoot 'CommonLib\libsumo\bin\libtracicpp.dll'
if (-not (Test-Path $dll)) { Write-Host "  libtracicpp.dll not present - skipping the TraCI relay symbol check."; exit 0 }

$vswhere = "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe"
if (-not (Test-Path $vswhere)) { Write-Host "  vswhere not found - skipping."; exit 0 }
$vs = & $vswhere -latest -property installationPath
$dumpbin = Get-ChildItem (Join-Path $vs 'VC\Tools\MSVC') -Recurse -Filter dumpbin.exe -ErrorAction SilentlyContinue |
           Where-Object { $_.FullName -match 'HostX64\\x64' } | Select-Object -First 1
if (-not $dumpbin) { Write-Host "  dumpbin not found - skipping."; exit 0 }

$exports = & $dumpbin.FullName /EXPORTS $dll
# Mangled exactly as TraciRelay.cpp's declaration of libtraci::Connection produces them.
$required = [ordered]@{
    'Connection::getActive()' = '?getActive@Connection@libtraci@@SAAEAV12@XZ'
    'Connection::doCommand(int, int, const std::string&, tcpip::Storage*, int)' =
        '?doCommand@Connection@libtraci@@QEAAAEAVStorage@tcpip@@HHAEBV?$basic_string@DU?$char_traits@D@std@@V?$allocator@D@2@@std@@PEAV34@H@Z'
}
$missing = @($required.Keys | Where-Object { -not ($exports | Select-String -SimpleMatch $required[$_] -Quiet) })
if ($missing.Count) {
    Write-Host ''
    Write-Host 'ERROR: libtracicpp.dll does not export what the TraCI relay (CommonLib/TraciRelay.cpp) calls:' -ForegroundColor Red
    foreach ($m in $missing) { Write-Host "    $m  [$($required[$m])]" -ForegroundColor Red }
    Write-Host '  Re-pack CommonLib/libsumo from a SUMO build that exports them, or drop the relay for this version.'
    exit 1
}
Write-Host "  TraCI relay symbols present in libtracicpp.dll ($($required.Count) of $($required.Count))."
exit 0
