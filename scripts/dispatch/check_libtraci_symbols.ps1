# ============================================================================
# #356: assert libtracicpp exports the three symbols the TraCI relay binds to.
#
# CommonLib/TraciRelay.cpp declares libtraci::Connection locally, because the real
# libtraci/Connection.h cannot be vendored -- it includes libsumo/Subscription.h,
# which includes utils/common/SUMOVehicleClass.h and utils/common/SUMOTime.h, neither
# of which ships. A local declaration links because MSVC mangles a member function
# from the namespace, class and signature alone.
#
# The bet that makes is version-sensitive, and it is not hypothetical: the SUMO
# 1.21.0 distribution does NOT export getActive() or getMutex() (they are inline in
# that build), while the 1.22.0 one does. So a SUMO bump can silently remove what the
# relay depends on. This check turns that into one explanatory failure before the
# build, instead of an unresolved external in the middle of it.
#
# Run by scripts/dispatch/2_core_components.bat. Exit 0 = fine (including "no libsumo
# fetched yet", which is not this script's business); exit 1 = the relay will not link.
# ============================================================================
param(
    [string]$RepoRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
)
$ErrorActionPreference = 'Stop'

$dll = Join-Path $RepoRoot 'CommonLib\libsumo\bin\libtracicpp.dll'
if (-not (Test-Path $dll)) {
    Write-Host "  libtracicpp.dll not present - skipping the #356 symbol check."
    exit 0
}

$vswhere = "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe"
if (-not (Test-Path $vswhere)) { Write-Host "  vswhere not found - skipping."; exit 0 }
$vs = & $vswhere -latest -property installationPath
$dumpbin = Get-ChildItem (Join-Path $vs 'VC\Tools\MSVC') -Recurse -Filter dumpbin.exe -ErrorAction SilentlyContinue |
           Where-Object { $_.FullName -match 'HostX64\\x64' } | Select-Object -First 1
if (-not $dumpbin) { Write-Host "  dumpbin not found - skipping."; exit 0 }

$exports = & $dumpbin.FullName /EXPORTS $dll

# Mangled exactly as CommonLib/TraciRelay.cpp's local declaration produces them.
$required = @{
    'Connection::doCommand(int, int, const std::string&, tcpip::Storage*, int)' =
        '?doCommand@Connection@libtraci@@QEAAAEAVStorage@tcpip@@HHAEBV?$basic_string@DU?$char_traits@D@std@@V?$allocator@D@2@@std@@PEAV34@H@Z'
    'Connection::getActive()' = '?getActive@Connection@libtraci@@SAAEAV12@XZ'
    'Connection::getMutex() const' = '?getMutex@Connection@libtraci@@QEBAAEAVmutex@std@@XZ'
}

$missing = @()
foreach ($name in $required.Keys) {
    if (-not ($exports | Select-String -SimpleMatch $required[$name] -Quiet)) {
        $missing += "$name`n      expected export: $($required[$name])"
    }
}

if ($missing.Count -gt 0) {
    Write-Host ''
    Write-Host 'ERROR (#356): libtracicpp.dll does not export what the TraCI relay needs.' -ForegroundColor Red
    foreach ($m in $missing) { Write-Host "    missing: $m" -ForegroundColor Red }
    Write-Host ''
    Write-Host '  The relay calls libtraci''s generic executor through a local declaration of'
    Write-Host '  libtraci::Connection (CommonLib/TraciRelay.cpp). A SUMO version whose build'
    Write-Host '  inlines or renames these cannot be relayed onto, and TrafficLayer would fail'
    Write-Host '  to link with an unresolved external instead of this message.'
    Write-Host ''
    Write-Host '  Fix: re-pack CommonLib/libsumo from a SUMO build that exports them (1.22.0'
    Write-Host '  does; the 1.21.0 distribution does not), or drop the relay for that version.'
    exit 1
}

Write-Host '  #356 TraCI relay symbols: all 3 present in libtracicpp.dll'
exit 0
