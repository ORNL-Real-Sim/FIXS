# #356 probe build. Compiles probe_relay.cpp against the vendored libtracicpp.
#
# Two headers the relay needs are NOT in the shipped native-deps set
# (CommonLib/libsumo): foreign/tcpip/storage.h and its .cpp. They are fetched here
# from the tagged SUMO source, pinned to the version in dependencies.yaml. That
# fetch is the packaging prerequisite this probe is measuring - see FINDINGS.md.
param(
    [string]$LibSumo  = (Join-Path $PSScriptRoot "..\..\..\..\CommonLib\libsumo"),
    [string]$SumoTag  = "v1_22_0",
    [string]$Work     = (Join-Path $env:TEMP "fixs356_probe")
)
$ErrorActionPreference = "Stop"

if (-not (Test-Path (Join-Path $LibSumo "bin\libtracicpp.lib"))) {
    throw "libtracicpp.lib not found under $LibSumo - run scripts\initialize_fixs.ps1, or pass -LibSumo <path to a checkout that has it>"
}
$LibSumo = (Resolve-Path $LibSumo).Path

$inc = Join-Path $Work "inc"
New-Item -ItemType Directory -Force -Path (Join-Path $inc "foreign\tcpip"), (Join-Path $inc "libsumo") | Out-Null
"" | Out-File -Encoding ascii (Join-Path $inc "config.h")      # SUMO's build config; empty is enough here
Copy-Item (Join-Path $LibSumo "*.h") (Join-Path $inc "libsumo") -Force

$base = "https://raw.githubusercontent.com/eclipse-sumo/sumo/$SumoTag/src"
foreach ($f in @("foreign/tcpip/storage.h", "foreign/tcpip/storage.cpp")) {
    $dst = Join-Path $inc ($f -replace "/", "\")
    if (-not (Test-Path $dst)) {
        Write-Host "fetching $f ($SumoTag)"
        Invoke-WebRequest -UseBasicParsing "$base/$f" -OutFile $dst
    }
}

$vs = & "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe" -latest -property installationPath
$vcvars = Join-Path $vs "VC\Auxiliary\Build\vcvars64.bat"
$out = Join-Path $PSScriptRoot "probe_relay.exe"
$cmd = "`"$vcvars`" >nul && cd /d `"$Work`" && cl /nologo /std:c++17 /EHsc /O2 /MD /DNDEBUG " +
       "/I`"$inc`" /I`"$inc\libsumo`" " +
       "`"$PSScriptRoot\probe_relay.cpp`" `"$inc\foreign\tcpip\storage.cpp`" " +
       "/Fe:`"$out`" " +
       "/link `"$LibSumo\bin\libtracicpp.lib`""
Write-Host "building..."
cmd /c $cmd
if ($LASTEXITCODE -ne 0) { throw "build failed ($LASTEXITCODE)" }
Write-Host "built $out"
