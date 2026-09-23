# ============================================================================
# Is <Dir> a complete, runnable FIXS install? (Windows)  #273
#
# Run by release.yml against what update_fixs.ps1 produced - once from the zip it
# just packed (before publishing) and once from the published release (after).
# scripts/check_fixs_install.sh is the Linux counterpart; change them together.
#
#   check_fixs_install.ps1 -Dir C:\tmp\t\FIXS -Tag v0.9.1-alpha
#
# Exit 0 = complete. Exit 1 = says what is missing.
# ============================================================================
param(
    [Parameter(Mandatory = $true)] [string]$Dir,
    [Parameter(Mandatory = $true)] [string]$Tag
)
$ErrorActionPreference = 'Stop'
$bad = @()

if (-not (Test-Path (Join-Path $Dir 'TrafficLayer.exe'))) { $bad += 'TrafficLayer.exe is missing' }

# A load, not a presence check: the vendored bin/ once lacked geos_c.dll for
# months with every probed DLL present (#70). libsumo_verify.ps1 walks the
# import graph the way TrafficLayer does at run time.
. (Join-Path $PSScriptRoot 'dispatch\libsumo_verify.ps1')
try {
    Test-LibsumoLoadable -BinDir (Join-Path $Dir 'CommonLib\libsumo\bin') -Context 'The installed native runtime is incomplete.'
} catch { $bad += "$_" }

# Line 1 is '<tag>' for a pinned release and '<tag> (<published>)' for a rolling
# one. run_cosim.py reads it byte-for-byte, where a BOM becomes part of the tag.
$vf = Join-Path $Dir 'FIXS_VERSION.txt'
if (-not (Test-Path $vf)) {
    $bad += 'FIXS_VERSION.txt is missing'
} else {
    $bytes = [System.IO.File]::ReadAllBytes($vf)
    if ($bytes.Length -ge 3 -and $bytes[0] -eq 0xEF -and $bytes[1] -eq 0xBB -and $bytes[2] -eq 0xBF) {
        $bad += 'FIXS_VERSION.txt starts with a UTF-8 BOM'
    }
    $line1 = ([System.IO.File]::ReadAllLines($vf) | Select-Object -First 1)
    if ($line1 -ne $Tag -and -not $line1.StartsWith("$Tag (")) {
        $bad += "FIXS_VERSION.txt line 1 is '$line1', expected '$Tag'"
    }
}

if ($bad) {
    $bad | ForEach-Object { Write-Host "[FAIL] $_" }
    exit 1
}
Write-Host "Install in $Dir is complete ($Tag)."
exit 0
