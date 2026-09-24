# ============================================================================
# Shared libsumo integrity check - dot-source this file, then call
# Test-LibsumoLoadable <binDir>.
#
# Why this exists: since #238 the SUMO runtime is no longer vendored in git, so
# the libsumo-<ver>-windows-x86_64.zip release asset is the ONLY source of it. A
# silently incomplete bin/ is therefore a repo-wide breakage, not a local
# annoyance.
#
# It stayed hidden for months before: CommonLib/libsumo/bin shipped without
# geos_c.dll + geos.dll (imported by gdal.dll, which libsumocpp.dll pulls in),
# and TrafficLayer delay-loads libsumocpp.dll - so the link succeeded and the
# failure only appeared at the first libsumo call, as a Win32 loader exception
# (0xC06D007E) that no C++ catch block can see. See #70 / #237.
#
# Presence checks alone would not have caught it (the four probed DLLs were all
# there); only an actual load walks the transitive import graph.
# ============================================================================

$script:LibsumoProbeDlls = @('libsumocpp.dll', 'libsumocppD.dll', 'libtracicpp.dll', 'libtracicppD.dll')

function Test-LibsumoLoadable {
    <#
    .SYNOPSIS
      Throw unless every probe DLL in $BinDir loads with its full dependency chain.
    .PARAMETER BinDir
      Path to CommonLib/libsumo/bin.
    .PARAMETER Context
      Short string used in the error message to say what should be done about it.
    #>
    param(
        [Parameter(Mandatory)] [string]$BinDir,
        [string]$Context = 'Repair CommonLib/libsumo/bin from the official SUMO Windows distribution.'
    )

    if (-not (Test-Path $BinDir)) { throw "libsumo bin/ not found: $BinDir. $Context" }

    $missing = $script:LibsumoProbeDlls | Where-Object { -not (Test-Path (Join-Path $BinDir $_)) }
    if ($missing) { throw "libsumo bin/ is missing: $($missing -join ', '). $Context" }

    # LoadLibraryEx + LOAD_WITH_ALTERED_SEARCH_PATH (0x8) so each DLL's own
    # directory is searched for its dependencies - the same way TrafficLayer
    # resolves them at runtime.
    if (-not ('Rs.LibsumoLoader' -as [type])) {
        Add-Type -Namespace Rs -Name LibsumoLoader -MemberDefinition @'
[System.Runtime.InteropServices.DllImport("kernel32", SetLastError=true, CharSet=System.Runtime.InteropServices.CharSet.Unicode)]
public static extern System.IntPtr LoadLibraryExW(string path, System.IntPtr h, uint flags);
[System.Runtime.InteropServices.DllImport("kernel32", SetLastError=true)]
public static extern bool FreeLibrary(System.IntPtr h);
'@
    }

    # Also prepend BinDir to PATH: a few of SUMO's bundled DLLs are pulled in by
    # dependencies rather than directly, and ALTERED_SEARCH_PATH only covers the
    # directory of the DLL being loaded.
    $bad = @()
    $savedPath = $env:PATH
    $env:PATH = "$BinDir;$env:PATH"
    try {
        foreach ($dll in $script:LibsumoProbeDlls) {
            $h = [Rs.LibsumoLoader]::LoadLibraryExW((Join-Path $BinDir $dll), [IntPtr]::Zero, 0x00000008)
            if ($h -eq [IntPtr]::Zero) {
                $err = [System.Runtime.InteropServices.Marshal]::GetLastWin32Error()
                $bad += "$dll (win32 error $err)"
            } else {
                # FreeLibrary matters: the packer probes its staging directory and
                # then deletes it. A leaked handle keeps the DLL mapped and the
                # cleanup fails with a sharing violation.
                [void][Rs.LibsumoLoader]::FreeLibrary($h)
            }
        }
    } finally {
        $env:PATH = $savedPath
    }
    if ($bad) {
        throw ("libsumo bin/ is incomplete - these did not load: {0}. " -f ($bad -join '; ')) +
              "A transitive dependency DLL is absent from bin/ (this is exactly the geos_c.dll/geos.dll case from #70). $Context"
    }

    Write-Host "  libsumo loadability check OK ($($script:LibsumoProbeDlls.Count)/$($script:LibsumoProbeDlls.Count) DLLs loaded)"
}
