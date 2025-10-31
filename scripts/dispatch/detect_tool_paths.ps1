param(
    [Parameter(Mandatory=$true)][string]$Tool
)

function Find-VisualStudio {
    $vsWhere = "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe"

    if (Test-Path $vsWhere) {
        $installPath = & $vsWhere -latest -property installationPath 2>$null
        if ($installPath) {
            return $installPath
        }
    }

    # Fallback: Check common locations
    $commonPaths = @(
        "${env:ProgramFiles}\Microsoft Visual Studio\2022",
        "${env:ProgramFiles(x86)}\Microsoft Visual Studio\2022",
        "${env:ProgramFiles}\Microsoft Visual Studio\2019",
        "${env:ProgramFiles(x86)}\Microsoft Visual Studio\2019"
    )

    foreach ($path in $commonPaths) {
        if (Test-Path $path) {
            return $path
        }
    }

    return $null
}

function Find-Matlab {
    # Check registry first
    try {
        $regPaths = @(
            "HKLM:\SOFTWARE\MathWorks\MATLAB",
            "HKLM:\SOFTWARE\WOW6432Node\MathWorks\MATLAB"
        )

        foreach ($regPath in $regPaths) {
            if (Test-Path $regPath) {
                $versions = Get-ChildItem $regPath | Sort-Object -Descending
                if ($versions.Count -gt 0) {
                    $matlabRoot = (Get-ItemProperty $versions[0].PSPath).MATLABROOT
                    if ($matlabRoot -and (Test-Path $matlabRoot)) {
                        return $matlabRoot
                    }
                }
            }
        }
    } catch {}

    # Fallback: Check common installation directory
    $matlabBase = "C:\Program Files\MATLAB"
    if (Test-Path $matlabBase) {
        $versions = Get-ChildItem $matlabBase -Directory | Where-Object { $_.Name -match '^R\d{4}[ab]$' } | Sort-Object Name -Descending
        if ($versions.Count -gt 0) {
            return $versions[0].FullName
        }
    }

    return $null
}

function Find-DSpace {
    # Check common installation paths
    $programFiles = @($env:ProgramFiles, ${env:ProgramFiles(x86)})

    foreach ($pf in $programFiles) {
        $dspaceBase = Join-Path $pf "dSPACE"
        if (Test-Path $dspaceBase) {
            $installations = Get-ChildItem $dspaceBase -Directory | Where-Object { $_.Name -like "ConfigurationDesk*" } | Sort-Object Name -Descending
            if ($installations.Count -gt 0) {
                return $installations[0].FullName
            }
        }
    }

    return $null
}

switch ($Tool.ToLower()) {
    "visual_studio" { $result = Find-VisualStudio }
    "matlab" { $result = Find-Matlab }
    "dspace" { $result = Find-DSpace }
    default { $result = $null }
}

if ($result) {
    Write-Output $result
}
