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
        # Look for dSPACE ConfigurationDesk installations directly in Program Files
        $installations = Get-ChildItem $pf -Directory -ErrorAction SilentlyContinue |
            Where-Object { $_.Name -like "dSPACE ConfigurationDesk*" } |
            Sort-Object Name -Descending

        if ($installations.Count -gt 0) {
            return $installations[0].FullName
        }
    }

    return $null
}

function Find-CarMaker {
    param(
        [string]$RequestedVersion = ""
    )

    # Check common installation paths
    $searchPaths = @(
        "C:\IPG\carmaker",
        "${env:ProgramFiles}\IPG\carmaker",
        "${env:ProgramFiles(x86)}\IPG\carmaker"
    )

    $installations = @()

    foreach ($basePath in $searchPaths) {
        if (-not (Test-Path $basePath)) {
            continue
        }

        # Get all version directories (format: win64-X.Y.Z) that contain CM_Office.exe
        $versionDirs = Get-ChildItem $basePath -Directory -ErrorAction SilentlyContinue |
            Where-Object { $_.Name -match '^win64-(\d+\.\d+\.\d+)$' }

        foreach ($dir in $versionDirs) {
            $cmOfficeExe = Join-Path $dir.FullName "bin\CM_Office.exe"
            if (Test-Path $cmOfficeExe) {
                # Extract version number from directory name
                if ($dir.Name -match '^win64-(\d+)\.(\d+)\.(\d+)$') {
                    $installations += [PSCustomObject]@{
                        Path = $dir.FullName
                        Version = [version]"$($matches[1]).$($matches[2]).$($matches[3])"
                        DirName = $dir.Name
                    }
                }
            }
        }
    }

    if ($installations.Count -eq 0) {
        return $null
    }

    # Sort by version descending
    $installations = $installations | Sort-Object Version -Descending

    # If a specific version is requested, try to find the best match
    if ($RequestedVersion) {
        # Try exact match first
        $exactMatch = $installations | Where-Object { $_.Version -eq [version]$RequestedVersion } | Select-Object -First 1
        if ($exactMatch) {
            return $exactMatch.Path
        }

        # Try matching major.minor version (e.g., 13.1.3 matches 13.1.x)
        if ($RequestedVersion -match '^(\d+)\.(\d+)') {
            $majorMinor = "$($matches[1]).$($matches[2])"
            $majorMinorMatch = $installations | Where-Object { $_.Version.ToString() -like "$majorMinor.*" } | Select-Object -First 1
            if ($majorMinorMatch) {
                return $majorMinorMatch.Path
            }
        }

        # Try matching major version (e.g., 13.1.3 matches 13.x.x)
        if ($RequestedVersion -match '^(\d+)\.') {
            $major = $matches[1]
            $majorMatch = $installations | Where-Object { $_.Version.Major -eq [int]$major } | Select-Object -First 1
            if ($majorMatch) {
                return $majorMatch.Path
            }
        }
    }

    # Return latest version
    return $installations[0].Path
}

switch ($Tool.ToLower()) {
    "visual_studio" { $result = Find-VisualStudio }
    "matlab" { $result = Find-Matlab }
    "dspace" { $result = Find-DSpace }
    "carmaker" {
        $requestedVersion = ""
        if ($args.Count -gt 0 -and $null -ne $args[0]) {
            $requestedVersion = [string]$args[0]
        }
        $result = Find-CarMaker -RequestedVersion $requestedVersion
    }
    default { $result = $null }
}

if ($result) {
    Write-Output $result
}
