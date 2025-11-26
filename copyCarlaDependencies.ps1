param(
    [Parameter(Mandatory = $true, Position = 0)]
    [string]$CarlaRoot
)

$ErrorActionPreference = "Stop"

function Show-Usage {
    Write-Host "Usage: .\copy_carla_dependencies.ps1 <CARLA_ROOT>"
    Write-Host "Copies CARLA's PythonAPI dependency bundle into CommonLib\libcarla."
    Write-Host "  Source: <CARLA_ROOT>\PythonAPI\carla\dependencies"
    Write-Host "  Dest  : <FIXS_ROOT>\CommonLib\libcarla"
}

if ($CarlaRoot -eq "-h" -or $CarlaRoot -eq "--help") {
    Show-Usage
    exit 0
}

try {
    $CarlaRoot = (Resolve-Path -LiteralPath $CarlaRoot).Path
} catch {
    Write-Error "Invalid CARLA_ROOT path: $CarlaRoot"
    exit 1
}

$src = Join-Path $CarlaRoot "PythonAPI\carla\dependencies"
$repoRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
$dst = Join-Path $repoRoot "CommonLib\libcarla"

if (-not (Test-Path -LiteralPath $src)) {
    Write-Error "Dependencies directory not found at: $src"
    exit 2
}

if (Test-Path -LiteralPath $dst) {
    Write-Error "Destination already exists: $dst"
    Write-Error "Remove it first if you want to replace it."
    exit 3
}

Write-Host "Copying CARLA dependencies..."
Write-Host "  from: $src"
Write-Host "  to  : $dst"

Copy-Item -LiteralPath $src -Destination $dst -Recurse

Write-Host "Done. CARLA dependencies copied to $dst"
