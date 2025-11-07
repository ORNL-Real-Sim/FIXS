<#
.SYNOPSIS
    RealSim User.c Patcher - PowerShell Edition

.DESCRIPTION
    Automatically patches CarMaker User.c files with RealSim integration code.
    Supports both GUI file selection and command-line automation.

.PARAMETER FilePath
    Path(s) to User.c file(s) to patch. Can specify multiple files.

.PARAMETER DryRun
    Preview changes without modifying files.

.PARAMETER NoBackup
    Skip backup creation (not recommended).

.EXAMPLE
    # Interactive GUI mode
    .\patch_user_c.ps1

.EXAMPLE
    # Patch specific file
    .\patch_user_c.ps1 -FilePath "..\CM13_proj\src\User.c"

.EXAMPLE
    # Dry run (preview changes)
    .\patch_user_c.ps1 -FilePath "User.c" -DryRun

.EXAMPLE
    # Patch multiple files
    .\patch_user_c.ps1 -FilePath "User1.c","User2.c"

.EXAMPLE
    # Patch without backup
    .\patch_user_c.ps1 -FilePath "User.c" -NoBackup
#>

[CmdletBinding()]
param(
    [Parameter(ValueFromPipeline=$true)]
    [string[]]$FilePath,

    [switch]$DryRun,

    [switch]$NoBackup
)

# =============================================================================
# Code Snippets to Insert
# =============================================================================

$HEADER_CODE = @'
#define REALSIM

#ifdef REALSIM
// ===========================================================================
// 			 RealSim
// ===========================================================================
#include "VirEnv_Wrapper.h"

struct VirEnvHelper* VirEnv_c;

char* RS_configFile;
char* RS_signalTable;
// ===========================================================================
// ===========================================================================
#endif

'@

$SCAN_CMDLINE_REALSIM_SECTION = @'
    // ===========================================================================
    // 			 RealSim
    // ===========================================================================
    else if (strcmp(*argv, "-f") == 0) {
		RS_configFile = *++argv;
	}
	else if (strcmp(*argv, "-s") == 0) {
		RS_signalTable = *++argv;
	}
    // ===========================================================================
    // ===========================================================================
'@

$USER_INIT_FIRST_REALSIM_CODE = @'
    // ===========================================================================
    // 			 RealSim
    // ===========================================================================
    #ifdef REALSIM
        if (VirEnv_c == NULL) {
            VirEnv_c = newVirEnvHelper();
            if (VirEnv_c == NULL) {
                LogErrF(EC_General, "Failed to create VirEnvHelper instance");
                return -1;
            }
        }
    #endif
    // ===========================================================================
    // ===========================================================================

'@

$USER_INIT_REALSIM_CODE = @'
#ifdef REALSIM
    // ===========================================================================
    // 			 RealSim
    // ===========================================================================
    VirEnv_c = newVirEnvHelper();
    // ===========================================================================
    // ===========================================================================
#endif

'@

$TESTRUN_START_REALSIM_CODE = @'
#ifdef REALSIM
    // ===========================================================================
    // 			 RealSim
    // ===========================================================================
    if (VirEnv_isVeryFirstStep && SimCore.State >= SCState_Start) {
        VirEnv_initialization(VirEnv_c, RS_configFile, RS_signalTable);
    }
    // ===========================================================================
    // ===========================================================================
#endif

'@

$USER_CALC_REALSIM_CODE = @'
#ifdef REALSIM
    // ===========================================================================
    // 			 RealSim
    // ===========================================================================
    if (SimCore.State != SCState_Simulate) {
        return 0;
    }

    VirEnv_runStep(VirEnv_c, SimCore.Time);

    // ===========================================================================
    // ===========================================================================
#endif

'@

$TESTRUN_END_REALSIM_CODE = @'
#ifdef REALSIM
	// ===========================================================================
    // 			 RealSim
    // ===========================================================================
	VirEnv_shutdown(VirEnv_c);
    // ===========================================================================
    // ===========================================================================
#endif
'@

# =============================================================================
# Helper Functions
# =============================================================================

function Test-AlreadyPatched {
    param([string]$Content)

    return ($Content -match '#define REALSIM') -or ($Content -match 'VirEnv_Wrapper\.h')
}

function Get-FileDialog {
    <#
    .SYNOPSIS
        Opens Windows file picker dialog for User.c selection
    #>
    try {
        Add-Type -AssemblyName System.Windows.Forms -ErrorAction Stop
    }
    catch {
        Write-Error "Failed to load Windows Forms. Error: $_"
        return $null
    }

    try {
        $FileBrowser = New-Object System.Windows.Forms.OpenFileDialog
        $FileBrowser.Filter = "C Source Files (User.c)|User.c|All C Files (*.c)|*.c|All files (*.*)|*.*"
        $FileBrowser.Title = "Select User.c file to patch"
        $FileBrowser.InitialDirectory = (Get-Location).Path
        $FileBrowser.Multiselect = $true

        $result = $FileBrowser.ShowDialog()

        if ($result -eq [System.Windows.Forms.DialogResult]::OK) {
            return $FileBrowser.FileNames
        }

        return $null
    }
    catch {
        Write-Error "Failed to open file dialog. Error: $_"
        return $null
    }
}

function Patch-HeaderSection {
    param([string]$Content)

    # Check if already patched
    if ($Content -match '#define REALSIM' -and $Content -match 'VirEnv_Wrapper\.h') {
        return @{Content=$Content; Success=$true; AlreadyPresent=$true}
    }

    # Find position before #include <Global.h>
    if ($Content -match '(?s)(.*?)#include\s+<Global\.h>') {
        $beforeGlobal = $Matches[1]
        $afterGlobal = $Content.Substring($Matches[0].Length)

        # Check for windows.h warning
        if ($beforeGlobal -match 'windows\.h') {
            Write-Warning "  windows.h found before Global.h - manual review recommended"
        }

        $newContent = $beforeGlobal + $HEADER_CODE + "#include <Global.h>" + $afterGlobal
        return @{Content=$newContent; Success=$true; AlreadyPresent=$false}
    }

    Write-Warning "  Could not find '#include <Global.h>' - skipping header patch"
    return @{Content=$Content; Success=$false; AlreadyPresent=$false}
}

function Patch-UserScanCmdLine {
    param([string]$Content)

    # Check if already patched
    if ($Content -match 'RS_configFile = \*\+\+argv' -or $Content -match 'strcmp\(\*argv, "-f"\)') {
        return @{Content=$Content; Success=$true; AlreadyPresent=$true}
    }

    # Find the else if block just before else if ((*argv)[0] == '-')
    # Look for help section as insertion point
    $pattern = '(?s)(User_PrintUsage\(Pgm\);.*?return\s+NULL;\s*\})'

    if ($Content -match $pattern) {
        $matchEnd = $Matches[0].Length + $Content.IndexOf($Matches[0])
        $newContent = $Content.Substring(0, $matchEnd) + "`n" + $SCAN_CMDLINE_REALSIM_SECTION + "`n" + $Content.Substring($matchEnd)
        return @{Content=$newContent; Success=$true; AlreadyPresent=$false}
    }

    Write-Warning "  Could not find insertion point in User_ScanCmdLine"
    return @{Content=$Content; Success=$false; AlreadyPresent=$false}
}

function Patch-UserInitFirst {
    param([string]$Content)

    # Check if already patched
    if ($Content -match 'User_Init_First.*?VirEnv_c\s*==\s*NULL') {
        return @{Content=$Content; Success=$true; AlreadyPresent=$true}
    }

    # Find User_Init_First function and insert before return statement
    $pattern = '(?s)(int\s+User_Init_First\s*\(void\)\s*\{.*?)(return\s+0;\s*\})'

    if ($Content -match $pattern) {
        $beforeReturn = $Matches[1]
        $returnStatement = $Matches[2]
        $matchStart = $Content.IndexOf($Matches[0])
        $matchEnd = $matchStart + $Matches[0].Length

        $newContent = $Content.Substring(0, $matchStart) + $beforeReturn + $USER_INIT_FIRST_REALSIM_CODE + "    " + $returnStatement + $Content.Substring($matchEnd)
        return @{Content=$newContent; Success=$true; AlreadyPresent=$false}
    }

    Write-Warning "  Could not find User_Init_First function"
    return @{Content=$Content; Success=$false; AlreadyPresent=$false}
}

function Patch-UserInit {
    param([string]$Content)

    # Check if already patched
    if ($Content -match 'User_Init\s*\(void\).*?newVirEnvHelper\(\)') {
        return @{Content=$Content; Success=$true; AlreadyPresent=$true}
    }

    # Find User_Init function and insert at the beginning
    $pattern = '(?s)(int\s+User_Init\s*\(void\)\s*\{)'

    if ($Content -match $pattern) {
        $matchEnd = $Matches[0].Length + $Content.IndexOf($Matches[0])
        $newContent = $Content.Substring(0, $matchEnd) + "`n" + $USER_INIT_REALSIM_CODE + $Content.Substring($matchEnd)
        return @{Content=$newContent; Success=$true; AlreadyPresent=$false}
    }

    Write-Warning "  Could not find User_Init function"
    return @{Content=$Content; Success=$false; AlreadyPresent=$false}
}

function Patch-UserTestRunStartAtEnd {
    param([string]$Content)

    # Check if already patched
    if ($Content -match 'User_TestRun_Start_atEnd.*?VirEnv_initialization') {
        return @{Content=$Content; Success=$true; AlreadyPresent=$true}
    }

    # Find the function and insert before return statement
    $pattern = '(?s)(int\s+User_TestRun_Start_atEnd\s*\(void\)\s*\{.*?)(return\s+0;\s*\})'

    if ($Content -match $pattern) {
        $beforeReturn = $Matches[1]
        $returnStatement = $Matches[2]
        $matchStart = $Content.IndexOf($Matches[0])
        $matchEnd = $matchStart + $Matches[0].Length

        $newContent = $Content.Substring(0, $matchStart) + $beforeReturn + "`n" + $TESTRUN_START_REALSIM_CODE + "    " + $returnStatement + $Content.Substring($matchEnd)
        return @{Content=$newContent; Success=$true; AlreadyPresent=$false}
    }

    Write-Warning "  Could not find User_TestRun_Start_atEnd function"
    return @{Content=$Content; Success=$false; AlreadyPresent=$false}
}

function Patch-UserCalc {
    param([string]$Content)

    # Check if already patched
    if ($Content -match 'User_Calc.*?VirEnv_runStep') {
        return @{Content=$Content; Success=$true; AlreadyPresent=$true}
    }

    # Find User_Calc function and insert after the commented warning
    $pattern = '(?s)(int\s+User_Calc\s*\(double\s+dt\)\s*\{[^\}]*?/\*if \(!UserCalcCalledByAppTestRunCalc\) return 0;\*/)'

    if ($Content -match $pattern) {
        $matchEnd = $Matches[0].Length + $Content.IndexOf($Matches[0])
        $newContent = $Content.Substring(0, $matchEnd) + "`n`n" + $USER_CALC_REALSIM_CODE + $Content.Substring($matchEnd)
        return @{Content=$newContent; Success=$true; AlreadyPresent=$false}
    }

    Write-Warning "  Could not find User_Calc function"
    return @{Content=$Content; Success=$false; AlreadyPresent=$false}
}

function Patch-UserTestRunEnd {
    param([string]$Content)

    # Check if already patched
    if ($Content -match 'User_TestRun_End.*?VirEnv_shutdown') {
        return @{Content=$Content; Success=$true; AlreadyPresent=$true}
    }

    # Find the function and insert before return statement
    $pattern = '(?s)(int\s+User_TestRun_End\s*\(void\)\s*\{.*?)(return\s+0;\s*\})'

    if ($Content -match $pattern) {
        $beforeReturn = $Matches[1]
        $returnStatement = $Matches[2]
        $matchStart = $Content.IndexOf($Matches[0])
        $matchEnd = $matchStart + $Matches[0].Length

        $newContent = $Content.Substring(0, $matchStart) + $beforeReturn + $TESTRUN_END_REALSIM_CODE + "`n`n    " + $returnStatement + $Content.Substring($matchEnd)
        return @{Content=$newContent; Success=$true; AlreadyPresent=$false}
    }

    Write-Warning "  Could not find User_TestRun_End function"
    return @{Content=$Content; Success=$false; AlreadyPresent=$false}
}

function Invoke-PatchUserC {
    <#
    .SYNOPSIS
        Main patching function for User.c file
    #>
    param(
        [string]$Path,
        [switch]$DryRun,
        [switch]$CreateBackup
    )

    # Validate file
    if (-not (Test-Path $Path)) {
        return @{
            Success = $false
            Message = "File not found: $Path"
        }
    }

    $file = Get-Item $Path
    if ($file.Name -ne 'User.c') {
        return @{
            Success = $false
            Message = "File must be named 'User.c', got: $($file.Name)"
        }
    }

    Write-Host "`nProcessing: $Path" -ForegroundColor Cyan

    # Read file
    try {
        $content = Get-Content -Path $Path -Raw -Encoding UTF8
    }
    catch {
        return @{
            Success = $false
            Message = "Error reading file: $_"
        }
    }

    # Apply patches and track results
    $patchesApplied = @()
    $patchesAlreadyPresent = @()
    $patchesFailed = @()

    $result = Patch-HeaderSection -Content $content
    $content = $result.Content
    if ($result.Success -and $result.AlreadyPresent) {
        $patchesAlreadyPresent += "Header section"
    } elseif ($result.Success) {
        $patchesApplied += "Header section"
    } else {
        $patchesFailed += "Header section"
    }

    $result = Patch-UserScanCmdLine -Content $content
    $content = $result.Content
    if ($result.Success -and $result.AlreadyPresent) {
        $patchesAlreadyPresent += "User_ScanCmdLine"
    } elseif ($result.Success) {
        $patchesApplied += "User_ScanCmdLine"
    } else {
        $patchesFailed += "User_ScanCmdLine"
    }

    $result = Patch-UserInitFirst -Content $content
    $content = $result.Content
    if ($result.Success -and $result.AlreadyPresent) {
        $patchesAlreadyPresent += "User_Init_First"
    } elseif ($result.Success) {
        $patchesApplied += "User_Init_First"
    } else {
        $patchesFailed += "User_Init_First"
    }

    $result = Patch-UserInit -Content $content
    $content = $result.Content
    if ($result.Success -and $result.AlreadyPresent) {
        $patchesAlreadyPresent += "User_Init"
    } elseif ($result.Success) {
        $patchesApplied += "User_Init"
    } else {
        $patchesFailed += "User_Init"
    }

    $result = Patch-UserTestRunStartAtEnd -Content $content
    $content = $result.Content
    if ($result.Success -and $result.AlreadyPresent) {
        $patchesAlreadyPresent += "User_TestRun_Start_atEnd"
    } elseif ($result.Success) {
        $patchesApplied += "User_TestRun_Start_atEnd"
    } else {
        $patchesFailed += "User_TestRun_Start_atEnd"
    }

    $result = Patch-UserCalc -Content $content
    $content = $result.Content
    if ($result.Success -and $result.AlreadyPresent) {
        $patchesAlreadyPresent += "User_Calc"
    } elseif ($result.Success) {
        $patchesApplied += "User_Calc"
    } else {
        $patchesFailed += "User_Calc"
    }

    $result = Patch-UserTestRunEnd -Content $content
    $content = $result.Content
    if ($result.Success -and $result.AlreadyPresent) {
        $patchesAlreadyPresent += "User_TestRun_End"
    } elseif ($result.Success) {
        $patchesApplied += "User_TestRun_End"
    } else {
        $patchesFailed += "User_TestRun_End"
    }

    # Build summary message
    $messageParts = @()
    if ($patchesApplied.Count -gt 0) {
        $messageParts += "$($patchesApplied.Count) new patches applied"
    }
    if ($patchesAlreadyPresent.Count -gt 0) {
        $messageParts += "$($patchesAlreadyPresent.Count) already present"
    }
    if ($patchesFailed.Count -gt 0) {
        $messageParts += "$($patchesFailed.Count) failed"
    }
    $summaryMessage = $messageParts -join ", "

    # Determine if this is a success or failure
    $totalExpected = 7
    $allPatchesPresent = ($patchesApplied.Count + $patchesAlreadyPresent.Count) -eq $totalExpected

    if ($patchesApplied.Count -eq 0 -and $patchesAlreadyPresent.Count -eq 0) {
        return @{
            Success = $false
            Message = "No patches could be applied"
        }
    }

    if (-not $allPatchesPresent -and $patchesFailed.Count -gt 0) {
        Write-Warning "  Some patches failed: $($patchesFailed -join ', ')"
    }

    # Dry run - just report
    if ($DryRun) {
        if ($patchesApplied.Count -gt 0) {
            Write-Host "  [DRY RUN] Would apply: $($patchesApplied -join ', ')" -ForegroundColor Yellow
        }
        if ($patchesAlreadyPresent.Count -gt 0) {
            Write-Host "  [DRY RUN] Already present: $($patchesAlreadyPresent -join ', ')" -ForegroundColor Cyan
        }
        return @{
            Success = $true
            Message = "Dry run: $summaryMessage"
        }
    }

    # Only write file if there are new patches to apply
    if ($patchesApplied.Count -eq 0) {
        Write-Host "  [OK] All patches already present" -ForegroundColor Green
        return @{
            Success = $true
            Message = $summaryMessage
        }
    }

    # Create backup
    if ($CreateBackup) {
        $timestamp = Get-Date -Format "yyyyMMdd_HHmmss"
        $backupPath = "$Path.bak.$timestamp"
        try {
            Copy-Item -Path $Path -Destination $backupPath -Force
            Write-Host "  Created backup: $(Split-Path $backupPath -Leaf)" -ForegroundColor Green
        }
        catch {
            return @{
                Success = $false
                Message = "Error creating backup: $_"
            }
        }
    }

    # Write patched file
    try {
        # Use UTF8 without BOM and Unix line endings (LF)
        $utf8NoBom = New-Object System.Text.UTF8Encoding $false
        [System.IO.File]::WriteAllText($Path, $content, $utf8NoBom)

        Write-Host "  [OK] Applied: $($patchesApplied -join ', ')" -ForegroundColor Green
        if ($patchesAlreadyPresent.Count -gt 0) {
            Write-Host "  [OK] Already had: $($patchesAlreadyPresent -join ', ')" -ForegroundColor Cyan
        }
        return @{
            Success = $true
            Message = $summaryMessage
        }
    }
    catch {
        # Restore from backup if write failed
        if ($CreateBackup -and (Test-Path $backupPath)) {
            Copy-Item -Path $backupPath -Destination $Path -Force
        }
        return @{
            Success = $false
            Message = "Error writing file: $_"
        }
    }
}

# =============================================================================
# Main Script
# =============================================================================

# Banner
Write-Host "=" * 70 -ForegroundColor Cyan
Write-Host "RealSim User.c Patcher - PowerShell Edition" -ForegroundColor Cyan
Write-Host "=" * 70 -ForegroundColor Cyan

if ($DryRun) {
    Write-Host "MODE: Dry Run (no changes will be made)" -ForegroundColor Yellow
    Write-Host ""
}

# Determine files to patch
$filesToPatch = @()

if ($FilePath -and $FilePath.Count -gt 0) {
    # Command-line mode
    $filesToPatch = $FilePath
}
else {
    # GUI mode
    Write-Host "Opening file picker..." -ForegroundColor Yellow
    $selectedFiles = Get-FileDialog

    if (-not $selectedFiles -or $selectedFiles.Count -eq 0) {
        Write-Host "No file selected. Exiting." -ForegroundColor Yellow
        Write-Host ""
        Read-Host "Press Enter to exit"
        exit 0
    }

    $filesToPatch = @($selectedFiles)
}

if ($filesToPatch.Count -eq 0) {
    Write-Host "No files to process. Exiting." -ForegroundColor Yellow
    Read-Host "Press Enter to exit"
    exit 0
}

# Process files
$results = @()
foreach ($file in $filesToPatch) {
    $result = Invoke-PatchUserC -Path $file -DryRun:$DryRun -CreateBackup:(-not $NoBackup)
    $results += [PSCustomObject]@{
        File = $file
        Success = $result.Success
        Message = $result.Message
    }
}

# Summary
Write-Host "`n$("=" * 70)" -ForegroundColor Cyan
Write-Host "SUMMARY" -ForegroundColor Cyan
Write-Host "=" * 70 -ForegroundColor Cyan

$successCount = ($results | Where-Object Success).Count

foreach ($result in $results) {
    $status = if ($result.Success) { "[OK]" } else { "[FAIL]" }
    $color = if ($result.Success) { "Green" } else { "Red" }
    $fileName = Split-Path $result.File -Leaf
    Write-Host "$status $fileName`: $($result.Message)" -ForegroundColor $color
}

Write-Host "`nTotal: $successCount/$($results.Count) files patched successfully" -ForegroundColor Cyan

if (-not $DryRun -and $successCount -gt 0) {
    Write-Host "`nNext steps:" -ForegroundColor Yellow
    Write-Host "1. Review the patched User.c file(s)"
    Write-Host "2. Rebuild your CarMaker project"
    Write-Host "3. Configure config.yaml with CarMaker settings"
}

# Pause before exit so user can see results
Write-Host ""
Read-Host "Press Enter to exit"

# Exit code
exit $(if ($successCount -gt 0) { 0 } else { 1 })