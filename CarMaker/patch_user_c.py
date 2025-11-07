#!/usr/bin/env python3
"""
RealSim User.c Patcher
======================
Automatically patches CarMaker User.c files with RealSim integration code.

Usage:
    Interactive (GUI file picker):
        python patch_user_c.py

    Command-line (automated):
        python patch_user_c.py --file path/to/User.c
        python patch_user_c.py --file path/to/User.c --dry-run
        python patch_user_c.py --file path/to/User.c --no-backup

    Batch mode (patch multiple files):
        python patch_user_c.py --file User1.c --file User2.c
"""

import argparse
import re
import sys
from pathlib import Path
from datetime import datetime
import shutil

# Try to import tkinter for GUI file dialog
try:
    import tkinter as tk
    from tkinter import filedialog
    HAS_GUI = True
except ImportError:
    HAS_GUI = False
    print("Warning: tkinter not available. GUI file picker disabled. Use --file argument.")


# =============================================================================
# Code Snippets to Insert
# =============================================================================

HEADER_CODE = """#define REALSIM

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
"""

SCAN_CMDLINE_REALSIM_SECTION = """    // ===========================================================================
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
"""

USER_INIT_FIRST_REALSIM_CODE = """    // ===========================================================================
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
"""

USER_INIT_REALSIM_CODE = """#ifdef REALSIM
    // ===========================================================================
    // 			 RealSim
    // ===========================================================================
    VirEnv_c = newVirEnvHelper();
    // ===========================================================================
    // ===========================================================================
#endif
"""

TESTRUN_START_REALSIM_CODE = """#ifdef REALSIM
    // ===========================================================================
    // 			 RealSim
    // ===========================================================================
    if (VirEnv_isVeryFirstStep && SimCore.State >= SCState_Start) {
        VirEnv_initialization(VirEnv_c, RS_configFile, RS_signalTable);
    }
    // ===========================================================================
    // ===========================================================================
#endif
"""

USER_CALC_REALSIM_CODE = """#ifdef REALSIM
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
"""

TESTRUN_END_REALSIM_CODE = """#ifdef REALSIM
	// ===========================================================================
    // 			 RealSim
    // ===========================================================================
	VirEnv_shutdown(VirEnv_c);
    // ===========================================================================
    // ===========================================================================
#endif"""


# =============================================================================
# Patching Logic
# =============================================================================

def check_already_patched(content):
    """Check if User.c already contains RealSim patches."""
    return '#define REALSIM' in content or 'VirEnv_Wrapper.h' in content


def find_insert_position(content, pattern, search_from=0):
    """Find position to insert code based on regex pattern."""
    match = re.search(pattern, content[search_from:], re.MULTILINE | re.DOTALL)
    if match:
        return search_from + match.end()
    return None


def patch_header_section(content):
    """Patch the header section (beginning of file)."""
    # Find position before #include <Global.h>
    global_h_pattern = r'#include\s+<Global\.h>'
    pos = content.find('#include <Global.h>')

    if pos == -1:
        print("  Warning: Could not find '#include <Global.h>' - skipping header patch")
        return content, False

    # Check if we need to insert before windows.h include
    windows_check = content[:pos]
    if 'windows.h' in windows_check:
        print("  Warning: windows.h found before Global.h - manual review recommended")

    # Insert RealSim header code before Global.h
    content = content[:pos] + HEADER_CODE + '\n' + content[pos:]
    return content, True


def patch_user_scan_cmdline(content):
    """Patch User_ScanCmdLine function."""
    # Find the else if block just before the else if ((*argv)[0] == '-')
    pattern = r'(\s+else if \(strcmp\(\*argv, "-help?"\) == 0\)[^\}]+\})'
    match = re.search(pattern, content, re.MULTILINE | re.DOTALL)

    if not match:
        # Alternative pattern - look for help section
        pattern = r'(User_PrintUsage\(Pgm\);[^\}]+return\s+NULL;\s*\})'
        match = re.search(pattern, content, re.MULTILINE | re.DOTALL)

    if match:
        insert_pos = match.end()
        content = content[:insert_pos] + '\n' + SCAN_CMDLINE_REALSIM_SECTION + content[insert_pos:]
        return content, True
    else:
        print("  Warning: Could not find insertion point in User_ScanCmdLine")
        return content, False


def patch_user_init_first(content):
    """Patch User_Init_First function."""
    # Find the function and insert before return statement
    pattern = r'(int\s+User_Init_First\s*\(void\)\s*\{.*?)(return\s+0;\s*\})'
    match = re.search(pattern, content, re.MULTILINE | re.DOTALL)

    if match:
        insert_pos = match.start(2)
        content = content[:insert_pos] + USER_INIT_FIRST_REALSIM_CODE + '\n    ' + content[insert_pos:]
        return content, True
    else:
        print("  Warning: Could not find User_Init_First function")
        return content, False


def patch_user_init(content):
    """Patch User_Init function."""
    # Find User_Init function and insert at the beginning
    pattern = r'(int\s+User_Init\s*\(void\)\s*\{)'
    match = re.search(pattern, content)

    if match:
        insert_pos = match.end()
        content = content[:insert_pos] + '\n' + USER_INIT_REALSIM_CODE + content[insert_pos:]
        return content, True
    else:
        print("  Warning: Could not find User_Init function")
        return content, False


def patch_user_testrun_start_atend(content):
    """Patch User_TestRun_Start_atEnd function."""
    # Find the function and insert before return statement
    pattern = r'(int\s+User_TestRun_Start_atEnd\s*\(void\)\s*\{.*?)(return\s+0;\s*\})'
    match = re.search(pattern, content, re.MULTILINE | re.DOTALL)

    if match:
        insert_pos = match.start(2)
        content = content[:insert_pos] + '\n' + TESTRUN_START_REALSIM_CODE + '\n    ' + content[insert_pos:]
        return content, True
    else:
        print("  Warning: Could not find User_TestRun_Start_atEnd function")
        return content, False


def patch_user_calc(content):
    """Patch User_Calc function."""
    # Find User_Calc function and insert after the commented warning
    pattern = r'(int\s+User_Calc\s*\(double\s+dt\)\s*\{[^\}]*?/\*if \(!UserCalcCalledByAppTestRunCalc\) return 0;\*/)'
    match = re.search(pattern, content, re.MULTILINE | re.DOTALL)

    if match:
        insert_pos = match.end()
        content = content[:insert_pos] + '\n\n' + USER_CALC_REALSIM_CODE + content[insert_pos:]
        return content, True
    else:
        print("  Warning: Could not find User_Calc function")
        return content, False


def patch_user_testrun_end(content):
    """Patch User_TestRun_End function."""
    # Find the function and insert before return statement
    pattern = r'(int\s+User_TestRun_End\s*\(void\)\s*\{.*?)(return\s+0;\s*\})'
    match = re.search(pattern, content, re.MULTILINE | re.DOTALL)

    if match:
        insert_pos = match.start(2)
        content = content[:insert_pos] + TESTRUN_END_REALSIM_CODE + '\n\n    ' + content[insert_pos:]
        return content, True
    else:
        print("  Warning: Could not find User_TestRun_End function")
        return content, False


def patch_user_c(file_path, dry_run=False, create_backup=True):
    """
    Main patching function.

    Args:
        file_path: Path to User.c file
        dry_run: If True, only show what would be changed
        create_backup: If True, create backup before modifying

    Returns:
        (success, message) tuple
    """
    file_path = Path(file_path)

    if not file_path.exists():
        return False, f"File not found: {file_path}"

    if file_path.name != 'User.c':
        return False, f"File must be named 'User.c', got: {file_path.name}"

    print(f"\nProcessing: {file_path}")

    # Read file
    try:
        with open(file_path, 'r', encoding='utf-8', errors='ignore') as f:
            content = f.read()
    except Exception as e:
        return False, f"Error reading file: {e}"

    # Check if already patched
    if check_already_patched(content):
        return False, "File already contains RealSim patches (skipping)"

    # Apply patches
    original_content = content
    patches_applied = []

    content, success = patch_header_section(content)
    if success:
        patches_applied.append("Header section")

    content, success = patch_user_scan_cmdline(content)
    if success:
        patches_applied.append("User_ScanCmdLine")

    content, success = patch_user_init_first(content)
    if success:
        patches_applied.append("User_Init_First")

    content, success = patch_user_init(content)
    if success:
        patches_applied.append("User_Init")

    content, success = patch_user_testrun_start_atend(content)
    if success:
        patches_applied.append("User_TestRun_Start_atEnd")

    content, success = patch_user_calc(content)
    if success:
        patches_applied.append("User_Calc")

    content, success = patch_user_testrun_end(content)
    if success:
        patches_applied.append("User_TestRun_End")

    if not patches_applied:
        return False, "No patches could be applied"

    # Dry run - just report
    if dry_run:
        print(f"  [DRY RUN] Would apply patches: {', '.join(patches_applied)}")
        return True, f"Dry run complete. {len(patches_applied)} patches would be applied"

    # Create backup
    if create_backup:
        backup_path = file_path.with_suffix('.c.bak.' + datetime.now().strftime('%Y%m%d_%H%M%S'))
        try:
            shutil.copy2(file_path, backup_path)
            print(f"  Created backup: {backup_path.name}")
        except Exception as e:
            return False, f"Error creating backup: {e}"

    # Write patched file
    try:
        with open(file_path, 'w', encoding='utf-8', newline='\n') as f:
            f.write(content)
        print(f"  ✓ Successfully patched: {', '.join(patches_applied)}")
        return True, f"Successfully applied {len(patches_applied)} patches"
    except Exception as e:
        # Restore from backup if write failed
        if create_backup and backup_path.exists():
            shutil.copy2(backup_path, file_path)
        return False, f"Error writing file: {e}"


# =============================================================================
# GUI File Selection
# =============================================================================

def select_file_gui():
    """Open file dialog to select User.c file."""
    if not HAS_GUI:
        print("Error: GUI not available. Install tkinter or use --file argument.")
        return None

    root = tk.Tk()
    root.withdraw()  # Hide main window

    file_path = filedialog.askopenfilename(
        title="Select User.c file to patch",
        filetypes=[
            ("C Source Files", "User.c"),
            ("All C Files", "*.c"),
            ("All Files", "*.*")
        ],
        initialdir=str(Path.cwd())
    )

    root.destroy()

    return file_path if file_path else None


# =============================================================================
# Main Entry Point
# =============================================================================

def main():
    parser = argparse.ArgumentParser(
        description='Patch CarMaker User.c files with RealSim integration code',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Interactive GUI mode
  python patch_user_c.py

  # Patch specific file
  python patch_user_c.py --file ../CM13_proj/src/User.c

  # Dry run (preview changes)
  python patch_user_c.py --file User.c --dry-run

  # Patch without backup
  python patch_user_c.py --file User.c --no-backup

  # Patch multiple files
  python patch_user_c.py --file User1.c --file User2.c
        """
    )

    parser.add_argument(
        '--file', '-f',
        action='append',
        dest='files',
        help='Path to User.c file (can be specified multiple times)'
    )
    parser.add_argument(
        '--dry-run', '-d',
        action='store_true',
        help='Preview changes without modifying files'
    )
    parser.add_argument(
        '--no-backup', '-n',
        action='store_true',
        help='Skip backup creation (not recommended)'
    )

    args = parser.parse_args()

    # Determine which files to patch
    files_to_patch = []

    if args.files:
        # Command-line mode
        files_to_patch = args.files
    else:
        # GUI mode
        if not HAS_GUI:
            print("Error: No --file argument provided and GUI not available")
            print("Install tkinter or use: python patch_user_c.py --file path/to/User.c")
            return 1

        file_path = select_file_gui()
        if not file_path:
            print("No file selected. Exiting.")
            return 0

        files_to_patch = [file_path]

    # Process files
    print("=" * 70)
    print("RealSim User.c Patcher")
    print("=" * 70)

    if args.dry_run:
        print("MODE: Dry Run (no changes will be made)")

    results = []
    for file_path in files_to_patch:
        success, message = patch_user_c(
            file_path,
            dry_run=args.dry_run,
            create_backup=not args.no_backup
        )
        results.append((file_path, success, message))

    # Summary
    print("\n" + "=" * 70)
    print("SUMMARY")
    print("=" * 70)

    success_count = sum(1 for _, success, _ in results if success)

    for file_path, success, message in results:
        status = "✓" if success else "✗"
        print(f"{status} {Path(file_path).name}: {message}")

    print(f"\nTotal: {success_count}/{len(results)} files patched successfully")

    if not args.dry_run and success_count > 0:
        print("\nNext steps:")
        print("1. Review the patched User.c file(s)")
        print("2. Rebuild your CarMaker project")
        print("3. Configure config.yaml with CarMaker settings")

    return 0 if success_count > 0 else 1


if __name__ == '__main__':
    sys.exit(main())
