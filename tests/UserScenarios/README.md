# User Scenarios Directory

This directory is for debugging user-provided proprietary scenarios uploaded via the [OneDrive file request portal](https://outlookuga-my.sharepoint.com/:f:/g/personal/ys04893_uga_edu/EmyW7v04lJhGh263oGvKDfwBMunLjrKmkOkgKDoGfshXVQ).

## Workflow for User Scenario Integration

### 1. Receiving a User Scenario
When a user reports an issue and uploads their scenario files via OneDrive:

1. Download the scenario files from OneDrive
2. Create a new issue in GitHub for the problem (e.g., #85)
3. Create a branch for debugging: `git checkout -b bug/#85` (or appropriate label)

### 2. Setting Up the Scenario for Debugging
```bash
# Create a subdirectory for this specific case
mkdir tests/UserScenarios/issue_85_description

# Extract user files into this directory
cd tests/UserScenarios/issue_85_description
unzip ~/Downloads/user_scenario.zip

# The scenario is now ready for debugging with full IDE support
```

### 3. Debugging Process
- Files in `tests/UserScenarios/*` are gitignored (except this README and .gitkeep)
- You can freely modify/test these files without git tracking them
- Use Visual Studio debugging with the scenario's config.yaml
- Document findings and solutions in the GitHub issue

### 4. Privacy Considerations
**IMPORTANT**:
- All contents of `tests/UserScenarios/*` (except README.md and .gitkeep) are gitignored
- **DO NOT** commit proprietary user scenarios to the repository
- **DO NOT** push branches containing user scenario files
- Each developer's local copy of this directory remains private
- When creating PRs from bug/feature branches, verify no proprietary files are included

### 5. Resolution and Cleanup
After resolving the issue:
1. Merge the fix (without user files) to main via PR
2. Locally delete the user scenario subdirectory: `rm -rf tests/UserScenarios/issue_X_*`
3. Archive the original user files on OneDrive if needed for future reference

## Directory Structure
```
tests/UserScenarios/
├── .gitkeep              # Tracks folder structure across branches
├── README.md             # This file (tracked in git)
└── issue_X_*/           # User scenario subdirectories (gitignored)
    ├── config.yaml
    ├── *.inpx (VISSIM)
    ├── *.net.xml (SUMO)
    └── ...
```

## Notes
- The folder structure (`tests/UserScenarios/`) exists in all branches via `.gitkeep`
- This provides a consistent location for proprietary testing across all development branches
- See issue #80 for the original workflow design discussion
