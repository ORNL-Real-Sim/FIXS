# Contributing Guide

Thank you for your interest in contributing to FIXS!
Follow this guide to ensure your contributions are properly tracked and merged.

---

## Branch Strategy

- **`main`** — stable release branch; never commit directly
- **`dev`** — integration branch; all PRs target here
- **Feature/fix branches** — branch off `dev`, named `<type>/<issue_number>_<short_desc>`

```
main  ←── (release PR) ←── dev  ←── maintenance/128_unify_fixs_branding
                                ←── bug/84_python_xil_client
                                ←── feature/70_libsumo_direct_launch
```

Branch name types: `feature/`, `bug/`, `maintenance/`, `docs/`

---

## Step 1: Fork and Clone

```bash
git clone https://github.com/<your-username>/FIXS.git
cd FIXS
git submodule update --init --recursive
```

---

## Step 2: Create a Branch off `dev`

```bash
git fetch origin
git checkout -b maintenance/NNN_short_desc origin/dev
```

Always include the GitHub issue number in the branch name.

---

## Step 3: Make Changes and Test

- Run Python unit tests (no simulator required):
  ```bash
  python -m pytest tests/Python/unit/ -v
  python -m pytest tests/Python/test_repo_hygiene.py tests/Python/test_naming_consistency.py tests/Python/test_build_system.py -v
  ```
- Add a test case if your change is testable without simulators

---

## Step 4: Commit and Open a PR

PR title format: `#<issue_number>/<short_description>`

Example: `#130/python commonlib package structure and unit tests`

PR must target the **`dev`** branch, not `main`. Fill in all sections of the PR template.

---

## ProprietaryFiles Submodule

`ProprietaryFiles` is a **private** submodule containing CarMaker/VISSIM/dSPACE integration
code (proprietary commercial software APIs). It lives at
[ORNL-Real-Sim/ProprietaryFiles](https://github.com/ORNL-Real-Sim/ProprietaryFiles)
(private — accessible to ORNL team members only).

**Why it's structured this way:**
The public FIXS repo stores only a commit hash pointer to the private submodule.
Public contributors cannot see the proprietary contents, but can still build and use
the open-source parts of FIXS without them.

### If you are an ORNL team member modifying ProprietaryFiles

**Goal:** every merged FIXS commit must point to a `ProprietaryFiles/main` commit, so anyone cloning FIXS gets a stable PF state. The two PRs are reviewed in parallel but merged back-to-back: PF first, then FIXS.

```bash
# Step 1: Create a matching branch in the private submodule
cd ProprietaryFiles
git checkout -b maintenance/NNN_short_desc
# ... make changes, commit ...
git push origin maintenance/NNN_short_desc

# Step 2: Stage the updated submodule pointer in FIXS (points at your PF branch tip)
cd ..
git add ProprietaryFiles
git commit -m "chore: bump ProprietaryFiles to maintenance/NNN"
git push origin maintenance/NNN_short_desc

# Step 3: Open BOTH PRs at the same time, cross-referencing each other:
#   - ProprietaryFiles PR  →  PF/main
#   - FIXS PR              →  dev

# Step 4: When both PRs are approved, merge PF first, then re-point FIXS to PF/main:
cd ProprietaryFiles && git fetch origin && git checkout main && git pull
cd ..
git add ProprietaryFiles
git commit -m "chore: bump ProprietaryFiles pointer to merged main"
git push
# Then merge the FIXS PR.
```

**Rules:**
- Never push directly to `ProprietaryFiles/main` — branch-protected (requires PR + review)
- The ProprietaryFiles branch name must mirror the FIXS issue branch name
- Reference the companion PR in each PR's description so reviewers can find it
- Step 4 re-point is mandatory: do not merge a FIXS PR while its submodule pointer is at a PF feature-branch tip
- If the FIXS PR is rejected after the PF PR is merged, open a revert PR in PF — don't leave orphan code on PF/main

### If you are an external contributor

You do not need access to `ProprietaryFiles` to contribute to the open-source parts of FIXS
(SUMO integration, Python helpers, build system, tests, documentation). Simply ignore the
`ProprietaryFiles/` directory — it will show as an inaccessible submodule on your clone.

---

## PR Checklist

- [ ] Branch is based on `dev` (not `main`)
- [ ] PR title follows `#NNN/short_description` format
- [ ] All template sections filled in
- [ ] Python unit tests pass locally
- [ ] If ProprietaryFiles was modified: companion PR opened, PF merged first, submodule pointer re-pointed to PF/main HEAD before merging FIXS
- [ ] Relevant GitHub issue is linked in PR body
