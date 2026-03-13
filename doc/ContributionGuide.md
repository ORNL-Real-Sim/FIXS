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

```bash
# Step 1: Create a matching branch in the private submodule
cd ProprietaryFiles
git checkout -b maintenance/NNN_short_desc
# ... make changes, commit ...
git push origin maintenance/NNN_short_desc

# Step 2: Stage the updated submodule pointer in FIXS
cd ..
git add ProprietaryFiles
git commit -m "chore: bump ProprietaryFiles to maintenance/NNN"
git push origin maintenance/NNN_short_desc

# Step 3: Open PR in FIXS (public) — only the hash pointer is visible publicly
# Step 4: After FIXS PR merges to dev, open a PR in ProprietaryFiles (private) to merge to main
```

**Rules:**
- Never push directly to `ProprietaryFiles/main` — it is branch-protected (requires PR + 1 review)
- The ProprietaryFiles branch name must mirror the FIXS issue branch name
- Reference the ProprietaryFiles branch in the FIXS PR description so internal reviewers can find it

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
- [ ] If ProprietaryFiles was modified: matching branch created and pushed there first
- [ ] Relevant GitHub issue is linked in PR body
