"""
Test that FIXS branding is consistent across key source files.

Checks:
- C++ runtime log strings use FIXS, not legacy RealSim prefix
- ConfigHelper.h has EnableFIXS field (not EnableRealSim)
- All tests/**/config.yaml files use EnableFIXS (not EnableRealSim)

Excludes:
- MATLAB API files (RealSimSocket.m, etc.)
- RealSimSocket.cpp (MEX S-function, name must match S_FUNCTION_NAME macro)
- archive/, tmp/ directories
- Vendor libraries: CommonLib/libsumo/, CommonLib/libevent/, CommonLib/yaml-cpp/
- Binary files
"""

import re
import os
import glob
import pytest

# Root of the repository (two levels up from this file's directory)
REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))


def _read_file(path):
    with open(path, "r", encoding="utf-8", errors="replace") as f:
        return f.read()


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _find_realsim_in_log_strings(content):
    """
    Return all lines where "RealSim" appears inside a string literal that is
    part of a log/print/cout call.  We deliberately skip:
      - pure comment lines (starting with // after optional whitespace)
      - lines that are #include directives
      - lines that only reference file names (e.g. RealSimVersion.h)
    """
    suspicious = []
    for lineno, line in enumerate(content.splitlines(), start=1):
        stripped = line.strip()
        # Skip comment-only lines
        if stripped.startswith("//") or stripped.startswith("*") or stripped.startswith("/*"):
            continue
        # Skip preprocessor includes
        if stripped.startswith("#include"):
            continue
        # Skip blank lines
        if not stripped:
            continue
        # Look for "RealSim" inside a string literal on this line
        # Regex: a double-quoted string that contains RealSim
        if re.search(r'"[^"]*RealSim[^"]*"', line):
            suspicious.append((lineno, line.rstrip()))
    return suspicious


# ---------------------------------------------------------------------------
# Test 1: SocketHelper.cpp — no old "RealSim" runtime log strings
# ---------------------------------------------------------------------------

def test_sockhelper_no_realsim_log_strings():
    path = os.path.join(REPO_ROOT, "CommonLib", "SocketHelper.cpp")
    assert os.path.isfile(path), f"File not found: {path}"
    content = _read_file(path)
    hits = _find_realsim_in_log_strings(content)
    assert hits == [], (
        f"Found legacy 'RealSim' in string literals in {path}:\n"
        + "\n".join(f"  line {ln}: {text}" for ln, text in hits)
    )


# ---------------------------------------------------------------------------
# Test 2: VirEnvHelper.cpp — no old "RealSim" in active log calls
# ---------------------------------------------------------------------------

def test_virenvhelper_no_realsim_log_strings():
    path = os.path.join(REPO_ROOT, "CommonLib", "VirEnvHelper.cpp")
    assert os.path.isfile(path), f"File not found: {path}"
    content = _read_file(path)
    hits = _find_realsim_in_log_strings(content)
    # Filter out the commented-out block that is surrounded by /* ... */
    # We already skip // lines; the block comment lines start with * in stripped form
    assert hits == [], (
        f"Found legacy 'RealSim' in string literals in {path}:\n"
        + "\n".join(f"  line {ln}: {text}" for ln, text in hits)
    )


# ---------------------------------------------------------------------------
# Test 3: TrafficHelper.cpp — no old "RealSim" vehicle type string
# ---------------------------------------------------------------------------

def test_traffichelper_no_realsim_log_strings():
    path = os.path.join(REPO_ROOT, "CommonLib", "TrafficHelper.cpp")
    assert os.path.isfile(path), f"File not found: {path}"
    content = _read_file(path)
    hits = _find_realsim_in_log_strings(content)
    assert hits == [], (
        f"Found legacy 'RealSim' in string literals in {path}:\n"
        + "\n".join(f"  line {ln}: {text}" for ln, text in hits)
    )


# ---------------------------------------------------------------------------
# Test 4: ConfigHelper.h — struct has EnableFIXS, not EnableRealSim
# ---------------------------------------------------------------------------

def test_confighelper_h_has_enable_fixs():
    path = os.path.join(REPO_ROOT, "CommonLib", "ConfigHelper.h")
    assert os.path.isfile(path), f"File not found: {path}"
    content = _read_file(path)
    assert "EnableFIXS" in content, (
        f"ConfigHelper.h does not contain 'EnableFIXS' field. "
        f"Was the struct renamed from EnableRealSim to EnableFIXS?"
    )


def test_confighelper_h_no_enable_realsim_field():
    path = os.path.join(REPO_ROOT, "CommonLib", "ConfigHelper.h")
    assert os.path.isfile(path), f"File not found: {path}"
    content = _read_file(path)
    # The field declaration should be gone; only a comment might mention it
    for lineno, line in enumerate(content.splitlines(), start=1):
        stripped = line.strip()
        if stripped.startswith("//") or stripped.startswith("*"):
            continue
        if "EnableRealSim" in line:
            pytest.fail(
                f"ConfigHelper.h still has 'EnableRealSim' on line {lineno}: {line.rstrip()}"
            )


# ---------------------------------------------------------------------------
# Test 5: All tests/**/config.yaml files use EnableFIXS, not EnableRealSim
# ---------------------------------------------------------------------------

def _get_test_yaml_files():
    tests_dir = os.path.join(REPO_ROOT, "tests")
    pattern = os.path.join(tests_dir, "**", "*.yaml")
    return glob.glob(pattern, recursive=True)


def test_all_test_yamls_use_enable_fixs():
    yaml_files = _get_test_yaml_files()
    assert yaml_files, "No YAML files found under tests/"

    violations = []
    for fpath in yaml_files:
        try:
            content = _read_file(fpath)
        except Exception:
            continue
        for lineno, line in enumerate(content.splitlines(), start=1):
            if "EnableRealSim" in line:
                violations.append((fpath, lineno, line.rstrip()))

    assert violations == [], (
        "Found legacy 'EnableRealSim' in test YAML files (should be 'EnableFIXS'):\n"
        + "\n".join(f"  {fp}:{ln}: {text}" for fp, ln, text in violations)
    )
