"""
test_vissim_driver_parity.py

Structural tests for the consolidated VISSIM driver model.

History:
  - Issue #129: consolidated two driver model variants behind a shared
    DriverModel_FIXS_Common.h.
  - Issue #147: renamed the int-API (VISSIM 2021+) build to the unmarked
    default DriverModel_RealSim.dll and froze the long-API (VISSIM <= 2020)
    build as DriverModel_RealSim_legacy.dll.

These tests validate that:
  - DEFAULT_CPP (DriverModel_RealSim/DriverModel_RealSim.cpp) and LEGACY_CPP
    (DriverModel_RealSim_legacy/DriverModel_RealSim_legacy.cpp) are thin
    wrappers (< 100 lines each) after the refactor
  - DriverModel_FIXS_Common.h exists and is substantial (> 500 lines)
  - Both .cpp files include DriverModel_FIXS_Common.h
  - Neither .cpp file duplicates large blocks of logic from the other
  - The common header uses the VISSIM_API_INT macro for API versioning
  - The default .cpp DOES define VISSIM_V2021_API (int API, the supported path)
  - The legacy .cpp does NOT define VISSIM_V2021_API (long API, frozen)
  - The legacy wrapper carries the FROZEN policy marker so contributors know
    it is not accepting new features or refactors.

We do NOT compile the DLLs here -- VISSIM is not installed in CI.
"""

import os
import re
import pytest

REPO_ROOT = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "..")
)

VISSIM_SERVER_DIR = os.path.join(
    REPO_ROOT,
    "ProprietaryFiles", "VISSIMserver"
)

DEFAULT_CPP = os.path.join(
    VISSIM_SERVER_DIR,
    "DriverModel_RealSim", "DriverModel_RealSim.cpp"
)
LEGACY_CPP = os.path.join(
    VISSIM_SERVER_DIR,
    "DriverModel_RealSim_legacy", "DriverModel_RealSim_legacy.cpp"
)
COMMON_H = os.path.join(
    VISSIM_SERVER_DIR,
    "DriverModel_FIXS_Common.h"
)


def _read(path: str) -> str:
    with open(path, "r", encoding="utf-8") as fh:
        return fh.read()


def _line_count(path: str) -> int:
    return sum(1 for _ in open(path, "r", encoding="utf-8"))


# ---------------------------------------------------------------------------
# Existence checks
# ---------------------------------------------------------------------------

def test_default_cpp_exists():
    assert os.path.isfile(DEFAULT_CPP), f"Missing: {DEFAULT_CPP}"


def test_legacy_cpp_exists():
    assert os.path.isfile(LEGACY_CPP), f"Missing: {LEGACY_CPP}"


def test_common_header_exists():
    assert os.path.isfile(COMMON_H), f"Missing: {COMMON_H}"


# ---------------------------------------------------------------------------
# Line-count checks
# ---------------------------------------------------------------------------

def test_default_cpp_is_thin_wrapper():
    n = _line_count(DEFAULT_CPP)
    assert n < 100, (
        f"DriverModel_RealSim/DriverModel_RealSim.cpp should be a thin "
        f"wrapper (< 100 lines) but has {n} lines"
    )


def test_legacy_cpp_is_thin_wrapper():
    n = _line_count(LEGACY_CPP)
    assert n < 100, (
        f"DriverModel_RealSim_legacy/DriverModel_RealSim_legacy.cpp should "
        f"be a thin wrapper (< 100 lines) but has {n} lines"
    )


def test_common_header_is_substantial():
    n = _line_count(COMMON_H)
    assert n > 500, (
        f"DriverModel_FIXS_Common.h should contain the shared implementation "
        f"(> 500 lines) but has {n} lines"
    )


# ---------------------------------------------------------------------------
# Include checks
# ---------------------------------------------------------------------------

def test_default_cpp_includes_common_header():
    content = _read(DEFAULT_CPP)
    assert "DriverModel_FIXS_Common.h" in content, (
        "Default DriverModel_RealSim.cpp must include DriverModel_FIXS_Common.h"
    )


def test_legacy_cpp_includes_common_header():
    content = _read(LEGACY_CPP)
    assert "DriverModel_FIXS_Common.h" in content, (
        "Legacy DriverModel_RealSim_legacy.cpp must include "
        "DriverModel_FIXS_Common.h"
    )


# ---------------------------------------------------------------------------
# Version-specific macro checks
# ---------------------------------------------------------------------------

def test_default_cpp_defines_v2021_api():
    """The default wrapper is the int-API (VISSIM 2021+) build."""
    content = _read(DEFAULT_CPP)
    active_defines = [
        line.strip()
        for line in content.splitlines()
        if re.match(r"^\s*#define\s+VISSIM_V2021_API", line)
    ]
    assert len(active_defines) >= 1, (
        "Default DriverModel_RealSim.cpp must define VISSIM_V2021_API "
        "before including the common header (int API is the FIXS default)"
    )


def test_legacy_cpp_does_not_define_v2021_api():
    """The legacy wrapper is the long-API (VISSIM <= 2020) frozen build."""
    content = _read(LEGACY_CPP)
    active_defines = [
        line.strip()
        for line in content.splitlines()
        if re.match(r"^\s*#define\s+VISSIM_V2021_API", line)
    ]
    assert len(active_defines) == 0, (
        "Legacy DriverModel_RealSim_legacy.cpp must NOT define "
        f"VISSIM_V2021_API (found: {active_defines})"
    )


def test_common_header_uses_vissim_api_int_macro():
    content = _read(COMMON_H)
    assert "VISSIM_API_INT" in content, (
        "DriverModel_FIXS_Common.h must define and use VISSIM_API_INT"
    )
    assert "#define VISSIM_API_INT" in content, (
        "DriverModel_FIXS_Common.h must define the VISSIM_API_INT macro"
    )


# ---------------------------------------------------------------------------
# No-duplication check: neither .cpp should contain API function bodies
# ---------------------------------------------------------------------------

_LARGE_CODE_PATTERN = re.compile(
    r"(DriverModelSetValue|DriverModelGetValue|DriverModelExecuteCommand)"
    r"\s*\(",
    re.IGNORECASE
)

def test_default_cpp_has_no_duplicate_api_implementations():
    content = _read(DEFAULT_CPP)
    matches = _LARGE_CODE_PATTERN.findall(content)
    assert len(matches) == 0, (
        "Default DriverModel_RealSim.cpp should not implement API functions "
        f"directly; found {len(matches)} definition(s): {matches}"
    )


def test_legacy_cpp_has_no_duplicate_api_implementations():
    content = _read(LEGACY_CPP)
    matches = _LARGE_CODE_PATTERN.findall(content)
    assert len(matches) == 0, (
        "Legacy DriverModel_RealSim_legacy.cpp should not implement API "
        f"functions directly; found {len(matches)} definition(s): {matches}"
    )


# ---------------------------------------------------------------------------
# Common header has all four required API entry points
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("func_name", [
    "DriverModelSetValue",
    "DriverModelSetValue3",
    "DriverModelGetValue",
    "DriverModelGetValue3",
    "DriverModelExecuteCommand",
])
def test_common_header_implements_api_function(func_name: str):
    content = _read(COMMON_H)
    pattern = re.compile(
        rf"DRIVERMODEL_API\s+int\s+{re.escape(func_name)}\s*\("
    )
    assert pattern.search(content), (
        f"DriverModel_FIXS_Common.h must implement {func_name}()"
    )


# ---------------------------------------------------------------------------
# TODO stub preservation check
# ---------------------------------------------------------------------------

def test_common_header_preserves_todo_stubs():
    content = _read(COMMON_H)
    todo_count = content.count("// TODO #129:")
    assert todo_count >= 5, (
        f"Expected at least 5 '// TODO #129:' stubs in DriverModel_FIXS_Common.h "
        f"to document unimplemented handlers, found {todo_count}"
    )


# ---------------------------------------------------------------------------
# FROZEN policy marker on the legacy wrapper (issue #147)
# ---------------------------------------------------------------------------

def test_legacy_cpp_has_frozen_policy_marker():
    """The legacy wrapper must clearly communicate that it is frozen so
    future contributors do not silently add new features there.
    """
    content = _read(LEGACY_CPP)
    assert "FROZEN" in content, (
        "Legacy DriverModel_RealSim_legacy.cpp must include a 'FROZEN' "
        "policy marker so contributors know the file is not accepting new "
        "features or refactors (only build-break and security fixes)."
    )
