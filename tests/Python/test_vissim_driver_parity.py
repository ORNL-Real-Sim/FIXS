"""
test_vissim_driver_parity.py

Structural tests for the consolidated VISSIM driver model (issue #129).

These tests validate that:
  - DriverModel_RealSim.cpp and DriverModel_RealSim_v2021.cpp are thin wrappers
    (< 100 lines each) after the refactor
  - DriverModel_FIXS_Common.h exists and is substantial (> 500 lines)
  - Both .cpp files include DriverModel_FIXS_Common.h
  - Neither .cpp file duplicates large blocks of logic from the other
  - The common header uses the VISSIM_API_INT macro for API versioning
  - The old .cpp does NOT define VISSIM_V2021_API
  - The v2021 .cpp DOES define VISSIM_V2021_API

We do NOT compile the DLLs here — VISSIM is not installed in CI.
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

OLD_CPP = os.path.join(
    VISSIM_SERVER_DIR,
    "DriverModel_RealSim", "DriverModel_RealSim.cpp"
)
V2021_CPP = os.path.join(
    VISSIM_SERVER_DIR,
    "DriverModel_RealSim_v2021", "DriverModel_RealSim_v2021.cpp"
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

def test_old_cpp_exists():
    assert os.path.isfile(OLD_CPP), f"Missing: {OLD_CPP}"


def test_v2021_cpp_exists():
    assert os.path.isfile(V2021_CPP), f"Missing: {V2021_CPP}"


def test_common_header_exists():
    assert os.path.isfile(COMMON_H), f"Missing: {COMMON_H}"


# ---------------------------------------------------------------------------
# Line-count checks
# ---------------------------------------------------------------------------

def test_old_cpp_is_thin_wrapper():
    n = _line_count(OLD_CPP)
    assert n < 100, (
        f"DriverModel_RealSim.cpp should be a thin wrapper "
        f"(< 100 lines) but has {n} lines"
    )


def test_v2021_cpp_is_thin_wrapper():
    n = _line_count(V2021_CPP)
    assert n < 100, (
        f"DriverModel_RealSim_v2021.cpp should be a thin wrapper "
        f"(< 100 lines) but has {n} lines"
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

def test_old_cpp_includes_common_header():
    content = _read(OLD_CPP)
    assert "DriverModel_FIXS_Common.h" in content, (
        "DriverModel_RealSim.cpp must include DriverModel_FIXS_Common.h"
    )


def test_v2021_cpp_includes_common_header():
    content = _read(V2021_CPP)
    assert "DriverModel_FIXS_Common.h" in content, (
        "DriverModel_RealSim_v2021.cpp must include DriverModel_FIXS_Common.h"
    )


# ---------------------------------------------------------------------------
# Version-specific macro checks
# ---------------------------------------------------------------------------

def test_old_cpp_does_not_define_v2021_api():
    content = _read(OLD_CPP)
    # Must not have an uncommented #define VISSIM_V2021_API
    active_defines = [
        line.strip()
        for line in content.splitlines()
        if re.match(r"^\s*#define\s+VISSIM_V2021_API", line)
    ]
    assert len(active_defines) == 0, (
        "DriverModel_RealSim.cpp must NOT define VISSIM_V2021_API "
        f"(found: {active_defines})"
    )


def test_v2021_cpp_defines_v2021_api():
    content = _read(V2021_CPP)
    active_defines = [
        line.strip()
        for line in content.splitlines()
        if re.match(r"^\s*#define\s+VISSIM_V2021_API", line)
    ]
    assert len(active_defines) >= 1, (
        "DriverModel_RealSim_v2021.cpp must define VISSIM_V2021_API "
        "before including the common header"
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
# No-duplication check: neither .cpp should contain large function bodies
# ---------------------------------------------------------------------------

_LARGE_CODE_PATTERN = re.compile(
    r"(DriverModelSetValue|DriverModelGetValue|DriverModelExecuteCommand)"
    r"\s*\(",
    re.IGNORECASE
)

def test_old_cpp_has_no_duplicate_api_implementations():
    content = _read(OLD_CPP)
    matches = _LARGE_CODE_PATTERN.findall(content)
    assert len(matches) == 0, (
        "DriverModel_RealSim.cpp should not implement API functions directly; "
        f"found {len(matches)} definition(s): {matches}"
    )


def test_v2021_cpp_has_no_duplicate_api_implementations():
    content = _read(V2021_CPP)
    matches = _LARGE_CODE_PATTERN.findall(content)
    assert len(matches) == 0, (
        "DriverModel_RealSim_v2021.cpp should not implement API functions "
        f"directly; found {len(matches)} definition(s): {matches}"
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
