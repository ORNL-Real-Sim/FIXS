#!/usr/bin/env python3
"""Fail if a target's TU list has drifted between MSBuild and CMake.

#65 leaves the .vcxproj files as the Windows build of record and adds a CMake
build for Linux, which means the translation-unit list now exists TWICE per
target. That is a deliberate trade (see CMakeLists.txt), but it needs a guard:
adding a .cpp on the Windows side and forgetting the CMake side would not fail
the Windows build at all, and the Linux build would simply never compile the
new code.

A .vcxproj list is the union of everything that target compiles, including TUs
that are Windows-only. On the CMake side those live in conditional blocks, so
the comparison for each target is:

    vcxproj  ==  <TARGET>_SOURCES  +  every conditional block listed below

Run standalone, or as a CI step:  python3 scripts/check_source_lists.py
"""

import re
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
CMAKELISTS = REPO / "CMakeLists.txt"

# name, .vcxproj, the set(...) variable in CMakeLists.txt, and any conditional
# list(APPEND ...) blocks that add to it. A target with no conditional block
# lists none -- VirCarlaEnv compiles the same TUs on both platforms, because
# nothing in it is Windows-only (the DSProxy path is TrafficLayer's).
TARGETS = [
    {
        "name": "TrafficLayer",
        "vcxproj": REPO / "TrafficLayer" / "TrafficLayer" / "TrafficLayer.vcxproj",
        "var": "TRAFFICLAYER_SOURCES",
        "conditionals": [
            # VISSIM's DSProxy path LoadLibrary()s a proprietary Windows DLL.
            r"if\(FIXS_ENABLE_DSPROXY\).*?list\(APPEND TRAFFICLAYER_SOURCES(.*?)\)",
        ],
    },
    {
        "name": "VirCarlaEnv",
        "vcxproj": REPO / "VirCarlaEnv" / "VirCarlaEnv" / "VirCarlaEnv.vcxproj",
        "var": "VIRCARLAENV_SOURCES",
        "conditionals": [],
    },
]


def normalize(path: str, base: Path) -> str:
    """Resolve a build-file path to a lowercase repo-relative one.

    .vcxproj entries are relative to the .vcxproj's own directory
    ('..\\..\\CommonLib\\Foo.cpp', 'mainTrafficLayer.cpp'); CMake entries are
    relative to the repo root. Both are resolved against their own base so the
    two lists become comparable.
    """
    resolved = (base / path.replace("\\", "/")).resolve()
    try:
        rel = resolved.relative_to(REPO)
    except ValueError:
        rel = resolved
    return rel.as_posix().lower()


def vcxproj_sources(vcxproj: Path) -> set:
    text = vcxproj.read_text(encoding="utf-8", errors="replace")
    return {
        normalize(m, vcxproj.parent)
        for m in re.findall(r'ClCompile\s+Include="([^"]+\.cpp)"', text)
    }


def cmake_sources(text: str, target: dict) -> set:
    var = target["var"]

    base = re.search(r"set\(" + var + r"(.*?)\)", text, re.S)
    if not base:
        sys.exit(f"check_source_lists: could not find set({var} ...) in CMakeLists.txt")

    blocks = [base.group(1)]
    for pattern in target["conditionals"]:
        extra = re.search(pattern, text, re.S)
        if not extra:
            sys.exit(
                f"check_source_lists: {target['name']}: could not find the "
                f"conditional source block /{pattern}/"
            )
        blocks.append(extra.group(1))

    found = set()
    for block in blocks:
        found.update(normalize(m, REPO) for m in re.findall(r"[\w./\\-]+\.cpp", block))
    return found


def check(target: dict, cmake_text: str) -> bool:
    name = target["name"]
    if not target["vcxproj"].exists():
        # Not a silent skip: a renamed or deleted .vcxproj means the guard is
        # no longer guarding anything, which is exactly what it exists to catch.
        print(f"FAIL: {name}: {target['vcxproj'].relative_to(REPO)} not found.")
        return False

    msbuild = vcxproj_sources(target["vcxproj"])
    cmake = cmake_sources(cmake_text, target)

    if msbuild == cmake:
        print(f"OK: {name} TU lists agree ({len(msbuild)} translation units).")
        return True

    print(f"FAIL: {name}'s MSBuild and CMake source lists have diverged.\n")
    only_msbuild = sorted(msbuild - cmake)
    only_cmake = sorted(cmake - msbuild)
    if only_msbuild:
        print(f"  In {target['vcxproj'].name} but NOT in CMakeLists.txt")
        print("  (the Linux build would silently not compile these):")
        for s in only_msbuild:
            print(f"    - {s}")
    if only_cmake:
        print(f"  In CMakeLists.txt but NOT in {target['vcxproj'].name}:")
        for s in only_cmake:
            print(f"    - {s}")
    print("\nAdd the file to both lists, then re-run.")
    return False


def main() -> int:
    cmake_text = CMAKELISTS.read_text(encoding="utf-8", errors="replace")
    ok = True
    for target in TARGETS:
        ok &= check(target, cmake_text)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
