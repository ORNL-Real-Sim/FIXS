#!/usr/bin/env python3
"""Fail if TrafficLayer's TU list has drifted between MSBuild and CMake.

#65 leaves the .vcxproj files as the Windows build of record and adds a CMake
build for Linux, which means the translation-unit list now exists TWICE. That is
a deliberate trade (see CMakeLists.txt), but it needs a guard: adding a .cpp on
the Windows side and forgetting the CMake side would not fail the Windows build
at all, and the Linux build would simply never compile the new code.

The .vcxproj list is the union of everything TrafficLayer compiles, including
the two Windows-only DSProxy TUs. On the CMake side those two live in a
conditional block, so the comparison is:

    vcxproj  ==  TRAFFICLAYER_SOURCES  +  the FIXS_ENABLE_DSPROXY block

Run standalone, or as a CI step:  python3 scripts/check_source_lists.py
"""

import re
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
VCXPROJ = REPO / "TrafficLayer" / "TrafficLayer" / "TrafficLayer.vcxproj"
CMAKELISTS = REPO / "CMakeLists.txt"


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


def vcxproj_sources() -> set:
    text = VCXPROJ.read_text(encoding="utf-8", errors="replace")
    return {
        normalize(m, VCXPROJ.parent)
        for m in re.findall(r'ClCompile\s+Include="([^"]+\.cpp)"', text)
    }


def cmake_sources() -> set:
    text = CMAKELISTS.read_text(encoding="utf-8", errors="replace")

    base = re.search(r"set\(TRAFFICLAYER_SOURCES(.*?)\)", text, re.S)
    if not base:
        sys.exit("check_source_lists: could not find set(TRAFFICLAYER_SOURCES ...) in CMakeLists.txt")

    extra = re.search(
        r"if\(FIXS_ENABLE_DSPROXY\).*?list\(APPEND TRAFFICLAYER_SOURCES(.*?)\)", text, re.S
    )
    if not extra:
        sys.exit("check_source_lists: could not find the FIXS_ENABLE_DSPROXY source block")

    found = set()
    for block in (base.group(1), extra.group(1)):
        found.update(normalize(m, REPO) for m in re.findall(r"[\w./\\-]+\.cpp", block))
    return found


def main() -> int:
    msbuild = vcxproj_sources()
    cmake = cmake_sources()

    if msbuild == cmake:
        print(f"OK: TrafficLayer TU lists agree ({len(msbuild)} translation units).")
        return 0

    print("FAIL: TrafficLayer's MSBuild and CMake source lists have diverged.\n")
    only_msbuild = sorted(msbuild - cmake)
    only_cmake = sorted(cmake - msbuild)
    if only_msbuild:
        print("  In TrafficLayer.vcxproj but NOT in CMakeLists.txt")
        print("  (the Linux build would silently not compile these):")
        for s in only_msbuild:
            print(f"    - {s}")
    if only_cmake:
        print("  In CMakeLists.txt but NOT in TrafficLayer.vcxproj:")
        for s in only_cmake:
            print(f"    - {s}")
    print("\nAdd the file to both lists, then re-run.")
    return 1


if __name__ == "__main__":
    sys.exit(main())
