# FixsVersion.cmake -- generate CommonLib/RealSimVersion.h on non-Windows builds.
#
# scripts/generate_version.ps1 does this on Windows and stays the generator
# there (MSBuild is unchanged by #65). This module mirrors its semantics for the
# CMake path so a Linux/CI build gets the same macros:
#
#   * semver comes from `git describe --tags --match v[0-9]* --abbrev=0`, which
#     deliberately ignores the rolling lightweight tags (latest, alpha_v0.9.0)
#     that would otherwise match and force a 0.0.0 fallback -- see #191.
#   * REALSIM_GIT_TAG is the FULL describe (e.g. v0.8.0-120-gce90f3c0) so a dev
#     build points at its exact commit.
#   * a tagless or git-less checkout degrades to 0.0.0 with a warning instead of
#     failing the build, matching the .ps1's fallback behaviour.

function(fixs_generate_version_header TEMPLATE_IN OUTPUT_H)
    set(VERSION_MAJOR 0)
    set(VERSION_MINOR 0)
    set(VERSION_PATCH 0)
    set(GIT_COMMIT "UNKNOWN")
    set(GIT_TAG "UNKNOWN")

    find_package(Git QUIET)
    if(GIT_FOUND AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/.git")
        execute_process(
            COMMAND "${GIT_EXECUTABLE}" describe --tags --match "v[0-9]*" --abbrev=0
            WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
            OUTPUT_VARIABLE _semver_tag
            OUTPUT_STRIP_TRAILING_WHITESPACE
            ERROR_QUIET
            RESULT_VARIABLE _semver_rc)

        if(_semver_rc EQUAL 0 AND _semver_tag MATCHES "^v?([0-9]+)\\.([0-9]+)\\.([0-9]+)")
            set(VERSION_MAJOR ${CMAKE_MATCH_1})
            set(VERSION_MINOR ${CMAKE_MATCH_2})
            set(VERSION_PATCH ${CMAKE_MATCH_3})
        else()
            message(WARNING
                "FIXS: no vX.Y.Z tag reachable from HEAD -- version falls back to 0.0.0. "
                "This matches generate_version.ps1's behaviour on a tagless checkout.")
        endif()

        execute_process(
            COMMAND "${GIT_EXECUTABLE}" describe --tags --match "v[0-9]*" --always
            WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
            OUTPUT_VARIABLE GIT_TAG
            OUTPUT_STRIP_TRAILING_WHITESPACE
            ERROR_QUIET)

        execute_process(
            COMMAND "${GIT_EXECUTABLE}" rev-parse --short HEAD
            WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
            OUTPUT_VARIABLE GIT_COMMIT
            OUTPUT_STRIP_TRAILING_WHITESPACE
            ERROR_QUIET)
    else()
        message(WARNING "FIXS: git unavailable -- RealSimVersion.h uses placeholder 0.0.0.")
    endif()

    set(VERSION_STRING "${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_PATCH}")

    # "{0:X2}{1:X2}{2:X2}" in the .ps1 -- two hex digits per component.
    foreach(_c MAJOR MINOR PATCH)
        math(EXPR _hex_${_c} "${VERSION_${_c}}" OUTPUT_FORMAT HEXADECIMAL)
        string(REGEX REPLACE "^0x" "" _hex_${_c} "${_hex_${_c}}")
        string(TOUPPER "${_hex_${_c}}" _hex_${_c})
        string(LENGTH "${_hex_${_c}}" _len)
        if(_len LESS 2)
            set(_hex_${_c} "0${_hex_${_c}}")
        endif()
    endforeach()
    set(VERSION_HEX "${_hex_MAJOR}${_hex_MINOR}${_hex_PATCH}")

    string(TIMESTAMP GENERATION_TIME "%Y-%m-%d %H:%M:%S" UTC)

    configure_file("${TEMPLATE_IN}" "${OUTPUT_H}" @ONLY)
    message(STATUS "FIXS version: ${VERSION_STRING} (${GIT_TAG}, ${GIT_COMMIT})")
endfunction()
