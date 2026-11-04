# FixsVersion.cmake -- generate CommonLib/RealSimVersion.h on non-Windows builds.
# Mirrors scripts/generate_version.ps1: the version comes from the tracked VERSION
# file; git only supplies REALSIM_GIT_COMMIT and the describe label REALSIM_GIT_TAG.

function(fixs_generate_version_header TEMPLATE_IN OUTPUT_H)
    set(_version_file "${CMAKE_CURRENT_SOURCE_DIR}/VERSION")
    if(NOT EXISTS "${_version_file}")
        message(FATAL_ERROR "FIXS: VERSION file not found: ${_version_file}")
    endif()
    file(STRINGS "${_version_file}" _declared LIMIT_COUNT 1)
    string(STRIP "${_declared}" _declared)
    if(NOT _declared MATCHES "^([0-9]+)\\.([0-9]+)\\.([0-9]+)$")
        message(FATAL_ERROR "FIXS: VERSION must be X.Y.Z, got '${_declared}'")
    endif()
    set(VERSION_MAJOR ${CMAKE_MATCH_1})
    set(VERSION_MINOR ${CMAKE_MATCH_2})
    set(VERSION_PATCH ${CMAKE_MATCH_3})
    # Re-run configure when VERSION changes.
    set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${_version_file}")

    set(GIT_COMMIT "UNKNOWN")
    set(GIT_TAG "UNKNOWN")
    find_package(Git QUIET)
    if(GIT_FOUND AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/.git")
        # --match skips the non-semver 'stable' tag; --always degrades to the short SHA.
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
        message(WARNING "FIXS: git unavailable -- REALSIM_GIT_COMMIT/TAG set to UNKNOWN.")
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
