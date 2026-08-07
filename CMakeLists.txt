# ---------------------------------------------------------------------------
# FIXS -- CMake build (issue #65)
#
# SCOPE: this builds TrafficLayer on Linux. It does NOT replace the MSBuild
# build on Windows -- the .vcxproj files remain the Windows build of record and
# are untouched by #65. Migrating Windows onto CMake is deliberately a separate,
# later piece of work; this file is written to make that migration a deletion
# rather than a rewrite (hence the if(WIN32) branches, which are present but
# unexercised today).
#
# Consequence worth knowing: the TU list below is a SECOND copy of the one in
# TrafficLayer.vcxproj. scripts/check_source_lists.py compares them and CI fails
# on divergence, so "added a TU on Windows, forgot Linux" cannot pass silently.
#
# Requires CMake >= 3.16 (Ubuntu 20.04 stock) and a C++17 compiler.
# ---------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.16)
project(FIXS LANGUAGES CXX)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(FixsVersion)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

# VISSIM's DSProxy path LoadLibrary()s PTV's proprietary DrivingSimulatorProxy.dll,
# so it can only exist on Windows. Off everywhere else; see the guard in
# ConfigHelper that rejects a VISSIM config at parse time on Linux.
option(FIXS_ENABLE_DSPROXY "Build the VISSIM DSProxy path (Windows only)" ${WIN32})

# --- version header ---------------------------------------------------------
# Generated into the build tree, never into the source tree, so a Linux build
# cannot leave a stray RealSimVersion.h next to the one generate_version.ps1
# writes on Windows.
fixs_generate_version_header(
    "${CMAKE_CURRENT_SOURCE_DIR}/CommonLib/RealSimVersion.h.in"
    "${CMAKE_CURRENT_BINARY_DIR}/generated/RealSimVersion.h")

# --- yaml-cpp ---------------------------------------------------------------
# Built from the vendored copy. This also runs generate_export_header(), which
# is what produces yaml-cpp/dll.h -- that header is NOT in git, so anything
# including yaml.h fails until this target has been configured.
set(YAML_CPP_BUILD_TESTS   OFF CACHE BOOL "" FORCE)
set(YAML_CPP_BUILD_TOOLS   OFF CACHE BOOL "" FORCE)
set(YAML_CPP_FORMAT_SOURCE OFF CACHE BOOL "" FORCE)
set(YAML_BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
add_subdirectory(CommonLib/yaml-cpp EXCLUDE_FROM_ALL)

# --- libtraci ---------------------------------------------------------------
# TrafficHelper.h leaves ENABLE_LIBSUMO commented out, so the active client is
# libtraci -- a thin TraCI socket client. That matters for cost: libtracicpp
# avoids the gdal/proj/fox/osg chain that libsumocpp drags in.
#
# #238 dropped the vendored CommonLib/libsumo from git -- on Windows it is
# restored by fetch_native_deps.ps1, which has no Linux counterpart. So the
# headers are LOCATED rather than assumed: the fetched copy when it is present,
# otherwise the SUMO source checkout we build libtracicpp from, which carries
# the headers for exactly the version we link against so the two cannot drift.
set(FIXS_SUMO_HOME "$ENV{SUMO_HOME}" CACHE PATH
    "Root of a built SUMO checkout (expects bin/libtracicpp.so, headers under src/ or include/)")

find_path(FIXS_LIBTRACI_INCLUDE_DIR
    NAMES libsumo/libtraci.h
    HINTS "${CMAKE_CURRENT_SOURCE_DIR}/CommonLib"
          "${FIXS_SUMO_HOME}/include"
          "${FIXS_SUMO_HOME}/src"
    NO_DEFAULT_PATH
    DOC "Directory containing libsumo/libtraci.h")

if(NOT FIXS_LIBTRACI_INCLUDE_DIR)
    message(FATAL_ERROR
        "libsumo/libtraci.h not found. These headers are no longer vendored in "
        "git (#238). Point -DFIXS_SUMO_HOME=<sumo checkout> at a SUMO source "
        "tree, or run fetch_native_deps to populate CommonLib/libsumo.")
endif()

find_library(FIXS_LIBTRACI_LIBRARY
    NAMES tracicpp libtracicpp
    HINTS "${CMAKE_CURRENT_SOURCE_DIR}/CommonLib/libsumo/bin"
          "${FIXS_SUMO_HOME}/bin" "${FIXS_SUMO_HOME}/lib"
    DOC "Path to libtracicpp")

if(NOT FIXS_LIBTRACI_LIBRARY)
    message(FATAL_ERROR
        "libtracicpp not found. Run scripts/fetch_native_deps.sh to build and "
        "install it into CommonLib/libsumo, or point -DFIXS_SUMO_HOME=<sumo "
        "checkout> at an existing build, or set -DFIXS_LIBTRACI_LIBRARY=<path>.")
endif()

# --- TrafficLayer -----------------------------------------------------------
set(TRAFFICLAYER_SOURCES
    CommonLib/ConfigHelper.cpp
    CommonLib/MsgHelper.cpp
    CommonLib/SocketHelper.cpp
    CommonLib/TrafficHelper.cpp
    CommonLib/XilGuard.cpp
    TrafficLayer/TrafficLayer/mainTrafficLayer.cpp)

if(FIXS_ENABLE_DSPROXY)
    list(APPEND TRAFFICLAYER_SOURCES
        CommonLib/VissimDSProxyHelper.cpp
        TrafficLayer/TrafficLayer/DSProxyMode.cpp)
endif()

add_executable(TrafficLayer ${TRAFFICLAYER_SOURCES})

target_compile_features(TrafficLayer PRIVATE cxx_std_17)

target_include_directories(TrafficLayer PRIVATE
    # generated/ FIRST: on a machine that also builds with MSBuild,
    # generate_version.ps1 leaves a CommonLib/RealSimVersion.h behind (gitignored,
    # so CI never sees it). Since the include is quoted, CommonLib would win the
    # search and the Linux build would silently embed the Windows run's version.
    "${CMAKE_CURRENT_BINARY_DIR}/generated"
    "${CMAKE_CURRENT_SOURCE_DIR}/CommonLib"
    "${FIXS_LIBTRACI_INCLUDE_DIR}")

target_link_libraries(TrafficLayer PRIVATE
    yaml-cpp
    "${FIXS_LIBTRACI_LIBRARY}")

if(WIN32)
    # Present for the eventual Windows migration; MSBuild supplies these today.
    target_compile_definitions(TrafficLayer PRIVATE
        WIN32 NOMINMAX _CRT_SECURE_NO_WARNINGS _WINSOCK_DEPRECATED_NO_WARNINGS)
    target_link_libraries(TrafficLayer PRIVATE ws2_32 Mswsock AdvApi32 shlwapi)
else()
    find_package(Threads REQUIRED)
    target_link_libraries(TrafficLayer PRIVATE Threads::Threads)
    # std::filesystem only merged into libstdc++ proper in GCC 9; older GCC
    # needs the separate library.
    if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 9.0)
        target_link_libraries(TrafficLayer PRIVATE stdc++fs)
    endif()
endif()
