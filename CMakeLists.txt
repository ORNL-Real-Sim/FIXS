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

# Where libsumo lives is declared in dependencies.yaml, not hardcoded here, so
# moving the vendor directory is a one-line yaml edit. scripts/fetch_native_deps.sh
# reads the same key, so the two cannot disagree.
set(FIXS_LIBSUMO_RELDIR "CommonLib/libsumo")
file(READ "${CMAKE_CURRENT_SOURCE_DIR}/dependencies.yaml" _fixs_deps_yaml)
string(REGEX MATCH "\n  sumo:[^\n]*\n((  [^\n]*\n|    [^\n]*\n)+)" _m "${_fixs_deps_yaml}")
if(CMAKE_MATCH_1)
    string(REGEX MATCH "location:[ \t]*\"([^\"]+)\"" _m2 "${CMAKE_MATCH_1}")
    if(CMAKE_MATCH_1)
        set(FIXS_LIBSUMO_RELDIR "${CMAKE_MATCH_1}")
    endif()
endif()
message(STATUS "FIXS libsumo location (dependencies.yaml): ${FIXS_LIBSUMO_RELDIR}")

find_path(FIXS_LIBTRACI_INCLUDE_DIR
    NAMES libsumo/libtraci.h
    HINTS "${CMAKE_CURRENT_SOURCE_DIR}/${FIXS_LIBSUMO_RELDIR}/.."
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
    HINTS "${CMAKE_CURRENT_SOURCE_DIR}/${FIXS_LIBSUMO_RELDIR}/bin"
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

if(NOT WIN32)
    # Bind libtracicpp.so RELATIVE TO THE EXECUTABLE, the Linux analogue of the
    # exe-relative DLL search on Windows.
    #
    # This matters because the two platforms bind SUMO at different times.
    # Windows delay-loads libsumocpp.dll (/DELAYLOAD), so SetDllDirectoryA() at
    # startup is early enough. On Linux libtracicpp.so is a DT_NEEDED entry the
    # loader resolves BEFORE main() runs, so ConfigureSumoLibraryPath's
    # setenv(LD_LIBRARY_PATH) cannot affect this process at all -- by the time
    # it runs, the library is already bound (it still helps a child SUMO).
    #
    # Without this, CMake bakes the BUILD MACHINE's absolute library path into
    # RUNPATH, so the binary only runs where it was built: a CI artifact would
    # carry /__w/... and fail to start anywhere else. $ORIGIN makes it
    # relocatable -- move or ship the tree and it still resolves.
    # SEVERAL $ORIGIN-relative candidates, because the layout around the
    # executable differs by how FIXS was obtained, and one hardcoded relative
    # path would only serve one of them:
    #
    #   cloned repo / CI   build/TrafficLayer   + <repo>/CommonLib/libsumo/bin
    #   released bundle    TrafficLayer         + libraries beside the exe
    #                      (8_create_zip.ps1 stages executables at the zip root)
    #   staged install     bin/TrafficLayer     + lib/
    #
    # $ORIGIN is resolved by the loader at run time, so all three work from one
    # binary and nothing depends on where it was built.
    # CMake also appends the link directory to RPATH automatically, as an
    # ABSOLUTE build-machine path. That both leaks the build location into a
    # shipped binary and masks a broken relative RPATH on the build machine
    # (the binary keeps working there while failing everywhere else). Rewrite
    # those automatic entries $ORIGIN-relative too, so nothing absolute
    # survives into the artifact.
    set(_fixs_rpath
        "$ORIGIN"
        "$ORIGIN/${FIXS_LIBSUMO_RELDIR}/bin"
        "$ORIGIN/../${FIXS_LIBSUMO_RELDIR}/bin"
        "$ORIGIN/../lib")
    set_target_properties(TrafficLayer PROPERTIES
        BUILD_RPATH   "${_fixs_rpath}"
        INSTALL_RPATH "${_fixs_rpath}"
        # NOT SKIP_BUILD_RPATH: that drops the explicit list above as well, and
        # then nothing resolves. CMake additionally appends the absolute
        # directory of the library it linked, which stays as a harmless LAST
        # entry -- the loader tries the $ORIGIN candidates first and simply
        # skips a path that does not exist. INSTALL_RPATH (used by
        # `cmake --install`) carries only the relative list.
        BUILD_RPATH_USE_ORIGIN ON)
endif()

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

# --- VirCarlaEnv ------------------------------------------------------------
# OPTIONAL, unlike TrafficLayer: it needs the CARLA C++ client SDK, which no
# distro packages and CARLA publishes nowhere (its releases carry no assets at
# all). A machine without it should still build TrafficLayer, so a missing SDK
# is a skip with an explanation, never a configure error.
#
#   scripts/fetch_native_deps.sh --component carla
#
# As with libsumo, WHERE it lives is declared in dependencies.yaml and read
# here, so the fetch script and this file cannot disagree.
set(FIXS_LIBCARLA_RELDIR "CommonLib/libcarla")
string(REGEX MATCH "\n  carla:[^\n]*\n((  [^\n]*\n|    [^\n]*\n)+)" _m "${_fixs_deps_yaml}")
if(CMAKE_MATCH_1)
    string(REGEX MATCH "location:[ \t]*\"([^\"]+)\"" _m2 "${CMAKE_MATCH_1}")
    if(CMAKE_MATCH_1)
        set(FIXS_LIBCARLA_RELDIR "${CMAKE_MATCH_1}")
    endif()
endif()

set(FIXS_LIBCARLA_DIR "${CMAKE_CURRENT_SOURCE_DIR}/${FIXS_LIBCARLA_RELDIR}" CACHE PATH
    "Root of the CARLA client SDK (expects include/carla and lib/libcarla_client.a)")

option(FIXS_BUILD_VIRCARLAENV "Build VirCarlaEnv (requires the CARLA client SDK)" ON)

if(FIXS_BUILD_VIRCARLAENV AND NOT EXISTS "${FIXS_LIBCARLA_DIR}/lib/libcarla_client.a")
    message(STATUS
        "VirCarlaEnv: SKIPPED -- no CARLA client SDK at ${FIXS_LIBCARLA_DIR}. "
        "Run scripts/fetch_native_deps.sh --component carla to acquire it.")
    set(FIXS_BUILD_VIRCARLAENV OFF)
endif()

if(FIXS_BUILD_VIRCARLAENV)
    # Second copy of VirCarlaEnv.vcxproj's TU list, guarded by
    # scripts/check_source_lists.py exactly as TrafficLayer's is.
    set(VIRCARLAENV_SOURCES
        CommonLib/ConfigHelper.cpp
        CommonLib/DataLogger.cpp
        CommonLib/EgoDriver.cpp
        CommonLib/MsgHelper.cpp
        CommonLib/SocketHelper.cpp
        CommonLib/VirEnvCore.cpp
        CommonLib/XilGuard.cpp
        VirCarlaEnv/VirCarlaEnv/CarlaBackend.cpp
        VirCarlaEnv/VirCarlaEnv/BridgeHelper.cpp
        VirCarlaEnv/VirCarlaEnv/DebugHelper.cpp
        VirCarlaEnv/VirCarlaEnv/mainVirCarla.cpp)

    add_executable(VirCarlaEnv ${VIRCARLAENV_SOURCES})
    target_compile_features(VirCarlaEnv PRIVATE cxx_std_17)

    target_include_directories(VirCarlaEnv PRIVATE
        "${CMAKE_CURRENT_BINARY_DIR}/generated"          # see TrafficLayer above
        "${CMAKE_CURRENT_SOURCE_DIR}/CommonLib"
        "${CMAKE_CURRENT_SOURCE_DIR}/VirCarlaEnv/VirCarlaEnv"
        "${FIXS_LIBCARLA_DIR}/include")
    # SYSTEM: boost 1.81 and rpclib are third-party headers that do not compile
    # warning-free under this project's flags, and they are not ours to fix.
    target_include_directories(VirCarlaEnv SYSTEM PRIVATE
        "${FIXS_LIBCARLA_DIR}/include/system")

    # Static archives, listed dependency-first because a static link is
    # order-sensitive: carla_client pulls rpc, boost::filesystem and the
    # Recast/Detour navigation code.
    #
    # NOT linked, though VirCarlaEnv.vcxproj lists them: libsumocpp/libtracicpp
    # (no TU in this target references libsumo, libtraci or TrafficHelper),
    # boost_python310 (nothing uses the Python bindings), and zlib/zlibstatic
    # (the system libz here). They are vestigial on Windows too.
    set(_fixs_carla_libs
        carla_client rpc
        boost_filesystem boost_system boost_atomic
        Recast Detour DetourCrowd DetourTileCache DebugUtils)
    foreach(_lib IN LISTS _fixs_carla_libs)
        target_link_libraries(VirCarlaEnv PRIVATE "${FIXS_LIBCARLA_DIR}/lib/lib${_lib}.a")
    endforeach()

    target_link_libraries(VirCarlaEnv PRIVATE yaml-cpp)

    if(WIN32)
        target_compile_definitions(VirCarlaEnv PRIVATE
            WIN32 NOMINMAX _CRT_SECURE_NO_WARNINGS _WINSOCK_DEPRECATED_NO_WARNINGS)
        target_link_libraries(VirCarlaEnv PRIVATE ws2_32 Mswsock AdvApi32 shlwapi)
    else()
        find_package(Threads REQUIRED)
        target_link_libraries(VirCarlaEnv PRIVATE Threads::Threads ${CMAKE_DL_LIBS})
        if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 9.0)
            target_link_libraries(VirCarlaEnv PRIVATE stdc++fs)
        endif()
    endif()
endif()
