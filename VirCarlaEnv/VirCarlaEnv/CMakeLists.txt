cmake_minimum_required(VERSION 3.16)
project(VirCarlaEnv LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# ------------------------------------------------------------------
# Paths to external dependencies (CARLA)
# Looks first for CommonLib/libcarla (populated by copyCarlaDependencies),
# otherwise uses CARLA_ROOT (env or cache) and searches standard layouts.
# ------------------------------------------------------------------
set(FIXS_ROOT "${CMAKE_SOURCE_DIR}/../.." CACHE PATH "FIXS repository root")
set(DEFAULT_CARLA_ROOT "${FIXS_ROOT}/CommonLib/libcarla")

if(NOT DEFINED CARLA_ROOT OR CARLA_ROOT STREQUAL "")
  if(DEFINED ENV{CARLA_ROOT} AND NOT "$ENV{CARLA_ROOT}" STREQUAL "")
    set(CARLA_ROOT "$ENV{CARLA_ROOT}")
  elseif(EXISTS "${DEFAULT_CARLA_ROOT}")
    set(CARLA_ROOT "${DEFAULT_CARLA_ROOT}")
  else()
    set(CARLA_ROOT "" )
  endif()
endif()
set(CARLA_ROOT "${CARLA_ROOT}" CACHE PATH "Path to CARLA checkout/install or extracted dependencies (include/ and lib/)")
if(CARLA_ROOT STREQUAL "")
  message(FATAL_ERROR "Set CARLA_ROOT (or copy dependencies to CommonLib/libcarla).")
endif()

# Pick an include dir by probing common layouts
set(CARLA_INCLUDE_CANDIDATES
  "${CARLA_ROOT}/include"
  "${CARLA_ROOT}/include/carla"
  "${CARLA_ROOT}/LibCarla/source"
)
foreach(dir IN LISTS CARLA_INCLUDE_CANDIDATES)
  if(EXISTS "${dir}/carla/client/Client.h")
    set(CARLA_INCLUDE_DIR "${dir}")
    break()
  endif()
endforeach()
set(CARLA_INCLUDE_DIR "${CARLA_INCLUDE_DIR}" CACHE PATH "Directory containing CARLA headers (expects carla/client/Client.h)")
if(NOT CARLA_INCLUDE_DIR OR NOT EXISTS "${CARLA_INCLUDE_DIR}/carla/client/Client.h")
  message(FATAL_ERROR "Could not find carla/client/Client.h under ${CARLA_ROOT}; set CARLA_INCLUDE_DIR explicitly.")
endif()

# Locate the client library in common lib locations
set(CARLA_LIB_CANDIDATES
  "${CARLA_ROOT}/lib"
  "${CARLA_ROOT}/lib64"
  "${CARLA_ROOT}/Build/libcarla-client-release"
)
find_library(CARLA_CLIENT_LIBRARY NAMES carla_client carla_client.lib PATHS ${CARLA_LIB_CANDIDATES} NO_DEFAULT_PATH)
if(NOT CARLA_CLIENT_LIBRARY)
  find_library(CARLA_CLIENT_LIBRARY NAMES carla_client carla_client.lib)
endif()
set(CARLA_CLIENT_LIBRARY "${CARLA_CLIENT_LIBRARY}" CACHE FILEPATH "Path to CARLA client library")
if(NOT CARLA_CLIENT_LIBRARY)
  message(FATAL_ERROR "Could not find carla_client library. Set CARLA_ROOT or CARLA_CLIENT_LIBRARY.")
endif()
get_filename_component(CARLA_LIB_DIR "${CARLA_CLIENT_LIBRARY}" DIRECTORY)

find_library(BOOST_FILESYSTEM_LIB NAMES boost_filesystem PATHS "${CARLA_LIB_DIR}" "${CARLA_LIB_DIR}/.." NO_DEFAULT_PATH)
find_library(BOOST_SYSTEM_LIB NAMES boost_system PATHS "${CARLA_LIB_DIR}" "${CARLA_LIB_DIR}/.." NO_DEFAULT_PATH)
find_library(RPC_LIB NAMES rpc PATHS "${CARLA_LIB_DIR}" "${CARLA_LIB_DIR}/.." NO_DEFAULT_PATH)
find_library(DETOUR_LIB NAMES Detour PATHS "${CARLA_LIB_DIR}" "${CARLA_LIB_DIR}/.." NO_DEFAULT_PATH)
find_library(DETOUR_CROWD_LIB NAMES DetourCrowd PATHS "${CARLA_LIB_DIR}" "${CARLA_LIB_DIR}/.." NO_DEFAULT_PATH)
find_library(DETOUR_TILE_LIB NAMES DetourTileCache PATHS "${CARLA_LIB_DIR}" "${CARLA_LIB_DIR}/.." NO_DEFAULT_PATH)
find_library(RECAST_LIB NAMES Recast PATHS "${CARLA_LIB_DIR}" "${CARLA_LIB_DIR}/.." NO_DEFAULT_PATH)
if(NOT RPC_LIB)
  message(WARNING "rpclib (librpc) not found alongside CARLA libs; set RPC_LIB or adjust CARLA_LIB_DIR.")
endif()

# ------------------------------------------------------------------
# Third-party: yaml-cpp (use system if present, otherwise the vendored copy)
# ------------------------------------------------------------------
find_package(yaml-cpp QUIET)
if(NOT yaml-cpp_FOUND)
  set(YAML_BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
  set(YAML_CPP_BUILD_TESTS OFF CACHE BOOL "" FORCE)
  set(YAML_CPP_BUILD_TOOLS OFF CACHE BOOL "" FORCE)
  add_subdirectory(${FIXS_ROOT}/CommonLib/yaml-cpp ${CMAKE_BINARY_DIR}/yaml-cpp EXCLUDE_FROM_ALL)
endif()

find_package(Threads REQUIRED)

# ------------------------------------------------------------------
# Common library (shared helpers used by the Carla bridge)
# ------------------------------------------------------------------
set(COMMONLIB_DIR "${FIXS_ROOT}/CommonLib" CACHE PATH "FIXS CommonLib directory")
set(COMMONLIB_SOURCES
  ${COMMONLIB_DIR}/ConfigHelper.cpp
  ${COMMONLIB_DIR}/MsgHelper.cpp
  ${COMMONLIB_DIR}/SocketHelper.cpp
)

add_library(commonlib STATIC ${COMMONLIB_SOURCES})
target_include_directories(commonlib PUBLIC ${COMMONLIB_DIR})
target_link_libraries(commonlib PUBLIC yaml-cpp)
target_compile_definitions(commonlib PUBLIC RS_SKIP_CARMAKER)

# ------------------------------------------------------------------
# VirCarlaEnv executable
# ------------------------------------------------------------------
set(VIRCARLAENV_DIR "${CMAKE_SOURCE_DIR}")
set(VIRCARLAENV_SOURCES
  ${VIRCARLAENV_DIR}/mainVirCarla.cpp
  ${VIRCARLAENV_DIR}/BridgeHelper.cpp
  ${VIRCARLAENV_DIR}/DebugHelper.cpp
)

add_executable(VirCarlaEnv ${VIRCARLAENV_SOURCES})
target_include_directories(VirCarlaEnv PRIVATE
  ${VIRCARLAENV_DIR}
  ${COMMONLIB_DIR}
  ${CARLA_INCLUDE_DIR}
  ${CARLA_INCLUDE_DIR}/system
)

target_link_directories(VirCarlaEnv PRIVATE ${CARLA_LIB_DIR})
target_link_libraries(VirCarlaEnv PRIVATE
  commonlib
  ${CARLA_CLIENT_LIBRARY}
  ${RPC_LIB}
  ${DETOUR_LIB}
  ${DETOUR_CROWD_LIB}
  ${DETOUR_TILE_LIB}
  ${RECAST_LIB}
  Threads::Threads
)

if(BOOST_FILESYSTEM_LIB)
  target_link_libraries(VirCarlaEnv PRIVATE ${BOOST_FILESYSTEM_LIB})
else()
  message(WARNING "boost_filesystem not found alongside CARLA libs; add it to CARLA_LIB_DIR or system paths.")
endif()

if(BOOST_SYSTEM_LIB)
  target_link_libraries(VirCarlaEnv PRIVATE ${BOOST_SYSTEM_LIB})
else()
  message(WARNING "boost_system not found alongside CARLA libs; add it to CARLA_LIB_DIR or system paths.")
endif()

if(UNIX AND NOT APPLE)
  target_link_libraries(VirCarlaEnv PRIVATE dl)
endif()

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# Ensure the executable can find CARLA shared libs at runtime
# - prefer a relative rpath so it works from the build tree without env tweaks
set(CMAKE_SKIP_BUILD_RPATH FALSE)
set(CMAKE_BUILD_RPATH_USE_ORIGIN TRUE)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
file(RELATIVE_PATH CARLA_LIB_RPATH_FROM_BIN
  "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}"
  "${CARLA_LIB_DIR}"
)
set(CARLA_RPATH_ENTRIES
  "${CARLA_LIB_DIR}"
  "\$ORIGIN/${CARLA_LIB_RPATH_FROM_BIN}"
)
set_target_properties(VirCarlaEnv PROPERTIES
  BUILD_RPATH "${CARLA_RPATH_ENTRIES}"
  INSTALL_RPATH "${CARLA_RPATH_ENTRIES}"
)
