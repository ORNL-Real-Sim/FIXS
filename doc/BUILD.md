# Building Real-Sim FIXS

This document describes how to build Real-Sim FIXS from source. For general project information, see [README.md](../README.md).

## Table of Contents
* [Quick Start](#quick-start)
* [First-Run Setup (fresh clone)](#first-run-setup-fresh-clone)
* [Build System Overview](#build-system-overview)
* [Prerequisites](#prerequisites)
* [Release Builds](#release-builds)
* [Automated Release CI & Proprietary Binaries Bundle](#automated-release-ci--proprietary-binaries-bundle)
* [Development Builds](#development-builds)
* [Debug vs Release Configuration](#debug-vs-release-configuration)
* [Build System Architecture](#build-system-architecture)
* [Troubleshooting](#troubleshooting)

## Quick Start

For a complete release build of all components:

```batch
dispatch.bat
```

This single command will:
- Generate the version header (RealSimVersion.h) from git tags
- Initialize the clone: submodules, native deps (libsumo, libcarla), yaml-cpp
- Build core components (TrafficLayer, VirtualEnvironment)
- Build VISSIM driver model DLLs
- Build CarMaker executables for all detected versions
- Build dSPACE libraries (if dSPACE is installed)
- Build MEX files for MATLAB/Simulink
- Generate BUILD_INFO.txt
- Copy all artifacts to `build/` directory

## First-Run Setup (fresh clone)

`dispatch.bat` initializes the clone itself (step 1), so on a fresh checkout **`dispatch.bat` alone is enough**. If you want to do the setup separately — or you build individual components rather than running the full dispatch — run:

```powershell
powershell -ExecutionPolicy Bypass -File scripts\initialize_fixs.ps1
```

It is idempotent: every step short-circuits when its output is already present, so re-running it is cheap and safe.

| Step | What it does | Required? |
|------|--------------|-----------|
| ProprietaryFiles submodule | `git submodule update --init --recursive` | Optional — private repo. External contributors have no access; the public core still builds. |
| Native deps | Downloads checksum-verified zips from the rolling `fixs-native-deps` release | **libsumo: required.** libcarla: optional (Carla only). |
| yaml-cpp | CMake build of the vendored source (Release + Debug) | **Required** |

Useful flags: `-Force` re-acquires the native deps, `-CarlaMode prebuilt|source` picks the libcarla path, and `-SkipSubmodules` / `-SkipNativeDeps` / `-SkipYamlCpp` opt out of individual steps.

> Not to be confused with `scripts/update_fixs.ps1` / `scripts/update_fixs.sh`, which are the **consumer-side** installers for a published FIXS release zip. `initialize_fixs.ps1` is for a developer checkout.

### Native dependencies are not in git

`CommonLib/libsumo` and `CommonLib/libcarla` are **gitignored** and acquired at setup time from per-component, version- and platform-named assets on the public rolling release `fixs-native-deps` (`libsumo-<ver>-windows-x86_64.zip`, `libcarla-<ver>-windows-x86_64.zip`), each verified against its `.sha256` sidecar. Versions come from `dependencies.yaml`. One release carries every platform, so the name states which one an asset is for — read the suffix before downloading by hand.

libsumo used to be committed — 105 binary files, ~430 MB in the working tree. It was dropped in #238 because binaries in git get no verification, and the vendored copy proved it: it was silently missing `geos_c.dll` and `geos.dll` for months (#70). `libsumocpp.dll` could not load at all, and because TrafficLayer *delay-loads* it, the breakage surfaced only at the first libsumo call, as a Win32 loader exception no `catch` block can see. Both the packer and the fetcher now **load-test** `libsumo/bin` rather than trusting a file count, so that class of gap cannot ship again.

Because libsumo is required to link `TrafficLayer` (`TrafficLayer.vcxproj` references `..\..\CommonLib\libsumo\bin\libsumocpp.lib` directly), the fetch must happen before any core build — which is precisely why `dispatch.bat` runs initialization as step 1.

**Bumping the SUMO version** in `dependencies.yaml` requires publishing the matching asset first, otherwise every clone breaks:

```powershell
powershell -File scripts\build_libsumo.ps1                                  # rebuild from SUMO source
powershell -File scripts\dispatch\pack_native_deps.ps1 -Component sumo -Publish
```

**Offline / air-gapped:** the fetch needs network access to GitHub Releases. Without it, obtain `libsumo-<ver>-windows-x86_64.zip` out of band — the `-linux-x86_64` asset next to it on the same release contains a `.so` and will not link — and extract it into `CommonLib/` so that `CommonLib/libsumo/bin/libsumocpp.lib` exists, then run `initialize_fixs.ps1` — it detects the sentinel and skips the download. Same for `libcarla`.

> Removing libsumo from git only shrinks **shallow** clones. The blobs remain in repo history, so a full `git clone` still transfers them.

### Recommended clone: skip the history blobs

A plain `git clone` of this repo transfers roughly **1 GB packed**, because git sends every version of every file ever committed. Only a small fraction of that is content any build reads — the bulk is historical binaries that were committed and later deleted (`CM9_proj`/`CM10_proj`/`CM11_proj`, a committed `.venv/`, old `build/` outputs). Deleting a file from HEAD does not remove it from history, so the download cost stays (#255).

Clone with a **blob filter** instead:

```bash
git clone --filter=blob:none https://github.com/ORNL-Real-Sim/FIXS.git
```

This is a *partial clone*: git fetches the full commit graph and directory trees, but no file contents up front. It then downloads each blob lazily, the first time something actually reads it. You get a complete, fully functional repository — `git log`, `git describe`, `git blame`, branch switching and `dispatch.bat` all behave normally — you just never pay for the ~2.5 GiB of file versions no build opens. The one tradeoff is that operations reaching into old history (checking out an ancient commit, `git log -p` over the whole repo) fetch on demand and so need network access.

If you also do not need the test networks and docs locally, add a sparse checkout — `tests/` (197 MiB) and `doc/` (21 MiB) are 218 MiB of the 231 MiB present at HEAD, and neither is read by `scripts/dispatch/dispatch.bat`. Measured, this takes the working tree from 231 MiB to **13 MiB**:

```powershell
git clone --filter=blob:none --no-checkout https://github.com/ORNL-Real-Sim/FIXS.git
cd FIXS
git sparse-checkout set --no-cone '/*' '!/tests/' '!/doc/'
git checkout
```

> **Run this in PowerShell or cmd, not Git Bash.** MSYS path conversion rewrites any argument starting with `/`, so in Git Bash `!/tests/` silently becomes `!C:/Program Files/Git/tests/` — the pattern then matches nothing and the exclusion appears to do nothing at all. If you must use Git Bash, prefix the command with `MSYS_NO_PATHCONV=1`.

Verify it worked with `git sparse-checkout list` — the three patterns must come back exactly as typed. Undo the narrowing at any time with `git sparse-checkout disable`. The release CI uses this same pair of flags in [`.github/workflows/release.yml`](../.github/workflows/release.yml), where `actions/checkout` writes the patterns to a file directly and no shell mangling applies.

## Build System Overview

The Real-Sim FIXS build system uses a modular script-based architecture that automatically detects installed tools and versions. Key features:

- **Automated tool detection**: Automatically finds Visual Studio, MATLAB, dSPACE, and CarMaker installations
- **Version management**: Central `dependencies.yaml` file defines all tool versions and configurations
- **Modular scripts**: Each subsystem has its own build script for independent development
- **Intelligent configuration**: Automatically generates CarMaker BuildConfig files based on detected tool versions
- **Comprehensive logging**: Detailed build logs in `scripts/dispatch/build.log` and `scripts/dispatch/build_summary.log`

## Prerequisites

### Required Tools

1. **Visual Studio 2022** (Community, Professional, or Enterprise)
   - Required for building all C++ components
   - Must have C++ desktop development workload installed
   - MSBuild must be in PATH (automatically detected by build scripts)
   - **C++ Standard**: The project uses mixed C++ standards for compatibility:
     - **TrafficLayer**: C++17 (uses `std::filesystem` for runtime library discovery)
     - **All other components**: C++14 (VirtualEnvironment, DriverModel, SC_DLL, CarMaker projects)
     - C++14 is required for compatibility with CarMaker SDK and dSPACE toolchain

2. **CMake** (version 3.10 or higher)
   - Required for building external libraries (yaml-cpp)
   - Must be in PATH
   - Download from: https://cmake.org/download/

### Optional Tools (for specific features)

3. **MATLAB/Simulink** (version 2024a or compatible)
   - Required for MEX file compilation and Simulink integration
   - Automatically detected from registry or common installation paths
   - Version can be configured in `dependencies.yaml`

4. **CarMaker** (versions 13.1.3, 11.1.2, or compatible)
   - Required for XIL/HIL vehicle dynamics integration
   - Multiple versions can coexist
   - Automatically detected from common installation paths

5. **dSPACE ConfigurationDesk** (version 2024a or compatible)
   - Required for real-time HIL system integration
   - Automatically detected from Program Files
   - Version can be configured in `dependencies.yaml`

### Configuration File

The `dependencies.yaml` file in the root directory defines all tool versions and configurations:

```yaml
simulators:
  carmaker:
    version: "13.1.3"
    versions:
      - "13.1.3"
      - "11.1.2"

development_tools:
  visual_studio:
    version: "2022"

  dspace:
    version: "2024a"
    product: "ConfigurationDesk 2024-A"
    release: "24.1"

  matlab:
    version: "2024a"
```

The build system reads this file to:
- Determine which CarMaker versions to build
- Generate version-specific BuildConfig files
- Locate correct tool installations
- Create version-specific output artifacts

## Release Builds

A release build compiles all components and packages them into the `build/` directory for distribution.

### Running a Release Build

```batch
dispatch.bat
```

### What Gets Built

The release build process executes the following steps:

0. **Version Header** (`generate_version.ps1`)
   - Generates `CommonLib/RealSimVersion.h` from git tags
   - Falls back to existing header or default (0.0.0) if git is unavailable
   - Output: `CommonLib/RealSimVersion.h` (auto-generated, not committed)

1. **External Libraries** (`1_external_libraries.bat`)
   - yaml-cpp (YAML configuration parser)
   - Built in both Debug and Release configurations
   - Output: `CommonLib/yaml-cpp/build/`
   - Note: libevent was removed in issue #131 as it was confirmed unused

2. **Core Components** (`2_core_components.bat`)
   - `TrafficLayer.exe` - Main interface executable
   - `CoordMerge.exe` - Coordinated merge controller
   - `VirtualEnvironment.lib` - Shared library for virtual environment integration
   - Output: Component directories + copied to `build/`

3. **VISSIM Components** (`3_vissim_components.bat`)
   - `DriverModel_RealSim.dll` - VISSIM driver model, default (int API, VISSIM 2021+, including 2022)
   - `DriverModel_RealSim_legacy.dll` - VISSIM driver model, legacy/frozen (long API, VISSIM ≤ 2020)
   - Output: `ProprietaryFiles/VISSIMserver/` + copied to `build/`

4. **CarMaker Components** (`4a_carmaker_components.ps1`)
   - Generates BuildConfig Python files for each CarMaker version
   - `CarMaker.win64.exe` - CarMaker Office executables
   - `libcarmaker4sl.mexw64` - CarMaker for Simulink MEX files
   - Built for each version in `dependencies.yaml` (e.g., CM11, CM13)
   - Output: `CarMaker/CM*/` + copied to `build/CarMaker/`

5. **dSPACE Libraries** (`4b_carmaker_dspace.ps1`) - Optional
   - `libRealSimDsLib_2024a_CM*.a` - Version-specific dSPACE libraries
   - Only built if dSPACE ConfigurationDesk is detected
   - Built for each CarMaker version
   - Output: `CommonLib/` + copied to `build/CarMaker/`

6. **MEX RealSimSocket** (`5_mex_realsim_socket.ps1`) - Optional
   - `RealSimSocket.mexw64` - MATLAB interface for socket communication
   - Only built if MATLAB is detected
   - Output: `CommonLib/` + copied to `build/`

7. **Build Information** (`6_build_info.ps1`)
   - Generates `BUILD_INFO.txt` with version information
   - Lists all built components and their versions
   - Output: `build/BUILD_INFO.txt`

### Build Output Structure

After a successful release build, the `build/` directory contains:

```
build/
├── BUILD_INFO.txt                    # Build metadata and versions
├── TrafficLayer.exe                  # Core interface
├── CoordMerge.exe                    # Controller
├── DriverModel_RealSim.dll           # VISSIM interface, default (int API, VISSIM 2021+)
├── DriverModel_RealSim_legacy.dll    # VISSIM interface, legacy (long API, VISSIM ≤ 2020)
├── RealSimSocket.mexw64              # MATLAB MEX file
├── RealSim*.m                        # MATLAB helper scripts
├── CommonLib/                        # Helper libraries and headers
│   ├── *.h                           # Header files
│   └── *.cpp                         # Source files (selective)
└── CarMaker/                         # CarMaker components
    ├── CM11/                         # CarMaker 11.1.2 build
    │   ├── CarMaker.win64.exe
    │   └── libcarmaker4sl.mexw64
    ├── CM13/                         # CarMaker 13.1.3 build
    │   ├── CarMaker.win64.exe
    │   └── libcarmaker4sl.mexw64
    └── libRealSimDsLib_*.a           # dSPACE libraries (if built)
```

## Automated Release CI & Proprietary Binaries Bundle

FIXS releases are produced by a GitHub Actions pipeline (`.github/workflows/release.yml`) so downstream consumers always get a complete, consistent zip without a developer hand-running the full build. The automation boundary is drawn at **source visibility**:

- **Public source → built on the hosted runner every push** (`windows-2022`, matching the VS 2022 generator; `windows-latest` ships VS 2026 and is incompatible): yaml-cpp, `TrafficLayer.exe`, `CoordMerge.exe`, `VirtualEnvironment.lib` (SDK-free on the 0.9.0 train, #174), and all `Carla/` + `CommonLib` Python. No private submodule, no token.
- **Licensed-toolchain source → built manually on a licensed workstation**: the VISSIM DriverModel DLLs, CarMaker executables + CM4SL MEX, dSPACE libraries, and the MATLAB MEX. These need installed proprietary toolchains, so the hosted runner cannot build them. They ship as a prebuilt **bundle** that CI overlays into the build.

The hosted path is made proprietary-aware by the `RS_FIXS_AUTOMATION` environment flag, which tells `dispatch.bat` to skip the licensed steps (3 / 4a / 4b / 5) and consume the downloaded bundle instead.

### Rolling release channels

| Branch | Rolling prerelease | Notes |
|--------|--------------------|-------|
| `main` | `latest` | current train |
| `dev_v0.9.0` | `v0.9.0-alpha` | 0.9.0 train (SDK-free VirtualEnvironment, #174) |

On every push to a release branch the pipeline builds the public core, overlays the matching proprietary bundle, packs one canonical zip, and (re)publishes the rolling prerelease anchored to that commit. Pull requests build + package only (no publish), and upload the zip as a workflow artifact for inspection.

### The proprietary-binaries bundle

The proprietary binaries live in a per-key GitHub Release on this repo, `Binaries-<key>`, where `<key>` is the **`ProprietaryFiles` submodule commit SHA** (first 12 chars; see `scripts/dispatch/bundle_key.ps1`). The key pairs a bundle to the exact proprietary source it was built from. CI reads the pinned submodule SHA from the FIXS tree (`git ls-tree HEAD ProprietaryFiles`), downloads `Binaries-<key>`, unzips it into `build/`, and packs the release zip — no private submodule checkout or cross-repo token needed.

A PR check, **`bundle-guard`** (`.github/workflows/bundle-guard.yml`), makes it impossible to merge a PR that moves the `ProprietaryFiles` pointer unless the matching `Binaries-<key>` is already published — so a "bumped the SHA but forgot to rebuild the binaries" mistake cannot reach a release branch.

### Publishing a new bundle (when ProprietaryFiles changes)

Whenever `ProprietaryFiles` moves, a fresh bundle must be built and published on a **licensed workstation** before the FIXS submodule bump can merge:

1. **Sync the submodule** to the target proprietary commit, and **clean the tree** so a crashed or partial build can't be masked by a stale artifact:
   ```batch
   git submodule update --init --force ProprietaryFiles
   ```
   Remove prior build outputs first — `build/`, the `x64/` output dirs, and the CarMaker `src*/Release/` obj dirs. Otherwise a stale output gets copied into `build/` and hides a failed rebuild.

2. **Full dispatch** — no `RS_FIXS_AUTOMATION`, so the proprietary steps run:
   ```batch
   dispatch.bat
   ```
   Confirm `scripts/dispatch/build_summary.log` shows **zero failures** and every proprietary artifact in `build/` is **freshly timestamped**. (A transient `CL.exe` crash — `exit code -1073741819` / `0xC0000005`, seen on Win11 24H2 — can fail one target while the rest succeed; the clean tree makes that show up as a *missing* file rather than a stale one.)

3. **Pack + publish** the bundle:
   ```powershell
   powershell -File scripts\dispatch\pack_binaries.ps1 -Publish
   ```
   This zips the proprietary subset of `build/` into `fixs-binaries-<key>.zip` + `manifest.json` and creates the `Binaries-<key>` prerelease on `ORNL-Real-Sim/FIXS`.

4. **Bump the FIXS submodule pointer** to the merged `ProprietaryFiles` commit in a PR to the release branch. `bundle-guard` verifies `Binaries-<key>` exists; once it's green (and reviewed), merge. The push then triggers the pipeline, which overlays the new bundle and publishes the complete rolling zip.

> **Order matters:** publish the bundle (step 3) **before** opening the submodule-bump PR, or `bundle-guard` will (correctly) block the merge.

### Notes

- **Bundle contents** (the files `pack_binaries.ps1` collects): both VISSIM DLLs (`DriverModel_RealSim.dll` + `_legacy.dll`), per-CM `CarMaker.win64.exe` + `libcarmaker4sl.mexw64`, the dSPACE `libRealSimDsLib_*.a`, and `RealSimSocket.mexw64`.
- **Versioning** uses `git describe --match 'v[0-9]*'` so the rolling non-semver tag can't shadow the semver tag (which previously fell back to `0.0.0`). `CommonLib/RealSimVersion.h` is regenerated at build time by a pre-build step in `TrafficLayer.vcxproj`.
- **`pack_binaries.ps1 -Publish`** works under both PowerShell 7 (`pwsh`, used by CI) and stock Windows PowerShell 5.1.

## Development Builds

For active development, you can build individual components without running the full dispatch. This is faster and allows focused debugging.

### Building Individual Components

Each component has its own build script in `scripts/dispatch/`:

#### 1. External Libraries Only
```batch
scripts\dispatch\1_external_libraries.bat
```
Builds yaml-cpp. Only needed once or after clean builds.

#### 2. Core Components Only
```batch
scripts\dispatch\2_core_components.bat
```
Builds TrafficLayer.exe, CoordMerge.exe, VirtualEnvironment.lib. Run this when modifying core C++ code.

#### 3. VISSIM Components Only
```batch
scripts\dispatch\3_vissim_components.bat
```
Builds VISSIM driver model DLLs. Run this when modifying VISSIM interface code.

#### 4. CarMaker Components Only
```batch
powershell -ExecutionPolicy Bypass -File scripts\dispatch\4a_carmaker_components.ps1
```
Builds CarMaker executables. Automatically:
- Reads CarMaker versions from `dependencies.yaml`
- Generates BuildConfig Python files
- Builds for each detected version
- Run this when modifying CarMaker integration code

#### 5. dSPACE Libraries Only
```batch
powershell -ExecutionPolicy Bypass -File scripts\dispatch\4b_carmaker_dspace.ps1
```
Builds dSPACE libraries for CarMaker HIL integration. Run this when modifying dSPACE interface code.

#### 6. MEX RealSimSocket Only
```batch
powershell -ExecutionPolicy Bypass -File scripts\dispatch\5_mex_realsim_socket.ps1
```
Builds MATLAB MEX file for socket communication. Run this when modifying MATLAB interface code.

### Common Development Workflows

**Scenario 1: Modified TrafficLayer C++ code**
```batch
scripts\dispatch\2_core_components.bat
```

**Scenario 2: Modified VISSIM driver model**
```batch
scripts\dispatch\3_vissim_components.bat
```

**Scenario 3: Modified CarMaker User.c or integration**
```batch
powershell -ExecutionPolicy Bypass -File scripts\dispatch\4a_carmaker_components.ps1
```

**Scenario 4: Testing full release package**
```batch
dispatch.bat
```

## Debug vs Release Configuration

The build system supports both Debug and Release configurations.

### Release Configuration (Default)

Release builds are optimized for performance and distribution:
- Compiler optimizations enabled
- Debug symbols minimal or stripped
- Default for `dispatch.bat`
- Used for distributable builds in `build/` directory

### Debug Configuration

Debug builds include full debugging information:
- No compiler optimizations
- Full debug symbols
- Easier to debug with Visual Studio debugger
- Larger executable sizes

### Switching to Debug Configuration

#### For Full Dispatch:
Edit `scripts/dispatch/dispatch.bat` line 17:
```batch
set "RS_BUILD_CONFIG=Debug"
```

#### For Individual Components:
Each component script checks the `RS_BUILD_CONFIG` environment variable:

```batch
set RS_BUILD_CONFIG=Debug
scripts\dispatch\2_core_components.bat
```

Or pass configuration directly to msbuild:
```batch
msbuild TrafficLayer\TrafficLayer.sln /p:Configuration=Debug /p:Platform=x64
```

### External Libraries Debug Build

yaml-cpp is built in both Debug and Release configurations automatically:
- Release: `yaml-cpp.lib`
- Debug: `yaml-cppd.lib`

When building components in Debug mode, make sure to link against the Debug library (`yaml-cppd.lib`).

## Build System Architecture

### Script Organization

```
scripts/
├── initialize_fixs.ps1              # Fresh-clone setup (submodules, native deps, yaml-cpp)
├── generate_version.ps1             # Generate RealSimVersion.h from git tags
├── update_fixs.ps1                  # CONSUMER-side: install a published release (Windows)
├── update_fixs.sh                   # CONSUMER-side: install a published release (POSIX)
├── build_libsumo.ps1                # STANDALONE: Build libsumo DLLs from SUMO source
├── build_sumo_executables.ps1       # STANDALONE: Build SUMO executables from source
└── dispatch/
    ├── dispatch.bat                 # Main orchestrator
    ├── detect_tool_paths.ps1        # Auto-detection of tools
    ├── yaml_helper.ps1              # Parse dependencies.yaml
    ├── fetch_native_deps.ps1        # Acquire libsumo (required) + libcarla (optional)
    ├── pack_native_deps.ps1         # Pack/publish the native-deps release assets
    ├── libsumo_verify.ps1           # Shared libsumo bin/ load test (used by both)
    ├── 1_external_libraries.bat     # Build yaml-cpp
    ├── 2_core_components.bat        # Build TrafficLayer
    ├── 3_vissim_components.bat      # Build VISSIM DLLs
    ├── 4_virtual_environment.bat    # Build VirtualEnvironment.lib
    ├── 4c_carla_virenv.ps1          # Build VirCarlaEnv.exe (needs libcarla)
    ├── 5a_carmaker_components.ps1   # Build CarMaker executables
    ├── 5b_carmaker_dspace.ps1       # Build dSPACE libraries
    ├── 6_mex_realsim_socket.ps1     # Build MATLAB MEX file
    ├── 7_build_info.ps1             # Generate BUILD_INFO.txt
    └── 8_create_zip.ps1             # Pack the release zip
```

### Setup vs. build scripts

- **`initialize_fixs.ps1`** (in `scripts/`) prepares a checkout — submodules, native deps, yaml-cpp. Called by `dispatch.bat` as step 1; idempotent, so running it directly is fine too. See [First-Run Setup](#first-run-setup-fresh-clone).
- **`fetch_native_deps.ps1`** / **`pack_native_deps.ps1`** are the two halves of the native-deps channel: `fetch` pulls checksum-verified `libsumo-<ver>-windows-x86_64.zip` / `libcarla-<ver>-windows-x86_64.zip` from the rolling `fixs-native-deps` release (which also carries the `-linux-x86_64` assets, hence the suffix); `pack` builds and publishes those assets. Both call `libsumo_verify.ps1`, which **loads** every libsumo probe DLL rather than just checking that files exist — an incomplete `bin/` is otherwise invisible until runtime (#70).

### Standalone Utility Scripts

Two scripts in `scripts/` are standalone utilities not called by `dispatch.bat`:

- **`build_libsumo.ps1`**: Builds libsumo DLLs from SUMO source into `CommonLib/libsumo/`. Use this when the SUMO version in `dependencies.yaml` changes. Since #238 that directory is gitignored, so a rebuild is **not** the end of the job — publish the result so every other clone can fetch it: `pack_native_deps.ps1 -Component sumo -Publish`.
- **`build_sumo_executables.ps1`**: Builds the full SUMO application suite (sumo.exe, sumo-gui.exe, etc.) from source. Most users should download a pre-built SUMO release instead.

### Tool Auto-Detection

The `detect_tool_paths.ps1` script automatically locates installed tools:

**Visual Studio Detection:**
- Searches for VS 2022 (Community, Professional, Enterprise)
- Locates MSBuild.exe path
- Validates C++ workload installation

**MATLAB Detection:**
- Checks Windows registry: `HKLM:\SOFTWARE\MathWorks\MATLAB`
- Searches common installation paths
- Validates MEX compiler availability
- Prioritizes version from `dependencies.yaml`

**dSPACE Detection:**
- Searches Program Files for ConfigurationDesk
- Locates version matching `dependencies.yaml`
- Finds DsBuildLibrary.mk for library compilation

**CarMaker Detection:**
- Searches Program Files for IPG installations
- Identifies available CarMaker versions
- Matches against versions in `dependencies.yaml`

### BuildConfig Auto-Generation

CarMaker builds require a BuildConfig Python file that specifies compiler settings, include paths, and library dependencies. The build system automatically generates these:

**Process:**
1. `4a_carmaker_components.ps1` reads `dependencies.yaml`
2. For each CarMaker version (e.g., 11.1.2, 13.1.3):
   - Detects MATLAB version
   - Locates CarMaker installation
   - Generates `RS_CM{major}_{minor}_{patch}_BuildConfig_{matlab}.py`
3. BuildConfig files are created in `CarMaker/` directory
4. CarMaker uses these during compilation

**Generated BuildConfig Features:**
- Correct MATLAB version paths
- RealSim-specific defines (`RS_DSPACE`, `RS_DEBUG`)
- Include paths for VirEnv_Wrapper.h
- Links to libRealSimDsLib_*.a (for dSPACE builds)
- Architecture-specific settings (dsrtlx, dsrt64)

**Manual Modifications (Advanced):**
If you need custom BuildConfig settings, you can:
1. Generate BuildConfig using the script
2. Manually edit the generated Python file
3. Re-run CarMaker build

Note: Re-running `4a_carmaker_components.ps1` will regenerate and overwrite custom changes.

## Troubleshooting

### Build Failures

**Problem:** CMake not found
```
'cmake' is not recognized as an internal or external command
```
**Solution:** Install CMake and add to PATH, or restart command prompt after CMake installation.

---

**Problem:** MSBuild not found
```
'msbuild' is not recognized as an internal or external command
```
**Solution:**
- Ensure Visual Studio 2022 is installed with C++ workload
- Build scripts should auto-detect MSBuild
- Manually add to PATH: `%ProgramFiles%\Microsoft Visual Studio\2022\<Edition>\MSBuild\Current\Bin`

---

**Problem:** yaml-cpp.lib not found during TrafficLayer build
```
LINK : fatal error LNK1181: cannot open input file 'yaml-cpp.lib'
```
**Solution:** Build external libraries first:
```batch
scripts\dispatch\1_external_libraries.bat
```

---

**Problem:** MATLAB MEX build fails
```
Error: Could not find MATLAB installation
```
**Solution:**
- Ensure MATLAB is installed
- Update `dependencies.yaml` with correct MATLAB version
- Check MATLAB registry entries exist
- Try specifying MATLAB path manually in `5_mex_realsim_socket.ps1`

---

**Problem:** CarMaker build fails with missing include files
```
fatal error C1083: Cannot open include file: 'VirEnv_Wrapper.h'
```
**Solution:**
- Ensure VirtualEnvironment.lib was built: `scripts\dispatch\2_core_components.bat`
- Check that `CommonLib/VirEnv_Wrapper.h` exists
- Verify BuildConfig includes correct paths

---

**Problem:** dSPACE library build fails
```
Error: Could not find dSPACE ConfigurationDesk
```
**Solution:**
- This is optional - skip if you don't need dSPACE HIL
- Install dSPACE ConfigurationDesk matching version in `dependencies.yaml`
- Ensure DsBuildLibrary.mk exists in dSPACE installation

---

**Problem:** Debug/Release mismatch errors
```
LNK2038: mismatch detected for 'RuntimeLibrary'
```
**Solution:**
- Ensure all libraries built in same configuration (Debug or Release)
- If building Debug, rebuild external libraries in Debug
- Check that yaml-cppd.lib (Debug) or yaml-cpp.lib (Release) matches your build config

### Checking Build Logs

Build logs are located in `scripts/dispatch/`:

- **build.log**: Detailed compilation output from all build steps
- **build_summary.log**: High-level summary of each component's build status

Check these logs for detailed error messages if builds fail.

### Verifying Tool Detection

To verify that tools are correctly detected, run:

```batch
powershell -ExecutionPolicy Bypass -File scripts\dispatch\detect_tool_paths.ps1
```

This will print detected paths for Visual Studio, MATLAB, dSPACE, and CarMaker.

### Clean Builds

To perform a clean build:

1. Delete the `build/` directory
2. Delete component-specific output directories:
   - `TrafficLayer/x64/`
   - `VirtualEnvironment/x64/`
   - `ProprietaryFiles/VISSIMserver/x64/`
   - `CarMaker/CM*/src*/CarMaker.win64.exe`
   - `CarMaker/CM*/src*/libcarmaker4sl.mexw64`
3. Optionally delete external library builds:
   - `CommonLib/yaml-cpp/build/`
4. Run `dispatch.bat`

### Common Build Order Issues

Components must be built in this order due to dependencies:

1. External libraries (yaml-cpp) - Required by VirtualEnvironment
2. VirtualEnvironment.lib - Required by CarMaker
3. All other components - Can be built in any order

If you get linker errors, ensure you've built dependencies first.

### Getting Help

If you encounter build issues not covered here:

1. Check build logs: `scripts/dispatch/build.log` and `build_summary.log`
2. Verify tool versions match `dependencies.yaml`
3. Ensure all prerequisites are installed
4. Try a clean build
5. Contact: realsimxil@gmail.com
