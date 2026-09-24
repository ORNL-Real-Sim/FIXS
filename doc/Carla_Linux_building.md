# CARLA 0.9.15 Linux Build Guide

This guide documents building **CARLA 0.9.15** (Unreal Engine 4 line) from source on **Linux (Ubuntu)**, and is the Linux counterpart to `Carla_Windows_building.md`. It focuses on the **common issues and workarounds** encountered when building this 2023 release on a modern (2026) toolchain, since the official docs assume the original environment. Refer to the [official Linux build docs](https://carla.readthedocs.io/en/0.9.15/build_linux/) for the baseline procedure.

> **Why 0.9.15 (UE4) and not 0.10.x (UE5)?** The 0.9.x line has the mature, stable client API and the complete tooling ecosystem (scenario_runner, leaderboard, ROS bridge, Traffic Manager). The UE5 0.10.x line is still under active development (glitchy, incomplete feature parity). Stay on 0.9.15 for anything that must be reliable and reproducible.

> **✅ Verified build environment:** **Ubuntu 22.04.5 LTS** (x86-64, kernel 6.8, glibc 2.35) · Unreal Engine 4.26 · clang 10.0.1 (UE4-bundled) · CMake 3.28.3 · Ninja 1.10.1 · GNU Make 4.3 · Python 3.10.20 (conda/Miniconda 25.1.1). CARLA tag **0.9.15.2**.

---

## Part One: Prerequisites

The versions in **bold** are what this build was verified on (✅ this machine); others are minimum/standard.

| Requirement | Version / Notes |
|---|---|
| OS | **Ubuntu 22.04.5 LTS** (x86-64) ✅ — kernel 6.8.0-60-generic, glibc 2.35 |
| Unreal Engine | **4.26** (CARLA fork), pre-built. Set `UE4_ROOT` to its path. |
| Compiler | **clang 10.0.1** — bundled inside UE4 (`.../v17_clang-10.0.1-centos7/...`); no system clang needed |
| CMake | **3.28.3** (3.28+) |
| Ninja | **1.10.1** |
| Make | **GNU Make 4.3** |
| Python | **3.10.20** (CARLA 0.9.15 ships an official `cp310` wheel) |
| conda | **25.1.1** (Miniconda) — used for the Python env |
| git, wget, tar, xz | standard |
| Disk | ~40 GB free for the build |
| Display | An X/Wayland display is required to *launch* the editor (`DISPLAY` set) |

**Verify the toolchain:**

```bash
echo "$UE4_ROOT"                       # must point at a built UnrealEngine 4.26
ls "$UE4_ROOT/Engine/Binaries/Linux/UE4Editor"   # engine must already be built
cmake --version
ninja --version
python3 --version
```

### Set up the Python environment (conda)

CARLA 0.9.15's pinned `requirements.txt` predates Python 3.10 (`numpy==1.18.4`, ancient `Shapely`), so use a dedicated env with bumped pins. Create `carla-0915-py310.yml`:

```yaml
name: carla-0915
channels:
  - conda-forge
dependencies:
  - python=3.10
  - pip
  - pip:
      - setuptools
      - wheel
      - auditwheel
      - networkx
      - numpy==1.23.5      # repo pins 1.18.4 (incompatible with py3.10)
      - distro
      - Shapely            # repo pins 1.6.4 (won't build on py3.10)
      - psutil
      - py-cpuinfo
      - pygame
      - python-tr
      - future
      - matplotlib
      - Pillow
      - open3d
      - pyyaml
```

```bash
conda env create -f carla-0915-py310.yml
conda activate carla-0915
conda install -n carla-0915 -y sqlite   # needed for the PROJ fix (Issue 3)
```

---

## Part Two: Build CARLA

> **Run every build command inside the activated `carla-0915` env, and with a large finite stack** (see **Issue 4** — this is the single most important Linux-specific fix):
>
> ```bash
> conda activate carla-0915
> ulimit -s 131072        # 128 MB; do NOT use 'unlimited'
> ```

### Step 1 — Clone the repository

Ensure tag **0.9.15** (or `0.9.15.2`) is checked out.

```bash
git clone --depth 1 -b 0.9.15 https://github.com/carla-simulator/carla.git
cd carla
```

### Step 2 — Get assets (map/mesh content)

Run `./Update.sh` (Linux equivalent of `Update.bat`) to fetch the content pack into `Unreal/CarlaUE4/Content/Carla`. The asset version is listed in `Util/ContentVersions.txt`.

> **Common Issue — Asset download fails / content is root-owned.** If the content was extracted by another process (e.g. a root/Docker step), `Unreal/CarlaUE4/Content` may be owned by `root`, which blocks the editor build later (see **Issue 7**). If `Update.sh` fails, manually download the matching package from `Util/ContentVersions.txt` and extract it into `Unreal/CarlaUE4/Content/Carla`.

### Step 3 — Compile the Python API client

```bash
make PythonAPI
```

This runs `Setup.sh` (downloads/builds Boost, rpclib, gtest, Recast, libpng, xerces, Eigen, sqlite, PROJ), builds LibCarla + osm2odr, then packages the wheel/egg into `PythonAPI/carla/dist/`.

Building this 2023 release today triggers the following issues, **in order**:

---

#### Issue 1 — Boost 1.80.0 download fails (dead mirror)

`Setup.sh` downloads Boost from `boostorg.jfrog.io`, which was **decommissioned** — it now returns an 11 KB HTML landing page instead of the tarball, and extraction fails with `gzip: stdin: not in gzip format`.

**Solution:** point at the permanent official archive. In `Util/BuildTools/Setup.sh`:

```bash
# was: wget "https://boostorg.jfrog.io/artifactory/main/release/${BOOST_VERSION}/source/${BOOST_PACKAGE_BASENAME}.tar.gz"
wget -nc "https://archives.boost.io/release/${BOOST_VERSION}/source/${BOOST_PACKAGE_BASENAME}.tar.gz" || true
```

(Or manually download `boost_1_80_0.tar.gz` from `https://archives.boost.io/release/1.80.0/source/` into `Build/`.)

---

#### Issue 2 — libpng 1.6.37 download fails (SourceForge 404)

`Setup.sh` fetches libpng from a SourceForge URL that now **404s** (1.6.37 was moved to *older-releases*).

**Solution:** in `Util/BuildTools/Setup.sh`:

```bash
# was: LIBPNG_REPO=https://sourceforge.net/projects/libpng/files/libpng16/${LIBPNG_VERSION}/libpng-${LIBPNG_VERSION}.tar.xz
LIBPNG_REPO=https://download.sourceforge.net/libpng/libpng-${LIBPNG_VERSION}.tar.xz
...
wget -nc ${LIBPNG_REPO}
```

---

#### Issue 3 — PROJ `proj.db` generation segfaults

Building PROJ generates `proj.db` by feeding a ~10 MB SQL script to `sqlite3`. The statically-built sqlite3 (and any sqlite3 run under a small stack) **segfaults** parsing it — sqlite's parser recurses, and a stack below ~1 MB crashes it.

**Solution:** use an on-PATH sqlite3 via a wrapper that raises the stack first. In `Util/BuildTools/Setup.sh`, near the sqlite section, generate a wrapper and point PROJ's `EXE_SQLITE3` at it:

```bash
REAL_SQLITE_EXE=$(command -v sqlite3 || true)
[[ -z "${REAL_SQLITE_EXE}" ]] && REAL_SQLITE_EXE=${SQLITE_EXE}
PROJ_SQLITE_EXE=${CARLA_BUILD_FOLDER}/host-sqlite3
cat > "${PROJ_SQLITE_EXE}" <<EOF
#!/usr/bin/env bash
ulimit -s unlimited 2>/dev/null || ulimit -s 65536 2>/dev/null || true
exec "${REAL_SQLITE_EXE}" "\$@"
EOF
chmod +x "${PROJ_SQLITE_EXE}"
```

Then replace `-DEXE_SQLITE3=${SQLITE_EXE}` with `-DEXE_SQLITE3=${PROJ_SQLITE_EXE}` in **both** PROJ `cmake` calls. (Requires `sqlite` installed in the env — see Part One.)

---

#### Issue 4 — Segfaults from a constrained stack (`ulimit -s`)  ⚠️ most important

Several build steps **segfault (SIGSEGV)** when the build runs under a small stack — PROJ's `proj.db`, Python's `setup.py`, and especially the **`lld` link of `libUE4Editor-Carla.so`** (Part Three). This is the key Linux-specific gotcha.

**Counter-intuitive detail:** `ulimit -s unlimited` makes it *worse* for the multithreaded linker. On glibc, when `RLIMIT_STACK` is *unlimited*, pthread worker threads fall back to a tiny ~2 MB default stack — so threaded `lld` overflows and crashes.

**Solution:** run all build commands under a **large finite** stack:

```bash
ulimit -s 131072        # 128 MB — gives BOTH main and pthread worker threads a big stack
make PythonAPI
```

For the editor build, wrap the whole invocation:

```bash
bash -c 'ulimit -s 131072; exec make launch'
```

---

#### Issue 5 — clang rejects conda Python's gcc flag (`-fno-merge-constants`)

The extension compile fails with:

```
clang++: error: optimization flag '-fno-merge-constants' is not supported [-Werror,-Wignored-optimization-argument]
```

Conda's Python was compiled with **gcc**, so its baked-in `sysconfig` `CFLAGS` include `-fno-merge-constants`. `distutils` passes those to **clang** (UE4's compiler), and CARLA builds with `-Werror`, making the unsupported-flag warning fatal.

**Solution:** keep `-Werror` but silence that one diagnostic. In `PythonAPI/carla/setup.py`, in the Linux `extra_compile_args` list:

```python
'-Werror', '-Wno-ignored-optimization-argument',
'-Wall', '-Wextra', '-Wpedantic', '-Wno-self-assign-overloaded',
```

---

#### Issue 6 — PROJ static lib not built with `-fPIC` (shared-lib link error)

Linking `libcarla.so` fails:

```
libproj.a(geodesic.c.o): relocation R_X86_64_32S against `.rodata.cst8'
  can not be used when making a shared object; recompile with -fPIC
```

CARLA's PROJ build sets `-DCMAKE_CXX_FLAGS="...-fPIC"` but **not** the C flags, so PROJ's C sources (geodesic.c, the WKT parsers) aren't position-independent — and they get linked into the shared `libcarla.so`.

**Solution:** add C-side PIC to **both** PROJ `cmake` calls in `Util/BuildTools/Setup.sh`:

```bash
-DCMAKE_C_FLAGS="-fPIC" -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
```

Then clean and rebuild PROJ so the change takes effect:

```bash
rm -rf Build/proj-src Build/proj-install Build/proj-install-server Build/proj-7.2.1.tar.gz
rm -f PythonAPI/carla/dependencies/lib/libproj.a
```

**Verify the client:**

```bash
pip install --force-reinstall PythonAPI/carla/dist/carla-0.9.15-cp310-cp310-linux_x86_64.whl
python3 -c "import carla; print(carla.Client('localhost',2000).get_client_version())"
# -> 0.9.15.2
```

---

## Part Three: Build & launch the CarlaUE4 editor

```bash
conda activate carla-0915
bash -c 'ulimit -s 131072; exec make launch'      # build server LibCarla + plugin + UE4 project, then open the editor
```

This compiles the Carla UE4 plugin and the `CarlaUE4Editor` target (long the first time), then runs `Launching UE4Editor...` and opens the GUI. First launch also compiles shaders, so the window takes a few minutes to appear.

> **Issue 4 applies here too** — without `ulimit -s 131072`, `lld` segfaults at `[N/M] Link (lld) libUE4Editor-Carla.so`. Use the finite-stack wrapper shown above.

---

#### Issue 7 — `Permission denied` creating content directories

The build fails late with:

```
mkdir: cannot create directory '.../Unreal/CarlaUE4/Content/Carla/ExportedMaps': Permission denied
```

The `Content/` tree is owned by **root** (e.g. extracted by a root/Docker step), so the build (running as your user) can't write into it.

**Solution** (needs sudo):

```bash
sudo chown -R "$USER":"$USER" /path/to/carla/Unreal/CarlaUE4/Content
```

Then re-run `make launch`. Since everything is already compiled, it just creates the directory and opens the editor.

---

## Quick reference — full build from scratch

```bash
# 0. one-time env
conda env create -f carla-0915-py310.yml
conda activate carla-0915
conda install -n carla-0915 -y sqlite

# 1. apply the Setup.sh / setup.py patches (Issues 1,2,3,5,6) — see above

# 2. build, always with the finite stack
ulimit -s 131072
make PythonAPI                          # client wheel/egg
sudo chown -R "$USER":"$USER" Unreal/CarlaUE4/Content   # Issue 7, if content is root-owned
bash -c 'ulimit -s 131072; exec make launch'            # editor

# later: relaunch the editor without rebuilding
make launch-only
```

## Part Four: the C++ client SDK that FIXS ships (#65)

`VirCarlaEnv` links the **C++** client, not the Python one. CARLA publishes no
release assets at all, so that SDK exists only inside a built source tree, at
`PythonAPI/carla/dependencies/{include,lib}` — which is why FIXS packs and
publishes it itself:

```bash
scripts/pack_native_deps.sh --component carla --carla-root ~/carla_0.9.15 --publish
```

Consumers never build it:

```bash
scripts/fetch_native_deps.sh --component carla        # ~3 s, SHA-256 verified
```

### The client does NOT need UE4

`Setup.sh` exports `CC`/`CXX` from `$UE4_ROOT` because the **server** half must
be built with UE4's clang against libc++. The **client** half is plain
libstdc++ (`Util/BuildTools/BuildLibCarla.sh` has separate CLIENT and SERVER
paths, and only SERVER uses `LIBCPP_TOOLCHAIN_FILE`), and CARLA's own
`Examples/CppClient/Makefile` builds it with a system `g++`. So once the
third-party dependencies exist in `Build/`, the client rebuilds with the
distro's compiler and no engine in sight.

### Build it with the OLDEST supported distro's compiler ⚠️

This is the trap, and it cost a published asset. A dev box that builds CARLA
usually has a **newer GCC installed than the distro ships** — this one is
Ubuntu 20.04 with `libstdc++6` 13.1.0. UE4's clang-10 then compiles against
GCC 13's libstdc++ headers, and the result references symbols focal's stock
`libstdc++.so.6` (10.5.0) does not export:

| symbol | introduced | from |
|---|---|---|
| `std::__exception_ptr::exception_ptr::_M_addref` / `_M_release` | GCC 12 | `libcarla_client.a` (boost::asio) |
| `std::__throw_bad_array_new_length` | GCC 11 | `librpc.a` |

The asset linked on the packing box **and** on 22.04/24.04, and failed to link
on stock 20.04 — the one distro it exists to serve. `/etc/os-release` says
20.04 either way, so the distro guard cannot see this.

Rebuild both archives with the baseline compiler (no UE4, no `make setup`; the
`Build/` dependencies are reused as-is):

```bash
# 1. LibCarla client, with focal's stock g++-9
cat > /tmp/tc-gcc9.cmake <<'EOF'
set(CMAKE_C_COMPILER /usr/bin/gcc-9)
set(CMAKE_CXX_COMPILER /usr/bin/g++-9)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -pthread -fPIC -O3 -DNDEBUG" CACHE STRING "" FORCE)
EOF
cmake -S ~/carla_0.9.15 -B /tmp/libcarla-build -G Ninja \
      -DCMAKE_BUILD_TYPE=Client -DLIBCARLA_BUILD_RELEASE=ON \
      -DLIBCARLA_BUILD_DEBUG=OFF -DLIBCARLA_BUILD_TEST=OFF \
      -DCMAKE_TOOLCHAIN_FILE=/tmp/tc-gcc9.cmake \
      -DCMAKE_INSTALL_PREFIX=/tmp/libcarla-install
cmake --build /tmp/libcarla-build -j"$(nproc)" && cmake --install /tmp/libcarla-build

# 2. rpclib, the same fork/tag Setup.sh uses, also with g++-9
git clone -b v2.2.1_c5 https://github.com/carla-simulator/rpclib.git /tmp/rpclib-src
sed -i s/"3.9.0"/"3.5.0"/g /tmp/rpclib-src/CMakeLists.txt
cmake -S /tmp/rpclib-src -B /tmp/rpclib-build -G Ninja \
      -DCMAKE_C_COMPILER=/usr/bin/gcc-9 -DCMAKE_CXX_COMPILER=/usr/bin/g++-9 \
      -DCMAKE_CXX_FLAGS="-fPIC -std=c++14" -DCMAKE_INSTALL_PREFIX=/tmp/rpclib-install
cmake --build /tmp/rpclib-build -j"$(nproc)" && cmake --install /tmp/rpclib-build

# 3. drop both into the tree the packer reads
cp /tmp/libcarla-install/lib/libcarla_client.a /tmp/rpclib-install/lib/librpc.a \
   ~/carla_0.9.15/PythonAPI/carla/dependencies/lib/
```

The other shipped archives (Recast, Detour\*, DebugUtils, boost) are clean as
built — they are C or use no post-GCC-10 libstdc++ entry points.

`pack_native_deps.sh` now enforces all of this: it screens the staged archives
for those symbol names and then **link-tests the payload inside a stock
`ubuntu:20.04` container**, which is the only place that can answer the
question (a static archive carries unversioned names, so probing on the packing
box passes even when the asset is unusable). With no container runtime it warns
loudly and packs anyway.

### Boost 1.81, deliberately

`Setup.sh` here is patched from 1.80.0 → 1.81.0 (Issue 1 above). The asset ships
its own boost headers, so it stays self-consistent even though the Windows SDK
is on 1.80; `dependencies.yaml` records it under `carla.boost`.

---

## Summary of fixes (7 issues, on par with the Windows guide)

| # | Stage | Fix |
|---|---|---|
| 1 | setup deps | Boost URL → `archives.boost.io` |
| 2 | setup deps | libpng URL → `download.sourceforge.net` |
| 3 | PROJ proj.db | `host-sqlite3` stack-raising wrapper |
| 4 | whole build | `ulimit -s 131072` (finite, **not** unlimited) |
| 5 | PythonAPI compile | `-Wno-ignored-optimization-argument` (gcc flag vs clang `-Werror`) |
| 6 | PythonAPI link | PROJ `-fPIC` / `POSITION_INDEPENDENT_CODE=ON` for C sources |
| 7 | editor launch | `chown` root-owned `Content/` to the build user |

Most of these are **"old release, modern toolchain" rot** (dead mirrors, newer CMake/setuptools/numpy) — the same class of problem the Windows guide documents, just spelled differently. The genuinely Linux-specific items are **Issue 4** (stack/threaded-lld) and **Issue 5** (UE4 clang vs conda gcc flags).
