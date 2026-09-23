# CARLA 0.9.16 Windows Build Guide

Practical guide for building CARLA **0.9.16** from source on Windows, written from a
build actually carried out at `C:\src_ext\CarlaSrc_0_9_16` (tag `0.9.16`).

The 0.9.15 guide is [`Carla_Windows_building.md`](./Carla_Windows_building.md), and the
Linux counterpart is [`Carla_Linux_building.md`](./Carla_Linux_building.md). **Read this
one instead of the 0.9.15 guide if you are building 0.9.16** — the toolchain changed and
most of the 0.9.15 workarounds no longer apply.

Official baseline: [CARLA 0.9.16 Windows build docs](https://carla.readthedocs.io/en/0.9.16/build_windows/).

> **Why we care about 0.9.16.** It adds `vehicle.get_telemetry_data()`, which reports
> per-wheel tire force, wheel speed, slip and load, plus the vehicle's aero drag. That is
> what the XIL dyno coupling needs — see
> [`xil/CarlaDynoCoupling.md`](./xil/CarlaDynoCoupling.md). Migration status for FIXS is
> tracked in issue [#319](https://github.com/ORNL-Real-Sim/FIXS/issues/319).

---

## 1. What changed from 0.9.15

| | 0.9.15 | 0.9.16 |
|---|---|---|
| Visual Studio | 2019 (`vc142`) | **2022** (`vc143`) — hardcoded, see §2.1 |
| Boost | 1.80.0 | 1.84.0 |
| Unreal Engine | CARLA fork of UE 4.26 | **same fork, no engine rebuild needed** |
| Python wheel build | `setup.py bdist_wheel` | `python -m build` |
| Content assets | `20231108_c5101a5` | `20250912_2171890` (20.1 GB) |

The single most useful thing to know: **the UE 4.26 fork does not change.** If you
already have `CarlaUnreal/UnrealEngine` (the `carla` branch) built for 0.9.15, point
`UE4_ROOT` at it and reuse it as-is.

---

## 2. Prerequisites

Same as 0.9.15 except the compiler:

- **Visual Studio 2022** with the C++ toolset and Windows 10/11 SDK. 0.9.15 needed
  VS2019; 0.9.16 does not build with it (§2.1).
- **Make 3.81** exactly — CARLA pins this version. Install via GnuWin32.
- **CMake** 3.15+ (3.31.9 verified).
- **7-Zip**, on `PATH` or installed to `%ProgramW6432%\7-Zip`.
- **Python 3.10** with `py` and `py3` resolving to it. The `py.ini` procedure in the
  0.9.15 guide §Part Two.6 still applies verbatim.
- **UE 4.26 CARLA fork** at `%UE4_ROOT%`.

Python packages, under Python 3.10 specifically:

```
py -3.10 -m pip install build distro wheel setuptools "numpy<2.0.0"
```

`build` is new for 0.9.16 (§3.3) and `numpy<2.0.0` is still required.

### 2.1 The VS2022 requirement is hardcoded

0.9.15 passed the boost toolset through a variable; 0.9.16 pins it:

```
0.9.15   Setup.bat  --boost-toolset $(TOOLSET)     configurable
0.9.16   Setup.bat  --boost-toolset msvc-14.3      hardcoded  (vc143 = VS2022)
```

So build with VS2022 and pass the matching generator (§3.1).

---

## 3. Build procedure

Run everything from a **VS2022 x64 Native Tools** environment. The CARLA Makefile does
not source `vcvars64.bat` itself.

Two Windows-specific traps before you start:

- **Do not drive the build from Git Bash.** Its POSIX `PATH` leaks into `cmd` and breaks
  `vcvars64.bat` with `Unknown parameter: path`. Use `cmd` or PowerShell.
- **If launching from PowerShell**, PowerShell sets
  `NoDefaultCurrentDirectoryInExePath=1`, which `cmd` inherits and which makes
  `call bootstrap.bat` fail even though the file is right there. Clear it first:

  ```bat
  set "NoDefaultCurrentDirectoryInExePath="
  ```

### 3.1 Dependencies

```bat
call "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat"
set "UE4_ROOT=C:\src_ext\CarlaUnreal"
cd /d C:\src_ext\CarlaSrc_0_9_16
make setup GENERATOR="Visual Studio 17 2022"
```

**`make setup` and `make PythonAPI` can exit 0 while having failed.** The batch wrappers
swallow the real exit code. Always verify the artifacts exist rather than trusting the
return code.

### 3.2 Patch: trailing-backslash quote escape (7 sites)

**Symptom**

```
'bootstrap.bat' is not recognized as an internal or external command
```

**Cause** The build directory is passed as `"C:\...\Build\"`. In batch, `\"` escapes the
quote rather than closing it, so `cd "%VAR%"` silently fails and the script then cannot
find the file it is about to run.

**Fix** Append a dot so the quote closes properly, in all 7 places:

```bat
cd "%BOOST_SRC_DIR%"      ->   cd "%BOOST_SRC_DIR%."
```

Files under `Util/InstallersWin/`: `install_boost.bat`, `install_chrono.bat`,
`install_fastDDS.bat`, `install_gtest.bat`, `install_recast.bat` (twice),
`install_rpclib.bat`.

> The 0.9.15 guide describes this same fix but gives the directory as
> `Util/InstallerWin`. The correct path is **`Util/InstallersWin`** (with the `s`).

### 3.3 Patch: boost bootstrap toolset

`Util/InstallersWin/install_boost.bat` calls `bootstrap.bat vc141` (VS2017), which is not
installed. Change to match the toolset the same file already defaults to:

```bat
call bootstrap.bat vc143
```

### 3.4 Patch: `python -m build` isolation

**Symptom** `make PythonAPI` exits `4294967295`.

**Cause** 0.9.16 builds the wheel with `python -m build`, which creates an isolated venv
whose bundled pip (23.0.1) fails against modern setuptools.

**Fix** `Util/BuildTools/BuildPythonAPI.bat` line 99 — build in the current environment:

```bat
python -m build --wheel --no-isolation --outdir dist\.tmp .
```

### 3.5 Patch: MSVC internal compiler error and COFF section limit

**Symptom 1**

```
boost/math/special_functions/trigamma.hpp(147): fatal error C1001: Internal compiler error.
```

MSVC 14.43 ICEs on boost 1.84's math headers under setuptools' default `/O2 /GL`.

**Symptom 2**, once that is past:

```
class.hpp(590): fatal error C1128: number of sections exceeded object file format limit
```

**Fix** Both in `PythonAPI/carla/setup.py`. setuptools injects its flags first and CARLA
appends after, so a later `/GL-` wins:

```python
'/DLIBCARLA_WITH_PYTHON_SUPPORT', '-DLIBCARLA_IMAGE_WITH_PNG_SUPPORT=true', '/MD',
# MSVC 14.43 ICEs (C1001) on boost 1.84 math headers under /GL.
'/GL-',
# boost.python generates more sections than the COFF limit allows
'/bigobj']
```

### 3.6 Build the Python client

```bat
make PythonAPI
```

Verify — do not trust the exit code:

```
PythonAPI\carla\dist\carla-0.9.16-cp310-cp310-win_amd64.whl
```

Install it and confirm the telemetry API is present:

```bat
py -3.10 -m pip install PythonAPI\carla\dist\carla-0.9.16-cp310-cp310-win_amd64.whl
py -3.10 -c "import carla; print(carla.Vehicle.get_telemetry_data, carla.WheelTelemetryData.omega)"
```

### 3.7 Content assets

`Util/ContentVersions.txt` maps versions to asset packages. For 0.9.16 it is
`20250912_2171890`. If `Update.bat` fails (it often does), download directly — 20.1 GB:

```
https://carla-assets.s3.us-east-005.backblazeb2.com/20250912_2171890.tar.gz
```

Extract into `Unreal/CarlaUE4/Content/Carla/`. The archive has **no top-level directory
prefix**, so it expands straight into that folder — do not create a nested one.

---

## 4. Running the server

### 4.1 `make package` does not work

```
ScriptCompiler.CompileAutomationProjects
  -> MSBuild.exe  ExitCode=-1073741819      (0xC0000005, access violation)
```

UE 4.26's AutomationTool crashes under the VS2022 toolchain that 0.9.16 requires (§2.1).
This is unresolved. It blocks producing a **distributable package**; it does not block
using CARLA from source.

### 4.2 Launch from source instead

```bat
make CarlaUE4Editor
```

This succeeds (751/751 modules). It produces `CarlaUE4.exe`, but that is the *game*
target and will refuse to start:

```
Your application is built to load COOKED content. No COOKED content was found
```

Cooking is what `make package` would have done. Launch the way CARLA itself does
(`Util/BuildTools/BuildCarlaUE4.bat:205`) — through the **engine's** editor binary, which
loads uncooked content directly:

```bat
"%UE4_ROOT%\Engine\Binaries\Win64\UE4Editor.exe" ^
    C:\src_ext\CarlaSrc_0_9_16\Unreal\CarlaUE4\CarlaUE4.uproject ^
    -game -carla-server -carla-rpc-port=2000 -RenderOffScreen -quality-level=Low -nosound
```

Drop `-RenderOffScreen` if you want a window.

### 4.3 Confirm it works

```python
import carla
c = carla.Client("127.0.0.1", 2000); c.set_timeout(10.0)
print(c.get_server_version(), c.get_client_version())   # both: 294096e-dirty
```

Verified on this build: `Town10HD_Opt`, 25 NPC vehicles spawned with 24 driving under
Traffic Manager, four synchronous-mode client sessions totalling ~5000 ticks, no errors
in the server log.

Probe scripts that exercise the server and the telemetry API live on the investigation
branch under `doc/xil/carla0916_probes/` — see issue
[#319](https://github.com/ORNL-Real-Sim/FIXS/issues/319).

---

## 5. Known-good configuration

| | |
|---|---|
| CARLA | tag `0.9.16`, reports `294096e-dirty` |
| Unreal Engine | CARLA fork of 4.26 at `C:\src_ext\CarlaUnreal` (shared with 0.9.15) |
| Visual Studio | 2022 Community, MSVC 14.43.34808 |
| CMake / Make | 3.31.9 / 3.81 |
| Python | 3.10.11 |
| Boost | 1.84.0 (`vc143`) |
| Assets | `20250912_2171890` |
| Source tree | `C:\src_ext\CarlaSrc_0_9_16` |

---

## 6. Upstream defects

Four of the patches above are CARLA bugs, not environment quirks, and are worth
reporting upstream:

- §3.2 trailing-backslash quote escape in `Util/InstallersWin/*.bat`
- §3.3 `bootstrap.bat vc141` against a `msvc-14.3` default
- §3.5 no `/GL-` guard for the boost 1.84 ICE
- §3.5 no `/bigobj` for boost.python
