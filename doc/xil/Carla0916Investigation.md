# CARLA 0.9.16 Investigation

Tracking issue: [#319](https://github.com/ORNL-Real-Sim/FIXS/issues/319) (Backlog milestone).
Companion to [`CarlaDynoCoupling.md`](CarlaDynoCoupling.md), which explains **why** 0.9.16
matters for the XIL bench.

**This branch is an investigation. Nothing here should land on a major branch until we
decide to migrate.**

---

## 1. Why 0.9.16

`get_telemetry_data` on the vehicle actor returns, **per wheel**:

```
omega          wheel speed            [rad/s]
torque         Trq_T2W                [Nm]
long_slip      kappa
lat_slip       alpha
tire_load      Fz                     [N]
long_force     Fx                     [N]
lat_force      Fy                     [N]
tire_friction, normalized_tire_load, normalized_long_force, normalized_lat_force
```

and at vehicle level `speed`, `steer`, `throttle`, `brake`, `engine_rpm`, `gear`, `drag`.

`omega` and `torque` are exactly the two quantities the dyno loop needs and 0.9.15 does
not expose. On 0.9.15 they must be reconstructed from the vehicle's longitudinal
equation, with an unverified drag area and an estimated wheel inertia. On 0.9.16 they
are read directly.

The server-side builder runs unconditionally — it reads `PxWheelQueryResult` and the
`UVehicleWheel` debug fields, with no dependence on `show_debug_telemetry` or the HUD.

---

## 2. Reproducible build recipe (Windows)

Built at `C:\src_ext\CarlaSrc_0_9_16`, tag `0.9.16`.

### 2.1 What did NOT need to change

**The UE 4.26 fork.** 0.9.16 uses the same `CarlaUnreal/UnrealEngine` `carla` branch we
already have built at `C:\src_ext\CarlaUnreal`. No engine rebuild.

Prerequisites already satisfied: `make` 3.81 (exact version CARLA pins), CMake 3.31.9,
7-Zip, Python 3.10.11 with `py` resolving to it.

### 2.2 Toolchain change

0.9.15 was built with **VS2019** — confirmed from the boost libraries in
`C:\src_ext\Carla\Build\boost-1.80.0-install\lib`, which are named `libboost_*-vc142-*`.

0.9.16 **hardcodes** the VS2022 toolset:

```
0.9.15   Setup.bat --boost-toolset $(TOOLSET)     configurable
0.9.16   Setup.bat --boost-toolset msvc-14.3      hardcoded  (= vc143 = VS2022)
```

So build with VS2022 and `GENERATOR="Visual Studio 17 2022"`. Boost is also newer:
1.84.0, against 1.80.0 for 0.9.15.

### 2.3 Patches required

All applied to the 0.9.16 tree; **none of these are needed for 0.9.15**.

**(a) `Util/InstallersWin/*.bat` — trailing-backslash quote escape.** The build dir is
passed as `"C:\...\Build\"`. In batch, `\"` escapes the quote, so `cd "%VAR%"` fails
silently and the script then cannot find the file it was about to run. Symptom:

```
'bootstrap.bat' is not recognized as an internal or external command
```

Fix, in 7 places — append a dot so the quote closes properly:

```
cd "%BOOST_SRC_DIR%"      ->   cd "%BOOST_SRC_DIR%."
```
Files: `install_boost.bat`, `install_chrono.bat`, `install_fastDDS.bat`,
`install_gtest.bat`, `install_recast.bat` (x2), `install_rpclib.bat`.

**(b) `install_boost.bat` — bootstrap toolset.** Calls `bootstrap.bat vc141` (VS2017),
which is not installed. Change to `vc143`.

**(c) Python `build` module.** 0.9.16 builds the wheel with `python -m build`, which was
absent: `No module named build.__main__`. Install for Python 3.10:

```
pip install build distro wheel setuptools "numpy<2.0.0"
```

**(d) `Util/BuildTools/BuildPythonAPI.bat` — isolated env.** `build` creates an isolated
venv whose pip (23.0.1) fails against setuptools 84, exiting `4294967295`. Add
`--no-isolation`:

```
python -m build --wheel --no-isolation --outdir dist\.tmp .
```

**(e) `PythonAPI/carla/setup.py` — MSVC internal compiler error.** MSVC 14.43.34808
ICEs compiling boost 1.84's math headers under setuptools' default `/O2 /GL`:

```
boost/math/special_functions/trigamma.hpp(147): fatal error C1001: Internal compiler error.
```

setuptools injects the flags first and CARLA appends after, so a later `/GL-` wins.

**(f) `PythonAPI/carla/setup.py` — COFF section limit.** boost.python then exceeds it:

```
class.hpp(590): fatal error C1128: number of sections exceeded object file format limit
```

Append `/bigobj`.

Items (a), (b), (e) and (f) are upstream defects worth reporting.

### 2.4 Correction to our existing build doc

`doc/Carla_Windows_building.md` says `Util/InstallerWin`; the real path is
`Util/InstallersWin`. Its boost-URL fix is now **obsolete** — 0.9.16 already ships
`archives.boost.io`. Its trailing-dot fix (#3) is still needed and is the same issue as
(a) above.

`doc/Carla_0915_Windows_Fixes.md` is a PDF with a `.md` extension whose entire content
is the string *"Placeholder for Carla 0915 Windows Fixes"*. Removed on `dev_v0.9.0`.

### 2.5 Content assets

`Util/ContentVersions.txt` gives `0.9.16: 20250912_2171890`. Direct download, 20.1 GB:

```
https://carla-assets.s3.us-east-005.backblazeb2.com/20250912_2171890.tar.gz
```

The archive has no top-level prefix, so it extracts straight into
`Unreal/CarlaUE4/Content/Carla/`.

---

## 3. Migration impact on FIXS

### 3.1 VirCarlaEnv (C++)

Every CARLA header `VirCarlaEnv` includes was diffed between the two trees.

| Header | Change | Affects us? |
|---|---|---|
| `client/Vehicle.h` | additive: `GetTelemetryData`, wheel pitch, bone transforms, `RestorePhysXPhysics` | no |
| `client/Actor.h` | additive: component / bone / socket accessors; `GetBoundingBox` became a real method | no |
| `client/Map.h`, `client/Waypoint.h` | additive: road marks, `IsRHT` | no |
| `client/World.h` | `SpawnActor` / `TrySpawnActor` gained a **defaulted** `socket_name` | no — source compatible |
| `client/Client.h` | `ReplayFile` gained a **required** `geom::Transform offset` | no — **0 call sites** |
| `trafficmanager/TrafficManager.h` | `SetKeepRightPercentage` **renamed** `SetKeepSlowLanePercentage` | not called by VirCarlaEnv |
| `Memory.h`, `geom/*`, `rpc/VehicleControl.h`, `rpc/TrafficLightState.h`, `client/TrafficLight.h`, `client/ActorList.h`, `client/BlueprintLibrary.h`, `client/ActorBlueprint.h`, `client/TimeoutException.h` | unchanged | no |

**No source change to VirCarlaEnv appears necessary.** It must be relinked against the
0.9.16 client library.

### 3.2 The vendored libcarla — the real cost

`CommonLib/libcarla` is a checked-in copy of the CARLA client library:

```
767 source files, 782 MB
version.h  ->  "0.9.15.2-3-g114e1f2fa-dirty"
```

Migrating means regenerating it. This is also where the `SetKeepRightPercentage` rename
actually lands (`trafficmanager/Parameters.{h,cpp}`, `TrafficManager.h`,
`TrafficManagerBase.h`).

### 3.3 Python

All 44 FIXS scripts that reference CARLA were scanned. Every `carla.<Class>` attribute
they use exists in 0.9.16.

Checked explicitly against the breaking items in the 0.9.16 changelog:

| Changelog item | FIXS exposure |
|---|---|
| `Sensor.is_listening` property -> method | **not used** |
| `waypoint.next` / `.previous` loop behaviour fixed | not used in Python; VirCarlaEnv uses only `GetWaypoint()` |
| `keep_right_rule_percentage` -> `keep_slow_lane_rule_percentage` | **not used** |
| BoundingBox synced server/client, gained `actor_id` | 11 use sites — **behavioural check needed** |
| `carla.command` now importable; `carla.ad` import change | not used |

---

## 4. Status

| | |
|---|---|
| `make setup` | clean, with the patches in §2.3 |
| `make PythonAPI` | `carla-0.9.16-cp310-cp310-win_amd64.whl` built |
| telemetry API in the built client | verified — `get_telemetry_data`, all 11 `WheelTelemetryData` fields |
| `make package` | see issue #319 |
| launch + traffic + live telemetry | see issue #319 |

---

## 5. Open questions

1. Regenerate the vendored `CommonLib/libcarla`, or restructure to consume an external
   CARLA build? The 782 MB checked-in copy is the main migration cost and the main
   argument for restructuring.
2. Does the BoundingBox change alter behaviour at any of the 11 use sites?
3. Do we need per-wheel *command* as well as per-wheel *measurement*? Telemetry is
   read-only; `apply_control` remains one lumped throttle split by the differential.
   See `CarlaDynoCoupling.md` §7.
4. Report (a), (b), (e), (f) upstream to CARLA.
