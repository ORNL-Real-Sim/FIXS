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
omega          wheel spin state       [rad/s]
torque         tire-to-wheel torque   [Nm]     (identically -long_force * radius)
long_slip      PhysX contact-patch longitudinal slip
lat_slip       PhysX contact-patch lateral slip   [deg]
tire_load      Fz                     [N]
long_force     Fx                     [N]
lat_force      Fy                     [N]
tire_friction, normalized_tire_load, normalized_long_force, normalized_lat_force
```

and at vehicle level `speed`, `steer`, `throttle`, `brake`, `engine_rpm`, `gear`, `drag`.

`omega` and the tire force are the quantities the dyno loop needs and 0.9.15 does not
expose. On 0.9.15 the tire force has to be reconstructed from the vehicle's
longitudinal equation using an unverified drag area; the wheel speed is not
recoverable at all. On 0.9.16 both are read directly.

**§5 documents what these fields actually are, measured against a live server.** Two
of the names mislead: `torque` is not the drivetrain torque, and `long_slip` is not a
body-referenced slip ratio.

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

## 4. Running the 0.9.16 server

### 4.1 `make package` fails; launch through the editor binary instead

`make package` dies in `ScriptCompiler.CompileAutomationProjects`:

```
MSBuild.exe  ExitCode=-1073741819      (0xC0000005, access violation)
```

UE 4.26's AutomationTool crashes under the VS2022 toolchain that 0.9.16 forces
(§2.2). This blocks the *packaging* step only, not the build.

`make CarlaUE4Editor` succeeds (751/751 modules, exit 0). It produces
`CarlaUE4.exe`, but that is the **game** target and refuses to start:

```
Your application is built to load COOKED content. No COOKED content was found
```

Cooking is what `make package` would have done. The way around it is the way CARLA
itself launches — `Util/BuildTools/BuildCarlaUE4.bat:205` runs the **engine's**
editor binary, which loads uncooked content directly:

```
C:\src_ext\CarlaUnreal\Engine\Binaries\Win64\UE4Editor.exe ^
    C:\src_ext\CarlaSrc_0_9_16\Unreal\CarlaUE4\CarlaUE4.uproject ^
    -game -carla-server -carla-rpc-port=2000 -RenderOffScreen -quality-level=Low -nosound
```

This is enough for all API work. A distributable package still needs the
AutomationTool problem solved.

### 4.2 Runtime verification

Server and client both report `294096e-dirty`. Map `Town10HD_Opt`.

- 25/25 NPC vehicles spawned; 24/25 under Traffic Manager control after 3 s.
- Four separate synchronous-mode client sessions, `fixed_delta_seconds` 0.05 and
  0.005, ~5000 ticks total, **zero errors in the server log** and the server stayed
  up throughout.

---

## 5. What the telemetry actually contains

Measured against the live server, then traced back to source. The field names are
misleading in one important way.

### 5.1 `torque` is the tire reaction, not the drivetrain torque

`CarlaWheeledVehicle.cpp:864` copies `UVehicleWheel::DebugWheelTorque`, which is set
in the tire shader callback (`WheeledVehicleMovementComponent.cpp:92`) from the
output of `PxVehicleComputeTireForceDefault`. PhysX defines that output as
`-LongForce * WheelRadius`.

Measured over 1362 wheel-samples across drive and brake:

```
worst relative error between torque and (-long_force * radius) :  1.2e-7
```

So **`torque` is algebraically identical to `long_force`** and carries no extra
information. What it *is* is the tire-to-wheel reaction torque — our `Trq_T2W`.
That is the useful quantity for the dyno loop, but it is not the drivetrain torque,
and **0.9.16 still exposes no way to read the torque the drivetrain delivers to a
wheel.** Only `engine_rpm` and `gear`, from which it would have to be
reconstructed through the engine curve and gear ratios in `physics_control`.

Sign convention, consistent with `Trq_T2W` in
[`AxleDynoCarMakerCoupling.md`](AxleDynoCarMakerCoupling.md): negative under drive,
positive under braking.

### 5.2 `omega` is the genuine wheel-spin state

`CarlaWheeledVehicle.cpp:859` reads
`PVehicle->mWheelsDynData.getWheelRotationSpeed(w)` — the actual PhysX wheel DOF in
rad/s, not a kinematic reconstruction from body speed. This is the first time the
wheel-spin state is observable from the client API.

It is genuinely dynamic: at throttle 0.5 the Tesla Model 3 shows per-wheel `omega`
between 23 and 41 rad/s while `v/r` is 28 rad/s — real wheelspin, not noise.

### 5.3 `long_slip` is PhysX's per-wheel slip, not `(wr - v)/v`

`PxWheelQueryResult::longitudinalSlip` is computed at the contact patch. Under
clean straight-line braking it tracks `(wr - v_body)/v_body`; once the vehicle is
sliding, it does not, because the wheel's own contact-patch velocity is no longer
the body velocity. Do not treat it as a body-referenced slip ratio.

### 5.4 The longitudinal balance closes exactly

```
m * dv/dt  =  sum_i long_force_i  -  drag
```

Residual `r = m*a - (sum Fx - drag)`, binned by speed, 1600 ticks at 5 ms through
accelerate / coast / brake-to-standstill, zero collisions:

| speed [m/s] | n | median \|r\| [N] | max \|r\| [N] | median \|m·a\| [N] |
|---|---|---|---|---|
| 0.0 – 0.1 | 649 | 0.0 | 61254 | 0.0 |
| 0.1 – 0.5 | 8 | 2406 | 15751 | 46832 |
| 0.5 – 1.0 | 21 | 748 | 15496 | 4072 |
| 1.0 – 3.0 | 307 | 9.4 | 676 | 4256 |
| 3.0 – 6.0 | 615 | 1.8 | 89 | 2363 |

Above ~1 m/s the residual is 2–9 N against 2400 N of `m·a`, i.e. **0.1 %**. Nothing
is unmodelled: rolling resistance, driveline drag and tire losses are all already
inside `sum(long_force)`.

Below ~1 m/s it blows up. That is PhysX's standstill handling — the sticky-tire
*velocity constraint* replaces the tire force, so `long_force` there is a constraint
impulse and not a force. **This is exactly the regime where the dyno has to hand
over**, so it is a real constraint on the design, not a measurement artifact. See
[`CarlaDynoCoupling.md`](CarlaDynoCoupling.md) §7.

### 5.5 Aero drag — the 0.9.15 open question, settled

`WheeledVehicleMovementComponent.cpp:980-993`, reduced to SI (the `1.25/100^3`
kg/cm³ density, cm/s speed, cm² area and the `/100` in the telemetry getter all
cancel down):

```
drag [N] = 0.5 * 1.25 * Cd * A * v^2
```

Measured `drag / v^2` over 941 samples above 2 m/s: median **0.2358**, spread
3.2 % — constant, as the formula requires. So

```
Cd * A = 0.377 m^2
```

`physics_control.drag_coefficient` reports **0.150** for the Model 3, giving

```
ChassisDragArea = 0.377 / 0.150 = 2.515 m^2
```

`CarlaDynoCoupling.md` §4.1 inferred **2.52 m²** from the UE4 class defaults
(180 cm × 140 cm), on the argument that CARLA's Blueprints do not serialise those
properties and UE4 only stores non-defaults. It flagged the number *"not yet verified
in motion"* and left `Cd` symbolic. **That inference is now confirmed to 0.2 %**, and
the reasoning behind it — that CARLA never touches `ChassisWidth`/`ChassisHeight` —
holds.

Note that `Cd` was never the unknown: `physics_control.drag_coefficient` is exposed on
0.9.15 too. It is worth stating explicitly only because it is **0.15**, half the UE4
class default of 0.3, so anyone carrying the class default through by habit gets twice
the drag. Item C-1 in `CarlaDynoCoupling.md` §8 can be closed.

### 5.6 Net effect on the dyno bench

0.9.16 improves **measurement**, not **actuation**.

| | 0.9.15 | 0.9.16 |
|---|---|---|
| per-wheel tire force `Fx` | reconstruct from `m·vdot`, with an unverified drag | read directly, balance closes to 0.1 % |
| aero drag | inferred, wrong by 2× | reported per tick |
| wheel speed `omega` | not observable | read directly |
| tire-to-wheel torque `Trq_T2W` | not observable | read directly (= `-Fx·r`) |
| drivetrain torque to a wheel | not observable | **still not observable** |
| any per-wheel *command* | none | **still none** |

The injection problem is unchanged: telemetry is read-only, and `apply_control`
still offers one lumped throttle split by the differential. The design options in
`CarlaDynoCoupling.md` §7 stand as written. What 0.9.16 removes is the need to
*estimate* the quantity being injected against.

---

## 6. Open questions

1. Regenerate the vendored `CommonLib/libcarla`, or restructure to consume an external
   CARLA build? The 782 MB checked-in copy is the main migration cost and the main
   argument for restructuring.
2. Does the BoundingBox change alter behaviour at any of the 11 use sites?
3. Do we need per-wheel *command* as well as per-wheel *measurement*? Telemetry is
   read-only; `apply_control` remains one lumped throttle split by the differential.
   See `CarlaDynoCoupling.md` §7. §5.6 above narrows this to the actuation side only.
4. How does the dyno hand over below 1 m/s, where PhysX switches from tire forces to a
   standstill velocity constraint and `long_force` stops being a force (§5.4)?
5. `make package` — solve the UE 4.26 AutomationTool crash under VS2022 (§4.1) if we
   ever need a distributable 0.9.16 build rather than a source launch.
6. Report the four build defects (§2.3 a, b, e, f) upstream to CARLA.
