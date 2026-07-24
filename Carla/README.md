# `Carla/` — FIXS Carla automation tools

Shippable helpers for launching CARLA and staging a scenario world, so probes
and users get one-click runs instead of hand-running scenario-specific bats.
Generalized from `FIXS_SUMO_CARLA_ANL` (`apps/roosevelt/helper_scripts`),
de-hardcoded from Roosevelt / `C:\src_ext` paths — everything machine-specific
lives in `carla.env`.

> **Scope (now):** lean + **signal-free**. The OpenDRIVE-standalone path
> (SimpleLoop) needs no map package and no traffic lights. The traffic-light
> placement / Unreal tooling (`auto_place_tls`, `unreal_placing_tls`, …) is being
> finalized in `FIXS_SUMO_CARLA_ANL` first and will be ported into `Carla/tls/`
> once it ships there — one port, no divergence.

## One-time setup

```cmd
copy carla.env.example carla.env
```
Edit `carla.env` for your box: `UE4_ROOT`, `CARLA_UPROJECT` (your source build —
`C:\src_git\Carla_0915\Unreal\CarlaUE4\CarlaUE4.uproject`), and `PY` (a Python
with the `carla` 0.9.15 package). `carla.env` is gitignored.

## Files

| File | Purpose |
| --- | --- |
| `carla.env.example` | Template for per-machine settings (copy → `carla.env`) |
| `launch_carla.bat` | Launch CARLA from the source build via `UE4Editor.exe … -game` (no editor GUI). `launch_carla.bat [map]` |
| `wait_for_rpc.ps1` | Block until the CARLA RPC port accepts connections (exit 0 up / 1 timeout) — gate launchers on it |
| `load_opendrive_world.py` | Import a `.xodr` into the running server via `generate_opendrive_world()` (no map package). Coordinate-matched to the CarMaker xodr |
| `set_spectator_view.py` | Aim the spectator camera (top-down at a point, or `--follow <role_name>`) |
| `tls/` | *(later)* traffic-light placement/extract, ported from ANL after it ships there |

## Typical flow (signal-free, OpenDRIVE standalone)

```cmd
REM 1. start CARLA server (source build, -game)
launch_carla.bat

REM 2. wait until it accepts RPC
powershell -NoProfile -ExecutionPolicy Bypass -File wait_for_rpc.ps1 -Port 2000

REM 3. replace the world with the scenario road (coordinate-matched .xodr)
%PY% load_opendrive_world.py ..\tests\Vissim\SimpleEcho\simple_loop.xodr --sync --delta 0.1

REM 4. (optional) frame the camera
%PY% set_spectator_view.py --x 100 --y 100 --z 150

REM 5. now start SUMO + TrafficLayer + VirCarlaEnv (see the SUMO-Carla probe)
```

The SUMO-Carla SimpleLoop probe
([../tests/Sumo/Probes/TrafficLayer_SUMO_Carla/](../tests/Sumo/Probes/TrafficLayer_SUMO_Carla/))
wires steps 1–5 into its one-click launcher + self-checking verify.
