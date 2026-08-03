# VirEnvCore — simulator-free guard (#174)

The **baseline that protects the `VirEnvHelper` → `VirEnvCore` extraction**: a test
that runs green with **no CarMaker, no Carla, no Carla server** installed. It pins
the bridge core's *decisions* (the verb calls in the canonical frame) so the
extract-class refactor can be proven behavior-preserving.

## Pieces

| File | Role |
| --- | --- |
| `../../CommonLib/IVirEnvBackend.h` | The backend-agnostic **verb interface** the core will call (SDK-free: no `CarMaker.h`, no Carla). Lifted from the seven-step skeleton in `VirEnvHelper.cpp`. |
| `MockVirEnvBackend.h` | An `IVirEnvBackend` **recording double** — bounded per-class handle pool (mimics CarMaker's pre-placed slots) + records every verb call as a diffable string. |
| `smoke_interface.cpp` | Standalone smoke: exercises every verb + spawn/despawn pool-exhaustion, prints the transcript. Stands in for CI until the core lands. |

## Build & run (SDK-free)

```cmd
build_and_run.bat
```
Finds VS 2022 via `vswhere`, compiles `smoke_interface.cpp` with `/std:c++17`, runs
it. Expected tail: `SMOKE PASS: IVirEnvBackend + MockVirEnvBackend compile & run SDK-free.`

## Where this is going (the meat of #174)

1. **[done]** `IVirEnvBackend` verb interface + `MockVirEnvBackend` + smoke.
2. **next** Extract `VirEnvCore` from `VirEnvHelper`: move the SDK-free orchestration
   (sockets, id↔handle map, interpolation, refresh gating, TLS lookup, heading→yaw)
   into a core that calls verbs through `IVirEnvBackend*`. The lib stops including
   `CarMaker.h`; the CarMaker-specific code (slot pool, `TrfObj->t_0[]`, anchor
   `Cfg.l/h/zOff`, `Traffic_Lights_*`, `tlsChar2CmState`, `Vehicle.*` readback)
   becomes `CarMakerBackend` (compiled in the CM project where `CarMaker.h` lives).
3. Replace this smoke with a **recorded-trace replay**: feed a captured FIXS traffic
   trace into `VirEnvCore` + `MockVirEnvBackend`, assert the verb transcript is
   unchanged vs the golden — the actual refactor guard.
4. `CarlaBackend` + thin `mainVirCarla`; then `CarlaSetup` `EgoDynamicsOwner` /
   `EgoControl` (mode B wiring).
