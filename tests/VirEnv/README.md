# Verifying the Python VirEnvCore (#325)

Two implementations of one bridge have to agree, and #325 names where disagreement
is most expensive: **who owns the ego**. `carlaOwnsId` (TrafficHelper) and
`carlaOwnsEgo` (VirCarlaEnv) once answered that differently and produced a run
where TrafficLayer kept the ego on traffic-sim kinematics while a physics ego
free-ran with no anchor — 0.7 m of agreement for 30 s, then background traffic
teleported through the physics body at 10.34 m/s and separation ended at 124.8 m.
Two processes deciding one thing with no arbiter.

So parity is **demonstrated, not assumed**, in four stages. Each ends in a number.
Run them in order: each one needs more of the stack than the last, and if an early
one fails there is no point running the later ones.

| stage | needs | answers |
| --- | --- | --- |
| 1 | nothing | do the two cores agree on hand-authored scenarios? |
| 2 | a SUMO + TrafficLayer run | do they agree over a real corridor, 6501 exchanges? |
| 3 | the same | does attaching a bridge change the eco-driving result? |
| 4 | + a CARLA server | does the live Python bridge change it? |

---

## Stage 1 — the two cores, no simulator (seconds)

```bash
python -m pytest tests/VirEnv/ -q
```

**Pass:** `67 passed`.

That runs the Python core through the same two scenarios `replay_core.cpp` runs,
and asserts its verb transcript against a golden captured from the C++ binary
(`tests/VirEnv/golden/replay_core_cpp.json`) — step for step, character for
character. It also checks the Python `BridgeHelper` against CARLA's own
`standalone/run_synchronization/sumo_integration/bridge_helper.py`, the upstream
the C++ one was ported from, so agreeing with it is evidence about the port rather
than a restatement of it.

To confirm the guard can actually fail, break something and watch it name the step:

```bash
# in CommonLib/VirEnv/VirEnvCore.py, in the 1:1 branch of step 6:
#     out.gradeRad = nS.pose.gradeRad + 0.25
python -m pytest tests/VirEnv/test_core_parity.py -q
```

```
scenario1, step "step t=0.10 onUpdate=1": the two cores decided differently.
  only in the C++ golden: ['setVehiclePose 0 (10.000,0.000,0.100) hdg=90.000 grade=0.000']
  only in the Python core: ['setVehiclePose 0 (10.000,0.000,0.100) hdg=90.000 grade=0.250']
```

**If you also have MSVC**, rebuild the C++ side so the golden cannot go stale
against a change to `VirEnvCore.cpp`:

```cmd
tests\VirEnvCore\build_and_run.bat
```

**Pass:** `ALL VIRENVCORE GUARD TESTS PASSED`. `test_cpp_golden_is_current` then stops
skipping and regenerates the golden from the binary to compare.

---

## Stage 2 — both cores over a real corridor (~15 min)

Four hand-authored steps and two vehicles cannot exercise what actually diverges:
the tick an id appears, disappears, or is skipped. A recorded corridor can.

**Record the feed.** Start the MLK stack (this launches SUMO and TrafficLayer
itself), and attach the recorder on the bridge port. It needs **no CARLA** — it
stands in for the bridge and replies with nothing, which is what a render-only
bridge sends anyway.

```bash
cd <FIXS_Applications>/dev
CFG=apps/mlk_eco_driving/MLK_Sumo_Scenario/config_Sumo_Carla_ecoDriving.yaml

python apps/mlk_eco_driving/eco_driving_mpr_SUMO_noPython_MLK_debug_prepared.py \
       --endTime 29750 --runTag trace --trafficlayerConfig $CFG --fixsTimeout 0 &

python <FIXS>/tests/VirEnv/record_feed.py -f $CFG --port 440 --out mlk.trace
```

**Pass:** `[record] 6501 exchanges -> mlk.trace`.

**Replay it through both cores**, at 1:1 and with interpolation:

```bash
cd <FIXS>
for N in 1 4; do
  ./tests/VirEnvCore/replay_core.exe --trace mlk.trace --substeps $N --digest-out mlk.$N.cpp.json
  python tests/VirEnv/replay_core.py  --trace mlk.trace --substeps $N --digest-out mlk.$N.py.json
  python tests/VirEnv/compare_digests.py mlk.$N.cpp.json mlk.$N.py.json --label-a cpp --label-b py
done
```

**Pass:** `RESULT: identical decisions`, for both. Measured:

```
  exchanges    6501
  substeps     1                                    substeps     4
  OK   despawn    cpp=1356  py=1356                 OK   despawn    1356
  OK   pose       cpp=1259058  py=1259058           OK   pose       5036232
  OK   spawn      cpp=1547  py=1547                 OK   spawn      1547
  OK   tls        cpp=71511  py=71511               OK   tls        71511
  OK   digests   all 6501 ticks identical           OK   digests   all 26004 ticks identical
```

Without `replay_core.exe`, run the Python side alone and check its overall digest
is `72609cdeff1a4df4` at `--substeps 1` and `50633ff7424fe0f9` at `--substeps 4`,
for that trace.

### What "identical" does and does not mean here

Compared: the verb, the vehicle it applies to, the pose to three decimals, the
light bits, the ego id, per host tick. Not compared: which backend handle a
vehicle got, and the order of verbs within one step — both are C++ `unordered_map`
iteration order, which the standard leaves unspecified and an insertion-ordered
Python dict cannot reproduce. They are independent writes that CARLA batches
anyway. Every handle is rewritten to its owning vehicle id before the step is
sorted, so what survives the comparison is the decision.

---

## Stage 3 — attaching a bridge must not move the answer (~15 min)

The eco-driving result is the oracle: CARLA observes, SUMO and the controller
decide. A bridge that changes it is a participant, and that is the failure mode.

The run from stage 2 already produced one. Compare it:

```bash
cd <FIXS_Applications>/dev/apps/mlk_eco_driving
python tools/compare_to_reference.py MLK_Sumo_Scenario/MPR/<the trace run>
```

**Pass:**

```
  ego_speed_mps                0.000000     0.000000   never
  raw_eco_speed_mps            0.000000     0.000000   never
  final_sent_to_FIXS_mps       0.000000     0.000000   never
  dist2Stop_ft                 0.000000     0.000000   never
  ego_accel_mps2               0.000000     0.000000   never

  RESULT: identical to ..._0731_main_baseline on every compared column.
```

`max|diff| = 0.000000` on five columns × 6501 timesteps, first divergence "never".
Anything else means the bridge perturbed the run.

---

## Stage 4 — the live Python bridge (~20 min, needs CARLA)

```bash
# 1. CARLA, matching the client in the env (0.9.15):
#    a source build launches through the editor in -game mode
"<UE4>/Engine/Binaries/Win64/UE4Editor.exe" "<CARLA>/Unreal/CarlaUE4/CarlaUE4.uproject" \
    -game -carla-rpc-port=2000 -quality-level=Low

# 2. load the map
python -c "import carla; carla.Client('127.0.0.1',2000).load_world('mlk_untextured')"

# 3. the traffic stack (as stage 2), and then the bridge instead of the recorder
cd <FIXS_Applications>/dev
CFG=apps/mlk_eco_driving/MLK_Sumo_Scenario/config_Sumo_Carla_ecoDriving.yaml
TLS=apps/mlk_eco_driving/MLK_Sumo_Scenario/traffic_light_table.csv
RS_POSE_LOG=py_pose.csv python <FIXS>/Carla/VirEnv/mainVirCarla.py -f $CFG -t $TLS
```

Or, in one command, through the front door — `CarlaSetup.EnablePythonBackend: true`
selects this bridge, `false` selects `VirCarlaEnv.exe`, and `--engine` overrides:

```bash
run_cosim --app mlk_eco_driving --map mlk_untextured --config $CFG --engine py --fast
```

**Pass:** the same `compare_to_reference.py` output as stage 3 —
`max|diff| = 0.000000`, "never". Measured: 1,258,864 poses applied across 1547
vehicles, clean shutdown on `traffic simulator ended the run`.

`RS_POSE_LOG` writes the applied CARLA pose per SUMO id. The C++ driver writes the
same columns under the same variable, so a py run and a cpp run of one scenario
diff per `(simTime, id)` with no CARLA-readback confound.

### If the client will not connect

```
WARNING: Client API version = ...   Simulator API version = ...
Assertion failed: (_data.size() - _offset) % sizeof(T) == 0u
```

is a client/server version mismatch, not a bridge fault. Check which `carla` the
interpreter actually resolves — a `pip install --user` of a different CARLA lands
in `%APPDATA%\Roaming\Python\Python310\site-packages`, which precedes every conda
env on `sys.path` and shadows the correct one in **all** of them:

```bash
python -c "import carla; print(carla.__file__)"
```

It should be inside the env, not under `Roaming`. If it is not,
`python -m pip uninstall carla` removes the user-site copy; keep a different CARLA
version in its own env instead.

---

## The files

| file | role |
| --- | --- |
| `MockVirEnvBackend.py` | recording double; same transcript format as the C++ mock |
| `replay_core.py` | the scripted scenarios, and `--trace` for a recorded corridor |
| `record_feed.py` | attaches to a live TrafficLayer and writes a trace; `--probe <id>` reports the raw wire fields for one vehicle |
| `compare_digests.py` | diffs two digest files and names the first exchange that differs |
| `test_core_parity.py` | stages 1; the golden and the check that it is current |
| `test_bridge_helper.py` | the frame, blueprint and signal mappings against upstream CARLA |
| `golden/replay_core_cpp.json` | the C++ transcript, committed so stage 1 needs no MSVC |
