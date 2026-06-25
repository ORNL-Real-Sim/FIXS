# #177 — SUMO TraCI per-step cost: root cause (measured)

Companion to #168's `tests/Vissim/Probes/TrafficLayer_DSProxy_CMoffice/PERF_N_TRAFFIC.md`.
That study measured the CarMaker side (CM-core + `VirtualEnvironment.lib`). This one
measures the **SUMO/TraCI side**, which #168 did not, and which is where the
SUMO-CM slowdown actually lives.

All numbers are from the in-isolation harnesses in this folder — **no CarMaker,
no TrafficLayer, no socket to CM** — driving the exact `tests/Sumo/network/simple_loop`
scenario (40-vehicle ego-free loop demand, `--step-length 0.1`). They replicate the
CarMaker-side SUMO path from `CommonLib/TrafficHelper.cpp` faithfully.

Reproduce (SUMO 1.21):

```
pip install eclipse-sumo==1.21.0 libsumo==1.21.0 traci==1.21.0 sumolib==1.21.0
python bench_traci_sub.py        # in-process libsumo (compute floor)
python bench_traci_socket.py     # socket traci == libtraci == the shipping build
python micro_nexttls.py          # getNextTLS cost vs route length
python analyze_bench.py          # cost-vs-N slope (ms/veh) from the CSVs
```

Outputs land in `_perf/`.

---

## TL;DR

**The dominant per-step cost is `Vehicle::getNextTLS(vehId)`, called per traffic
vehicle per step inside `parserSumoSubscription` (`TrafficHelper.cpp` L1525) — NOT
the context subscription and NOT its variable list.** It costs **~2.5 ms per
vehicle per step** (≈88 ms/step at N=36, RTF≈1.1 at 10 Hz), and it scales
**linearly with the vehicle's upcoming route length**, which the SimpleLoop demand
inflates to ~400 000 edges via `repeat="100000"`. The traffic-light data it produces
is **never sent** to CarMaker in this scenario (no `signalLight*` field in
`VehicleMessageField`, `SynchronizeTrafficSignal: false`). It is pure waste.

A second, ~30× smaller waste is `getLeader(v,1000)` + `getSpeed(leader)` (L1512/1519),
~2.7 ms/step total, whose `precedingVehicle*` output is likewise never sent.

The issue's filed hypotheses (#1 trim the subscription variable list, #2 shrink the
radius) are **not** the cause: the subscription is ~free in-process and only ~1.6 ms/step
over the socket for all 22 vars at N=36 — ~55× smaller than `getNextTLS`.

**Why VISSIM is immune:** the VISSIM/DSProxy path (`recvFromVISSIM`, L1695) makes
**zero** per-vehicle TraCI calls — one socket `recvData` delivers every vehicle's
state pre-packaged from the driver-model DLL. The SUMO path layers N×(getNextTLS +
getLeader) route-lookahead queries on top of the subscription every step.

---

## Measurement 1 — variable list & radius are NOT the cost (in-process libsumo)

`bench_traci_sub.py`, steady-state N≈37, per-step total:

| config | sub vars | radius | total ms/step | per-veh |
|---|---:|---:|---:|---:|
| nosub | 0 | — | 0.048 | 1.3 µs |
| full | 22 | 500 | 0.360 | 9.7 µs |
| full_leader | 22 (+getLeader/veh) | 500 | 0.380 | 10.3 µs |
| **full_nexttls** | 22 (+getNextTLS/veh) | 500 | **90.42** | **2444 µs** |
| full_faithful | 22 (+both) | 500 | 91.83 | 2482 µs |

→ adding/removing subscription variables moves the needle by µs; adding the per-veh
`getNextTLS` call moves it by **90 ms**. (In-process = pure compute, no socket.)

## Measurement 2 — the real transport is the socket (libtraci)

`CommonLib/TrafficHelper.h` L10 leaves `ENABLE_LIBSUMO` **undefined**, so the build
compiles the **`libtraci`** namespace (L17) and `TrafficLayer.vcxproj` delay-loads
`libtracicpp.dll`. The shipping co-sim therefore talks to a **separate `sumo-gui.exe`
over TCP** (`run_sumo_cm_demo.bat`: `sumo-gui --remote-port 1337`); every individual
TraCI getter is a synchronous round-trip. `bench_traci_socket.py` (Python `traci` =
same socket client), steady-state N=36:

| config | step ms | subread ms | per-veh ms | total ms/step | RTF @10 Hz |
|---|---:|---:|---:|---:|---:|
| nosub | 0.09 | — | — | 0.09 | 1115× |
| sub_minimal (3 var) | 0.32 | 0.002 | 0.001 | 0.32 | 313× |
| sub_full (22 var) | 1.62 | 0.006 | 0.001 | 1.62 | 62× |
| leader | 1.60 | 0.007 | **2.74** | 4.34 | 23× |
| **nexttls** | 1.77 | 0.012 | **88.2** | **90.0** | **1.11×** |

Cost-vs-N slope (`analyze_bench.py`): nexttls **2.49 ms/veh**, leader 0.099 ms/veh,
sub_full 0.045 ms/veh.

Decomposition over the socket:
- `getNextTLS` 88 ms/step — **compute-bound** (same ~88 ms as in-process; the socket
  adds little because the returned TLS list is empty on this net).
- `getLeader`+`getSpeed` 2.7 ms/step — **round-trip-bound** (~75 µs/call × N).
- 22-var subscription serialization ~1.3 ms/step over the 3-var minimal — the issue's
  hypothesis #1, real but ~70× smaller than `getNextTLS`.

The issue measured ~7.5 ms/veh / 205 ms at N=27 vs the isolated 2.5 ms/veh / 67 ms
here. Same linear-in-N shape and same dominant term; the ~3× magnitude gap is the
issue's `sumo-gui` render overhead + CarMaker co-sim blocking (#171) on top.

## Measurement 3 — why `getNextTLS` is 2.5 ms: it walks the whole route ahead

`micro_nexttls.py`, single vehicle, in-process, getNextTLS cost vs route length:

| route `repeat` | route edges | getNextTLS / call |
|---:|---:|---:|
| 10 | 40 | 0.001 ms |
| 100 | 400 | 0.003 ms |
| 1 000 | 4 000 | 0.027 ms |
| 10 000 | 40 000 | 0.265 ms |
| **100 000** | **400 000** | **2.687 ms** |

Linear in upcoming-route length. `simple_loop.rou.xml` declares the background route
`repeat="100000"` so the 4-edge loop becomes a ~400 000-edge route, and `getNextTLS`
scans all of it (no TLS on the net → it walks to the end and returns empty). 2.69 ms
matches the 2.49 ms/veh field slope.

---

## Root cause (causal chain)

1. `simple_loop.rou.xml` background route uses `repeat="100000"` → each vehicle's
   route is ~400 000 edges.
2. `parserSumoSubscription` (`TrafficHelper.cpp` L1446) runs **per context vehicle per
   step** and unconditionally calls `getNextTLS(v)` (L1525) and `getLeader(v,1000)` +
   `getSpeed(leader)` (L1512/1519).
3. `getNextTLS` is O(upcoming-route length) ≈ O(400 000) ≈ 2.7 ms/call.
4. × N vehicles × every step → ~88 ms/step at N=36, **linear in N** → RTF collapses
   exactly as observed, worsening as traffic spawns in.
5. The `signalLight*` / `precedingVehicle*` fields these calls populate are **gated out
   of the wire** by `MsgHelper` (not in `VehicleMessageField`) — computed, never sent.

## Proposed minimal fix (NOT yet applied — research-mode per the issue)

Gate the two per-vehicle call blocks in `parserSumoSubscription` on whether their
output is actually requested, reusing the `VehicleMessageField_set` member that
TrafficHelper already builds (L401–405), the same way `MsgHelper` gates packing:

- skip `getNextTLS` unless any `signalLight*` field is requested **or** signal sync is
  on (the next-speed-limit path also keys off it);
- skip `getLeader`/`getSpeed` unless any `precedingVehicle*` field is requested.

Expected effect (from the table): SUMO-CM step cost drops from ~90 ms to ~1.6 ms
(the `sub_full` row), RTF 1.1 → ~62 at N=36 — real time restored. Trimming the
subscription variable list to the consumed set is an *optional* extra ~1.3 ms/step;
do it second, it is not the fix.

**Do NOT change:** the context-subscription mechanism, the radius, or the variable
list as the *primary* lever (measured ~free / minor); the VISSIM path; behaviour when
signal/preceding fields *are* requested (gating must preserve it).

### Secondary / open
- `repeat="100000"` is itself pathological for any per-vehicle route-walk TraCI call.
  Even with the gate, a future signal scenario on this demand would be slow; consider a
  bounded lookahead or a smaller repeat. Tracked thinking, not part of the minimal fix.
- Whether fixing this restores lockstep and resolves the ego stop/spin/off-road
  (#177 secondary) needs a co-sim re-run after the gate lands.

---

## Optimization study (measured, `bench_opt.py` + `analyze_opt.py`)

Beyond gating, three independent levers were measured head-to-head over BOTH
transports at steady state (N=36). `getNextTLS(vehID)` has **no** bounded-lookahead
variant in the TraCI API (Vehicle.h L91), so it is optimized caller-side.

**Lever 1 — make `getNextTLS` efficient (cache on edge-change).** The next-TLS
*identity* changes only when the vehicle crosses to a new edge, which the subscription
already reports via `VAR_ROAD_ID`. So call `getNextTLS` only when `VAR_ROAD_ID`
changes; between edge changes reuse the identity and recompute distance O(1) from the
subscribed lane position. Drops calls/step from 36.4 (every veh every step) to 0.41.

**Lever 2 — trim the subscription to the consumed 12 vars** (from 22).

**Lever 3 — transport: in-process libsumo vs socket libtraci** (`ENABLE_LIBSUMO`).

| config | transport | getNextTLS | vars | calls/step | mean ms | p95 ms | RTF |
|---|---|---|---|---:|---:|---:|---:|
| T_base **(current build)** | socket | every step | full | 36.4 | 93.7 | 106.9 | 1.1× |
| T_cached | socket | cached | full | 0.41 | 2.69 | 6.17 | 37× |
| T_opt | socket | cached | consumed | 0.41 | 2.08 | 5.53 | 48× |
| T_floor *(O(1)-map ceiling)* | socket | none | consumed | 0 | 1.36 | 1.91 | 74× |
| L_base | in-proc | every step | full | 36.4 | 95.2 | 103.7 | 1.1× |
| L_cached | in-proc | cached | full | 0.41 | 1.40 | 4.96 | 71× |
| **L_opt** | in-proc | cached | consumed | 0.41 | 1.27 | 4.83 | 79× |
| **L_floor** *(O(1)-map ceiling)* | in-proc | none | consumed | 0 | **0.29** | **0.33** | **345×** |

Read-out:
- **`getNextTLS` cache = the big win:** 93.7 → 2.69 ms on the *current* socket build
  (35×), real time restored, GUI kept. No SUMO change.
- **Subscription trim** is a real but secondary ~0.5 ms (socket) / ~0.1 ms (in-proc).
- **Transport:** in-process libsumo removes ~1 ms of per-step TCP tax — socket step
  alone is ~1.0–1.8 ms even with zero per-veh calls (localhost TraCI round-trip +
  result serialization), vs ~0.17–0.35 ms in-process. This is what gets the pipeline
  comfortably under the 1 ms CarMaker step.
- **Cache vs O(1) map (tail latency):** the cache still pays one 2.7 ms `getNextTLS`
  walk per edge-change → p95 ~5 ms spikes (the route is 400k edges). The proper
  **compute-once-per-route + O(1) distance** map removes the spike entirely:
  in-process p95 0.33 ms (L_floor) vs 4.83 ms (L_opt). For a 1 ms CM thread the tail,
  not the mean, is what overruns — so the O(1) map is the *proper* form, the cache is
  the *quick* form.

**libsumo feasibility caveat (Windows):** `libsumo.start(["sumo-gui", ...])` prints
*"Libsumo on Windows does not work with GUI, falling back to plain libsumo"* — so
adopting in-process libsumo **loses the SUMO GUI on Windows** (this repo's target).
The build already has the `ENABLE_LIBSUMO` `#ifdef` (TrafficHelper.h L10–18): keep
libtraci for interactive/demo runs (GUI, ~2 ms/step after the getNextTLS fix), select
libsumo for headless/max-throughput runs (~0.3 ms/step, no GUI). It is a per-build
choice, not a one-way door.

### Recommended implementation (tiers, each independently verified above)
1. **getNextTLS: compute-once-per-route + O(1) distance** (proper) — or cache-on-edge-
   change (quick). 35× on the current build, GUI kept. Also gate it off entirely when
   no `signalLight*` field is requested / `SynchronizeTrafficSignal:false`.
2. **`getSpeed(leader)`**: drop it — the leader is itself a context vehicle whose speed
   was already received this step; look it up from the parsed buffer (0 calls).
3. **Trim `VehDataSubscribeList`** to the consumed set (~0.5 ms).
4. **Expose `ENABLE_LIBSUMO`** as a build/run option for headless runs (~5× transport,
   under the 1 ms CM step) — documented as GUI-less on Windows.
