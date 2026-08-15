# CATK / AlpaSim world model — established facts (#189)

Hard-won findings from debugging the SUMO ↔ CATK co-simulation. **Read this before
changing `backends/alpasim.py`, `catk_map.py`, or any signal handling.** Several of
these took many wrong turns to establish; a few are the *opposite* of what the code
or intuition suggests. Claims here are backed by a reproducible experiment, named
per section.

---

## 1. What we run, and what is actually ours

| piece | whose |
|---|---|
| `catk_v120` checkpoint + `alpasim_trafficsim.catk.model_adapter.CATK` | NVIDIA AlpaSim |
| upstream training/preprocessing reference (`NVlabs/catk`, cloned to `C:/src_git/WM_CATK_upstream`) | NVIDIA |
| `env_data` construction, map conversion, write-back, handback | **ours** |

We call `CATK.create_model_input()` + `CATK.inference()` directly. We do **not** use
AlpaSim's `CATKSceneAdapter` (needs USD/`pxr` + `alpasim_runtime`, not installed) nor
trajdata's `map_adapter` — but our `catk_map.py` deliberately reproduces the latter's
layer split and segmentation.

---

## 2. CATK **can** simulate signalized intersections — do not claim otherwise

On AlpaSim's own ClipGT scene `026d6a39` (`src/utils/tests/data/xodr_artifact/`), with
**no SUMO involved**, CATK launches a genuinely dead-stopped 3-car queue at a signal:

| start | queue speed at start | GT travel | CATK travel | launches |
|---|---|---|---|---|
| 9.5 s | 0.22 m/s | 54 m | 0.1–20 m | 2/6 |
| 10.0 s | 0.29 m/s | 59 m | 6.7–42 m | **5/6** |

It does this with `light_type = 0` everywhere (AlpaSim's runtime hardcodes it), i.e.
**from motion cues in the history, not from a signal channel.**

> Any conclusion of the form "CATK can't do signalized intersections, so SUMO must own
> the signals" is refuted by this experiment. Do not propose it.

**Watch for a confound:** at `start=10.5` the queue is *already rolling* (0.81 and
1.46 m/s), so "launch" there is just continuation. Check the speed at the chosen start
before interpreting. Also, tracks become valid mid-clip — at t=8.0 s tracks 168/166
jump from a (0,0) placeholder, producing a fake **302 m/s**. Always verify
`valid_mask` and the start-time speed.

---

## 3. The real defect was ours: SUMO emits **bit-exact zero** motion

Every vehicle CATK ever saw came from tracked real-world data, where a stopped car
still moves ~2–7 cm/frame. SUMO stops vehicles at mathematically exact zero. An
all-zero 16-frame history is outside the training distribution and the model holds the
queue indefinitely.

Proven both directions:

- our int_east snapshot (t=175.4, red, green at 180): queue never launches in 25 s
  (mean 0.07–0.08 m/s), across seeds, with the correct multi-layer map and correct
  signal painting;
- **freezing the ClipGT history** on the scene that otherwise launches 8/10 drops it
  to **0/10** — same defect, same symptom, on *their* data.

**Fix (in our adapter, not the simulator):** perturb only the history we *feed* the
model; the write-back is untouched. `--obs-noise` in the harness,
`ObservationNoise` in config.

| σ | result on the int_east snapshot |
|---|---|
| 0 | never launches |
| 0.02 m | launches, 0 red-light violations |
| 0.05 m | launches, 0 red-light violations |
| 0.10 m | launches, 1 violation across 3 seeds |

**Gate the noise to near-static vehicles only** (`ObservationNoiseMaxTravel`, default
0.5 m of travel over the whole 1.6 s history). Only bit-exact-stopped vehicles need it.
Applying it to movers corrupts their heading: at 2 m/s a frame is 0.2 m, so 5 cm of
noise is a quarter of the signal. Measured >90 deg heading flips: vanilla 0%,
WM without noise **18%**, WM with UNGATED noise **29%**, WM with gated noise: per-step
yaw back to 2-9.5 deg.

**Zero-mean noise, not a forward creep.** A constant creep (`--creep 0.5`) also
launches the queue (3/3) but makes vehicles **run the red** (crossings at t=176.9 and
177.3, 3 s early) and drift up to 3.7 m off-lane — because a creep tells the model
"I am rolling" while noise only makes the vehicle look *tracked*.

---

## 4. The map must be AlpaSim's multi-layer format

CATK builds map tokens from **3 consecutive polyline points** and embeds a per-token
layer identity (`pl_type`). Lane centrelines alone strip junction semantics.

Same scene, same seeds, only the map swapped (`tools/wm_sumo_format_probe.py`):

| map | queue launches |
|---|---|
| native 8-layer | 8/10 |
| our old `lane_centers`-only | 2/10 |
| ours + native map | 8/10 (matches native seed-for-seed) |

Ego-centering and the constant-velocity ego plan were **exonerated** by the same
bisection (2/10 and 1/10 — no change).

`catk_map.py` does: `net.xml --netconvert--> .xodr --trajdata--> 8 layers`, plus
`waitlines` and `intersection_areas` synthesised from the SUMO net (netconvert emits
neither). Verified 0.00 m median alignment to the source net; junction lanes present.
Point spacing must be ~1 m (CATK's own `map_resample_interval_m`); a fixed point
count gave ~20 m spacing on long lanes and broke lane-following.

**Apply CATK's own map budget** (`map_adv_max_*`: lane 1000 / road_boundary 750 /
other 250). It lives in the `scene_adapter` path we bypass, so we must apply it
ourselves — without it the map grows with agent count and inference degrades
74 ms → 3.8 s.

---

## 5. Traffic-signal encoding (upstream, authoritative)

From `NVlabs/catk` `src/data_preprocess.py` — the training-side convention:

- enum (`_polygon_light_type`, line 51):
  `NO_LANE_STATE=0, LANE_STATE_UNKNOWN=1, LANE_STATE_STOP=2, LANE_STATE_GO=3, LANE_STATE_CAUTION=4`
- the state is attached to the **whole controlled lane** and repeated onto **every
  3-point token of that lane** (lines 158–163; `smart/utils/preprocess.py:151`) — not a
  radius around a stop point. `src/utils/vis_waymo.py` draws it exactly that way, which
  is why the CATK website renders signals as coloured lane polylines.
- only the state at `current_time_index` is stamped and held for the whole rollout
  (line 462) — training never showed a light *changing* mid-rollout.

**`light_type` is LIVE, not dead.** Toggling it shifts map-encoder features by up to
1.06 absolute / **9.5 % relative**. An earlier "bit-identical output ⇒ disconnected"
claim was an artifact of testing under **greedy decoding** (`num_k=1`): argmax over a
discrete token vocabulary quantises away sub-margin logit shifts. Test the continuous
features, or sample.

**But it is not the launch mechanism here.** Forcing STOP vs GO on the queue's own
lanes changes nothing about whether it launches. Signal compliance in this checkpoint
comes from the motion context, not the token.

---

## 6. Bugs we introduced and fixed (do not reintroduce)

| bug | symptom | fix |
|---|---|---|
| `setSpeed` never released on handback | TraCI `setSpeed` **pins** speed until `setSpeed(-1)`. A vehicle handed back below `MinControlSpeed` crawled forever at its last command. 30/36 vehicles pinned; corridor median 1.58 vs vanilla 13.02 m/s | `TrafficHelper` tracks `WorldModelHeldIds` and releases with `setSpeed(-1)` |
| `SamplingTopK` set on the wrong object | the sampler lives on the **SMART instance inside** the CATK wrapper; the old assignment silently created a dead attribute, so "greedy" never took effect | `self.model.model.validation_rollout_sampling.num_k` |
| signal painted on the wrong lane, unresampled | painted `linkIndex 4` = `fromLane 0` (y=308) while the queue is at y=304.8/301.6 (links 5/6); 2 points on a 113 m lane ⇒ nothing tagged | `controlled_lanes()` — every controlled lane, approach + internal, resampled to 1 m, each with its own linkIndex |
| garbage-token teleport | CATK occasionally emits 85–130 m/s; `moveToXY` applied it as a ~21 m jump, stranding a car mid-junction | clamp displacement to `vmax*dt` per sub-step |
| non-finite pose written to SUMO | `moveToXY` with NaN/inf kills **TrafficLayer silently** -- no message in stdout or TrafficLayer.err, SUMO stops being stepped, the GUI just sits there and it looks like the sim is "super slow". Deterministic: died at sim t=34.30 every run once observation noise was fed in | guard every pose for finiteness before returning it; fall back to holding the current pose |
| unbounded map | inference 74 ms → 3.8 s as agents accumulated | per-group caps (§4) |

**Do not use `SamplingTopK: 1`.** Now that the path works, greedy is deterministic and
reliably locks up a stopped queue (0.02 m/s, 75 % stopped on ClipGT). Use the model
default (top-5).

---

## 7. Performance — the map is not the bottleneck

RTX 4000 Ada, 4 → 40 agents:

| component | cost |
|---|---|
| CATK inference | **70–81 ms/call, flat in agent count** |
| map crop | 2–21 ms |
| tokenise | 5 ms |
| map build (netconvert + trajdata) | 1.5 s, **once** |

At stride 3 that is ~3.6× faster than real time. Apparent co-sim slowness was **GPU
contention from stale processes** (GPU at 11.9/12.3 GB → inference degraded to 3.8 s)
plus sumo-gui rendering. **Kill leftover `python`/`TrafficLayer`/`sumo-gui` before a
run.**

---

## 8. Signal timing of the demo net (measured, not inferred)

`int_east` westbound (`linkIndex` 4/5/6) — cycle **90 s**: green 0–38, yellow 38–41,
red 41–90 ⇒ **red→green at t = 0, 90, 180, 270**. Reading the `tlLogic` phase strings
in `net.xml` gives the wrong answer (it misses the yellow phases netconvert inserts);
get the truth from SUMO with an additional file:

```xml
<additional><timedEvent type="SaveTLSStates" source="int_east" dest="tls_state.xml"/></additional>
```

Vanilla's own discharge is the baseline: startup lost time 0.1–0.2 s, headways
**2.5–2.7 s then 1.8 s**, every cycle.

---

## 9. Methodology — statistics alone will mislead you

A rollout in which **nothing moved at all** was reported by the checker as
"0/15 vehicles flagged, 0 red-light violations" — all true, and useless. Corridor
`%time<3 m/s` also matched vanilla while the traffic was frozen, because vanilla has
real signal queues.

**Always render and look**: `tools/wm_render_frames.py` writes a CATK-vs-vanilla PNG
frame grid. Judge behaviour at the red→green transition (queue discharge headways),
not with aggregate speed buckets.

---

## 10. Tools

| tool | purpose |
|---|---|
| `tools/wm_snapshot_rollout.py` | snapshot vanilla SUMO at a junction → roll CATK closed-loop with **no SUMO**; flags teleports/yaw/off-lane/red-running. The main debug loop. |
| `tools/wm_render_frames.py` | CATK vs vanilla PNG frame grid — **look at this** |
| `tools/wm_crosscheck.py` | runs our snapshot through the *reference* rollout loop; proved our loop is not at fault |
| `tools/wm_sumo_format_probe.py` | A/B of env_data formatting on the ClipGT queue scene |
| `tools/wm_map_convert_check.py` | verifies net → xodr → layers alignment |
| `tools/wm_native_reference.py` | native ClipGT rollout (from Codex) |

Typical: `python wm_snapshot_rollout.py --at 175.4 --half-extent 200 --seconds 25 --seed 0 --obs-noise 0.05 --light`

---

## 11. Open

- **Heading flips are a pre-existing WM defect**: 18% of vehicles show a >90 deg
  flip in one 0.1 s step even with no observation noise (vanilla: 0%). Mostly at
  junctions at 1.6-2.2 m/s. Gating the noise reduced it but did not remove it. This
  is very likely the "orientation messed up" seen in the GUI. NOT yet root-caused.
- Discharge is still messier than vanilla — vehicles bunch near the junction where
  vanilla has cleared. Quantify with headways, not eyeballing.
- Two seeds errored in the σ=0.02/0.05 sweep (no vehicles parsed) — undiagnosed.
- Upstream WOMD-trained checkpoint (trained *with* real signal states) is not public:
  obtained by emailing the author with Waymo Open Dataset registration proof.
