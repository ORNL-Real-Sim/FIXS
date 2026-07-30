# JunctionStopTest — minimal junction verification of the CM traffic-light stop rule (FIXS #172)

A clean 2-edge + 1-junction scene built to settle, without corridor confounds, **what CarMaker
actually requires for the ego to stop at a red light**, and whether a **far-side** signal head
(across the junction) still stops the ego.

Scene (netconvert → osc2cm): a signalized junction `jC`; the ego drives the main road straight
through, **W → jC → E**, on lane-path chain `15` (approach, RL.1) → `218` (connector) → `109`
(departure, RL.95). `build_junction_test.py` forces the straight controller (31) to **static
red**, puts a `DrvStop` on the approach referencing it, and varies only the head location.

## Result (controller forced red; ego must stop if the mechanism fires)

| Variant | Signal head | Ego |
|---|---|---|
| `approach` | straight head on the approach edge (RL.1) | **stops at s≈99.9** ✓ |
| `nohead`   | no head referencing the controller anywhere | **runs the red** (never stops) |
| `farside`  | head on the **departure edge across the junction** (RL.95); approach has none | **stops at s≈99.9** ✓ |

## Conclusions

1. **Both a `DrvStop` marker and a signal head (referencing the same controller) are required.**
   A `DrvStop` alone (no head) does **not** stop the ego — so the IPGDriver-manual wording
   *"respects traffic light controllers with associated stop markers"* is incomplete: a head
   must also exist.
2. **The head's location is free** — approach or far-side (across the junction) both stop the
   ego. So **true far-side (US-style) signal heads are achievable** with the ego still stopping.
3. This corrects an earlier wrong conclusion ("far-side breaks the stop, 1/6 crossings"): that
   was an implementation bug in the *corridor* relocation (undivided-2-way collinear matching
   dropped/mis-placed some controllers' heads), not a CarMaker constraint.

## Run

```bat
python build_junction_test.py approach   :: (or nohead / farside)
CM_Office.exe -projectdir <CM13_proj> -cmd "SaveMode save" -run junction_approach
python parse_erg.py <...junction_approach_*.erg>
```

Files: `nodes.nod.xml` / `edges.edg.xml` (SUMO net) → `junction.net.xml` / `junction.xodr`
(netconvert) → `junction.rd5` (osc2cm) → `junction_<variant>.rd5` + TestRun (build script).
`build_junction_test.py`, `parse_erg.py`, `junction.xosc`.
