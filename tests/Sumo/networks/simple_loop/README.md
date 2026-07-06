# simple_loop — the one shared SUMO network + demand (FIXS #174)

A 200 m × 200 m rounded-rectangle loop (R15 corners). This is the **single home**
for the SUMO `simple_loop` network and demand; every SUMO-side test references it
(SUMO-CM, SUMO-Carla, SimpleEchoClient) rather than keeping a private copy.

## Geometry is unified — single source of truth

`simple_loop.net.xml` is **generated from** the canonical OpenDRIVE file
[`../../../Vissim/SimpleEcho/simple_loop.xodr`](../../../Vissim/SimpleEcho/simple_loop.xodr)
— the *same* xodr that `osc2cm` turns into the CarMaker road (`simple_loop.rd5`)
and that `Carla/load_opendrive_world.py` loads into Carla. So SUMO traffic, the
CarMaker road, and the Carla world are all in **one coordinate frame** (0–200),
which is what makes absolute-X/Y co-simulation land on-road across backends.

Regenerate the SUMO net if the xodr changes:

```cmd
netconvert --opendrive-files ..\..\..\Vissim\SimpleEcho\simple_loop.xodr ^
           --output-file simple_loop.net.xml
```

netconvert names the four straight edges `-1 -2 -3 -4` (loop order, node
4→1→2→3→4); the routes use those ids. If you regenerate and the ids change,
update both route files to match.

## Demand is ego-free; the ego is an overlay

The background demand is **ego-free** so one demand drives every backend without a
stray ego leaking across backends. The ego is supplied per backend:

| Backend | sumocfg | Ego source |
| --- | --- | --- |
| SUMO-CM | `simple_loop.sumocfg` | CarMaker **injects** `egoCm` via FIXS (no SUMO ego) |
| SUMO-Carla | `simple_loop_ego.sumocfg` | the `ego` overlay vehicle (Carla visualizes it) |
| SimpleEchoClient | `simple_loop_ego.sumocfg` | the `ego` overlay vehicle (Python echoes it) |

> Why: on the SUMO-CM path, CarMaker owns the ego (`egoCm`) and reacts to forwarded
> traffic (`Driver.Consider.Traffic = 1`). A SUMO-side `ego` baked into the shared
> demand reached CarMaker as a phantom traffic object on top of its own ego and
> pushed IPGDriver off a corner (~71 s abort). Splitting the ego into an overlay
> loaded only where the ego must be SUMO-driven removes that coupling.
>
> A real FIXS-injected Carla ego (dropping the overlay for Carla too) is deferred
> to the #174 PR4/5 backend work.

## Files

| File | Purpose |
| --- | --- |
| `simple_loop.net.xml` | SUMO network — **generated from `simple_loop.xodr`** (do not hand-edit) |
| `simple_loop.rou.xml` | Background demand: 40 looping `car` vehicles (`bg.*`), **ego-free** |
| `simple_loop.ego.rou.xml` | Ego overlay: one SUMO-driven `ego` (self-contained vType/route) |
| `simple_loop.sumocfg` | net + background only → SUMO-CM |
| `simple_loop_ego.sumocfg` | net + background + ego overlay → SUMO-Carla, SimpleEchoClient |
