# SimpleLoop — shared SUMO scenario (FIXS #174)

A 200 m × 200 m rounded-rectangle loop (R15 corners), used as the **one shared
scenario** that drives every backend through the FIXS pipeline (SUMO-CM,
SUMO-Carla, and the VISSIM-CM probe).

## Geometry is unified — single source of truth

`simple_loop.net.xml` is **generated from** the canonical OpenDRIVE file
[`../../Vissim/SimpleEcho/simple_loop.xodr`](../../Vissim/SimpleEcho/simple_loop.xodr)
— the *same* xodr that `osc2cm` turns into the CarMaker road (`simple_loop.rd5`)
and that `Carla/load_opendrive_world.py` loads into Carla. So SUMO traffic, the
CarMaker road, and the Carla world are all in **one coordinate frame** (0–200),
which is what makes the absolute-X/Y co-simulation land on-road across backends.

Regenerate the SUMO net if the xodr changes:

```cmd
netconvert --opendrive-files ..\..\Vissim\SimpleEcho\simple_loop.xodr ^
           --output-file simple_loop.net.xml
```

netconvert names the four straight edges `-1 -2 -3 -4` (loop order, node
4→1→2→3→4); `simple_loop.rou.xml`'s `loop` route uses those ids. If you
regenerate and the ids change, update the route to match.

## Files

| File | Purpose |
| --- | --- |
| `simple_loop.net.xml` | SUMO network — **generated from `simple_loop.xodr`** (do not hand-edit) |
| `simple_loop.rou.xml` | One looping `ego` (vType `car`) on route `-1 -2 -3 -4` |
| `simple_loop.sumocfg` | Ties the net + route together |

> Earlier this net was authored independently of the xodr (square corners,
> edges `bottom/right/top/left`) and did **not** coordinate-match the CarMaker
> road. It was regenerated from the xodr for #174 so all backends share one
> geometry.
