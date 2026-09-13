# Dyno / CARLA forced-speed-match study

Companion to `FIXS_Applications#24`. No CARLA server, no dyno, no hardware.

## The question

The bench holds a real vehicle against a road load. CARLA meanwhile runs its own
vehicle, with its own mass and its own resistance. We tie the two together by
overwriting CARLA's speed with the bench's speed every sync interval — what
`set_target_velocity` does. **How much does that paper over, and what does it
cost?**

## What is real here and what is not

The bench is [`CommonLib/xil`](../../../../CommonLib/xil/) `DynoSimulator` in
chassis mode — the shipped simulator, not a copy of it. The CARLA side is a
**surrogate**: a longitudinal ODE written in this file, parameterised from
constants measured off a live 0.9.16 server. No server is contacted.

Both plants are driven by **pedals**, from the same driver, because that is what
the FIXS ego receives — a CARLA agent writes throttle and brake through
`ego.set` (`FIXS_Applications#59`). Feeding them anything else would compare two
different inputs rather than two plants.

They share a powertrain envelope on purpose, so the bench differs from the
surrogate in exactly three separable places: **resistance curve, mass, and the
bench's torque delivery lag**. Handing the surrogate CARLA's real torque map is
the next step and belongs with a server-connected run.

## Running it

Use the `realsim` conda env. The study and the bench need only the standard
library; matplotlib is optional and draws the figure, so use `fixs_applications`
if you want the plot.

```bash
PY=~/miniconda3/envs/realsim/python.exe

$PY -m pytest tests/Python/unit/test_xil_dyno.py -q        # the bench, 23 tests
$PY -m pytest tests/Xil/Probes/dyno_speed_sync/ -q         # the coupling, 12 tests
$PY tests/Xil/Probes/dyno_speed_sync/dyno_sync_sim.py --sync-sweep 0.002,0.005,0.02,0.05,0.10
```

## The four couplings

| coupling | what CARLA is doing | why it is here |
|---|---|---|
| `free` | its own driver, never synced | how far the two drift apart on their own |
| `track` | its own driver, closing the loop on the bench speed | the alternative to overwriting |
| `forced_coast` | coasting, speed overwritten | pessimistic bound: the sync supplies the whole tractive effort |
| `forced_driven` | its own driver **and** speed overwritten | **the architecture under test** |

## What it found

**1. The model disagreement changes sign at about 20 m/s.**

```
   v [m/s]     road load - CARLA resistance [N]
         0                   -106.1
        10                    -74.8
        20                     -0.6
        30                    116.5
        40                    276.5
```

CARLA's rolling resistance dominates low down, the bench's `C` dominates high up.
Below the crossing the sync pushes CARLA forward; above it, holds CARLA back. No
single bias correction removes that.

**2. Syncing faster buys accuracy, not quietness.**

```
  sync [s]   speed err rms [m/s]   F_sync rms [N]
     0.002              0.0007              882.9
     0.005              0.0015              875.2
     0.010              0.0028              863.0
     0.020              0.0053              840.5
     0.050              0.0122              780.5
     0.100              0.0218              692.0
```

A 50x range of sync interval moves the speed error 31x and the injected force
22%. `F_sync = m*dv/T` with `dv` growing in `T`, so the two largely cancel. **The
disturbance is set by the model mismatch, not the sync rate.** It is not exactly
invariant, because the plants also differ dynamically and that content in `dv`
does not accumulate linearly — which is the honest version of a claim an earlier
revision of this study made too strongly, when both plants were force-driven and
the figure was 4%.

**3. Whether CARLA drives itself dominates everything.**

```
  coupling        err rms [m/s]   F_sync rms [N]   F_sync max [N]
  free                   0.1120              n/a              n/a
  track                  0.2779              n/a              n/a
  forced_coast           0.0088           2698.8           5352.8
  forced_driven          0.0028            863.0           1647.5
```

Coasting makes the overwrite supply the whole tractive effort. With CARLA's own
agent driving, the sync supplies only the residual — 863 N rms with a near-zero
mean (−11.5 N), and −82 kJ against 4353 kJ of tractive work over the cycle,
**−1.89 %**. The near-zero mean matters more than the rms: the overwrite is
shuffling energy back and forth as the mismatch changes sign, not biasing it.

**4. Matching the resistance models does not zero it.** A residual remains from
the bench's torque lag alone, and widening the bench's torque bandwidth shrinks
it. Two independent error sources, one test each.

## Parameters, and which are trustworthy

| value | source |
|---|---|
| CARLA `CdA = 0.377 m^2`, `rho = 1.25` | CARLA source, **confirmed in motion**: `drag/v^2` constant to 3.2 % over 941 samples |
| CARLA `mass = 1845 kg` | `get_physics_control()` on the stock Tesla Model 3 |
| bench `wheel_radius = 0.3596 m` | **measured**: `median(v / omega_rear)` over 2.2 M samples of EV6 log |
| CARLA `roll_coeff = 0.012` | **not measured.** CARLA's rolling resistance lives inside the tire model and only appears lumped into `sum(long_force)`. It decides where the sign change in finding (1) sits |
| bench road load A/B/C | representative EV6 AWD values, not measured from our dyno |
| bench powertrain peaks | manufacturer-derived, **not measured**. The instrumented cycles reached 41 kW at the wheels against a rated 239 kW, so that data bounds demand and cannot confirm capability |
| bench torque bandwidth 5 Hz, roller inertia 40 kg m^2 | placeholders for the real bench |

**The first thing to replace with measurement is `roll_coeff`**, because it
decides where the sign change lands and that crossing is the whole character of
the mismatch.

## Feeding it a real cycle

```bash
python dyno_sync_sim.py --cycle-csv mycycle.csv --sync-dt 0.01
```

Two columns, `time [s], speed [m/s]`, header optional, resampled onto `--dt`.
