# Checks to Run on Real Dyno Data

Companion to [`AxleDynoCarMakerCoupling.md`](AxleDynoCarMakerCoupling.md).

That document describes how the bench and CarMaker are coupled, and closes with a
handful of questions that **cannot be answered from the architecture** — only from a
recorded run. This file lists those questions, the channels needed to answer them, and
what each result would mean.

Status: **written before any dyno data was available.** Nothing here has been run yet.

---

## 1. Why this file exists

Today's coupling sends the *commanded* wheel speed to CarMaker, not the measured one
(§4.2 of the coupling document). That is a deliberate choice and the right one for
stability, but it has a consequence: **the co-simulation cannot see the hardware.**
If the shaft fails to follow its command, CarMaker keeps producing a healthy-looking
trajectory regardless.

Everything below exists to close that gap from outside the loop, by looking at the raw
logs rather than by changing the architecture.

---

## 2. Channels to record

All at the loop rate (1 kHz) unless noted. Per driven corner where applicable.

| Channel | Source | Why |
|---|---|---|
| `w_cmd` | our Simulink block | the value sent to the dyno *and* to CarMaker |
| `w_act` | dyno speed feedback | the only signal that can contradict the model |
| `Taxl` | torque flange | the one physical input to the loop |
| `T_dyno` | dyno torque feedback or current | to detect saturation against the machine limit |
| `Trq_T2W` | CarMaker | tire torque returned each step |
| `vx` | CarMaker | vehicle speed, for slip and for the operating point |
| `Slp` | CarMaker | tire longitudinal slip, if exportable |
| ECU shaft speed | vehicle CAN | what the real controllers believe the speed is |
| gear / shift flag | vehicle CAN | to check shift points land where the model expects |

Also capture once per run, not per sample: the tire parameter file, the value of `I`
used, `Ts`, and the dyno's torque limit.

---

## 3. The checks

### 3.1 Servo tracking — the headline check

Plot `w_act - w_cmd` for the whole run. Report its distribution, its worst case, and
its correlation with `|dw/dt|`.

- **Small everywhere** (a few tenths of a rad/s, no trend): the commanded-versus-measured
  debate is moot. Keep the current architecture and stop worrying about it.
- **Grows with acceleration**: ordinary servo lag. Quantify it, then apply §3.4.
- **Large excursions, or a step change that does not recover**: saturation. Go to §3.2.
  This is the failure the architecture cannot report on itself.

### 3.2 Saturation

Compare `T_dyno` against the machine's absorb and motor limits, and separately check the
vehicle's peak axle torque (including regen braking) against the same limits.

Under speed control the dyno must sink whatever the powertrain produces. If the vehicle
can out-torque the machine, the shaft leaves the command and CarMaker never finds out.

**If found:** this is the case for switching to the re-anchored integrator,
`w_cmd[k+1] = w_act[k] + (Ts/I)(Taxl[k] + Trq_T2W[k])`, which makes the failure
self-limiting instead of silent. It costs run-to-run repeatability.

### 3.3 Oscillation

Take a spectrum of `w_cmd` and `Trq_T2W`, and look specifically at slow-speed sections
(roughly 2 to 5 m/s) and at tip-in and tip-out transients.

- **Clean:** no action. The predicted wheel-against-tire mode is well damped.
- **A peak in the tens of hertz** that grows as the vehicle slows: that is the mode
  described in §7.1 of the coupling document. Record its frequency — it also gives us
  the real slip stiffness, which is currently a textbook estimate.

Check while you are there that no extra delay has crept into the path
`Trq_T2W -> integrator -> rotv`. One integration step is expected; a second would show
here first.

### 3.4 Energy bias

Compute both:

```
E_cmd  = integral of  Taxl * w_cmd  dt
E_act  = integral of  Taxl * w_act  dt
```

`E_act` is the physically real one. The difference is the bias described in the
coupling document: it runs the same direction on acceleration and on regen braking, so
it does not cancel over a cycle, and it scales with how much transient content the drive
cycle has.

**Expect well under 1% with a healthy servo.** If it is larger, §3.1 or §3.2 will
already have said why.

**Regardless of size, report energy from `E_act`.** The model should keep advancing on
`w_cmd`; only the reported number changes. There is no reason those two uses of "speed"
have to be the same signal, and separating them costs nothing.

### 3.5 Do the real controllers agree with the model?

Compare the ECU's own shaft speed against `w_cmd`, and check that gear changes occur
where CarMaker expects them.

The physical ECU, TCU and hybrid supervisor act on the *actual* speed from their own
sensors, while the eco-driving strategy is planned against the model's speed. If those
two diverge, the controller under test and the controller being evaluated are working
from different beliefs about the vehicle — which sits directly inside the comparison
this bench exists to make.

### 3.6 Slip sanity

Look at steady-state cruise. Slip should be small and centred near zero.

A **standing slip offset** at constant speed points at one of: a bias in the torque
measurement, a mismatch between the radius used to relate wheel speed and vehicle speed,
or a road-load term that does not match the vehicle. §3.4 and §3.7 narrow it down.

### 3.7 Tire configuration

Read out of the tire parameter file, once:

- `Model.USE_MODE` — expected 13, 14 or 15. This decides both whether combined slip
  couples cornering back into the torque the bench feels, and whether the tire carries
  relaxation states at all. If it is 11 or 12, several statements in the coupling
  document do not hold.
- `LongFrc.Length` (or `ITRLLO`) — expected about 0.05 m.
- `StandStill.vMax` — expected 2.0 m/s.
- Longitudinal slip stiffness — currently a textbook figure in our estimates.

### 3.8 Wheel inertia, if a coastdown is practical

`I` cannot be measured on an axle dyno, because the tire and rim are not present and
they are most of it. It is estimated from the wheel specification, and that is
acceptable: wheel-end inertia is under 1% of the total inertia the vehicle accelerates,
so a 20% error moves vehicle acceleration by roughly 0.2%.

Treat `I` as a **stability parameter, not an accuracy one** — it sets how twitchy the
loop in §3.3 is, and very little else. Record the value used and where it came from.

---

## 4. What would change our mind about the architecture

Ranked by how strongly the evidence would push:

| Finding | Consequence |
|---|---|
| Servo saturation (§3.2) | move to the re-anchored integrator |
| Sustained oscillation in the 2–5 m/s band (§3.3) | check for an added delay first; then `Ts`, then `I` |
| ECU and model disagree on speed at shift points (§3.5) | the confound is real; decide whether the strategy comparison is still clean |
| Energy gap above ~1% (§3.4) | symptom, not cause — look at §3.1 and §3.2 |
| Everything clean | keep the current architecture and record that it was verified, with the numbers |

---

## 5. When data is available

Point an assistant at this file together with the logs. The checks are in priority
order: §3.1 first, since almost every other question is downstream of how well the
servo tracks.
