# Axle Dyno &harr; CarMaker Coupling

Status: **Step 1 — document the working implementation.** No CARLA content yet.
CM version referenced: 13.1.3.

**Slide version.** `AxleDynoCarMakerCoupling.pptx` sits beside this file: six slides
of native, editable PowerPoint shapes. It follows the same chain as this document —
the loop, when `Trq_T2W[k]` is computed within the integration step, how the tire
model computes it, what can make the loop ring, and a summary.

It is a repo-side deliverable rather than a Sphinx page, so it is not linked from the
rendered documentation.

| File | Use |
|---|---|
| `AxleDynoCarMakerCoupling.pptx` | the deck — for presenting |
| `make_slides_pptx.py` | regenerates it. **Edit this, not the binary** |

The `.pptx` is a build product checked in for convenience. A binary does not diff, so
treat `make_slides_pptx.py` as the source of truth and regenerate rather than
hand-editing, unless the change is genuinely one-off presentation polish:

```
pip install python-pptx
python doc/xil/make_slides_pptx.py
```

---

## 1. Scope

Records the as-built coupling between the axle dynamometer and CarMaker via the
CarMaker-for-Simulink (CM4SL) powertrain bypass, in which CarMaker's internal
PowerTrain module is disabled and replaced by a Simulink block that owns the wheel
rotational degree of freedom.

Every equation is indexed by the discrete step `k`: the sample-to-sample ordering is
what makes this loop behave, and it is the part most easily lost when the setup is
described informally — see §5.

Not covered here: CARLA, road load, the FIXS wire. Those attach on top of this loop and
are the subject of later work (§11).

**Source references.** Citations of the form *RefMan §x* / *Programmer's Guide §x* are to
the CarMaker 13.1.3 documentation set; `PowerTrain.h` is
`include/Car/PowerTrain.h` from the same install; `UserPowerTrain.mdl` is
`Templates/Car4SL_Extras/src_cm4sl/UserPowerTrain.mdl`, IPG's own reference
implementation of this bypass.

---

## 2. Notation

```
k           discrete step index; sample period Ts = 0.001 s (= CM's internal step)
i           corner index, i in {FL, FR, RL, RR}

I_i         CM parameter  Wheel.<i>.I               [kg m^2]
K           gain in the Simulink integrator block   [-]      = 1 (confirmed)

Taxl_i[k]   measured axle torque at corner i        [Nm]     = T_drive - T_brake, physical
Ttire_i[k]  Trq_T2W read out of CM                  [Nm]     tire torque about wheel spin axis
w_i[k]      rotv computed in Simulink               [rad/s]
w_act_i[k]  actual dyno shaft speed                 [rad/s]  NOT sent to CarMaker
```

`Wheel.<i>.I` is documented in the CM Reference Manual as *"the wheel and all other
rotating components (parts of the brake, ...)"* — i.e. the full wheel-end rotational
inertia, tire + rim + hub + rotating brake parts.

---

## 3. State ownership — the central fact

Exactly **one** wheel-spin integrator exists in the whole system, and it is in Simulink.

| Domain | Owns | Notes |
|---|---|---|
| **Simulink (our code)** | wheel spin DOF `w_i` | the only integrator for this state |
| **CarMaker** | vehicle body, suspension, tire, road | powertrain module removed |
| **Hardware** | powertrain, as a *torque source only* | its physical spin is servo-slaved |

CarMaker has no wheel-spin state because that integrator lives inside the PowerTrain
module, which we replaced. This is explicit in the CM documentation: the driveline
model diagram (RefMan Fig. 16.83) contains a block labelled **"Integration of rotation
speeds"**, fed by `TWheel`, `TTire`, `TBrake`; and `tPTDriveLineCfgIF` receives
`Wheel_Iyy[wheel]` at init precisely so it can perform that integration.

Our Simulink block is a like-for-like reimplementation of that CM block, using the
same inertia parameter. Interface evidence, `include/Car/PowerTrain.h:1069-1121`:

```c
    struct tPowerTrainIF_WheelIn {          /* Inputs to the powertrain  */
        double  Trq_Brake;
        double  Trq_T2W;        /* Tire torque, around wheel spin axis [Nm]  */
        double  Trq_WhlBearing;
    } WheelIn[VEHICLE_NWHLS];

    struct tPowerTrainIF_WheelOut {         /* Outputs from the powertrain */
        double  rot, rotv;      /* Wheel rotation [rad, rad/s]  */
        double  Trq_B2W;
        double  Trq_Drive;
        double  Trq_Supp2WC;
        ...
    } WheelOut[VEHICLE_NWHLS];
```

Tire torque in, wheel speed out. Replacing the module means owning `rotv`.

---

## 4. Signal flow at step k

```
                  Taxl_i[k]  (measured)
   HARDWARE ─────────────────────────────┐
      ▲                                  │
      │ w_cmd_i[k]                       ▼
      │                        ┌──────────────────────┐
      │                        │  SIMULINK            │
      └────────────────────────│  w_i[k] = w_i[k-1]   │
        (dyno internal speed   │   + (K*Ts/I_i)*      │
         servo; w_act_i[k]     │     (Taxl_i[k]       │
         stays inside it)      │      + Ttire_i[k])   │
                               └────┬────────────▲────┘
                    w_i[k], Taxl_i[k]│            │ Ttire_i[k]
                                     ▼            │
                               ┌──────────────────┴───┐
                               │  CARMAKER            │
                               │  tire + body + road  │
                               └──────────────────────┘
```

### 4.1 Simulink update

Integrator block `K*Ts*z/(z-1)` is **backward Euler**:
`Y/U = K*Ts*z/(z-1)`  =>  `y[k] = y[k-1] + K*Ts*u[k]` (uses the *current* input).

```
w_i[k] = w_i[k-1] + (K*Ts / I_i) * ( Taxl_i[k] + Ttire_i[k] )        (1)
```

Torques are **added**, which is correct provided CM's `Trq_T2W` is already signed
negative under forward drive (see OPEN-3).

### 4.2 Outputs at step k

```
to dyno:   w_cmd_i[k]            = w_i[k]
to CM:     PT.W<i>.rotv          = w_i[k]          <-- computed, NOT measured
           PT.W<i>.Trq_Drive     = Taxl_i[k]
```

### 4.3 Dyno inner loop (outside this loop)

```
w_act_i[k]  <- servo tracks w_cmd_i[k]
```

`w_act_i[k]` is consumed **only** by the dyno's internal speed controller. It never
enters Simulink or CarMaker.

### 4.4 CarMaker at step k

```
vBelt_i   = w_i[k] * rBelt_eff_i          (RefMan EQ 280)
s_i       = (vBelt_i - vx_i(P)) / |...|   (TYDEX, RefMan EQ 281)
Fx_i      = tire(s_i, alpha_i, Fz_i, muRoad)
Ttire_i   = tire torque about spin axis   -> back to (1)
body      = f(Fx, Fy, Fz, ...)            -> next vx_i(P)
```

Note `rBelt_eff` is a tire-model *output* and load-dependent. Our integrator (1) uses
only torques and inertia — no radius — so it is immune to any radius mismatch. This is
a genuine robustness property of the current design and should be preserved.

---

## 5. The k-indexing — RESOLVED, d = 1

Equation (1) is backward Euler and consumes `Ttire_i[k]`. But CM computes `Ttire_i`
*from* `w_i`. This is not an algebraic loop, because of CM's **main cycle ordering**
(Programmer's Guide Listing 1.2, section 1.6). Within one `Vehicle Model` evaluation:

```
- Vehicle
    - Steering
    - Suspension Kinematics and Compliance
    - Aerodynamics
    - Suspension Forces
    - Tire              <-- Trq_T2W produced HERE
- Trailer
- Brake
- PowerTrain            <-- consumed HERE; our bypass occupies this slot
    - ...
    - DriveLine         <-- "Integration of rotation speeds" (RefMan Fig. 16.83)
- Body Frame
```

The tire runs **before** the powertrain in the same cycle. Our Simulink bypass sits at
the powertrain slot *inside* the CM vehicle hierarchy — confirmed by the block path in
IPG's own reference model:

```
UserPowerTrain/CarMaker/IPG Vehicle/PowerTrain/User RearDriven PowerTrain/...
```

Therefore, within step `k`: the tire evaluates using `w_i[k-1]` and emits `Ttire_i[k]`;
our block consumes it and emits `w_i[k]`; the tire uses `w_i[k]` at step `k+1`. So

```
w_i[k] = w_i[k-1] + (K*Ts/I_i) * ( Taxl_i[k] + Ttire_i( w_i[k-1], body[k-1] ) )   (2)
```

**Backward Euler on the block, one step of delay on the tire term.** Everything in
section 7 follows from (2).

Critically: this is **CarMaker's own native structure**, not a co-simulation artifact
we introduced. When CM runs its internal powertrain it performs the same update in the
same order with the same delay.

### 5.2 Where the delay physically lives — and why it must exist

The tire's slip->force map is **memoryless** (§10.3). That is not in tension with the
one-step delay, because the delay is not in the tire — it is in the **shared variable**.

`Rim_rotv` is a field in CM's vehicle data structure, written once per cycle by the
PowerTrain module and read once per cycle by the Tire module. Because Tire is scheduled
*before* PowerTrain, it always reads the value written in the previous cycle:

```
- Vehicle
    - ... - Tire        <-- READS  Rim_rotv   (last cycle's value = w[k-1])
- Trailer
- Brake
- PowerTrain            <-- WRITES Rim_rotv   (w[k])
```

That shared field **is** the `z^-1`. A memoryless block fed a stale input still produces
a delayed output.

Moreover the delay is *required*. If the tire were memoryless and read `w[k]`, then

```
w[k] = w[k-1] + (Ts/I) * ( Taxl[k] + f(w[k]) )
```

is implicit in `w[k]` — an algebraic loop needing iteration, which real-time code cannot
do. Something must break the loop: either the tire's own relaxation states, or the
scheduling delay. See §7.1 for why which one matters.

### 5.3 Where the integration actually happens — two different integrations

These are frequently conflated. They are not the same thing and they do not happen at
the same point in the cycle.

| | Integration | Where in the sequence | Who owns it |
|---|---|---|---|
| **A** | **wheel spin** `w_i` | **at the PowerTrain slot** — it *is* the PowerTrain module | CM native: DriveLine "Integration of rotation speeds" (RefMan Fig 16.83). **Our build: our Simulink block.** |
| **B** | body / suspension multibody DOFs | `Body Frame`, *after* PowerTrain | CarMaker |

So the wheel-speed integration is **not** downstream of PowerTrain — it is what
PowerTrain *does*. The body integration is what follows it. Within one cycle:

```
Tire            reads  Rim_rotv = w[k-1],  emits Trq_T2W[k]      (memoryless map)
Brake
PowerTrain      INTEGRATION A: w[k] = w[k-1] + (Ts/I)(Taxl[k] + Trq_T2W[k])
Body Frame      INTEGRATION B: advances chassis/suspension states
```

Figure 1.23 brackets this whole vehicle-model evaluation as **"One integration step."**

### 5.4 CarMaker's solver is Backward Euler — which is why our block must be

Programmer's Guide, CM4SL FAQ, verbatim:

> *"When using integrator blocks, it is recommended to use discrete ones. This is the
> case because using discrete integrator blocks allows the user to select an alternate
> solver. **Since the solver used in CarMaker is backwards-euler**, the user needs to
> choose this option. Due to the better compatibility, vibrations are no longer an
> issue."*

Two consequences:

1. Our `K*Ts*z/(z-1)` (= Simulink "Backward Euler") is not merely copying IPG's
   reference model — it **matches CM's own solver**, and IPG explicitly warns that the
   wrong choice produces *"slight vibrations."* The choice is correct on two independent
   grounds.
2. Backward Euler is a **one-stage** method: the module sequence of §5 is evaluated
   **exactly once per step**. There is no Runge-Kutta-style multi-evaluation. The
   one-step delay of §5.2 is therefore exact, not an approximation.

### 5.5 Is "Tire before PowerTrain" sufficient? — yes for well-posedness, no for stability

- **Well-posedness: yes, and it is free.** The ordering is CM's fixed module sequence;
  we do not choose it and cannot break it. It guarantees the tire reads `w[k-1]`, giving
  exactly one delay in the loop, so the update is explicit and real-time solvable.
- **Stability: no.** The very ordering that creates the delay is what *costs* damping
  (§7.1). "Fine" here means "runs correctly", not "without consequence."

**What we DO control, and could get wrong: additional delay.** The ordering supplies
exactly one step. Anything added on top of it — a `Memory` block, a `Unit Delay`, a
rate transition, a subsystem introducing its own delay — makes `d = 2` or more, and each
extra step costs a further `dzeta ~ wn*Ts/2 ~ 0.14`. At 10 m/s, where `zeta ~ 0.35`, two
extra steps would take the loop to marginal.

> **Rule: the path `CM Trq_T2W -> our integrator -> CM rotv` must contain no delay
> element other than the integrator itself.**

Note IPG's reference model *does* contain four `Memory` blocks — but they sit on the
brake-torque-reduction path (`Trq_WB -> Saturation -> Product`, breaking the algebraic
loop in `Trq_B2W`), **not** on the `Trq_T2W -> integrator -> rotv` path. That path is
dead in our build anyway since all CM brake torque is zeroed (OPEN-5). The core loop in
IPG's reference contains exactly one delay: the scheduling one. Ours should match.

### 5.1 The implementation matches IPG's reference model block-for-block

`Templates/Car4SL_Extras/src_cm4sl/UserPowerTrain.mdl` is IPG's documented example for
"replace CarMaker's internal powertrain module on the Simulink level"
(Programmer's Guide §6.4.13). Its four wheel-speed integrators:

```
Block { BlockType        DiscreteIntegrator
        IntegratorMethod "Integration: Backward Euler"
        SampleTime       "0.001"
        Port { Name      "rotv FL" } }        (likewise FR, RL, RR)
```

`K*Ts*z/(z-1)` with `K=1, Ts=0.001` **is** Simulink's Backward Euler discrete
integrator. The reference also reads the wheel inertia from CM parameters via
`Read CM Parameter` blocks named `WheelInertia`, `WheelInertia1..3` — the same
`Wheel.<i>.I` route we use.

Our implementation is the sanctioned bypass, not an approximation of it.

---

## 6. Why it works

**6.1 One integrator, so nothing can diverge.** The classic HIL failure — a model
state and a plant state drifting apart — requires two integrators. There is one.
CM's tire slip is computed from exactly the number that commands the dyno, so model
and command are consistent *by construction*, not by tracking.

**6.2 The dyno's tracking error is structurally excluded.** Because `w_act` never
enters the model, servo lag and servo error cannot perturb CM's tire slip. The
simulation is repeatable regardless of dyno performance. (This is also the one
weakness — see 7.2.)

**6.3 The inertia is counted exactly once, with the right value, in the right place.**
`Wheel.<i>.I` is the parameter CM itself hands its driveline for this integration.
Under speed control the physical hub and rotor inertia are overridden by the servo, so
the model inertia is the one that governs — and it is correct.

**6.4 The hardware enters at exactly one point.** `Taxl_i[k]` is the sole reality
input. That makes the fidelity question a single, well-posed one: is the measured axle
torque correct, and was it produced at the right operating point?

---

## 7. Where the margins are

### 7.1 The wheel/tire torsional mode and what the delay costs it

> **`Model.USE_MODE` selects which of the two analyses below is the live one.**
> With relaxation ON (`+10`, i.e. 13/14/15) the tire's own states break the loop and
> §7.1a applies. With relaxation OFF (3/4) the tire is memoryless, the scheduling delay
> is the only dynamics between `w` and `Ttire`, and §7.1b applies. See OPEN-9.

#### 7.1a Relaxation ON (`USE_MODE` 13/14/15) — expected case

CM's tire is **not algebraic in slip**. It applies a first-order relaxation lag
(RefMan, Real-time Tire, `LongFrc.Length`, default **0.05 m**):

> *"Specifies the relaxation length to determine the transient response of the
> longitudinal force Fx ... The transient behavior is described using a low pass filter
> with following time constant: **T = K / Vx**."*

IPGTire has the same parameter, `ITRLLO = 0.05`. So the loop is second order, not
first order. Linearising about an operating point (`sigma` = relaxation length,
`Cs` = longitudinal slip stiffness [N per unit slip], `tau = sigma/vx`):

```
I*dw/dt      = -r*dFx
tau*dFx/dt   = (Cs*r/vx)*dw - dFx

=>   s^2 + s/tau + Cs*r^2/(I*tau*vx) = 0
```

Since `tau*vx = sigma`, the two modal parameters separate cleanly:

```
wn   =  r * sqrt( Cs / (I*sigma) )        <-- INDEPENDENT of vehicle speed
zeta =  vx / (2*sigma*wn)                 <-- PROPORTIONAL to vehicle speed
```

With `r=0.32 m`, `I=1.5 kg m^2`, `sigma=0.05 m`, `Cs ~ 60000 N/unit slip`:

```
wn   ~ 286 rad/s ~ 45 Hz
zeta ~ vx / 28.6
```

A one-step delay in the loop costs roughly `dzeta ~ wn*Ts/2 = 0.14` of damping:

| vx [m/s] | zeta physical | zeta after 1-step delay | verdict |
|---|---|---|---|
| 20 | 0.70 | 0.56 | well damped |
| 10 | 0.35 | 0.21 | fine |
| 5 | 0.175 | 0.03 | marginal |
| 2 | 0.070 | -0.07 | would ring — but see below |

**This mode is physical**, not a numerical artifact: it is the tire/wheel torsional
resonance that a real vehicle has. The delay only removes damping from it.

**Why the bottom of the band is covered by CM itself.** The Real-time Tire has
`StandStill.vMax`, default **2.0 m/s** — *"Boundary for switching between stand still
model and normal computation"* — below which CM switches to a stand-still formulation
(`StandStill.cLong = 0.01`). IPGTire has the equivalent `ITSSCLO = 0.01`. The normal
slip model is therefore never evaluated in the worst region.

**Residual exposure: roughly 2-5 m/s**, where the normal model runs and `zeta_eff` is
small. Testable prediction: **~45 Hz ringing in `rotv` and `Trq_T2W` during low-speed
creep between 2 and 5 m/s.**

Design levers, in order of leverage:

- `dzeta ~ wn*Ts/2`, so `Ts` is linear in damping loss. 1 ms is good; 2 ms doubles it.
- `wn ~ 1/sqrt(I)`: a *larger* wheel inertia lowers `wn` and costs less damping. A
  small `Wheel.<i>.I` is destabilizing.
- `wn ~ 1/sqrt(sigma)`: a *shorter* relaxation length is worse. If anyone reduces
  `LongFrc.Length` "for realism", the margin shrinks as `1/sqrt`.

**Caveat:** `Cs` above is an estimate, not read from our tire file. `wn ~ sqrt(Cs)`,
so a 60% error in `Cs` is a 26% error in `wn`. Read the actual value before treating
the 45 Hz figure as precise.

**Action:** inspect `Trq_T2W` and `rotv` traces through the 2-5 m/s band of a run.

#### 7.1b Relaxation OFF (`USE_MODE` 3/4) — qualitatively different loop

With no tire states, the scheduling delay of §5.2 is the *only* dynamics between `w`
and `Ttire`, and the loop collapses to first order explicit. With
`d(Ttire)/dw = -r^2*Cs/vx`:

```
pole  =  1 - Ts*r^2*Cs/(I*vx)          stable iff  0 < Ts*r^2*Cs/(I*vx) < 2
```

| vx [m/s] | loop gain | pole | verdict |
|---|---|---|---|
| 20 | 0.20 | 0.80 | fine |
| 5 | 0.82 | 0.18 | fine |
| 2 | 2.05 | -1.05 | **unstable** |

Same numbers as 7.1a at high speed, but the failure at low speed is a hard divergence
rather than light damping. If our tire file turns out to be in this mode, §7.4's
torque-mode stop is not a convenience — it is load-bearing.

### 7.2 The invisible servo error — the one failure this design cannot self-detect

The model believes `w_i[k]`; the hardware is at `w_act_i[k]`. The real powertrain
produces torque as a function of *its own* speed, so `Taxl_i[k]` is the torque at
`w_act_i[k]` — but equation (1) integrates it as though the wheel were at `w_i[k]`.

While `|w_act - w_cmd|` is small this is negligible. If the servo saturates (torque
limit on hard tip-in, or bandwidth limit on a fast transient), we integrate a torque
measured at the wrong operating point. **Nothing rings and nothing diverges** — the
simulation quietly becomes a different vehicle.

**Action:** log `w_act_i[k] - w_cmd_i[k]` and alarm on it, even though it is not in
the loop. It is the health metric for the entire coupling.

### 7.3 Speed control suppresses driveline shuffle

The servo sets the wheel-end impedance the real powertrain sees. First driveline
torsional mode (~2-10 Hz) is a wheel-speed oscillation, so a stiff servo damps it out.
Irrelevant for energy/eco work; relevant if drivability or tip-in transients are ever
in scope. Recorded as a known limitation, not a defect.

---

## 8. Open items

### Closed

| # | Item | Resolution |
|---|---|---|
| OPEN-1 | Value of `K` | `K = 1`. Emulated inertia is `Wheel.<i>.I` exactly, as intended. |
| OPEN-2 | CM4SL delay `d` | `d = 1`, established from CM main-cycle ordering (§5), not measured. Tire runs before PowerTrain in the same cycle. Matches CM's native behaviour. |
| OPEN-3 | Sign of `Trq_T2W` | Handled in the implementation. Written here as `Taxl + Trq_T2W` to match the Simulink block. |
| OPEN-4 | `Trq_WhlBearing` | RefMan §12.15: `T_Fric = amp * F_Axle * mu_Fric * R_B` (EQ 72). Defaults `On = 0` (**off**), `Coeff = 0.003`, `Radius = 0.03 m`. At `F_Axle ~ 4 kN` the term is ~**0.36 Nm** per corner, ~0.1% of typical axle torque. Non-issue. Only action: confirm `WhlBearing.On` is 0 in our vehicle file; if someone enabled it, record the omission. |
| OPEN-7 | `Ts` | `Ts = 0.001 s`, matching CM's internal step. Good for the §7.1 margin. |

| OPEN-5 | Brake accounting | **All CM brake torque is bypassed to zero.** Real brake torque is physical and arrives inside `Taxl` (`Taxl = T_drive - T_brake`). No double count. |
| OPEN-6 | `Trq_Supp2WC`, `Trq_Supp2Bdy1`, `Trq_Supp2BdyEng` | **All set to zero — recorded decision.** Consequence: no drive-torque squat/lift, no engine-mount reaction. `Fz` therefore omits the longitudinal-load-transfer contribution from drive torque (it still gets the contribution via body acceleration). Acceptable for energy/eco work; revisit if handling balance under drive torque matters. |

### Still open

| # | Item | Why it matters |
|---|---|---|
| OPEN-8 | Actual `Cs` from our tire file | §7.1's 45 Hz figure scales as `sqrt(Cs)`. Low priority — see §7.4. |
| OPEN-9 | `Model.USE_MODE` in our tire file | Determines whether combined slip is active at all. If `USE_MODE` is 11/12 there is **no Fx<->Fy coupling** and §10 path A/C do not exist. See §10.5. |

### 7.4 Low speed / standstill — handled outside this loop

The dyno is switched to **torque mode with commanded `Ttire = 0`** during stopping. The
speed-tracking loop of §4.3 is therefore not active in the regime where §7.1 predicts
low damping, and CM has separately switched to its stand-still tire model below
`StandStill.vMax`. The §7.1 low-speed margin is consequently not a live concern;
it is retained here as the explanation of *why* the mode switch is needed.

---

## 10. The tire model — what produces `Ttire`

### 10.1 `Trq_T2W` is the end of a chain, not the tire model itself

```
Vehicle model      contact-patch kinematics + normal load Fz
      |
      v
Tire model         plug-in: forces & moments at the contact point   (Fig. 18.2)
      |
      v
Tire_Calc_Frc2WC() transform contact point -> wheel center
      |
      v
"Calculation of tire reaction moment for powertrain"  ->  Trq_T2W   (Fig. 18.6)
```

Governing relation, RefMan EQ 279:

```
F, T = f( Fz, alpha, s, gamma, mu, turnslip )
```

### 10.2 Tire model interface (CPI = Contact Point Interface)

Per wheel, per step:

```
INPUTS  (vehicle -> tire)
  IF.P_v0_W[0..2]  velocity vector at the tire-road contact point, wheel frame FrW  [m/s]
  IF.Rim_rotv      rim rotation speed            <-- OUR w_i[k] ENTERS HERE         [rad/s]
  IF.Rim_turnv     rim turning speed about vertical axis (turn slip)                [rad/s]
  IF.Frc_W[2]      Fz, normal force at contact point                                [N]
  IF.InclinAngle   inclination / camber angle                                       [rad]
  IF.muRoad        road friction coefficient                                        [-]

OUTPUTS (tire -> vehicle)
  IF.Slp           longitudinal slip s (kappa)
  IF.Alpha         sideslip angle
  IF.TurnSlp       turn slip
  IF.vBelt         belt speed at contact point = Rim_rotv * rBelt_eff               [m/s]
  IF.rBelt_eff     effective rolling radius                                         [m]
  IF.Frc_W[0..2]   Fx, Fy, Fz at contact point                                      [N]
  IF.Trq_W[0..2]   Mx, My, Mz at contact point                                      [Nm]
```

Note the elegance: **one velocity vector** `P_v0_W` carries both the longitudinal and
lateral kinematics, and our `Rim_rotv` is the only other kinematic input.

Available models (RefMan 18.2): IPGTire (STI), Real-time Tire (CPI), MF 5.2 / 6.1,
TameTire, MF-Tyre/MF-Swift, FTire.

### 10.3 What happens inside — four stages

**Stage 1 — kinematics to slip.** Two scalars, from two different sources:

```
s      = (Rim_rotv * rBelt_eff - vx(P)) / |...|   = (vBelt - vx(P)) / |...|   EQ 281 (TYDEX)
alpha  = atan( vy(P) / |vx(P)| )
```

`s` is driven by **our** `Rim_rotv` against the longitudinal component of `P_v0_W`.
`alpha` is driven by the **lateral** component of the same vector — which comes from
steering, body yaw and body sideslip. This is where the two channels enter.

**Stage 2 — pure-slip forces.** Model-dependent (spline for IPGTire, Magic Formula for
MF*): `Fx0(s)` and `Fy0(alpha)`, each also a function of `Fz`, `gamma`, `mu`.

**Stage 3 — combined slip.** The coupling. Magic Formula form, RefMan 18.5.10/18.5.11:

```
Fx = Gx(alpha) * Fx0(s)            EQ 352
Fy = Gy(s)     * Fy0(alpha) + SVy  EQ 360

Gx = cos[ Cx*atan{ Bx*S - Ex(Bx*S - atan(Bx*S)) } ] / Gx0        EQ 353-354
```

`Gx` and `Gy` are cosine-shaped weighting functions, <= 1. **`Fx` is de-rated by the
presence of sideslip; `Fy` is de-rated by the presence of longitudinal slip.** This is
the friction ellipse, implemented as multiplicative de-rating rather than a geometric
clip.

**Stage 4 — spin-axis moment.** `Trq_T2W` is *not* simply `-r*Fx`. It also carries the
rolling-resistance torque (SAE J2452, EQ 286-287):

```
FRR   = P^a * Fz^b * (a + b*vBelt + c*vBelt^2)
TrqRR = -FRR * r * sgn(vBelt)
```

so there is a drag component about the spin axis even at zero slip.

### 10.4 How longitudinal and lateral interact, given longitudinal comes from the dyno

Our `w_i[k]` enters at exactly one point: `IF.Rim_rotv` -> `vBelt` -> `s`. From there
three coupling paths exist, all inside CarMaker:

| Path | Mechanism | Speed | Relevance to the bench |
|---|---|---|---|
| **A** long -> lat, in-tire | `s` -> `Gy(s)` -> `Fy` de-rated | instantaneous | a driven/braked wheel makes less lateral force at the same slip angle |
| **B** long -> lat, via body | `Fx` -> long. accel -> pitch/load transfer -> `Fz` -> both `Fx`,`Fy` scale; front/rear split shifts understeer balance | suspension timescale | partially suppressed here: `Trq_Supp2WC = 0` (OPEN-6) removes the drive-torque share |
| **C** lat -> long, **into our loop** | `alpha` -> `Gx(alpha)` -> `Fx` de-rated -> **`Trq_T2W` changes** | instantaneous | **the one that reaches the hardware** |

**Path C is the important one.** The bench is longitudinal-only, but the *load* it sees
is lateral-aware: during a corner, the same axle torque produces a different wheel
acceleration, because CM computes `Trq_T2W` with the combined-slip de-rating and our
integrator (eq. 2) consumes it directly. Cornering reaches the hardware through the
torque, without any lateral signal crossing the interface.

Each corner has its own `Trq_T2W` (different `Fz` from roll, different `alpha` from
steer/Ackermann), so per-wheel integrators preserve left/right asymmetry in a corner.
Lumping left and right would discard it.

**What the bench cannot provide:** the real tire is not present, so combined-slip
saturation is *modelled*, not measured. Path A and Path C are only as good as the tire
parameter set.

### 10.5 The one parameter to check — `Model.USE_MODE`

```
USE_MODE   1  pure longitudinal / pure lateral slip
           2  pure long. and lat. (NOT combined)
           3  combined slip
           4  combined slip + turn slip
         +10  transient (relaxation active)
RefMan: "Recommended modes for CarMaker are 13, 14 or 15."
```

If our tire file has `USE_MODE` 11 or 12, **combined slip is off** — paths A and C in
§10.4 do not exist, `Fx` is independent of `alpha`, and `Trq_T2W` carries no cornering
information to the hardware at all. Confirm it is 13/14/15.

---

## 11. Next steps

1. Close OPEN-5, OPEN-6 (fidelity/completeness — do not change correctness).
2. Close OPEN-8, OPEN-9 by reading the tire parameter file; then check a log for
   ~45 Hz content in the 2-5 m/s band (§7.1).
3. Add servo-error logging per §7.2.
4. Only then: how CARLA attaches to this loop.
