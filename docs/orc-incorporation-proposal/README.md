# Proposal: bringing ORC course models into `arm6dof`

> **Status:** proposal for discussion. This branch adds only this document;
> no code changes yet.
> **Branch:** `orc-incorporation-proposal` · **Date:** 2026-10-09

---

## 1. Summary

`arm6dof` sizes the servos with two checks. The **static** check covers gravity
only. The **dynamic** check adds gravity to inertia × one peak acceleration.
Both compare the demand with `servo_stall_torque × gear_ratio`.

The actuator, dynamics and trajectory-optimisation material from the
*Optimization-based Robot Control* (ORC) course shows that this comparison
leaves out three effects. All three matter exactly when the arm **moves**, and
they mean an external gear reduction is not free:

1. **Torque falls with speed.** A servo delivers its stall torque only at zero
   speed. A demand that occurs while the joint moves must sit under the
   servo's torque–speed line, not under its stall torque.
2. **A gear trades speed and inertia for torque.** A reduction of ratio N
   multiplies torque by N. It also divides the joint's top speed by N and
   multiplies the servo's own inertia, as seen by the joint, by N².
3. **Gears lose torque** through friction, which can be expressed as an
   efficiency below 1.

This proposal brings those models in. It fixes two known sizing problems first
(the worst-case pose and test isolation), then lays out a path to exact
dynamics through a URDF export, and to torque demand taken from real
trajectories.

**Impact in one paragraph:** the sizing verdicts become more honest and
generally stricter, so expect more `FAIL` rows before anything about the
hardware changes. The gear ratio stops being a free multiplier and becomes a
real trade-off between torque and speed. Longer term, the project gains a
route to exact dynamics, to sizing from real tasks, and to ROS 2 / simulation
tooling.

---

## 2. Sources and licence boundary

The ideas come from the public lab code of A. Del Prete's ORC course
(<https://github.com/andreadelprete/orc>):

| Topic | Course material | Used in |
|---|---|---|
| DC motor + gearbox model; the torque–speed line | `01_actuators/dc_motor_w_gear.py`, `compute_motor_efficiency.py` | Phase 3 |
| Optimal control with joint accelerations as the input and torque bounds from inverse dynamics; minimum time with a free time step | `optimal_control/casadi_adam/`, `min_time_reaching.ipynb` | Phase 6 |
| Full `M(q)` and `h(q, q̇)` (mass matrix, Coriolis and gravity) | `utils/robot_wrapper.py` | Phases 5–6 |
| Damped Gauss–Newton IK with backtracking | `reactive_control/ex_2_inverse_geometry.py` | Phase 7 |
| Planned trajectory → fixed-rate interpolation → position streaming; homing ramp; joint limits that allow for braking distance | `lab_disi/ur5_traj_track.py`, `legacy/interpolate.py`, `joint_pos_limits.ipynb` | Future `hardware/` layer |
| Simulation with Coulomb friction and randomised inertias | `utils/robot_simulator.py` | Future testing |

> ⚠️ **Licence boundary.** The ORC code is **GPL-3.0**; `arm6dof` is **MIT**.
> No course code is copied into this repository. Only the physics and the
> maths are reused, and every implementation is written from the equations.
> This rule applies to every phase below.

---

## 3. Where the project stands today

Both checks use the same pass rule:

```
servo_stall_torque × gear_ratio  ≥  |τ| × SAFETY_FACTOR   (2.5)
```

These are the last measured results (2026-09-29). They were taken with no
`config_default.json`, a 500 g payload, and the current `worst_case_pose`.
Torques are shown before the safety factor:

| Preset | Static | Dynamic (8 rad/s²) |
|---|---|---|
| `custom` | J2 3.88 N·m **FAIL**, J3 1.40 N·m **FAIL** | also J1 **FAIL** (1.52 N·m) |
| `IRB 1300-7/1.4` | J2 6.03 N·m **FAIL** | J1 / J2 / J3 **FAIL** (2.74 / 12.55 / 4.20 N·m) |

### Known gaps

| # | Gap | Effect on results | Fixed in |
|---|---|---|---|
| G1 | The servo rating is `stall × N`. It has no speed dependence, no gear losses and no reflected servo inertia. | Dynamic verdicts are optimistic, especially with external gearing. | Phase 3 |
| G2 | `JointSpec` has no fields for servo speed, servo inertia or gear efficiency. | G1 can't be fixed without them. | Phase 3 |
| G3 | `worst_case_pose` hard-codes q2 = −90°, which is only right for `custom`. | The IRB J2 static load is understated: 6.03 N·m reported, but a q2/q3 sweep within the joint limits reaches **10.08 N·m** at about (90°, −80°). | Phase 2 |
| G4 | The dynamic demand uses a diagonal point-mass inertia, one scalar acceleration on every joint at once, and no velocity (Coriolis) term. | The demand is a rough estimate, covered only by the 2.5 factor. | Phase 6 (Phase 5 enables it) |
| G5 | The tests read the machine-local `config_default.json`. There are no tests for the config round-trip, the preset registry, or `fk_chain` against `to_rtb`. | Results differ between machines, and regressions in `config.py` go unnoticed. | Phase 1 |
| G6 | Link centres of mass are fixed per preset and don't follow a/d edits. | An edited geometry is sized with the old lever arms. | Phase 4 |
| G7 | IK needs roboticstoolbox, and the IK tests skip without it. | No IK coverage on a lean install. | Phase 7 |

---

## 4. The core idea: an actuator model

### 4.1 What an external gear does

Take an external reduction of ratio **N** (the servo output turns N times per
joint turn) with efficiency **η**:

```
torque at the joint       τ_j     = N · η · τ_servo
speed at the joint        q̇_j     = q̇_servo / N
servo inertia at joint    I_refl  = N² · I_servo
friction                  servo viscous × N², servo Coulomb × N, plus the gear's own
```

### 4.2 Torque falls with speed

At a fixed supply voltage, a DC motor's torque falls linearly. It starts at
stall torque at zero speed and reaches zero at the no-load speed. A serial-bus
servo's datasheet stall torque and no-load speed are already measured at its
output, so at the joint:

```
τ_avail(q̇_j) = N · η · τ_stall · (1 − N · |q̇_j| / ω₀)        for |q̇_j| ≤ ω₀ / N
```

Here **ω₀** is the servo's no-load output speed **at the supply voltage
actually used**. This has two consequences:

- **More gearing means a slower joint.** Raising N lowers the joint's top speed
  to ω₀/N. Adding gearing until the static check passes can make the dynamic
  check fail at the speed the task needs.
- **Stall torque overstates what a moving joint can deliver.** It is the torque
  at zero speed, and the dynamic demand happens while the joint is moving.

### 4.3 The inertia hidden inside the servo

A hobby servo already contains a high internal reduction. Its motor rotor's
inertia shows up at the servo output multiplied by the **square of that
internal ratio**. As a result:

- The datum that matters is the servo's **output-side** inertia
  (`I_servo` = rotor inertia × internal ratio²), not the bare rotor inertia.
- It loads the joint **even with no external gear** (N = 1), and is multiplied
  by N² when there is one.

Today's dynamic check leaves this term out entirely.

### 4.4 The proposed checks

`q*` is the worst-case pose and `SF` the safety factor. For each joint j:

```
dynamic demand  τ_dyn,j = |g_j(q*)| + (I_eff,j(q*) + N_j² · I_servo,j) · q̈_peak

static   |g_j(q*)| · SF  ≤  N_j · η_j · τ_stall,j              (q̇ = 0)
dynamic  τ_dyn,j · SF    ≤  τ_avail,j(q̇_peak)
speed    q̇_peak          ≤  ω₀,j / N_j
```

These checks assume peak torque and peak speed happen at the same moment.
That is conservative, because in a real move they rarely coincide. Phase 6
removes the conservatism by checking the actual (q̇(t), τ(t)) pairs along a
trajectory.

The safety factor stays at 2.5 for now. Some of what it used to cover (speed,
gear losses, servo inertia) is now modelled explicitly. Lowering it is a
decision for later, once real servo data exists.

---

## 5. Proposed work, phase by phase

Each phase ends with a green test run. Existing tests stay green unless we
agree that a test was wrong.

### Phase 0: Baseline

- Create a virtual environment and record the test result and both sizing
  tables (`python -m arm6dof.torque`, `python -m arm6dof.dynamics`) for both
  presets.
- **Why:** every later phase changes numbers, so each one needs a "before" to
  compare against.
- **Code changes:** none.

### Phase 1: Test isolation (G5)

- Build the test models from `preset(...)`, or isolate the config loader, so a
  local `config_default.json` can't change the results.
- Add tests for:
  - a `config.py` save → load round-trip;
  - the preset registry;
  - `fk_chain` against `to_rtb` (skipped when roboticstoolbox is missing).
- **Done when:** the tests pass on a machine with and without
  `config_default.json`.
- **Why it comes first:** the later phases change numbers on purpose. They need
  tests whose expected values don't depend on the machine.

### Phase 2: Worst-case pose search (G3)

- Replace the hard-coded pose with a search over the joint limits, done per
  joint, because each joint's worst pose can differ.
- Keep it NumPy-only: a coarse grid over the joints that move the load, then a
  local refinement.
- Search the static objective (`|g_j|`) and the dynamic one
  (`|g_j| + I_j·q̈`) separately. Their worst poses can differ.
- **API, to be agreed:** keep `worst_case_pose()` for existing callers and add
  a per-joint variant used by the sizing reports.
- **Done when:** IRB J2 static is about 10.08 N·m (matching the known sweep),
  and `custom` is unchanged or higher.
- **Note:** maximising `|g_j(q)|` within the joint limits is a small nonlinear
  program. CasADi could solve it, but grid plus refinement keeps the core free
  of extra dependencies.

### Phase 3: Actuator model in sizing (G1, G2). The main course-derived change

New **optional** `JointSpec` fields. Their defaults keep today's behaviour:

| Field | Unit | Meaning | Default when unknown |
|---|---|---|---|
| `servo_no_load_speed` | rad/s | Servo output no-load speed at the supply voltage used | `None`: speed checks skipped, row reports "no data" |
| `servo_reflected_inertia` | kg·m² | Servo output-side inertia (rotor × internal ratio²) | `None`: treated as 0 and reported |
| `gear_efficiency` | – | Efficiency η of the external gear | `1.0` |

Sizing changes:

1. Add `N² · I_servo` to the inertia each joint accelerates.
2. Rate each joint at `N · η · τ_stall` instead of `N · τ_stall`.
3. Add a **torque–speed check** with a new peak-velocity input (default to be
   agreed, like `DEFAULT_PEAK_ACCEL = 8.0` rad/s²).
4. Add a **speed check**: `q̇_peak ≤ ω₀ / N`.

The reports gain these columns: joint no-load speed `ω₀/N`, available torque
at `q̇_peak`, and the reason for a FAIL (torque, speed, or no data). The GUI
table shows the same columns, and the GUI gets a peak-velocity control styled
through `theme.py` tokens. GUI changes need a visual check on the desktop.

**Where the data lives:** the new values are servo data, so they go in
`model.py` next to `servo_stall_torque`. They are not config overrides, so the
JSON schema stays at version 1. The figures must refer to the supply voltage
actually used, and that voltage is recorded next to the servo constants.

**Tests:**
- one unit test per formula;
- with the new fields unset, the results are identical to today.

**Data needed (it won't be guessed):**
- no-load speed at the supply voltage, and the voltage itself;
- the servo's output-side inertia, or the rotor inertia plus the internal
  gear ratio;
- the external gear's efficiency;
- a realistic peak joint velocity.

### Phase 4: Centres of mass follow the geometry (G6)

This isn't from the course, but every sizing number depends on it, and the
URDF export in Phase 5 needs masses that match the geometry. The
representation is still to be decided:

- **A.** Store the CoM as a fraction along the link's a/d vector, recomputed
  whenever the geometry is overridden.
- **B.** Store the CoM relative to the link's two end points.
- **C.** Keep it fixed, and warn when the geometry is edited.

### Phase 5: URDF export

`to_urdf(model)` builds a URDF from the DH table, the masses and the limits.

**DH → URDF mapping.** A standard DH row i is
`Rz(θ₀ᵢ + qᵢ) · Tz(dᵢ) · Tx(aᵢ) · Rx(αᵢ)`. A URDF joint applies a fixed origin
transform first and then rotates about its own axis. So joint i's origin
carries the fixed part of the *previous* row plus its own offset:

```
origin_i = Tz(d_{i−1}) · Tx(a_{i−1}) · Rx(α_{i−1}) · Rz(θ₀ᵢ)      axis = z
```

For the first joint the "previous row" is the identity. A final fixed joint
carries row 6's fixed part, and then the tool offset.

- **Inertias:** a point mass has zero rotational inertia, and dynamics
  libraries expect proper inertia tensors. The export uses simple solids (for
  example a cylinder along each link) and says plainly that this is an
  approximation.
- **Limits:**
  - position: from the model;
  - effort: `N · η · τ_stall`;
  - velocity: `ω₀ / N`, when known.
- **Validation:** forward kinematics from the URDF equals `fk_chain` at random
  q. The URDF parser stays an optional import.
- **What this enables:** the exact `M(q)`, `C(q, q̇)` and `g(q)` through
  Pinocchio or roboticstoolbox, plus visualisation in ROS 2 tools.

### Phase 6: Torque demand from real trajectories (optional, later)

- **Inverse dynamics along representative moves** (for example pick-and-place
  motions inside the workspace): `τ = M(q)·q̈ + h(q, q̇)`. This gives τ(t) and
  q̇(t) for each joint, and every point is checked against the torque–speed
  line. It replaces the single scalar acceleration applied to all joints at
  once.
- **Minimum-time optimal control:** joint accelerations are the input, with a
  free time step `h` in the cost. The constraints are:
  - torque bounds from the actuator model;
  - velocity bounds `ω₀/N`;
  - the target reached at rest.

  This answers directly "how fast can this arm, with these servos, do this
  move?"
- **Caveats from the course:**
  - With a free step, the accuracy of the integration depends on the solution.
  - The solver can exploit integration error to lower the cost artificially.

  So every result must be checked with a finer simulation.
- **Dependencies:** CasADi plus Pinocchio (or equivalent) are heavy. They stay
  out of the core install, either as an optional extra or as a separate tool.

### Phase 7: IK fallback without dependencies (optional)

- A damped Gauss–Newton step, `Δq = (JᵀJ + λI)⁻¹ Jᵀ e`, with a backtracking
  line search. It uses the full 6-D pose error (the course version is
  position-only) and a Jacobian built from `joint_frames`.
- It is used when roboticstoolbox is missing, so the IK tests stop skipping.
- Analytic IK with Pieper's method remains a separate item on the plan.

---

## 6. Impact on the project

### Short term (Phases 0–4)

| Area | Impact |
|---|---|
| **Sizing verdicts** | More honest and generally stricter. The IRB J2 static demand rises by about 67 % (6.03 → 10.08 N·m), and the dynamic check now accounts for speed and servo inertia. Expect more `FAIL` rows before any hardware changes. |
| **Gear ratio** | No longer a free multiplier. Each gear-ratio spinbox now shows a trade-off: torque × N·η, top speed ÷ N, servo inertia × N². |
| **Buying servos** | Each candidate servo needs two more datasheet figures: no-load speed at our voltage, and output-side inertia (or enough data to compute it). The shopping list gains a "required joint speed" column. |
| **Reproducibility** | Every machine gives the same test results. |
| **Backward compatibility** | The new fields are optional, and leaving them unset gives today's numbers. The exception is the worst-case pose fix, which changes the IRB numbers on purpose. The JSON config schema doesn't change. |
| **GUI** | Extra columns and a peak-velocity control, which need a visual check on the desktop. |
| **Dependencies** | Nothing new for the core, which stays NumPy-only. |

### Long term (Phases 5–7 and beyond)

| Area | Impact |
|---|---|
| **Exact dynamics** | The URDF gives the full `M`, `C` and `g`, which removes the diagonal-inertia and no-Coriolis simplifications. Once real servo data exists, the safety factor can then shrink to what it should really cover: datasheet optimism, voltage sag and thermal limits. |
| **Sizing from tasks** | The torque demand comes from the moves the arm must actually make, tied to a target cycle time, instead of one acceleration applied to every joint. |
| **Simulation / ROS 2** | The URDF is the entry point to RViz, Gazebo and MoveIt. |
| **Control** | Exact dynamics allow model-based control such as computed torque or operational-space control. But hobby serial-bus servos are **position-controlled**: torque-level control needs current-controlled actuators. This has to be decided when the hardware is chosen. Streaming optimised position trajectories works with hobby servos; torque control needs different actuators. |
| **`hardware/` layer** | The course gives a template for position-commanded joints: a planned trajectory, fixed-rate interpolation, position streaming, a homing ramp, user confirmation before motion, and joint limits that allow for braking distance. |
| **Testing** | Simulating Coulomb friction and randomised inertias stress-tests controllers before they run on hardware. |
| **Maintenance** | More model parameters to keep consistent. The heavy optional dependencies (CasADi, Pinocchio) must stay out of the core install. |

### Risks and limits

- **False precision.** A better model of placeholder geometry and masses makes
  the numbers more precise, not more accurate. Until the frame is measured,
  the results are for comparing designs, not for signing them off.
- **Datasheet quality.** Hobby servo datasheets are often optimistic, or quote
  figures at a different voltage. Measuring stall torque and no-load speed on
  the bench at the real supply voltage is worth more than any modelling
  refinement.
- **The linear torque–speed line is an idealisation.** Servo electronics, such
  as current limits and the internal control loop, can cut torque sooner.
  Treat the line as an upper bound.
- **Scope creep.** Phases 5–7 are only worth doing after Phases 0–3 have landed
  and real servo data exists.

---

## 7. What does not change

- **Build order:** model → FK → torque sizing → buy servos → IK → control.
  Sizing correctness comes before new features.
- `model.py` holds data only, and the maths lives in the other modules. The
  dataclasses stay `frozen=True`, and overrides go through
  `dataclasses.replace`.
- roboticstoolbox and spatialmath stay lazy imports, so the core and its tests
  run with NumPy alone.
- The model uses SI units. Millimetres, grams and degrees appear only in the
  GUI, the JSON config, and names ending in `_deg`.
- All GUI styling goes through `theme.py` tokens.

---

## 8. Decisions needed

1. Which phases, and in what order. Recommended: 0 → 1 → 2 → 3, then decide on
   4–7.
2. Servo data:
   - no-load speed at the supply voltage, and the voltage itself;
   - output-side inertia (or rotor inertia plus the internal ratio);
   - gear efficiency.

   Missing values stay empty, and the related check reports "no data".
3. A realistic peak joint velocity for the torque–speed check (the
   counterpart of the 8 rad/s² default acceleration).
4. The inertia field name: `servo_reflected_inertia` (output-side, as
   proposed) or `servo_rotor_inertia`.
5. The CoM representation for Phase 4.
6. Whether Phases 5–6 live in this repository as optional extras, or in a
   separate tool.

---

## 9. References

- A. Del Prete, *Optimization-based Robot Control*, course lab code,
  <https://github.com/andreadelprete/orc> (GPL-3.0). Only the ideas and
  equations are used; no code is copied.
