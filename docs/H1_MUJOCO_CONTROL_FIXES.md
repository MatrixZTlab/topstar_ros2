# H1 MuJoCo Upper-Body Control — Fixes & Tuning Notes

**Date:** 2026-04-24  
**Affects:**
- `topstar_mujoco/topstar_robots/h1/h1.xml`
- `topstar_mujoco/simulate_python/prepare_h1_model.py`
- `topstar_ros2/example/src/src/h1/topstar_h1/h1_upper_body_jog.py`

---

## Background: How H1 upper-body control actually works

The H1 firmware exposes a **position-only** servo interface.  `LowCmd.motor_cmd[i].kp`
and `.kd` are sent over the wire but the firmware ignores them and uses its own
internal gains.  The ROS 2 side mirrors this: `h1_ros2_node.py` extracts only
`motor_cmd[i].q` (mode == 1) and discards every other field.

In MuJoCo the entire stack is therefore a **trajectory planner**:

```
ROS 2 /h1/lowcmd  →  h1_ros2_node  →  H1Bridge  →  H1UpperBodyController
  →  MockInterpController (linear interpolation, 50 Hz)
  →  mj_data.ctrl[i]  →  MuJoCo <position> actuator  →  joint torque
```

The only knobs that govern simulation fidelity are the **static gains** (`kp`, `kv`)
baked into `h1.xml` at model-load time.  Per-command gain tuning has no effect.

---

## Fix 1 — Torso lift oscillation (critically underdamped gains)

### Symptom

`Robot_Body_Movement_Joint` (torso lift, prismatic) oscillated continuously when
jogged to any non-zero target.

### Root cause

The damping gain `kv = 100 N·s/m` was copied from revolute arm joints where the
effective inertia is small.  The torso lift carries the full upper body:

| Body | Mass (kg) |
|---|---|
| Robot_Body_Movement_Link (slider rod) | 1.016 |
| Robot_Body_Rotation_Link | 5.764 |
| Head links | 2.138 |
| Left arm (7 links) | 6.163 |
| Right arm (7 links) | 6.163 |
| **Total effective mass m_eff** | **21.24 kg** |

For a spring-damper position actuator, critical damping requires:

```
c_crit = 2 √(kp · m_eff) = 2 √(5000 × 21.24) = 652 N·s/m
ζ_old  = kv_old / c_crit = 100 / 652 = 0.15   ← severely underdamped
```

### Fix

`kv` raised from **100 → 652** N·s/m (ζ = 1.0, critical damping).

Additionally, the prismatic joint axis is anti-parallel to world-Z, so gravity
creates a constant generalized force along the positive joint direction:

```
F_grav = m_eff × g = 21.24 × 9.81 ≈ 208 N
```

With pure position feedback, the steady-state sag is:

```
e_ss = F_grav / kp = 208 / 5000 = 0.042 m  (42 mm)
```

A gravity feedforward constant was added via `biasprm[0]` to cancel this load,
giving < 0.1 mm steady-state error.

### Changes in `h1.xml`

The `<position>` shorthand does not accept `biastype` overrides, so the torso lift
actuator was converted to `<general>`:

```xml
<!-- Before -->
<position name="Robot_Body_Movement_Joint_act"
          joint="Robot_Body_Movement_Joint"
          kp="5000" kv="100" ctrlrange="-0.45 0"/>

<!-- After -->
<!-- f = kp*(ctrl−q) − kv*dq − F_grav
       = 5000*(ctrl−q) − 652*dq − 208               -->
<general name="Robot_Body_Movement_Joint_act"
         joint="Robot_Body_Movement_Joint"
         gaintype="fixed" gainprm="5000 0 0"
         biastype="affine" biasprm="-208 -5000 -652"
         ctrlrange="-0.45 0"/>
```

`biasprm = [b₀, b₁, b₂]` where the total bias force is `b₀ + b₁·q + b₂·q̇`.
Setting `b₀ = −208` injects a constant 208 N feedforward opposing gravity, while
`b₁ = −kp` and `b₂ = −kv` implement the standard position-plus-velocity feedback.

The `implicitfast` integrator (set in `scene.xml`) treats the `b₂·q̇` velocity
term implicitly, so numerical stability holds for any reasonable `kv`.

---

## Fix 2 — Torso pitch and shoulder oscillation (incorrect kv pass 1 & 2)

The first pass estimated effective inertias analytically.  The second pass used
`mujoco.mj_fullM` to read the exact diagonal of the joint-space mass matrix at
the zero configuration, which is the authoritative value for the simulation.

```python
M = np.zeros((m.nv, m.nv))
mujoco.mj_fullM(m, M, d.qM)
I_eff = M[dof, dof]          # effective inertia for DOF `dof`
c_crit = 2.0 * np.sqrt(kp * I_eff)
```

| Joint | kp | I_eff (kg·m²) | c_crit | kv old | kv new | ζ old → new |
|---|---|---|---|---|---|---|
| Robot_Body_Rotation_Joint | 3000 | 5.50 | 257 | 175 | **257** | 0.68 → **1.00** |
| Robot_Right/Left_Hand_1_Joint (shoulder) | 800→2000 | 0.640 | 72 | 30 | **72** | 0.66 → **1.01** |

The torso pitch (`Robot_Body_Rotation_Joint`) was also converted to `<general>` to
use the same formulation (no gravity bias since symmetric arm loading at rest keeps
net pitch torque near zero):

```xml
<!-- Before: kv=175 (ζ=0.68) -->
<!-- After:  kv=257 (ζ=1.00) -->
<general name="Robot_Body_Rotation_Joint_act"
         joint="Robot_Body_Rotation_Joint"
         gaintype="fixed" gainprm="3000 0 0"
         biastype="affine" biasprm="0 -3000 -257"
         ctrlrange="-1.6581 0"/>
```

---

## Fix 3 — Shoulder and elbow stiffness (kp too low)

With `kp = 800 N·m/rad` on shoulder and elbow joints, the gravity-induced
steady-state error at worst-case horizontal extension was:

```
τ_grav ≈ m_arm × g × r_COM ≈ 6.2 × 9.81 × 0.3 ≈ 18 N·m
e_ss   = τ_grav / kp       = 18 / 800            ≈ 0.023 rad  (1.3°)
```

`kp` was raised from **800 → 2000 N·m/rad** on all shoulder and elbow joints
(Hand_base, Hand_1, Hand_2, Hand_3 for both arms), reducing worst-case sag to
≈ 0.5°.  `kv` was re-derived for critical damping at the new `kp`:

| Joint (both arms) | kp old | kp new | kv old | kv new | I_eff | ζ |
|---|---|---|---|---|---|---|
| Hand_base (shoulder abduction) | 800 | **2000** | 30 | 30 | 0.021 | 2.3 |
| Hand_1 (shoulder flexion) | 800 | **2000** | 30 | **72** | 0.640 | 1.01 |
| Hand_2 (elbow yaw) | 800 | **2000** | 30 | 30 | 0.016 | 2.7 |
| Hand_3 (elbow flexion) | 800 | **2000** | 30 | 30 | 0.116 | 0.98 |

Wrist joints (Hand_4–6, `kp = 600`) were left unchanged; their small inertia
(< 0.015 kg·m²) means they settle quickly even when overdamped.

**Step-response verification** (2 s simulation, target 0.3–0.5 rad):

| Joint | Overshoot | Steady-state error | Final velocity |
|---|---|---|---|
| Robot_Body_Rotation_Joint | −3.5 % | 0.53° (gravity) | ≈ 0 |
| Robot_Right_Hand_1_Joint | −2.1 % | 0.38° (gravity) | ≈ 0 |
| Robot_Right_Hand_3_Joint | −1.8 % | 0.10° (gravity) | ≈ 0 |

Residual steady-state error is gravity-induced sag; it is configuration-dependent
for revolute joints and would require a per-pose feedforward to eliminate.

---

## Fix 4 — GUI joint limits incorrect (`h1_upper_body_jog.py`)

All arm joint limits in `JOINT_SPECS` were symmetric guesses.  They were replaced
with the exact values from `m.jnt_range` as loaded by MuJoCo.

Key corrections:

| Joint | Old range | Correct range | Issue |
|---|---|---|---|
| R/L Shoulder Base | ±1.5 | **−1.57 → +3.14** | positive half missing |
| R Shoulder | ±1.5 | **−1.5708 → +0.4363** | high limit 3.4× too large |
| R Elbow Yaw | ±2.0 | **±1.57** | too wide |
| R Elbow | ±1.571 | **−1.7977 → +0.4363** | asymmetric, both endpoints wrong |
| L Shoulder | ±1.5 | **−1.8 → +0.4363** | low limit wrong |
| **L Elbow** | ±1.571 | **−0.4363 → +1.7977** | completely inverted — opposite handedness |
| R/L Wrist Yaw | ±2.0 | **±2.9322** | too narrow |
| R/L Wrist Pitch | ±1.2 | **±2.3562** | too narrow |
| R/L Wrist Roll | ±3.142 | **±2.8798** | slightly too wide |

Note: torso and head limits were correct because they were derived analytically from
the sign-convention documentation (`H1_UPPER_BODY_JOINT_MAPPING.md`).

Sign convention reminder for the four torso/head joints:

| H1JointIndex | URDF joint | MuJoCo range | GUI (hardware) range |
|---|---|---|---|
| TORSO_LIFT | Robot_Body_Movement_Joint | [−0.45, 0] m | [0, 0.45] m |
| TORSO_PITCH | Robot_Body_Rotation_Joint | [−1.658, 0] rad | [0, 1.658] rad |
| HEAD_YAW | Robot_Head_Rotation_Joint | [−π/2, +π/2] | same |
| HEAD_PITCH | Robot_Head_Tonod_Joint | [−0.489, 0.559] rad | [−0.559, 0.489] rad |

The arm joints (indices 4–17) have no sign flip: hardware convention == MuJoCo
convention.

---

## prepare_h1_model.py — authoritative gain source

`h1.xml` is generated by `prepare_h1_model.py` (reads the URDF via
`mujoco.mj_saveLastXML` then injects actuators).  Editing `h1.xml` directly is
sufficient for the current session but will be overwritten on the next rebuild.
All gain changes above are also reflected in the `H1_UPPER_BODY_JOINTS` table in
`prepare_h1_model.py`, which is the authoritative source.

The `gravity_comp` column added to that table drives a conditional that emits
`<general … biasprm="b₀ …">` instead of `<position …>` for any joint with a
non-zero constant gravity load:

```python
# gravity_comp != 0  →  emit <general> with feedforward bias
# gravity_comp == 0  →  emit <position> (shorthand, no bias override needed)
```

---

## Complete final gain table

| Joint | kp | kv | ζ | gravity_comp |
|---|---|---|---|---|
| Robot_Body_Movement_Joint | 5000 | 652 | 1.00 | −208 N |
| Robot_Body_Rotation_Joint | 3000 | 257 | 1.00 | — |
| Robot_Head_Rotation_Joint | 500 | 20 | 3.0 | — |
| Robot_Head_Tonod_Joint | 500 | 20 | 3.5 | — |
| Robot_Right/Left_Hand_base_Joint | 2000 | 30 | 2.3 | — |
| Robot_Right/Left_Hand_1_Joint | 2000 | 72 | 1.01 | — |
| Robot_Right/Left_Hand_2_Joint | 2000 | 30 | 2.7 | — |
| Robot_Right/Left_Hand_3_Joint | 2000 | 30 | 0.98 | — |
| Robot_Right/Left_Hand_4_Joint | 600 | 20 | 3.8 | — |
| Robot_Right/Left_Hand_5_Joint | 600 | 20 | 3.3 | — |
| Robot_Right/Left_Hand_6_Joint | 600 | 20 | 4.1 | — |

Head and wrist joints are intentionally overdamped (ζ > 1): their very small
inertia (< 0.025 kg·m²) means they still settle in < 50 ms despite the high
damping ratio.
