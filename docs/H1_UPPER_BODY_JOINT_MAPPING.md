# H1 Upper Body Joint Mapping: Code ↔ URDF Correspondence

**Date:** 2026-04-10  
**Relevant files:**
- URDF: `~/topstar_h1/Topstar/Topstar.urdf`
- MuJoCo model: `~/topstar_mujoco/topstar_robots/h1/h1.xml`
- Controller code: `~/lerobot4/src/lerobot/robots/topstar/topstar_human.py`
- Teleop config: `~/lerobot4/configs/teleop_topstar_human.json`

---

## Background

`topstar_human.py` defines an 18-DOF humanoid with two 7-DOF arms plus 2 extra axes each:

| Controller | Arm joints | Extra axis 0 (joint[·,7]) | Extra axis 1 (joint[·,8]) |
|------------|-----------|--------------------------|--------------------------|
| robot0 (192.168.1.10) | joints[0, 0:7] | `torso_pitch` | `torso_z` |
| robot1 (192.168.1.9)  | joints[1, 0:7] | `head_yaw`    | `head_pitch` |

The URDF uses CAD-exported names that do not match these logical names.  
This document resolves the correspondence and documents the sign convention mismatch.

---

## Joint Correspondence Table

| Logical name (code) | URDF / MuJoCo joint name | Joint type | World-frame axis | URDF range |
|---------------------|--------------------------|------------|-----------------|------------|
| `torso_pitch` — robot0 joint[7] | `Robot_Body_Rotation_Joint` | revolute | **+Y** (pitch forward) | [-1.6581, 0] rad = [-95°, 0°] |
| `torso_z` / `torso_lift` — robot0 joint[8] | `Robot_Body_Movement_Joint` | prismatic | **−Z** (height) | [-0.45, 0] m |
| `head_yaw` — robot1 joint[7] | `Robot_Head_Rotation_Joint` | revolute | **−Z** (left-right turn) | [-1.5708, 1.5708] rad = [-90°, +90°] |
| `head_pitch` — robot1 joint[8] | `Robot_Head_Tonod_Joint` | revolute | **+Y** (nod up-down) | [-0.489, 0.559] rad = [-28°, +32°] |

### Kinematic chain in URDF

```
base_link
└── Robot_Body_Movement_Link   ← Robot_Body_Movement_Joint  [prismatic,  torso_z]
    └── Robot_Body_Rotation_Link ← Robot_Body_Rotation_Joint [revolute,   torso_pitch]
        ├── Robot_Head_Rotation_Link ← Robot_Head_Rotation_Joint [revolute, head_yaw]
        │   └── Robot_Head_Tonod_Link ← Robot_Head_Tonod_Joint   [revolute, head_pitch]
        ├── Robot_Left_Hand_base_Link  ← (left arm, 7 DOF)
        └── Robot_Right_Hand_base_Link ← (right arm, 7 DOF)
```

---

## World-Frame Axis Derivation

All URDF joints use local axis `[0 0 1]` or `[0 0 −1]` (SolidWorks export convention).  
The actual world-frame axis is found by propagating the RPY rotations through the kinematic chain.

### Robot_Body_Movement_Joint (torso_z)

Parent body origin: `rpy = (−π, 0, π)`

```
R_base→movement = Rz(π) · Ry(0) · Rx(−π)
                = [[-1, 0, 0],
                   [ 0, 1, 0],
                   [ 0, 0,-1]]

world axis = R · [0, 0, 1]ᵀ = [0, 0, −1]   ← vertical, downward
```

Prismatic along **−Z**: positive hardware displacement lifts the torso (moves along −[0,0,−1] = +Z).

### Robot_Body_Rotation_Joint (torso_pitch)

Parent body origin (in movement_link frame): `rpy = (−π/2, 0, 0)`

```
R_base→rotation = R_base→movement · Rx(−π/2)
               = [[-1, 0, 0],
                  [ 0, 0, 1],
                  [ 0, 1, 0]]

world axis = R · [0, 0, 1]ᵀ = [0, 1, 0]   ← world Y = pitch
```

Revolute around **+Y**: pitches the torso forward.  
> **Note:** The URDF name "Rotation" is ambiguous — it is **pitch**, not yaw.

### Robot_Head_Rotation_Joint (head_yaw)

Parent body origin (in rotation_link frame): `rpy = (π/2, 0, −π)`

```
R_base→head_rot = R_base→rotation · Rz(−π) · Rx(π/2)
               = identity  (numerically)

world axis = I · [0, 0, −1]ᵀ = [0, 0, −1]   ← world −Z = yaw
```

Revolute around **−Z**: left-right head turn.

### Robot_Head_Tonod_Joint (head_pitch)

Parent body origin (in head_rotation_link frame): `rpy = (π/2, 0, 0)`

```
R_base→tonod = R_base→head_rot · Rx(π/2)
             = [[1, 0, 0],
                [0, 0,-1],
                [0, 1, 0]]

world axis = R · [0, 0, −1]ᵀ = [0, 1, 0]   ← world +Y = pitch
```

Revolute around **+Y**: head nod (pitch). "Tonod" = "to nod".

---

## Cross-Verification via Joint Limits

| Joint | URDF range | Hardware config (teleop_topstar_human.json) | Match? |
|-------|-----------|----------------------------------------------|--------|
| `Robot_Body_Movement_Joint` | 0.45 m | limits [-10, 450] mm × scale 0.001 = [-0.01, 0.45] m | ✓ magnitude |
| `Robot_Body_Rotation_Joint` | 95° | limits [0, 95] deg × scale 0.01745 = [0, 1.658 rad] | ✓ magnitude |
| `Robot_Head_Rotation_Joint` | ±90° | limits [-90, 90] deg × scale 0.01745 | ✓ exact |
| `Robot_Head_Tonod_Joint` | [-28°, +32°] | limits [-40, 28] deg × scale 0.01745 | ~ (conservative) |

---

## Sign Convention Inconsistency

The hardware controller and URDF define the **positive direction** of motion oppositely for three joints.

| Joint | Hardware positive direction | URDF positive direction | Sign relationship |
|-------|----------------------------|------------------------|-------------------|
| `torso_pitch` | forward tilt → **+[0°, 95°]** | forward tilt → **−[0, −95°]** | `q_urdf = −q_hw` |
| `torso_z` | raise torso → **+[0, 450 mm]** | raise torso → **−[0, −0.45 m]** | `q_urdf = −q_hw` |
| `head_pitch` | look down → **+[0°, 28°]** | look down → **−[0, −28°]** | `q_urdf = −q_hw` |
| `head_yaw` | turn left → **+[0°, 90°]** | turn left → **+[0, 1.57 rad]** | `q_urdf = q_hw` ✓ |

**Root cause:** The SolidWorks URDF defines torso lift/pitch in the *negative* direction of the joint axis (joint at home = q=0, fully extended = q = lower limit < 0), while the firmware reports physical displacement as a positive value.

---

## Fix: Applying Sign Corrections in the MuJoCo Bridge

When commanding the MuJoCo model from hardware state (or vice versa), apply sign inversions:

```python
# Hardware (radians/meters) → MuJoCo URDF joint value
mj_data.joint("Robot_Body_Movement_Joint").qpos[0] = -torso_z_hw        # negate
mj_data.joint("Robot_Body_Rotation_Joint").qpos[0]  = -torso_pitch_hw   # negate
mj_data.joint("Robot_Head_Rotation_Joint").qpos[0]  =  head_yaw_hw      # no change
mj_data.joint("Robot_Head_Tonod_Joint").qpos[0]     = -head_pitch_hw    # negate

# MuJoCo URDF joint value → hardware command (inverse)
torso_z_cmd     = -mj_data.joint("Robot_Body_Movement_Joint").qpos[0]
torso_pitch_cmd = -mj_data.joint("Robot_Body_Rotation_Joint").qpos[0]
head_yaw_cmd    =  mj_data.joint("Robot_Head_Rotation_Joint").qpos[0]
head_pitch_cmd  = -mj_data.joint("Robot_Head_Tonod_Joint").qpos[0]
```

> The hardware controller and lerobot code are internally consistent (tested on real hardware).  
> The sign correction is only needed at the **MuJoCo interface layer**.

---

## MuJoCo Model Status

In `h1.xml`, both torso joints are currently **locked** (zero-torque equality constraints):

```xml
<!-- from prepare_h1_model.py injection -->
<joint joint1="Robot_Body_Movement_Joint" polycoef="0 0 0 0 0"/>
<joint joint1="Robot_Body_Rotation_Joint" polycoef="0 0 0 0 0"/>
```

This was added to prevent upper-body collapse during base motion testing.  
To enable upper-body control in simulation, remove these equality constraints and add  
position actuators for the four joints above, then apply the sign corrections when  
ingesting commands from the lerobot controller.
