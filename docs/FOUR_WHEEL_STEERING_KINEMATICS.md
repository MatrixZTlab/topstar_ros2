# Four-Wheel Steerable Chassis Kinematics

This note summarizes practical web references and the core kinematic model for a chassis with **4 independently steerable wheels** (often called **swerve drive** or **4-wheel independent steering**).

## Key web references

### 1. WPILib — Swerve Drive Kinematics
https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html

Useful for:
- converting a desired chassis motion `(v_x, v_y, \omega)` into each wheel's **steering angle** and **wheel speed**
- defining wheel module positions relative to the robot center
- field-oriented motion and custom centers of rotation

### 2. ChiefDelphi — Paper: 4 wheel independent drive & independent steering ("swerve")
https://www.chiefdelphi.com/t/paper-4-wheel-independent-drive-independent-steering-swerve/107383

Useful for:
- inverse kinematics derivation
- equations and pseudocode
- Ackermann-to-3DOF conversion ideas

### 3. Wikipedia — Steering
https://en.wikipedia.org/wiki/Steering

Relevant sections:
- **Geometry** → Ackermann steering idea
- **Four-wheel steering** → low-speed opposite-phase / high-speed same-phase steering
- **Crab steering** → all wheels point in the same direction

## Core kinematic model

For wheel or module `i` at position $(x_i, y_i)$ relative to the chassis center, and desired chassis twist:

- linear velocity: $(v_x, v_y)$
- yaw rate: $\omega$

The planar velocity at that wheel is modeled as:

```math
\mathbf{v}_i =
\begin{bmatrix}
 v_x \\
 v_y
\end{bmatrix}
+
\omega
\begin{bmatrix}
 -y_i \\
 x_i
\end{bmatrix}
```

Equivalently:

```math
v_{ix} = v_x - \omega y_i
```

```math
v_{iy} = v_y + \omega x_i
```

Then the wheel commands are:

### Steering angle

```math
\theta_i = \operatorname{atan2}(v_{iy}, v_{ix})
```

### Wheel speed

```math
s_i = \sqrt{v_{ix}^2 + v_{iy}^2}
```

This is the standard **inverse kinematics** used for swerve / independently steerable wheel systems.

## Interpretation for a 4-wheel chassis

Each wheel module usually has:
- **1 steering DOF** — rotate the wheel assembly to a target heading
- **1 rolling DOF** — spin the wheel to produce the desired tangential speed

Therefore:

- **4 wheels × 2 DOF = 8 DOF**

## Motion modes

### 1. Pure translation
If $\omega = 0$, then all wheels point in the same direction and spin at speeds that realize the desired body translation.

### 2. Pure rotation in place
If $v_x = v_y = 0$ and $\omega \neq 0$, each wheel points tangent to its circular path around the robot center.

### 3. General motion
If $(v_x, v_y, \omega)$ are all nonzero, each wheel gets its own angle and speed from the equations above.

### 4. Crab steering
If all wheels are commanded to the same angle and same forward speed, the robot translates diagonally without changing heading.

## Relevance to Topstar H1

For the `H1` robot, the wheel joint naming suggests each wheel is modeled with two joints:

- `Wheel_Rotation_[1-4]_1_Joint` → likely the **steering/swivel** joint
- `Wheel_Rotation_[1-4]_2_Joint` → likely the **wheel roll/spin** joint

So the `H1` mobile base matches the standard **4-module steerable-wheel** model well.

## Practical next step

To control the `H1` base in MuJoCo or in a controller:

1. define each wheel module position $(x_i, y_i)$ relative to the chassis center,
2. choose a desired chassis command $(v_x, v_y, \omega)$,
3. compute $(\theta_i, s_i)$ for all 4 wheels using the equations above,
4. map $\theta_i$ to the steering joints and $s_i$ to the wheel-spin joints.
