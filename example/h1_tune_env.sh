# ── Steering Stabilization Parameters (88° transition fix) ────────────────
# Problem: Robot tips over when steering from +X (forward) to +Y (lateral).
# Root cause: Resonant oscillation in steer↔drive coupling during large angle changes.
# Solution: Increase damping (35-40 Nm·s/rad), reduce steer gains, slow steer rate.

# Reduced steer gains: 120→60 (Kp), 70→40 (Kd)
export TOPSTAR_H1_STEER_KP=60
export TOPSTAR_H1_STEER_KD=40
export TOPSTAR_H1_MAX_LATERAL_SPEED=0.08
export TOPSTAR_H1_MAX_LATERAL_ACCEL=0.08
export TOPSTAR_H1_LATERAL_SHAPE_MIN_SCALE=0.35
export TOPSTAR_H1_LATERAL_SHAPE_STEER_ERR_REF=0.45
export TOPSTAR_H1_LATERAL_SHAPE_STEER_RATE_REF=0.25
export TOPSTAR_H1_LATERAL_SHAPE_ROLL_REF=0.20
export TOPSTAR_H1_LATERAL_SHAPE_PITCH_REF=0.20
export TOPSTAR_H1_DRIVE_TORQUE_BUDGET_MIN_SCALE=0.45
export TOPSTAR_H1_DRIVE_TORQUE_BUDGET_STEER_ERR_REF=0.40
export TOPSTAR_H1_DRIVE_TORQUE_BUDGET_STEER_RATE_REF=0.20

# CRITICAL: Increased drive damping during misalignment (18→40 N·m·s/rad)
# Suppresses oscillation in steer↔drive coupling. This is the primary tipover fix.
export TOPSTAR_H1_ALIGN_DRIVE_DAMPING=40

# Increased misalignment braking (15→35 N·m·s/rad)
export TOPSTAR_H1_LARGE_STEER_BRAKE_DAMPING=40

# Keep large-angle braking clamp aligned with controller default.
export TOPSTAR_H1_LARGE_STEER_MAX_BRAKE_TORQUE=90

# Reduced max steer torque (45→30 Nm) to prevent drive coupling
export TOPSTAR_H1_MAX_STEER_WAIT_TORQUE=25

# Slower steer rotation (0.3→0.15 rad/s) to allow damping to stabilize
export TOPSTAR_H1_MAX_STEER_RATE_COS=0.12

# Relaxed overspeed threshold (10→12 rad/s) - wheels couple during transitions
export TOPSTAR_H1_HARD_OVERSPEED_RATE=12
export TOPSTAR_H1_HARD_OVERSPEED_INSTANT_RATE=30
export TOPSTAR_H1_HARD_OVERSPEED_CONFIRM_STEPS=3
export TOPSTAR_H1_HARD_OVERSPEED_REARM_DELAY_S=0.25

# Per-wheel overspeed recovery: keep individual wheels in damp-to-zero mode
# until speed is genuinely settled, preventing repeated latch retriggers.
export TOPSTAR_H1_OVERSPEED_STEP_FRACTION=0.12
export TOPSTAR_H1_DRIVE_DAMPING_FF_ALIGNED=20
export TOPSTAR_H1_DRIVE_FF_ALIGN_MIN=0.98
export TOPSTAR_H1_DRIVE_FF_STEER_RATE_MAX=0.08
export TOPSTAR_H1_WHEEL_OVERSPEED_HOLD_S=0.8
export TOPSTAR_H1_WHEEL_OVERSPEED_RELEASE_RATE=2.0
export TOPSTAR_H1_WHEEL_OVERSPEED_BRAKE_DAMPING=80
export TOPSTAR_H1_WHEEL_OVERSPEED_MAX_BRAKE_TORQUE=90
export TOPSTAR_H1_DRIVE_TORQUE_SLEW=1800
export TOPSTAR_H1_WHEEL_FAULT_WINDOW_S=1.0
export TOPSTAR_H1_WHEEL_FAULT_HITS=3
export TOPSTAR_H1_WHEEL_FAULT_DISABLE_S=2.0

# ── Idle-mode tuning (from original h1_tune_env.sh) ──────────────────────
export TOPSTAR_H1_IDLE_STEER_HOLD_KP=60
export TOPSTAR_H1_IDLE_STEER_HOLD_KD=14
export TOPSTAR_H1_IDLE_DRIVE_DAMPING=12
export TOPSTAR_H1_IDLE_DRIVE_STATIC_TORQUE=6
export TOPSTAR_H1_MAX_IDLE_DRIVE_TORQUE=30