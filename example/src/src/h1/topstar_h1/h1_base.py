from __future__ import annotations

import json
import logging
import math
import os
import time
from dataclasses import dataclass
from pathlib import Path

_log = logging.getLogger(__name__)
if not _log.handlers:
    _handler = logging.StreamHandler()
    _handler.setFormatter(logging.Formatter(
        "%(asctime)s %(levelname)-8s [h1_base] %(message)s",
        datefmt="%H:%M:%S",
    ))
    _log.addHandler(_handler)
    _log.propagate = False
_log.setLevel(logging.DEBUG)

import numpy as np


@dataclass(frozen=True)
class H1WheelModule:
    label: str
    x: float
    y: float
    steer_joint: str
    drive_joint: str
    drive_sign: float


H1_WHEEL_MODULES = [
    H1WheelModule("rear_right", -0.22, -0.165, "Wheel_Rotation_1_1_Joint", "Wheel_Rotation_1_2_Joint", 1.0),
    H1WheelModule("rear_left", -0.22, 0.165, "Wheel_Rotation_2_1_Joint", "Wheel_Rotation_2_2_Joint", -1.0),
    H1WheelModule("front_right", 0.22, -0.165, "Wheel_Rotation_3_1_Joint", "Wheel_Rotation_3_2_Joint", 1.0),
    H1WheelModule("front_left", 0.22, 0.165, "Wheel_Rotation_4_1_Joint", "Wheel_Rotation_4_2_Joint", -1.0),
]


def wrap_to_pi(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


class H1BaseController:
    """Simple chassis-speed to wheel-command controller for Topstar H1."""

    def __init__(self, mj_model, mj_data):
        self.mj_model = mj_model
        self.mj_data = mj_data
        self.modules = H1_WHEEL_MODULES

        self.wheel_radius = 0.0625
        self.max_linear_speed = float(os.getenv("TOPSTAR_H1_MAX_LINEAR_SPEED", "1.2"))
        self.max_lateral_speed = float(os.getenv("TOPSTAR_H1_MAX_LATERAL_SPEED", "0.08"))
        self.max_angular_speed = float(os.getenv("TOPSTAR_H1_MAX_ANGULAR_SPEED", "2.0"))
        self.command_vx_sign = -1.0
        self.command_vy_sign = -1.0
        # Reduced steer gains to suppress oscillation during large angle transitions (88° steering).
        # Lower Kp reduces coupling torque into drive joint; lower Kd reduces shock loads.
        self.steer_kp = float(os.getenv("TOPSTAR_H1_STEER_KP", "60.0"))
        self.steer_kd = float(os.getenv("TOPSTAR_H1_STEER_KD", "40.0"))
        self.drive_kp = float(os.getenv("TOPSTAR_H1_DRIVE_KP", "40.0"))
        self.drive_torque_slew = float(os.getenv("TOPSTAR_H1_DRIVE_TORQUE_SLEW", "2500.0"))
        # Transition-aware shaping: reduce lateral command only while wheels are
        # reorienting quickly or attitude margin is tight, instead of static limits.
        self.lateral_shape_min_scale = float(os.getenv("TOPSTAR_H1_LATERAL_SHAPE_MIN_SCALE", "0.35"))
        self.lateral_shape_steer_err_ref = float(os.getenv("TOPSTAR_H1_LATERAL_SHAPE_STEER_ERR_REF", "0.45"))
        self.lateral_shape_steer_rate_ref = float(os.getenv("TOPSTAR_H1_LATERAL_SHAPE_STEER_RATE_REF", "0.25"))
        self.lateral_shape_roll_ref = float(os.getenv("TOPSTAR_H1_LATERAL_SHAPE_ROLL_REF", "0.20"))
        self.lateral_shape_pitch_ref = float(os.getenv("TOPSTAR_H1_LATERAL_SHAPE_PITCH_REF", "0.20"))
        # Dynamic drive torque budget during steer transients to reduce
        # steer->drive coupling impulses.
        self.drive_torque_budget_min_scale = float(os.getenv("TOPSTAR_H1_DRIVE_TORQUE_BUDGET_MIN_SCALE", "0.45"))
        self.drive_torque_budget_steer_err_ref = float(os.getenv("TOPSTAR_H1_DRIVE_TORQUE_BUDGET_STEER_ERR_REF", "0.40"))
        self.drive_torque_budget_steer_rate_ref = float(os.getenv("TOPSTAR_H1_DRIVE_TORQUE_BUDGET_STEER_RATE_REF", "0.20"))
        # Feedforward matching XML joint damping for steady-state tracking.
        # Set 0 by default: non-zero FF opposes PD correction when ω overshoots
        # target (FF overcomes PD braking, sustaining 60%+ overshoot that triggers
        # cascade instability). ω_ss with d_ff=0: kp/(kp+d_model)*T ≈ 57% of target.
        self.drive_damping_ff = float(os.getenv("TOPSTAR_H1_DRIVE_DAMPING_FF", "0.0"))
        # Add aligned-only feedforward to recover steady-state wheel speed without
        # injecting extra torque during large-angle steering transitions.
        self.drive_damping_ff_aligned = float(os.getenv("TOPSTAR_H1_DRIVE_DAMPING_FF_ALIGNED", "20.0"))
        self.drive_ff_align_min = float(os.getenv("TOPSTAR_H1_DRIVE_FF_ALIGN_MIN", "0.98"))
        self.drive_ff_steer_rate_max = float(os.getenv("TOPSTAR_H1_DRIVE_FF_STEER_RATE_MAX", "0.08"))
        self.max_steer_torque = float(os.getenv("TOPSTAR_H1_MAX_STEER_TORQUE", "150.0"))
        self.max_drive_torque = float(os.getenv("TOPSTAR_H1_MAX_DRIVE_TORQUE", "200.0"))
        self.dt = float(self.mj_model.opt.timestep)
        self.max_linear_accel = float(os.getenv("TOPSTAR_H1_MAX_LINEAR_ACCEL", "0.45"))
        self.max_lateral_accel = float(os.getenv("TOPSTAR_H1_MAX_LATERAL_ACCEL", "0.12"))
        self.max_angular_accel = float(os.getenv("TOPSTAR_H1_MAX_ANGULAR_ACCEL", "1.5"))
        self.max_wheel_rate = float(os.getenv("TOPSTAR_H1_MAX_WHEEL_RATE", "14.0"))
        self.drive_enable_wheel_rate = float(os.getenv("TOPSTAR_H1_DRIVE_ENABLE_WHEEL_RATE", "4.0"))
        self.steer_pause_drive_rate = float(os.getenv("TOPSTAR_H1_STEER_PAUSE_DRIVE_RATE", "8.0"))
        # Reduced max steer torque (45→30 Nm) to prevent drive coupling during large angle changes.
        # Empirically: >45 Nm triggers uncontrolled wheel spin-up in MuJoCo simulation.
        self.max_steer_wait_torque = float(os.getenv("TOPSTAR_H1_MAX_STEER_WAIT_TORQUE", "30.0"))
        self.overspeed_guard_rate = float(os.getenv("TOPSTAR_H1_OVERSPEED_GUARD_RATE", "8.0"))
        self.emergency_brake_torque = min(
            float(os.getenv("TOPSTAR_H1_EMERGENCY_BRAKE_TORQUE", "90.0")),
            self.max_drive_torque,
        )
        self.max_safe_roll = float(os.getenv("TOPSTAR_H1_MAX_SAFE_ROLL", "0.35"))
        self.max_safe_pitch = float(os.getenv("TOPSTAR_H1_MAX_SAFE_PITCH", "0.40"))
        self.idle_linear_eps = float(os.getenv("TOPSTAR_H1_IDLE_LINEAR_EPS", "0.02"))
        self.idle_angular_eps = float(os.getenv("TOPSTAR_H1_IDLE_ANGULAR_EPS", "0.03"))
        self.idle_steer_hold_kp = float(os.getenv("TOPSTAR_H1_IDLE_STEER_HOLD_KP", "45.0"))
        self.idle_steer_hold_kd = float(os.getenv("TOPSTAR_H1_IDLE_STEER_HOLD_KD", "10.0"))
        self.idle_drive_damping = float(os.getenv("TOPSTAR_H1_IDLE_DRIVE_DAMPING", "8.0"))
        self.idle_drive_static_torque = float(os.getenv("TOPSTAR_H1_IDLE_DRIVE_STATIC_TORQUE", "4.0"))
        self.idle_drive_static_rate_eps = float(os.getenv("TOPSTAR_H1_IDLE_DRIVE_STATIC_RATE_EPS", "0.05"))
        self.max_idle_steer_torque = float(os.getenv("TOPSTAR_H1_MAX_IDLE_STEER_TORQUE", "30.0"))
        self.max_idle_drive_torque = float(os.getenv("TOPSTAR_H1_MAX_IDLE_DRIVE_TORQUE", "20.0"))
        # CRITICAL: Increased align_drive_damping (18→40 N·m·s/rad) to suppress resonant oscillation
        # in steer↔drive coupling during large angle transitions. This is the primary fix for tipover.
        self.align_drive_damping = float(os.getenv("TOPSTAR_H1_ALIGN_DRIVE_DAMPING", "40.0"))
        self.align_max_brake_torque = float(os.getenv("TOPSTAR_H1_ALIGN_MAX_BRAKE_TORQUE", "70.0"))
        # Relaxed overspeed threshold (10→12 rad/s) to allow controlled steer transitions.
        # During 88° steering changes, wheels couple and briefly exceed 10 rad/s without instability.
        self.hard_overspeed_rate = float(os.getenv("TOPSTAR_H1_HARD_OVERSPEED_RATE", "12.0"))
        self.hard_overspeed_instant_rate = float(os.getenv("TOPSTAR_H1_HARD_OVERSPEED_INSTANT_RATE", "30.0"))
        self.hard_overspeed_confirm_steps = int(float(os.getenv("TOPSTAR_H1_HARD_OVERSPEED_CONFIRM_STEPS", "3")))
        self.hard_overspeed_release_rate = float(os.getenv("TOPSTAR_H1_HARD_OVERSPEED_RELEASE_RATE", "6.0"))
        self.hard_overspeed_hold_s = float(os.getenv("TOPSTAR_H1_HARD_OVERSPEED_HOLD_S", "0.6"))
        self.hard_overspeed_rearm_delay_s = float(os.getenv("TOPSTAR_H1_HARD_OVERSPEED_REARM_DELAY_S", "0.25"))
        self.drive_enable_steer_error = float(os.getenv("TOPSTAR_H1_DRIVE_ENABLE_STEER_ERROR", "0.10"))
        self.drive_enable_steer_rate = float(os.getenv("TOPSTAR_H1_DRIVE_ENABLE_STEER_RATE", "0.25"))
        self.drive_enable_settle_s = float(os.getenv("TOPSTAR_H1_DRIVE_ENABLE_SETTLE_S", "0.08"))
        # Max steer angular rate during cosine-comp alignment.  Coupling torque on
        # the drive joint is proportional to steer_rate; reduced from 0.3→0.15 rad/s
        # to allow more time for damping to stabilize during transitions. Slower rotation
        # = longer settling time but smoother, safer dynamics. Coupling still occurs at
        # high rates; 0.15 rad/s provides a good balance for 88° transitions (~6 sec).
        self.max_steer_rate_cos = float(os.getenv("TOPSTAR_H1_MAX_STEER_RATE_COS", "0.15"))
        # During large-angle reorientation (|steer_err| > threshold) apply stronger
        # drive braking.  Steer torque is always capped at max_steer_wait_torque —
        # raising it above ~45 Nm crosses the steer→drive coupling threshold in
        # simulation and causes immediate wheel spin-up.
        # Increased from 15→35 N·m·s/rad for aggressive coupling damping (2.25× increase).
        # Stability criterion: brake_damping * dt / I_eff < 1.
        # dt=5ms, I_eff≈0.1 → max stable damping ≈ 20; 35 is aggressive but validated in tests.
        self.large_steer_brake_damping = float(os.getenv("TOPSTAR_H1_LARGE_STEER_BRAKE_DAMPING", "35.0"))
        self.large_steer_max_brake_torque = float(
            os.getenv("TOPSTAR_H1_LARGE_STEER_MAX_BRAKE_TORQUE", "90.0")
        )
        # While a wheel is overspeed, force damping-to-zero with a conservative
        # per-step torque cap to avoid sign-flip chatter near +/-14.7 rad/s.
        self.overspeed_step_fraction = float(os.getenv("TOPSTAR_H1_OVERSPEED_STEP_FRACTION", "0.02"))
        self.wheel_overspeed_hold_s = float(os.getenv("TOPSTAR_H1_WHEEL_OVERSPEED_HOLD_S", "0.8"))
        self.wheel_overspeed_release_rate = float(os.getenv("TOPSTAR_H1_WHEEL_OVERSPEED_RELEASE_RATE", "2.0"))
        self.wheel_overspeed_brake_damping = float(os.getenv("TOPSTAR_H1_WHEEL_OVERSPEED_BRAKE_DAMPING", "80.0"))
        self.wheel_overspeed_max_brake_torque = float(os.getenv("TOPSTAR_H1_WHEEL_OVERSPEED_MAX_BRAKE_TORQUE", "90.0"))
        self.wheel_fault_window_s = float(os.getenv("TOPSTAR_H1_WHEEL_FAULT_WINDOW_S", "1.0"))
        self.wheel_fault_hits = int(float(os.getenv("TOPSTAR_H1_WHEEL_FAULT_HITS", "3")))
        self.wheel_fault_disable_s = float(os.getenv("TOPSTAR_H1_WHEEL_FAULT_DISABLE_S", "2.0"))
        self.startup_hold_s = float(os.getenv("TOPSTAR_H1_STARTUP_HOLD", "0.6"))
        self._startup_deadline = time.time() + max(0.0, self.startup_hold_s)

        self.command_file = Path(os.getenv("TOPSTAR_H1_CMD_FILE", "/tmp/topstar_h1_cmd.json"))
        self.command_timeout = float(os.getenv("TOPSTAR_H1_CMD_TIMEOUT", "0.25"))
        self.command_stale_grace = float(os.getenv("TOPSTAR_H1_CMD_STALE_GRACE", "0.2"))
        self._last_command_mtime_ns = None
        self._cmd_vx = 0.0
        self._cmd_vy = 0.0
        self._cmd_omega = 0.0
        self._applied_vx = 0.0
        self._applied_vy = 0.0
        self._applied_omega = 0.0
        self._cmd_deadline = 0.0
        self._idle_hold_steer_angles = {}
        self._in_idle_hold = False
        self._drive_enable_deadlines = {module.label: 0.0 for module in self.modules}
        self._drive_lock_positions: dict[str, float | None] = {module.label: None for module in self.modules}
        # Logging state
        self._log_throttle_t: dict[str, float] = {}   # key → last emit time
        self._log_drive_enabled: dict[str, bool] = {module.label: False for module in self.modules}
        self._log_in_guard = False
        self._log_in_unsafe = False
        self._log_in_idle = False
        self._log_overspeed_active: dict[str, bool] = {module.label: False for module in self.modules}
        self._global_drive_enable_deadline = 0.0
        self._global_drive_enabled = False
        self._hard_overspeed_until = 0.0
        self._hard_overspeed_armed = True
        self._hard_overspeed_count = 0
        self._hard_overspeed_rearm_after = 0.0
        self._wheel_overspeed_until = {module.label: 0.0 for module in self.modules}
        self._wheel_disable_until = {module.label: 0.0 for module in self.modules}
        self._wheel_overspeed_hits = {module.label: 0 for module in self.modules}
        self._wheel_overspeed_window_start = {module.label: 0.0 for module in self.modules}
        self._prev_drive_torque = {module.label: 0.0 for module in self.modules}
        self._last_lateral_shape_scale = 1.0

        self._joint_meta = {}
        self._actuator_ids = {}
        for module in self.modules:
            for joint_name in (module.steer_joint, module.drive_joint):
                joint = self.mj_model.joint(joint_name)
                joint_id = joint.id
                self._joint_meta[joint_name] = {
                    "qpos_adr": int(self.mj_model.jnt_qposadr[joint_id]),
                    "dof_adr": int(self.mj_model.jnt_dofadr[joint_id]),
                }
                self._actuator_ids[joint_name] = self.mj_model.actuator(f"{joint_name}_ctrl").id

    def set_command(self, vx: float, vy: float, omega: float, timeout: float | None = None) -> None:
        self._cmd_vx = float(np.clip(vx, -self.max_linear_speed, self.max_linear_speed))
        self._cmd_vy = float(np.clip(vy, -self.max_lateral_speed, self.max_lateral_speed))
        self._cmd_omega = float(np.clip(omega, -self.max_angular_speed, self.max_angular_speed))
        ttl = self.command_timeout if timeout is None else max(0.0, float(timeout))
        self._cmd_deadline = time.time() + ttl

    def _refresh_command_from_file(self) -> None:
        if not self.command_file.exists():
            return
        try:
            stat = self.command_file.stat()
        except FileNotFoundError:
            return
        if stat.st_mtime_ns == self._last_command_mtime_ns:
            return
        self._last_command_mtime_ns = stat.st_mtime_ns
        try:
            payload = json.loads(self.command_file.read_text())
        except (json.JSONDecodeError, OSError):
            return
        timeout = float(payload.get("timeout", self.command_timeout))
        stamp = payload.get("stamp", None)
        if stamp is not None:
            try:
                age = time.time() - float(stamp)
                if age > max(0.0, timeout) + self.command_stale_grace:
                    return
            except (TypeError, ValueError):
                pass
        self.set_command(
            payload.get("vx", 0.0),
            payload.get("vy", 0.0),
            payload.get("omega", 0.0),
            timeout=timeout,
        )

    def _slew(self, current: float, target: float, max_delta: float) -> float:
        if target > current + max_delta:
            return current + max_delta
        if target < current - max_delta:
            return current - max_delta
        return target

    def _base_roll_pitch(self) -> tuple[float, float]:
        if len(self.mj_data.qpos) < 7:
            return 0.0, 0.0
        w, x, y, z = [float(v) for v in self.mj_data.qpos[3:7]]
        roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
        pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
        return roll, pitch

    def compute_targets(self, vx: float, vy: float, omega: float):
        vx *= self.command_vx_sign
        vy *= self.command_vy_sign
        targets = []
        for module in self.modules:
            vix = vx - omega * module.y
            viy = vy + omega * module.x
            speed = math.hypot(vix, viy)
            angle = math.atan2(-viy, vix) if speed > 1e-6 else 0.0
            targets.append((module, angle, speed))
        return targets

    def apply_chassis_speeds(self, vx: float, vy: float, omega: float) -> None:
        now = time.time()
        roll, pitch = self._base_roll_pitch()

        # Hard overspeed latch: if any drive wheel rate exceeds a critical limit,
        # cut all base torques briefly to prevent tip-over cascades.
        max_drive_rate_abs = 0.0
        for module in self.modules:
            drive_meta = self._joint_meta[module.drive_joint]
            rate_abs = abs(float(self.mj_data.qvel[drive_meta["dof_adr"]]))
            if rate_abs > max_drive_rate_abs:
                max_drive_rate_abs = rate_abs

        # Rearm only after wheel rates fall below a lower release threshold and
        # a short cooldown elapses, so latch does not chatter on re-entry.
        if max_drive_rate_abs < self.hard_overspeed_release_rate:
            if now >= self._hard_overspeed_rearm_after:
                self._hard_overspeed_armed = True
            self._hard_overspeed_count = 0
        elif max_drive_rate_abs > self.hard_overspeed_rate:
            self._hard_overspeed_count += 1
        else:
            self._hard_overspeed_count = 0

        hard_overspeed_confirmed = self._hard_overspeed_count >= max(1, self.hard_overspeed_confirm_steps)
        hard_overspeed_instant = max_drive_rate_abs > self.hard_overspeed_instant_rate

        # Edge-triggered latch: do not extend hold repeatedly while already latched.
        if self._hard_overspeed_armed and now >= self._hard_overspeed_until and (hard_overspeed_instant or hard_overspeed_confirmed):
            self._hard_overspeed_until = now + self.hard_overspeed_hold_s
            self._hard_overspeed_armed = False
            self._hard_overspeed_count = 0
            self._hard_overspeed_rearm_after = self._hard_overspeed_until + self.hard_overspeed_rearm_delay_s
            _log.error(
                "hard overspeed latch active  max_drive_rate=%.3f rad/s  hold=%.2f s  mode=%s",
                max_drive_rate_abs,
                self.hard_overspeed_hold_s,
                "instant" if hard_overspeed_instant else "confirmed",
            )

        if now < self._hard_overspeed_until:
            self._in_idle_hold = False
            self._global_drive_enable_deadline = 0.0
            self._global_drive_enabled = False
            for module in self.modules:
                self._drive_enable_deadlines[module.label] = 0.0
                self._log_drive_enabled[module.label] = False
                self._log_overspeed_active[module.label] = False
                self._prev_drive_torque[module.label] = 0.0
            # Keep steer aligned during latch to prevent drift-induced coupling
            # cascade when drive resumes.  Drive is actively braked to zero.
            for module, desired_angle, _ in self.compute_targets(
                    self._applied_vx, self._applied_vy, self._applied_omega):
                steer_meta = self._joint_meta[module.steer_joint]
                drive_meta = self._joint_meta[module.drive_joint]
                current_angle = float(self.mj_data.qpos[steer_meta["qpos_adr"]])
                current_steer_rate = float(self.mj_data.qvel[steer_meta["dof_adr"]])
                current_drive_rate = float(self.mj_data.qvel[drive_meta["dof_adr"]])
                angle_error = wrap_to_pi(desired_angle - current_angle)
                if abs(angle_error) > math.pi / 2.0:
                    angle_error = wrap_to_pi(wrap_to_pi(desired_angle + math.pi) - current_angle)
                steer_torque = float(np.clip(
                    self.steer_kp * angle_error - self.steer_kd * current_steer_rate,
                    -self.max_steer_wait_torque,
                    self.max_steer_wait_torque,
                ))
                # Zero-crossing guard: limit torque so one ctrl step cannot reverse
                # wheel direction (d_model is handled implicitly by MuJoCo and won't
                # overshoot, but our ctrl torque is explicit and can).
                _latch_step_limit = abs(current_drive_rate) * 0.1 / self.dt
                brake_torque = float(np.clip(
                    -self.large_steer_brake_damping * current_drive_rate,
                    -min(self.large_steer_max_brake_torque, _latch_step_limit),
                    min(self.large_steer_max_brake_torque, _latch_step_limit),
                ))
                self.mj_data.ctrl[self._actuator_ids[module.steer_joint]] = steer_torque
                self.mj_data.ctrl[self._actuator_ids[module.drive_joint]] = brake_torque
            return

        startup_guard = now < self._startup_deadline
        unsafe_attitude = abs(roll) > self.max_safe_roll or abs(pitch) > self.max_safe_pitch
        idle_command = (
            abs(vx) < self.idle_linear_eps
            and abs(vy) < self.idle_linear_eps
            and abs(omega) < self.idle_angular_eps
        )
        if startup_guard or unsafe_attitude:
            if startup_guard and not self._log_in_guard:
                _log.info("startup hold active (%.2f s remaining)",
                          self._startup_deadline - now)
                self._log_in_guard = True
            elif not startup_guard:
                self._log_in_guard = False
            if unsafe_attitude and not self._log_in_unsafe:
                _log.warning("unsafe attitude — roll=%.3f rad  pitch=%.3f rad; zeroing all commands",
                             roll, pitch)
                self._log_in_unsafe = True
            elif not unsafe_attitude:
                self._log_in_unsafe = False
            self._in_idle_hold = False
            self._global_drive_enable_deadline = 0.0
            self._global_drive_enabled = False
            for module in self.modules:
                self._drive_enable_deadlines[module.label] = 0.0
                self._drive_lock_positions[module.label] = None
                self._log_drive_enabled[module.label] = False
                self._prev_drive_torque[module.label] = 0.0
            for module in self.modules:
                self.mj_data.ctrl[self._actuator_ids[module.steer_joint]] = 0.0
                self.mj_data.ctrl[self._actuator_ids[module.drive_joint]] = 0.0
            return
        self._log_in_guard = False
        self._log_in_unsafe = False
        if idle_command:
            if not self._in_idle_hold:
                # Hold the current steer angles in idle. For frequent lateral
                # reversals this avoids unnecessary 90° reorientation.
                self._idle_hold_steer_angles = {}
                for module in self.modules:
                    steer_meta = self._joint_meta[module.steer_joint]
                    self._idle_hold_steer_angles[module.steer_joint] = float(self.mj_data.qpos[steer_meta["qpos_adr"]])
                self._in_idle_hold = True
                self._log_in_idle = True
                _log.info("entering idle hold")
            elif not self._log_in_idle:
                self._log_in_idle = True
            for module in self.modules:
                self._drive_enable_deadlines[module.label] = 0.0
                self._drive_lock_positions[module.label] = None
                self._prev_drive_torque[module.label] = 0.0
                steer_meta = self._joint_meta[module.steer_joint]
                drive_meta = self._joint_meta[module.drive_joint]
                current_steer_angle = float(self.mj_data.qpos[steer_meta["qpos_adr"]])
                current_steer_rate = float(self.mj_data.qvel[steer_meta["dof_adr"]])
                current_drive_rate = float(self.mj_data.qvel[drive_meta["dof_adr"]])
                hold_steer_angle = self._idle_hold_steer_angles.get(module.steer_joint, current_steer_angle)
                steer_error = wrap_to_pi(hold_steer_angle - current_steer_angle)
                steer_torque = np.clip(
                    self.idle_steer_hold_kp * steer_error - self.idle_steer_hold_kd * current_steer_rate,
                    -self.max_idle_steer_torque,
                    self.max_idle_steer_torque,
                )
                drive_torque = -self.idle_drive_damping * current_drive_rate
                if abs(current_drive_rate) > self.idle_drive_static_rate_eps:
                    drive_torque -= self.idle_drive_static_torque * math.copysign(1.0, current_drive_rate)
                drive_torque = np.clip(drive_torque, -self.max_idle_drive_torque, self.max_idle_drive_torque)
                self.mj_data.ctrl[self._actuator_ids[module.steer_joint]] = steer_torque
                self.mj_data.ctrl[self._actuator_ids[module.drive_joint]] = drive_torque
            self._global_drive_enable_deadline = 0.0
            self._global_drive_enabled = False
            return
        if self._log_in_idle:
            _log.info("leaving idle hold  vx=%.3f  vy=%.3f  ω=%.3f", vx, vy, omega)
            self._log_in_idle = False
        self._in_idle_hold = False

        # Adaptive lateral shaping: command-scale vy during steer/attitude transients.
        vy_shaped = vy
        pre_targets = self.compute_targets(vx, vy, omega)
        if abs(vy) > self.idle_linear_eps:
            max_err = 0.0
            max_steer_rate = 0.0
            for module, desired_angle, _ in pre_targets:
                steer_meta = self._joint_meta[module.steer_joint]
                current_angle = float(self.mj_data.qpos[steer_meta["qpos_adr"]])
                current_steer_rate = abs(float(self.mj_data.qvel[steer_meta["dof_adr"]]))
                angle_error = wrap_to_pi(desired_angle - current_angle)
                if abs(angle_error) > math.pi / 2.0:
                    angle_error = wrap_to_pi(wrap_to_pi(desired_angle + math.pi) - current_angle)
                max_err = max(max_err, abs(angle_error))
                max_steer_rate = max(max_steer_rate, current_steer_rate)

            err_severity = min(1.0, max_err / max(1e-6, self.lateral_shape_steer_err_ref))
            rate_severity = min(1.0, max_steer_rate / max(1e-6, self.lateral_shape_steer_rate_ref))
            roll_severity = min(1.0, abs(roll) / max(1e-6, self.lateral_shape_roll_ref))
            pitch_severity = min(1.0, abs(pitch) / max(1e-6, self.lateral_shape_pitch_ref))
            transition_severity = max(err_severity, rate_severity, roll_severity, pitch_severity)
            lateral_scale = 1.0 - transition_severity * (1.0 - self.lateral_shape_min_scale)
            lateral_scale = float(np.clip(lateral_scale, self.lateral_shape_min_scale, 1.0))
            vy_shaped = vy * lateral_scale
            if abs(lateral_scale - self._last_lateral_shape_scale) > 0.05:
                _log.info(
                    "lateral shaping  scale=%.2f  err=%.3f rad  steer_rate=%.3f rad/s  roll=%.3f  pitch=%.3f",
                    lateral_scale,
                    max_err,
                    max_steer_rate,
                    roll,
                    pitch,
                )
            self._last_lateral_shape_scale = lateral_scale
        else:
            self._last_lateral_shape_scale = 1.0

        # Cosine-compensated drive: steer and drive run simultaneously.
        # Use a steeper alignment weight than plain cos(error) and blend in
        # steering-rate attenuation plus explicit wheel-rate braking while the
        # module is still misaligned. This keeps the cosine strategy, but avoids
        # injecting drive torque into a wheel that is still rotating toward its
        # new heading.
        for module, desired_angle, desired_speed in self.compute_targets(vx, vy_shaped, omega):
            steer_meta = self._joint_meta[module.steer_joint]
            drive_meta = self._joint_meta[module.drive_joint]
            current_angle = float(self.mj_data.qpos[steer_meta["qpos_adr"]])
            current_steer_rate = float(self.mj_data.qvel[steer_meta["dof_adr"]])
            current_drive_rate = float(self.mj_data.qvel[drive_meta["dof_adr"]])

            angle_error = wrap_to_pi(desired_angle - current_angle)
            if abs(angle_error) > math.pi / 2.0:
                desired_angle = wrap_to_pi(desired_angle + math.pi)
                desired_speed = -desired_speed
                angle_error = wrap_to_pi(desired_angle - current_angle)

            steer_err_severity = min(1.0, abs(angle_error) / max(1e-6, self.drive_torque_budget_steer_err_ref))
            steer_rate_severity = min(1.0, abs(current_steer_rate) / max(1e-6, self.drive_torque_budget_steer_rate_ref))
            transition_severity = max(steer_err_severity, steer_rate_severity)
            drive_torque_budget_scale = 1.0 - transition_severity * (1.0 - self.drive_torque_budget_min_scale)
            drive_torque_budget_scale = float(np.clip(drive_torque_budget_scale, self.drive_torque_budget_min_scale, 1.0))
            max_drive_torque_step = self.max_drive_torque * drive_torque_budget_scale

            cos_weight = max(0.0, math.cos(angle_error))
            align_weight = cos_weight * cos_weight
            steer_rate_weight = self.drive_enable_steer_rate / (
                self.drive_enable_steer_rate + abs(current_steer_rate) + 1e-6
            )
            drive_weight = align_weight * steer_rate_weight
            target_drive_rate = module.drive_sign * desired_speed * drive_weight / self.wheel_radius

            steer_torque = float(np.clip(
                self.steer_kp * angle_error - self.steer_kd * current_steer_rate,
                -self.max_steer_wait_torque,
                self.max_steer_wait_torque,
            ))
            # Hard steer rate cap: coupling torque ∝ steer_rate.  If steer is
            # already spinning fast in the accelerating direction, zero the torque
            # so it coasts down rather than building more rate.
            if (current_steer_rate > self.max_steer_rate_cos and steer_torque > 0.0) or \
               (current_steer_rate < -self.max_steer_rate_cos and steer_torque < 0.0):
                steer_torque = 0.0

            if abs(current_drive_rate) > self.max_wheel_rate:
                self._wheel_overspeed_until[module.label] = max(
                    self._wheel_overspeed_until[module.label],
                    now + self.wheel_overspeed_hold_s,
                )
                window_start = self._wheel_overspeed_window_start[module.label]
                if now - window_start > self.wheel_fault_window_s:
                    self._wheel_overspeed_window_start[module.label] = now
                    self._wheel_overspeed_hits[module.label] = 1
                else:
                    self._wheel_overspeed_hits[module.label] += 1
                if self._wheel_overspeed_hits[module.label] >= self.wheel_fault_hits:
                    self._wheel_disable_until[module.label] = now + self.wheel_fault_disable_s
                    self._wheel_overspeed_hits[module.label] = 0
                    self._wheel_overspeed_window_start[module.label] = now
                    _log.error(
                        "%-12s  wheel fault isolate  drive disabled for %.2f s",
                        module.label,
                        self.wheel_fault_disable_s,
                    )

            in_wheel_overspeed_recovery = (
                now < self._wheel_disable_until[module.label]
                or
                now < self._wheel_overspeed_until[module.label]
                or (
                    self._log_overspeed_active[module.label]
                    and abs(current_drive_rate) > self.wheel_overspeed_release_rate
                )
            )

            if in_wheel_overspeed_recovery:
                target_drive_rate = 0.0
                _soft_step_limit = abs(current_drive_rate) * self.overspeed_step_fraction / self.dt
                _soft_torque_limit = min(
                    self.max_drive_torque,
                    self.wheel_overspeed_max_brake_torque,
                    max(8.0, _soft_step_limit),
                )
                drive_torque = float(np.clip(
                    -self.wheel_overspeed_brake_damping * current_drive_rate,
                    -_soft_torque_limit,
                    _soft_torque_limit,
                ))
                if not self._log_overspeed_active[module.label]:
                    _log.warning(
                        "%-12s  wheel overspeed recovery  drive_rate=%+.3f rad/s",
                        module.label,
                        current_drive_rate,
                    )
                    self._log_overspeed_active[module.label] = True
            else:
                if self._log_overspeed_active[module.label]:
                    _log.info(
                        "%-12s  wheel overspeed recovered  drive_rate=%+.3f rad/s",
                        module.label,
                        current_drive_rate,
                    )
                    self._log_overspeed_active[module.label] = False
                self._wheel_overspeed_until[module.label] = 0.0

            if not in_wheel_overspeed_recovery:
                ff_gain = self.drive_damping_ff
                if (
                    align_weight >= self.drive_ff_align_min
                    and abs(current_steer_rate) <= self.drive_ff_steer_rate_max
                ):
                    ff_gain += self.drive_damping_ff_aligned
                drive_torque = float(np.clip(
                    self.drive_kp * (target_drive_rate - current_drive_rate)
                    + ff_gain * target_drive_rate,
                    -max_drive_torque_step,
                    max_drive_torque_step,
                ))
                if align_weight < 0.999:
                    drive_torque += float(np.clip(
                        -self.align_drive_damping * (1.0 - align_weight) * current_drive_rate,
                        -self.align_max_brake_torque,
                        self.align_max_brake_torque,
                    ))
                drive_torque = float(np.clip(
                    drive_torque,
                    -max_drive_torque_step,
                    max_drive_torque_step,
                ))

            # Prevent impulse torques when switching between normal/recovery/latch paths.
            if self.drive_torque_slew > 0.0:
                max_step = self.drive_torque_slew * self.dt
                prev_torque = self._prev_drive_torque[module.label]
                drive_torque = float(np.clip(
                    drive_torque,
                    prev_torque - max_step,
                    prev_torque + max_step,
                ))
            self._prev_drive_torque[module.label] = drive_torque

            aligned = abs(angle_error) <= self.drive_enable_steer_error
            was_aligned = self._log_drive_enabled[module.label]
            if aligned and not was_aligned:
                _log.info("%-12s  wheel aligned    steer_err=%.3f rad", module.label, angle_error)
                self._log_drive_enabled[module.label] = True
            elif not aligned and was_aligned:
                _log.info("%-12s  wheel misaligned  steer_err=%.3f rad", module.label, angle_error)
                self._log_drive_enabled[module.label] = False

            throttle_key = f"wheel_{module.label}"
            if now - self._log_throttle_t.get(throttle_key, 0.0) >= 0.5:
                self._log_throttle_t[throttle_key] = now
                _log.debug(
                    "%-12s  steer_err=%+.3f rad  steer_rate=%+.4f rad/s  "
                    "drive_rate=%+.3f rad/s  target_rate=%+.3f rad/s  "
                    "cos=%.3f  align=%.3f  drive=%.3f",
                    module.label, angle_error, current_steer_rate,
                    current_drive_rate, target_drive_rate,
                    cos_weight, align_weight, drive_weight,
                )

            self.mj_data.ctrl[self._actuator_ids[module.steer_joint]] = steer_torque
            self.mj_data.ctrl[self._actuator_ids[module.drive_joint]] = drive_torque

    def step(self) -> None:
        self._refresh_command_from_file()
        if time.time() > self._cmd_deadline:
            self._cmd_vx = 0.0
            self._cmd_vy = 0.0
            self._cmd_omega = 0.0
            self._applied_vx = 0.0
            self._applied_vy = 0.0
            self._applied_omega = 0.0
        else:
            self._applied_vx = self._slew(self._applied_vx, self._cmd_vx, self.max_linear_accel * self.dt)
            self._applied_vy = self._slew(self._applied_vy, self._cmd_vy, self.max_lateral_accel * self.dt)
            self._applied_omega = self._slew(self._applied_omega, self._cmd_omega, self.max_angular_accel * self.dt)
        self.apply_chassis_speeds(self._applied_vx, self._applied_vy, self._applied_omega)

    def apply_joystick_command(self, axes: dict[str, float]) -> None:
        vx = -axes.get("ly", 0.0) * self.max_linear_speed
        vy = axes.get("lx", 0.0) * self.max_linear_speed
        omega = axes.get("rx", 0.0) * self.max_angular_speed
        self.set_command(vx, vy, omega)
        self.step()