#!/usr/bin/env python3
"""
H2 RL Inference Runner — ROS2 version.

Subscribes to /lowstate, runs RL policy for leg joints (0-11), publishes
/lowcmd (with CRC).  Upper-body joints (12-28) are held at neutral by
default (waist+head+arms mode=1, q=0) to match the SHM bridge's FSM
carry-over behaviour.  Pass --no-hold-upper-body if upper_body_play_ros2.py
is running alongside and owns the arms via /arm_sdk.

Source setup before running:
    source ~/topstar_ros2/setup.sh

Usage:
    python3 ~/topstar_ros2/example/h2_rl_runner_ros2.py \\
        --deploy-yaml /path/to/deploy.yaml \\
        --policy-onnx /path/to/policy.onnx \\
        --kp-scale 2.0   # 2.0 = MuJoCo sim, 1.0 = real robot

Joystick (wireless_remote bytes in /lowstate):
    Left  stick   : forward/backward (vx), strafe left/right (vy)
    Right stick   : yaw rotation (vyaw)
    START/OPTIONS : enter RL inference mode
    SELECT/SHARE  : exit RL inference mode (DAMP)
    R1            : emergency DAMP

Full WBC stack:
    # Terminal 1 — RL legs
    python3 h2_rl_runner_ros2.py --deploy-yaml ... --auto-ai

    # Terminal 2 — recorded arm motion
    python3 upper_body_play_ros2.py take1.npz --loop
"""

import argparse
import json
import signal
import struct
import sys
import threading
import time
from collections import deque
from pathlib import Path

import numpy as np
import yaml
import onnxruntime as ort

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from topstar_hg.msg import LowCmd, LowState, MotorCmd
from topstar_api.msg import Request as ApiRequest

# ── Constants ─────────────────────────────────────────────────────────────────

NUM_DDS_MOTORS = 35

# Upper-body joint hold positions and gains, mirroring h2_fsm.c hold_upper_body().
# Motors 12–28: waist, head, arms.  Held at q=0 so the robot stays upright
# while the leg policy runs.  Disabled with --no-hold-upper-body if
# upper_body_play_ros2.py owns the arms via /arm_sdk.
_UPPER_BODY_HOLD = {
    # idx: (q_target, kp, kd)
    12: (0.0, 60.0, 3.0),   # WaistYaw
    13: (0.0, 20.0, 1.0),   # HeadYaw
    14: (0.0, 20.0, 1.0),   # HeadPitch
    15: (0.0, 30.0, 1.5),   # LeftShoulderPitch
    16: (0.0, 30.0, 1.5),   # LeftShoulderRoll
    17: (0.0, 30.0, 1.5),   # LeftShoulderYaw
    18: (0.0, 30.0, 1.5),   # LeftElbow
    19: (0.0, 20.0, 1.0),   # LeftWristYaw
    20: (0.0, 20.0, 1.0),   # LeftWristPitch
    21: (0.0, 20.0, 1.0),   # LeftWristRoll
    22: (0.0, 30.0, 1.5),   # RightShoulderPitch
    23: (0.0, 30.0, 1.5),   # RightShoulderRoll
    24: (0.0, 30.0, 1.5),   # RightShoulderYaw
    25: (0.0, 30.0, 1.5),   # RightElbow
    26: (0.0, 20.0, 1.0),   # RightWristYaw
    27: (0.0, 20.0, 1.0),   # RightWristPitch
    28: (0.0, 20.0, 1.0),   # RightWristRoll
}

# MotionSwitcher API IDs (rt/api/motion_switcher/request)
MS_API_SELECT_MODE  = 1002
MS_API_RELEASE_MODE = 1003

# Wireless remote button bitmasks (wireless_remote[40] in LowState)
PS2_KEY_R1     = (1 << 0)
PS2_KEY_L1     = (1 << 1)
PS2_KEY_START  = (1 << 2)   # OPTIONS button on some controllers
PS2_KEY_SELECT = (1 << 3)   # SHARE button on some controllers

_REMOTE_FMT  = '<2BHfffff'
_REMOTE_SIZE = struct.calcsize(_REMOTE_FMT)   # 24 bytes

# QoS profiles
_STATE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)
_CMD_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

# ── CRC helper ────────────────────────────────────────────────────────────────

def _crc32_core(data: bytes) -> int:
    words = struct.unpack(f'{len(data) // 4}I', data)
    crc = 0xFFFFFFFF
    poly = 0x04C11DB7
    for word in words:
        xbit = 1 << 31
        for _ in range(32):
            if crc & 0x80000000:
                crc = ((crc << 1) & 0xFFFFFFFF) ^ poly
            else:
                crc = (crc << 1) & 0xFFFFFFFF
            if word & xbit:
                crc ^= poly
            xbit >>= 1
    return crc


def compute_crc(msg: LowCmd) -> None:
    buf = bytearray()
    buf += struct.pack('BB2x', msg.mode_pr, msg.mode_machine)
    for m in msg.motor_cmd:
        buf += struct.pack('=B3xfffffI', m.mode, m.q, m.dq, m.tau, m.kp, m.kd, m.reserve)
    buf += struct.pack('4I', *list(msg.reserve))
    msg.crc = _crc32_core(bytes(buf))


# ── Wireless remote ───────────────────────────────────────────────────────────

class JoystickState:
    __slots__ = ("lx", "ly", "rx", "ry", "keys", "valid")

    def __init__(self):
        self.lx = self.ly = self.rx = self.ry = 0.0
        self.keys = 0
        self.valid = False


def parse_wireless_remote(raw) -> JoystickState:
    js = JoystickState()
    if len(raw) >= _REMOTE_SIZE:
        _, _, keys, lx, rx, ry, _L2, ly = struct.unpack_from(_REMOTE_FMT, bytes(raw))
        js.keys  = keys
        js.lx    = lx
        js.ly    = ly
        js.rx    = rx
        js.ry    = ry
        js.valid = True
    return js


# ── Policy helpers ────────────────────────────────────────────────────────────

def apply_deadzone(value, deadzone):
    if abs(value) < deadzone:
        return 0.0
    sign = 1.0 if value > 0 else -1.0
    return sign * (abs(value) - deadzone) / (1.0 - deadzone)


def quat_rotate_inverse(quat: np.ndarray, vec: np.ndarray) -> np.ndarray:
    w, x, y, z = quat
    q_vec = np.array([-x, -y, -z])
    t = 2.0 * np.cross(q_vec, vec)
    return vec + w * t + np.cross(q_vec, t)


def get_projected_gravity(quat: np.ndarray) -> np.ndarray:
    return quat_rotate_inverse(quat, np.array([0.0, 0.0, -1.0])).astype(np.float32)


def compute_obs_term(name, scale, omega, quat, cmd, q_policy, dq_policy,
                     default_pos_policy, action, dim):
    if name == "base_ang_vel":
        return (omega * scale).astype(np.float32)
    elif name == "projected_gravity":
        return (get_projected_gravity(quat) * scale).astype(np.float32)
    elif name in ("velocity_commands", "keyboard_velocity_commands"):
        return (cmd * scale).astype(np.float32)
    elif name == "joint_pos_rel":
        return ((q_policy - default_pos_policy) * scale).astype(np.float32)
    elif name == "joint_vel_rel":
        return (dq_policy * scale).astype(np.float32)
    elif name == "last_action":
        return (action * scale).astype(np.float32)
    else:
        return np.zeros(dim, dtype=np.float32)


class ButtonDebouncer:
    def __init__(self, cooldown_sec=0.5):
        self.cooldown = cooldown_sec
        self._last = {}

    def pressed(self, keys, mask):
        if not (keys & mask):
            return False
        now = time.monotonic()
        if (now - self._last.get(mask, 0)) >= self.cooldown:
            self._last[mask] = now
            return True
        return False


# ── ROS2 Node ─────────────────────────────────────────────────────────────────

class H2RLRunnerNode(Node):
    def __init__(self):
        super().__init__('h2_rl_runner_ros2')
        self._lock = threading.Lock()
        self._last_state: LowState | None = None

        self._state_sub = self.create_subscription(
            LowState, '/lowstate', self._on_lowstate, _STATE_QOS)
        self._cmd_pub = self.create_publisher(LowCmd, '/lowcmd', _CMD_QOS)
        self._api_pub = self.create_publisher(ApiRequest, '/api/motion_switcher/request', _CMD_QOS)

    def _on_lowstate(self, msg: LowState):
        with self._lock:
            self._last_state = msg

    def get_state(self) -> LowState | None:
        with self._lock:
            return self._last_state

    def send_lowcmd(self, msg: LowCmd):
        compute_crc(msg)
        self._cmd_pub.publish(msg)

    def send_motion_switcher(self, api_id: int, parameter: str):
        req = ApiRequest()
        req.header.identity.api_id = api_id
        req.parameter = parameter
        self._api_pub.publish(req)

    def select_mode(self, mode_name: str):
        param = json.dumps({"name": mode_name})
        self.send_motion_switcher(MS_API_SELECT_MODE, param)
        self.get_logger().info(f"select_mode → {mode_name}")

    def release_mode(self):
        param = json.dumps({"name": "ai"})
        self.send_motion_switcher(MS_API_RELEASE_MODE, param)
        self.get_logger().info("release_mode → normal")


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="H2 RL Inference Runner — ROS2")
    parser.add_argument("--lab-path", type=str, default="/home/test/topstar_rl_lab")
    parser.add_argument("--policy-onnx", type=str, default=None)
    parser.add_argument("--deploy-yaml", type=str, default=None)
    parser.add_argument("--kp-scale", type=float, default=2.0,
                        help="PD gain multiplier; 2.0 is the verified hardware value (stiffness "
                             "in deploy.yaml are half the real target gains)")
    parser.add_argument("--ankle-kp-scale", type=float, default=1.0)
    parser.add_argument("--action-clip", type=float, default=10.0)
    parser.add_argument("--vx", type=float, default=0.0)
    parser.add_argument("--vy", type=float, default=0.0)
    parser.add_argument("--vyaw", type=float, default=0.0)
    parser.add_argument("--no-joystick", action="store_true")
    parser.add_argument("--auto-ai", action="store_true",
                        help="Enter AI mode automatically at startup")
    parser.add_argument("--max-vx", type=float, default=0.5)
    parser.add_argument("--max-vx-back", type=float, default=None)
    parser.add_argument("--max-vy", type=float, default=0.15)
    parser.add_argument("--max-vyaw", type=float, default=0.8)
    parser.add_argument("--deadzone", type=float, default=0.1)
    parser.add_argument("--warmup-time", type=float, default=1.0)
    parser.add_argument("--policy-ramp", type=float, default=0.0)
    parser.add_argument("--action-ema", type=float, default=0.0)
    parser.add_argument("--cmd-ramp", type=float, default=0.0)
    parser.add_argument("--cmd-filter", type=float, default=0.0)
    parser.add_argument("--diag", action="store_true")
    parser.add_argument("--verbose", action="store_true")
    parser.add_argument("--no-hold-upper-body", action="store_true",
                        help="Leave upper-body joints (12-28) at mode=0. "
                             "Use only when upper_body_play_ros2.py is running alongside.")
    args = parser.parse_args()

    # ── Paths ──────────────────────────────────────────────────────────────────
    lab_path  = Path(args.lab_path)
    policy_dir = lab_path / "deploy" / "robots" / "h2" / "config" / "policy" / "velocity" / "v0"
    deploy_yaml = Path(args.deploy_yaml) if args.deploy_yaml else policy_dir / "params" / "deploy.yaml"
    policy_onnx = Path(args.policy_onnx) if args.policy_onnx else policy_dir / "exported" / "policy.onnx"

    if not deploy_yaml.exists():
        print(f"Error: Config not found at {deploy_yaml}")
        return

    # ── Load Config ────────────────────────────────────────────────────────────
    print(f"Loading config from {deploy_yaml}")
    with open(deploy_yaml, "r") as f:
        cfg = yaml.load(f, Loader=yaml.UnsafeLoader)

    step_dt          = cfg["step_dt"]
    joint_ids_map    = cfg["joint_ids_map"]
    default_pos_policy = np.array(cfg["default_joint_pos"], dtype=np.float32)
    num_joints       = len(default_pos_policy)

    kp_scale = args.kp_scale
    kd_scale = np.sqrt(kp_scale)
    kps = np.array(cfg["stiffness"], dtype=np.float32) * kp_scale
    kds = np.array(cfg["damping"],   dtype=np.float32) * kd_scale

    if args.ankle_kp_scale != 1.0:
        ankle_kd_scale = np.sqrt(args.ankle_kp_scale)
        kps[8:12] *= args.ankle_kp_scale
        kds[8:12] *= ankle_kd_scale

    action_cfg   = cfg["actions"]["JointPositionAction"]
    action_scale = np.array(action_cfg["scale"],  dtype=np.float32)
    action_offset = np.array(action_cfg["offset"], dtype=np.float32)

    obs_cfg = cfg["observations"]
    obs_groups = []
    for name, ocfg in obs_cfg.items():
        scale   = np.array(ocfg["scale"], dtype=np.float32)
        history = ocfg.get("history_length", 1)
        obs_groups.append({"name": name, "scale": scale, "dim": len(scale), "history": history})

    # ── Load Model ─────────────────────────────────────────────────────────────
    print(f"Loading ONNX model: {policy_onnx}")
    session     = ort.InferenceSession(str(policy_onnx))
    input_name  = session.get_inputs()[0].name
    onnx_obs_dim = session.get_inputs()[0].shape[-1]
    single_obs_dim = sum(g["dim"] for g in obs_groups)
    history_len = obs_groups[0]["history"]
    expected = single_obs_dim * history_len
    if onnx_obs_dim != expected and onnx_obs_dim % single_obs_dim == 0:
        history_len = onnx_obs_dim // single_obs_dim
        for g in obs_groups:
            g["history"] = history_len

    # ── ROS2 init ──────────────────────────────────────────────────────────────
    rclpy.init()
    node = H2RLRunnerNode()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # Wait for first lowstate
    print("Waiting for /lowstate...")
    t_wait = time.monotonic()
    while node.get_state() is None:
        if time.monotonic() - t_wait > 10.0:
            print("Timeout waiting for /lowstate")
            rclpy.shutdown()
            return
        time.sleep(0.05)
    print("Got /lowstate — ready.")

    # ── State variables ────────────────────────────────────────────────────────
    action          = np.zeros(num_joints, dtype=np.float32)
    smoothed_action = np.zeros(num_joints, dtype=np.float32)
    cmd_target      = np.array([args.vx, args.vy, args.vyaw], dtype=np.float32)
    cmd             = np.array([args.vx, args.vy, args.vyaw], dtype=np.float32)
    max_vx_back     = args.max_vx_back

    cmd_max_rates = None
    if args.cmd_ramp > 0.0:
        cmd_max_rates = np.array(
            [args.max_vx, args.max_vy, args.max_vyaw], dtype=np.float32
        ) * (step_dt / args.cmd_ramp)

    debouncer       = ButtonDebouncer(cooldown_sec=0.5)
    ai_mode_active  = False
    auto_ai_done    = False
    inference_count = 0
    WARMUP_STEPS    = max(1, round(args.warmup_time / step_dt))
    RAMP_STEPS      = max(0, round(args.policy_ramp / step_dt))
    warmup_remaining = 0
    policy_ramp_step = 0
    motor_fault_prev = False

    # Observation history
    state0   = node.get_state()
    quat0    = np.array(state0.imu_state.quaternion, dtype=np.float32)
    omega0   = np.array(state0.imu_state.gyroscope,  dtype=np.float32)
    q0       = np.array([state0.motor_state[joint_ids_map[i]].q  for i in range(num_joints)], dtype=np.float32)
    dq0      = np.array([state0.motor_state[joint_ids_map[i]].dq for i in range(num_joints)], dtype=np.float32)
    group_histories = []
    for g in obs_groups:
        init_val = compute_obs_term(g["name"], g["scale"], omega0, quat0, cmd,
                                    q0, dq0, default_pos_policy, action, g["dim"])
        hist = deque(maxlen=history_len)
        for _ in range(history_len):
            hist.append(init_val.copy())
        group_histories.append(hist)

    hold_upper_body = not args.no_hold_upper_body

    # Pre-build LowCmd template (legs set once; upper body held unless disabled)
    LOW_CMD_TEMPLATE = LowCmd()
    for policy_idx in range(num_joints):
        hw_idx = joint_ids_map[policy_idx]
        LOW_CMD_TEMPLATE.motor_cmd[hw_idx].mode = 1
        LOW_CMD_TEMPLATE.motor_cmd[hw_idx].kp   = float(kps[policy_idx])
        LOW_CMD_TEMPLATE.motor_cmd[hw_idx].kd   = float(kds[policy_idx])
    if hold_upper_body:
        for hw_idx, (q_hold, kp_hold, kd_hold) in _UPPER_BODY_HOLD.items():
            LOW_CMD_TEMPLATE.motor_cmd[hw_idx].mode = 1
            LOW_CMD_TEMPLATE.motor_cmd[hw_idx].q    = float(q_hold)
            LOW_CMD_TEMPLATE.motor_cmd[hw_idx].kp   = float(kp_hold)
            LOW_CMD_TEMPLATE.motor_cmd[hw_idx].kd   = float(kd_hold)

    joint_names = ["L_hip_p","R_hip_p","L_hip_r","R_hip_r",
                   "L_hip_y","R_hip_y","L_knee","R_knee",
                   "L_ank_p","R_ank_p","L_ank_r","R_ank_r"]

    print("=" * 60)
    print("H2 RL Runner — ROS2")
    print(f"  step_dt     : {step_dt*1000:.1f} ms  ({1/step_dt:.0f} Hz)")
    print(f"  kp_scale    : {kp_scale}")
    print(f"  warmup      : {args.warmup_time:.1f}s")
    print(f"  upper body  : {'held at neutral (waist+head+arms mode=1)' if hold_upper_body else 'passive mode=0 (upper_body_play_ros2.py expected)'}")
    if not args.no_joystick:
        print("  Joystick    : START=enter AI, SELECT=exit AI, R1=DAMP")
    if args.auto_ai:
        print("  --auto-ai   : will enter RL mode immediately")
    print("Press Ctrl+C to stop.")
    print("=" * 60)

    running = True
    def _stop(sig, frame):
        nonlocal running
        running = False
    signal.signal(signal.SIGINT,  _stop)
    signal.signal(signal.SIGTERM, _stop)

    # ── Control loop ───────────────────────────────────────────────────────────
    try:
        while running:
            cycle_start = time.monotonic()

            # 0. Auto-AI entry
            if args.auto_ai and not auto_ai_done and not ai_mode_active:
                print("[AUTO] --auto-ai: entering RL mode")
                node.select_mode("ai")
                ai_mode_active   = True
                auto_ai_done     = True
                warmup_remaining = WARMUP_STEPS
                policy_ramp_step = 0
                action[:]        = 0.0
                smoothed_action[:] = 0.0
                cmd[:]           = cmd_target
                print(f"[RL] Warmup {args.warmup_time:.1f}s ({WARMUP_STEPS} steps)")

            # 1. Read state
            state = node.get_state()
            if state is None:
                time.sleep(step_dt)
                continue

            quat  = np.array(state.imu_state.quaternion, dtype=np.float32)
            omega = np.array(state.imu_state.gyroscope,  dtype=np.float32)
            rpy   = np.array(state.imu_state.rpy,        dtype=np.float32)

            q_policy  = np.array([state.motor_state[joint_ids_map[i]].q   for i in range(num_joints)], dtype=np.float32)
            dq_policy = np.array([state.motor_state[joint_ids_map[i]].dq  for i in range(num_joints)], dtype=np.float32)
            tau_policy= np.array([state.motor_state[joint_ids_map[i]].tau_est for i in range(num_joints)], dtype=np.float32)

            # 1b. Joystick
            if not args.no_joystick:
                js = parse_wireless_remote(state.wireless_remote)
                if js.valid:
                    vx   = -apply_deadzone(js.ly, args.deadzone) * args.max_vx
                    vy   = -apply_deadzone(js.lx, args.deadzone) * args.max_vy
                    vyaw = -apply_deadzone(js.rx, args.deadzone) * args.max_vyaw
                    if max_vx_back is not None and vx < 0:
                        vx = max(vx, -max_vx_back)
                    cmd_target[:] = [vx, vy, vyaw]

                    if debouncer.pressed(js.keys, PS2_KEY_START) and not ai_mode_active:
                        print("[JOY] START → entering AI mode")
                        node.select_mode("ai")
                        ai_mode_active   = True
                        warmup_remaining = WARMUP_STEPS
                        policy_ramp_step = 0
                        action[:]        = 0.0
                        smoothed_action[:] = 0.0
                        cmd[:]           = cmd_target
                        print(f"[RL] Warmup {args.warmup_time:.1f}s ({WARMUP_STEPS} steps)")

                    if debouncer.pressed(js.keys, PS2_KEY_SELECT) and ai_mode_active:
                        print("[JOY] SELECT → exiting AI mode (DAMP)")
                        node.release_mode()
                        ai_mode_active = False

                    if debouncer.pressed(js.keys, PS2_KEY_R1):
                        print("[JOY] R1 → emergency DAMP")
                        node.release_mode()
                        ai_mode_active = False

            # 2. Command smoothing
            if cmd_max_rates is not None:
                delta = cmd_target - cmd
                cmd[:] = cmd + np.clip(delta, -cmd_max_rates, cmd_max_rates)
            elif args.cmd_filter > 0.0:
                cmd[:] = args.cmd_filter * cmd + (1.0 - args.cmd_filter) * cmd_target
            else:
                cmd[:] = cmd_target

            # 3. Observation history
            for gi, g in enumerate(obs_groups):
                val = compute_obs_term(g["name"], g["scale"], omega, quat, cmd,
                                       q_policy, dq_policy, default_pos_policy, action, g["dim"])
                group_histories[gi].append(val)

            obs_parts = []
            for hist in group_histories:
                obs_parts.extend(hist)
            full_obs = np.concatenate(obs_parts)
            if len(full_obs) != onnx_obs_dim:
                if len(full_obs) < onnx_obs_dim:
                    full_obs = np.pad(full_obs, (0, onnx_obs_dim - len(full_obs)))
                else:
                    full_obs = full_obs[:onnx_obs_dim]

            # 4. Inference (skip during warmup to keep last_action history = 0)
            in_warmup = ai_mode_active and warmup_remaining > 0
            if ai_mode_active and not in_warmup:
                obs_in = full_obs.reshape(1, -1).astype(np.float32)
                raw_action = session.run(None, {input_name: obs_in})[0].flatten()[:num_joints]
                raw_action = np.clip(raw_action, -args.action_clip, args.action_clip)
                if args.action_ema > 0.0:
                    smoothed_action = args.action_ema * smoothed_action + (1.0 - args.action_ema) * raw_action
                    action = smoothed_action
                else:
                    action = raw_action

            # 5. Publish /lowcmd
            if ai_mode_active:
                low_cmd = LowCmd()
                # Leg joints
                for policy_idx in range(num_joints):
                    hw_idx = joint_ids_map[policy_idx]
                    low_cmd.motor_cmd[hw_idx].mode = 1
                    low_cmd.motor_cmd[hw_idx].kp   = float(kps[policy_idx])
                    low_cmd.motor_cmd[hw_idx].kd   = float(kds[policy_idx])
                    low_cmd.motor_cmd[hw_idx].dq   = 0.0
                    low_cmd.motor_cmd[hw_idx].tau  = 0.0
                # Upper body: hold at neutral so the SHM bridge's FSM carry-over
                # behaviour is reproduced in the ROS2 re-publish path.
                if hold_upper_body:
                    for hw_idx, (q_hold, kp_hold, kd_hold) in _UPPER_BODY_HOLD.items():
                        low_cmd.motor_cmd[hw_idx].mode = 1
                        low_cmd.motor_cmd[hw_idx].q    = float(q_hold)
                        low_cmd.motor_cmd[hw_idx].kp   = float(kp_hold)
                        low_cmd.motor_cmd[hw_idx].kd   = float(kd_hold)

                if in_warmup:
                    for policy_idx in range(num_joints):
                        hw_idx = joint_ids_map[policy_idx]
                        low_cmd.motor_cmd[hw_idx].q = float(action_offset[policy_idx])
                    warmup_remaining -= 1
                    if warmup_remaining == 0:
                        if RAMP_STEPS > 0:
                            print(f"[RL] Warmup done — ramping policy over {args.policy_ramp:.1f}s  ← release band now")
                        else:
                            print("[RL] Warmup done — policy active  ← release band now")
                else:
                    if RAMP_STEPS > 0 and policy_ramp_step < RAMP_STEPS:
                        ramp_alpha = policy_ramp_step / RAMP_STEPS
                        policy_ramp_step += 1
                    else:
                        ramp_alpha = 1.0
                    target = ramp_alpha * action * action_scale + action_offset
                    for policy_idx in range(num_joints):
                        hw_idx = joint_ids_map[policy_idx]
                        low_cmd.motor_cmd[hw_idx].q = float(target[policy_idx])
                    inference_count += 1

                    if args.diag and inference_count % 50 == 1:
                        q_err_abs = np.abs(q_policy - default_pos_policy)
                        max_i = int(np.argmax(q_err_abs))
                        grav = get_projected_gravity(quat)
                        act_max = float(np.max(np.abs(action)))
                        print(f"[DIAG] #{inference_count:5d}  ramp={ramp_alpha:.2f}  "
                              f"grav=({grav[0]:+.3f},{grav[1]:+.3f},{grav[2]:+.3f})  "
                              f"act_max={act_max:.2f}  "
                              f"q_err_max={q_err_abs[max_i]:.3f}[{joint_names[max_i]}]  "
                              f"tau_max={float(np.max(np.abs(tau_policy))):.1f}  "
                              f"cmd=({cmd[0]:+.2f},{cmd[1]:+.2f},{cmd[2]:+.2f})")

                node.send_lowcmd(low_cmd)

            # 6. Sleep
            elapsed = time.monotonic() - cycle_start
            sleep_t = step_dt - elapsed
            if sleep_t > 0:
                time.sleep(sleep_t)

    except Exception as e:
        print(f"Error: {e}")
        import traceback; traceback.print_exc()
    finally:
        if ai_mode_active:
            node.release_mode()
            print("Released AI mode on shutdown.")
        rclpy.shutdown()
        print("Done.")


if __name__ == "__main__":
    main()
