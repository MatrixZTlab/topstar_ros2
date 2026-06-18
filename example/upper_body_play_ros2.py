#!/usr/bin/env python3
"""
Play back recorded upper-body motion via ROS2 /arm_sdk.

Subscribes to /lowstate (for ramp-in: reads current arm positions).
Publishes /arm_sdk (LowCmd type, NO CRC) with upper-body joints 12-28.

The topstar_bridge_v2 processes /arm_sdk separately from /lowcmd:
  - /arm_sdk  → writes SHM motors[12-28] and sets arm_sdk_active=1
  - /lowcmd   → skips mode=0 motor slots (bridge fix in process_lowcmd)
So /arm_sdk owns the arm joints exclusively.

Run alongside h2_rl_runner_ros2.py for the full WBC stack:
    # Terminal 1 — RL legs
    python3 h2_rl_runner_ros2.py --deploy-yaml ... --auto-ai

    # Terminal 2 — arm motion replay
    python3 upper_body_play_ros2.py take1.npz --loop

Standalone mode (without rl_runner):
    python3 upper_body_play_ros2.py take1.npz --arm-sdk

Safety:
  RAMP-IN : interpolates from current position to frame-0 over --ramp-in seconds.
  RAMP-OUT: moves arms to neutral (all-zeros) at full gains over --ramp-out seconds.
            Arms stay stiff at neutral after exit — never go limp under gravity.

Source setup before running:
    source ~/topstar_ros2/setup.sh
"""

import argparse
import signal
import struct
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from topstar_hg.msg import LowCmd, LowState, MotorCmd

NUM_DDS_MOTORS = 35

# Default impedance gains (from h2_robot_config.h)
DEFAULT_KP = [
    60, 60, 60, 100, 40, 40,       # Left leg  (0-5)
    60, 60, 60, 100, 40, 40,       # Right leg (6-11)
    60,                             # Waist     (12)
    20, 20,                         # Head      (13-14)
    40, 40, 40, 40, 40, 40, 40,    # Left arm  (15-21)
    40, 40, 40, 40, 40, 40, 40,    # Right arm (22-28)
]
DEFAULT_KD = [
    1, 1, 1, 2, 1, 1,
    1, 1, 1, 2, 1, 1,
    1,
    1, 1,
    1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1,
]

ALL_JOINT_NAMES = [
    "LeftHipPitch",    "LeftHipRoll",    "LeftHipYaw",    "LeftKnee",
    "LeftAnklePitch",  "LeftAnkleRoll",
    "RightHipPitch",   "RightHipRoll",   "RightHipYaw",   "RightKnee",
    "RightAnklePitch", "RightAnkleRoll",
    "WaistYaw",
    "HeadYaw",         "HeadPitch",
    "LShoulderPitch",  "LShoulderRoll",  "LShoulderYaw",  "LElbow",
    "LWristYaw",       "LWristPitch",    "LWristRoll",
    "RShoulderPitch",  "RShoulderRoll",  "RShoulderYaw",  "RElbow",
    "RWristYaw",       "RWristPitch",    "RWristRoll",
]

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


def lerp(a: np.ndarray, b: np.ndarray, t: float) -> np.ndarray:
    return a + (b - a) * t


class ArmSDKNode(Node):
    def __init__(self):
        super().__init__("upper_body_play_ros2")
        self._lock  = threading.Lock()
        self._state = None
        self._state_sub = self.create_subscription(
            LowState, "/lowstate", self._on_state, _STATE_QOS)
        # /arm_sdk uses the LowCmd message type but NO CRC check in the bridge
        self._arm_pub = self.create_publisher(LowCmd, "/arm_sdk", _CMD_QOS)

    def _on_state(self, msg):
        with self._lock:
            self._state = msg

    def get_state(self):
        with self._lock:
            return self._state

    def publish_arm_cmd(self, joint_ids, q_target, kps, kds):
        """Publish arm joint targets to /arm_sdk (NO CRC required)."""
        msg = LowCmd()
        for idx, jid in enumerate(joint_ids):
            msg.motor_cmd[jid].mode = 1                  # 1 = enabled
            msg.motor_cmd[jid].q   = float(q_target[idx])
            msg.motor_cmd[jid].dq  = 0.0
            msg.motor_cmd[jid].tau = 0.0
            msg.motor_cmd[jid].kp  = float(kps[idx])
            msg.motor_cmd[jid].kd  = float(kds[idx])
        # crc left at 0 — bridge does not verify CRC for /arm_sdk
        self._arm_pub.publish(msg)


def main():
    parser = argparse.ArgumentParser(
        description="Play back H2 upper-body recording via ROS2 /arm_sdk",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("recording", metavar="FILE",
                        help="NPZ file from upper_body_record_ros2.py (or SHM version)")
    parser.add_argument("--loop", action="store_true",
                        help="Loop recording indefinitely (Ctrl+C to stop)")
    parser.add_argument("--ramp-in", type=float, default=2.0, metavar="SEC",
                        help="Seconds to interpolate from current pose to frame-0 (default: 2.0)")
    parser.add_argument("--ramp-out", type=float, default=2.0, metavar="SEC",
                        help="Seconds to move arms to neutral after last frame (default: 2.0)")
    parser.add_argument("--speed", type=float, default=1.0, metavar="X",
                        help="Playback speed multiplier (default: 1.0)")
    parser.add_argument("--kp-scale", type=float, default=1.0)
    parser.add_argument("--kd-scale", type=float, default=1.0)
    args = parser.parse_args()

    if args.speed <= 0:
        print("Error: --speed must be > 0")
        return

    # ── Load recording ─────────────────────────────────────────────────────────
    rec         = np.load(args.recording, allow_pickle=True)
    t_rec       = rec["t"].astype(np.float64)
    q_rec       = rec["q"].astype(np.float32)
    joint_ids   = rec["joint_ids"].tolist()
    joint_names = rec["joint_names"].tolist()
    hz_rec      = float(rec["hz"])
    n_frames    = len(t_rec)
    n_joints    = len(joint_ids)

    if n_frames < 2:
        print(f"Error: recording has only {n_frames} frame(s) — need at least 2")
        return

    duration_rec = t_rec[-1] - t_rec[0]
    hz_play  = hz_rec / args.speed
    step_dt  = 1.0 / hz_play

    kps = np.array([DEFAULT_KP[i] * args.kp_scale for i in joint_ids], dtype=np.float32)
    kds = np.array([DEFAULT_KD[i] * args.kd_scale for i in joint_ids], dtype=np.float32)
    q_neutral = np.zeros(n_joints, dtype=np.float32)

    print(f"Recording : {args.recording}")
    print(f"  Frames  : {n_frames}  Duration: {duration_rec:.2f}s  Rate: {hz_rec:.0f} Hz")
    print(f"  Joints  : {list(zip(joint_ids, joint_names))}")
    print(f"Playback  : {hz_play:.1f} Hz  Speed: {args.speed}x  Loop: {args.loop}")
    print(f"Ramp-in   : {args.ramp_in:.1f}s   Ramp-out: {args.ramp_out:.1f}s")

    # ── ROS2 init ──────────────────────────────────────────────────────────────
    rclpy.init()
    node = ArmSDKNode()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    print("Waiting for /lowstate...")
    t_wait = time.monotonic()
    while node.get_state() is None:
        if time.monotonic() - t_wait > 10.0:
            print("Timeout waiting for /lowstate")
            rclpy.shutdown()
            return
        time.sleep(0.05)
    print("Got /lowstate — ready.")

    running = True
    def _stop(sig, frame):
        nonlocal running
        running = False
    signal.signal(signal.SIGINT,  _stop)
    signal.signal(signal.SIGTERM, _stop)

    # Read current arm positions for ramp-in
    state0   = node.get_state()
    q_init   = np.array([state0.motor_state[i].q for i in joint_ids], dtype=np.float32)
    q_frame0 = q_rec[0]

    # ── Phase 1: Ramp-in ───────────────────────────────────────────────────────
    ramp_in_steps = max(1, round(args.ramp_in * hz_play))
    print(f"\n[RAMP-IN] Moving to frame-0 over {args.ramp_in:.1f}s...")
    for step in range(ramp_in_steps):
        if not running:
            rclpy.shutdown()
            return
        alpha = step / ramp_in_steps
        node.publish_arm_cmd(joint_ids, lerp(q_init, q_frame0, alpha), kps, kds)
        time.sleep(step_dt)

    # ── Phase 2: Playback ──────────────────────────────────────────────────────
    play_count = 0
    q_last     = q_frame0.copy()

    while running:
        play_count += 1
        if args.loop:
            print(f"[PLAY] Replay #{play_count}")
        else:
            print(f"[PLAY] Playing {duration_rec:.1f}s of motion...")

        for frame_idx in range(n_frames):
            if not running:
                break
            cycle_start = time.monotonic()

            node.publish_arm_cmd(joint_ids, q_rec[frame_idx], kps, kds)
            q_last = q_rec[frame_idx]

            elapsed = time.monotonic() - cycle_start
            sleep_t = step_dt - elapsed
            if sleep_t > 0:
                time.sleep(sleep_t)

        if not running or not args.loop:
            break

    # ── Phase 3: Ramp-out — move arms to neutral at full gains ─────────────────
    # Arms move to zero while gains stay up.  The bridge holds neutral after exit.
    ramp_out_steps = max(1, round(args.ramp_out * hz_play))
    print(f"\n[RAMP-OUT] Moving arms to neutral over {args.ramp_out:.1f}s...")
    for step in range(ramp_out_steps):
        alpha = step / ramp_out_steps
        node.publish_arm_cmd(joint_ids, lerp(q_last, q_neutral, alpha), kps, kds)
        time.sleep(step_dt)

    # Hold neutral at full gains one last time (bridge will hold this position)
    node.publish_arm_cmd(joint_ids, q_neutral, kps, kds)

    rclpy.shutdown()
    print("Done.")


if __name__ == "__main__":
    main()
