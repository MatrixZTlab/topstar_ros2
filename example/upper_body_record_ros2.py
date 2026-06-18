#!/usr/bin/env python3
"""
Record H2 upper-body joint positions to NPZ via ROS2.

Subscribes to /lowstate and samples motor_state[i].q for upper-body joints
at --hz rate.  Saves t, q, dq, joint_ids, joint_names, hz to an NPZ file.
Press Ctrl+C to stop and save.

Source setup before running:
    source ~/topstar_ros2/setup.sh

Usage:
    python3 upper_body_record_ros2.py --out take1.npz
    python3 upper_body_record_ros2.py --out take1.npz --joints arms --hz 100
    python3 upper_body_record_ros2.py --out take1.npz --joints 15-21

Joint presets:
    all / upper : 12-28 (waist + head + both arms)
    arms        : 15-28 (both arms, no head/waist)
    left-arm    : 15-21
    right-arm   : 22-28
    head        : 13-14
    waist       : 12
"""

import argparse
import signal
import sys
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from topstar_hg.msg import LowState

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

PRESETS = {
    "all":       list(range(12, 29)),
    "upper":     list(range(12, 29)),
    "arms":      list(range(15, 29)),
    "left-arm":  list(range(15, 22)),
    "right-arm": list(range(22, 29)),
    "head":      [13, 14],
    "waist":     [12],
}

_STATE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)


def parse_joints(spec: str) -> list:
    spec = spec.strip()
    if spec in PRESETS:
        return PRESETS[spec]
    if "-" in spec and "," not in spec:
        lo, hi = spec.split("-", 1)
        return list(range(int(lo), int(hi) + 1))
    return [int(x.strip()) for x in spec.split(",")]


class StateSubscriber(Node):
    def __init__(self):
        super().__init__("upper_body_record_ros2")
        self._lock = threading.Lock()
        self._state = None
        self.create_subscription(LowState, "/lowstate", self._cb, _STATE_QOS)

    def _cb(self, msg):
        with self._lock:
            self._state = msg

    def get_state(self):
        with self._lock:
            return self._state


def main():
    parser = argparse.ArgumentParser(
        description="Record H2 upper-body joints to NPZ via ROS2",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--out", required=True, metavar="FILE")
    parser.add_argument("--hz", type=float, default=50.0)
    parser.add_argument("--joints", default="all",
                        help="all/arms/left-arm/right-arm/head/waist or range '12-28' or list '15,16'")
    parser.add_argument("--quiet", action="store_true")
    args = parser.parse_args()

    joint_ids = parse_joints(args.joints)
    for jid in joint_ids:
        if not (0 <= jid < len(ALL_JOINT_NAMES)):
            print(f"Error: joint {jid} out of range")
            return
    joint_names = [ALL_JOINT_NAMES[i] for i in joint_ids]
    n_joints    = len(joint_ids)

    rclpy.init()
    node = StateSubscriber()

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

    print(f"Recording {n_joints} joints at {args.hz} Hz → {args.out}")
    print(f"  Joints : {list(zip(joint_ids, joint_names))}")
    print("Press Ctrl+C to stop and save.\n")

    step_dt  = 1.0 / args.hz
    t_list, q_list, dq_list = [], [], []
    sample_count = 0
    t_start  = time.monotonic()
    PRINT_EVERY = max(1, round(args.hz / 5))

    running = True
    def _stop(sig, frame):
        nonlocal running
        running = False
    signal.signal(signal.SIGINT,  _stop)
    signal.signal(signal.SIGTERM, _stop)

    while running:
        cycle_start = time.monotonic()

        state = node.get_state()
        if state is not None:
            t_now  = time.monotonic() - t_start
            q_row  = np.array([state.motor_state[i].q  for i in joint_ids], dtype=np.float32)
            dq_row = np.array([state.motor_state[i].dq for i in joint_ids], dtype=np.float32)

            t_list.append(t_now)
            q_list.append(q_row)
            dq_list.append(dq_row)
            sample_count += 1

            if not args.quiet and sample_count % PRINT_EVERY == 0:
                parts = [f"{ALL_JOINT_NAMES[jid]}={q:+.3f}"
                         for jid, q in zip(joint_ids, q_row)]
                print(f"\r[{t_now:6.2f}s]  " + "  ".join(parts), end="", flush=True)

        sleep_t = step_dt - (time.monotonic() - cycle_start)
        if sleep_t > 0:
            time.sleep(sleep_t)

    rclpy.shutdown()
    print()

    if not t_list:
        print("No samples recorded.")
        return

    duration   = t_list[-1]
    actual_hz  = sample_count / duration if duration > 0 else 0
    print(f"Recorded {sample_count} samples over {duration:.1f}s "
          f"(actual rate: {actual_hz:.1f} Hz)")

    np.savez(
        args.out,
        t=np.array(t_list, dtype=np.float64),
        q=np.array(q_list, dtype=np.float32),
        dq=np.array(dq_list, dtype=np.float32),
        joint_ids=np.array(joint_ids, dtype=np.int32),
        joint_names=np.array(joint_names),
        hz=np.float64(args.hz),
    )
    print(f"Saved → {args.out}")


if __name__ == "__main__":
    main()
