#!/usr/bin/env python3

"""h1_drive_example.py — Drive H1 forward, wave arms, and print states.

Run (after sourcing all workspaces and launching h1_sim):
  ros2 run topstar_h1 h1_drive_example
"""
from __future__ import annotations

import math
import struct
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from topstar_hg.msg import LowCmd, LowState, MotorCmd

from topstar_h1.joint_defs import H1JointIndex, H1_DEFAULT_KP, H1_DEFAULT_KD

# ── CRC (ported from example/src/src/common/motor_crc_hg.cpp) ────────────────

def _crc32_core(data: bytes) -> int:
    """Custom CRC32 used by topstar_hg LowCmd."""
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
    """Pack LowCmd into its C struct layout and fill msg.crc in-place.

    C struct layout (little-endian, default alignment):
      uint8  mode_pr          )
      uint8  mode_machine     ) + 2 bytes padding = 4 bytes
      MotorCmd[35]            : each = B + 3x + 5f + I = 28 bytes → 980 bytes
      uint32[4] reserve       : 16 bytes
      ─────────────────────────────────────────────────────── 1000 bytes (250 u32)
      uint32 crc              : excluded from CRC input
    """
    buf = bytearray()
    buf += struct.pack('BB2x', msg.mode_pr, msg.mode_machine)
    for m in msg.motor_cmd:
        buf += struct.pack('=B3xfffffI',
                           m.mode, m.q, m.dq, m.tau, m.kp, m.kd, m.reserve)
    buf += struct.pack('4I', *list(msg.reserve))
    msg.crc = _crc32_core(bytes(buf))


# ── Helper ─────────────────────────────────────────────────────────────────────

def make_lowcmd(positions: list[float],
                kp: list[float] | None = None,
                kd: list[float] | None = None) -> LowCmd:
    """Build a position-control LowCmd for the 18 upper-body joints (slots 0–17)."""
    if kp is None:
        kp = H1_DEFAULT_KP.tolist()
    if kd is None:
        kd = H1_DEFAULT_KD.tolist()

    msg = LowCmd()
    msg.mode_pr = 0
    msg.mode_machine = 0
    for i, (q, p, d) in enumerate(zip(positions, kp, kd)):
        msg.motor_cmd[i] = MotorCmd(mode=1, q=float(q), dq=0.0, tau=0.0,
                                    kp=float(p), kd=float(d), reserve=0)
    compute_crc(msg)
    return msg


# ── QoS (must match h1_ros2_node) ─────────────────────────────────────────────

_SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)
_CMD_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

# ── Node ───────────────────────────────────────────────────────────────────────

class H1DriveExample(Node):
    CONTROL_HZ = 50          # command publish rate
    FORWARD_VX = 0.3         # m/s
    ARM_AMPLITUDE = 0.4      # rad
    ARM_FREQ_HZ = 0.3        # Hz

    def __init__(self) -> None:
        super().__init__('h1_drive_example')

        self._lowcmd_pub = self.create_publisher(LowCmd, '/lowcmd', _CMD_QOS)
        self._base_pub   = self.create_publisher(Twist,  '/base_cmd', _CMD_QOS)
        self._state_sub  = self.create_subscription(
            LowState, '/lowstate', self._on_state, _SENSOR_QOS)

        self._lock = threading.Lock()
        self._latest_state: LowState | None = None
        self._t0 = self.get_clock().now().nanoseconds * 1e-9

        self._timer = self.create_timer(1.0 / self.CONTROL_HZ, self._control_step)
        self.get_logger().info('H1DriveExample started.')

    # ── State callback ─────────────────────────────────────────────────────

    def _on_state(self, msg: LowState) -> None:
        with self._lock:
            self._latest_state = msg
        self._print_state(msg)

    def _print_state(self, msg: LowState) -> None:
        joints = [H1JointIndex.RIGHT_SHOULDER_BASE,
                  H1JointIndex.RIGHT_ELBOW,
                  H1JointIndex.LEFT_SHOULDER_BASE,
                  H1JointIndex.LEFT_ELBOW,
                  H1JointIndex.TORSO_LIFT,
                  H1JointIndex.TORSO_PITCH]
        lines = ['── H1 state ─────────────────────────────────']
        for j in joints:
            s = msg.motor_state[j]
            lines.append(f'  {j.name:<24} q={s.q:+.3f} rad  dq={s.dq:+.3f} rad/s')
        self.get_logger().info('\n'.join(lines), throttle_duration_sec=1.0)

    # ── Control step ───────────────────────────────────────────────────────

    def _control_step(self) -> None:
        t = self.get_clock().now().nanoseconds * 1e-9 - self._t0

        # 1. Drive base forward
        twist = Twist()
        twist.linear.x = self.FORWARD_VX
        self._base_pub.publish(twist)

        # 2. Wave both arms in opposing phase
        phase = 2.0 * math.pi * self.ARM_FREQ_HZ * t
        right_elbow = self.ARM_AMPLITUDE * math.sin(phase)
        left_elbow  = self.ARM_AMPLITUDE * math.sin(phase + math.pi)

        # 18 joints, all at 0 except the two elbows
        positions = [0.0] * 18
        positions[H1JointIndex.RIGHT_ELBOW] = right_elbow
        positions[H1JointIndex.LEFT_ELBOW]  = left_elbow

        self._lowcmd_pub.publish(make_lowcmd(positions))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = H1DriveExample()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop base on exit; publish a few frames and spin once to flush QoS.
        stop = Twist()
        for _ in range(5):
            node._base_pub.publish(stop)
            rclpy.spin_once(node, timeout_sec=0.02)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
