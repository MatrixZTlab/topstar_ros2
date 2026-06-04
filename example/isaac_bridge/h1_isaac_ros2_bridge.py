#!/usr/bin/env python3
"""h1_isaac_ros2_bridge.py — ROS2 bridge between Isaac Sim and H1 topics.

Subscribes:
  /lowcmd      topstar_hg/LowCmd    → ZMQ "lowcmd" to Isaac Sim
  /base_cmd    geometry_msgs/Twist  → ZMQ "basecmd" to Isaac Sim

Publishes:
  /lowstate    topstar_hg/LowState  ← ZMQ state from Isaac Sim (50 Hz)

Run AFTER h1_isaac_sim.py is started:
  source ~/topstar_ros2_ws/install/setup.bash
  python3 scripts/h1_isaac_bridge/h1_isaac_ros2_bridge.py
"""
from __future__ import annotations
import json
import sys
import threading
import time

import numpy as np
import zmq

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist
from topstar_hg.msg import LowCmd, LowState, MotorState, IMUState

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

N_UPPER = 18


class H1IsaacBridge(Node):
    """ROS2 node that bridges H1 topics to/from Isaac Sim via ZMQ."""

    def __init__(self) -> None:
        super().__init__("h1_isaac_ros2_bridge")

        self.declare_parameter("state_hz", 50)
        state_hz = float(self.get_parameter("state_hz").value)

        # ZMQ: receive state from Isaac Sim, push commands to Isaac Sim
        ctx = zmq.Context()

        self._state_sock = ctx.socket(zmq.PULL)
        self._state_sock.setsockopt(zmq.RCVHWM, 2)
        self._state_sock.connect("tcp://127.0.0.1:15555")

        self._cmd_sock = ctx.socket(zmq.PUSH)
        self._cmd_sock.setsockopt(zmq.SNDHWM, 2)
        self._cmd_sock.connect("tcp://127.0.0.1:15556")

        # Latest state from sim (protected by a lock)
        self._state_lock = threading.Lock()
        self._latest_state: dict | None = None

        # State receiver thread
        self._recv_thread = threading.Thread(target=self._recv_loop, daemon=True)
        self._recv_thread.start()

        # ROS2 subscriptions
        self._lowcmd_sub = self.create_subscription(
            LowCmd, "/lowcmd", self._on_lowcmd, _CMD_QOS
        )
        self._base_cmd_sub = self.create_subscription(
            Twist, "/base_cmd", self._on_base_cmd, _CMD_QOS
        )

        # ROS2 publisher + timer
        self._lowstate_pub = self.create_publisher(LowState, "/lowstate", _SENSOR_QOS)
        self._state_timer = self.create_timer(1.0 / state_hz, self._publish_lowstate)

        self.get_logger().info(
            f"H1IsaacBridge ready — /lowstate at {state_hz:.0f} Hz"
        )

    # ── ZMQ receive thread ─────────────────────────────────────────────────

    def _recv_loop(self) -> None:
        """Block on ZMQ state socket and keep latest packet."""
        while rclpy.ok():
            try:
                raw = self._state_sock.recv(flags=0)  # blocking
                state = json.loads(raw)
                with self._state_lock:
                    self._latest_state = state
            except Exception as exc:
                self.get_logger().warn(f"ZMQ recv error: {exc}", throttle_duration_sec=5)
                time.sleep(0.01)

    # ── ROS2 callbacks ─────────────────────────────────────────────────────

    def _on_lowcmd(self, msg: LowCmd) -> None:
        q = [0.0] * N_UPPER
        modes = [0] * N_UPPER
        for i in range(min(N_UPPER, len(msg.motor_cmd))):
            q[i] = float(msg.motor_cmd[i].q)
            modes[i] = int(msg.motor_cmd[i].mode)
        payload = json.dumps({"type": "lowcmd", "q": q, "mode": modes}).encode()
        try:
            self._cmd_sock.send(payload, zmq.NOBLOCK)
        except zmq.Again:
            pass
        self.get_logger().info(
            "LowCmd → sim | R_elbow={:.3f} L_elbow={:.3f}".format(q[7], q[14]),
            throttle_duration_sec=2.0,
        )

    def _on_base_cmd(self, msg: Twist) -> None:
        payload = json.dumps({
            "type": "basecmd",
            "vx": msg.linear.x,
            "vy": msg.linear.y,
            "omega": msg.angular.z,
        }).encode()
        try:
            self._cmd_sock.send(payload, zmq.NOBLOCK)
        except zmq.Again:
            pass
        self.get_logger().info(
            "BasCmd → sim | vx={:.3f} vy={:.3f} ω={:.3f}".format(
                msg.linear.x, msg.linear.y, msg.angular.z
            ),
            throttle_duration_sec=2.0,
        )

    # ── State publisher ────────────────────────────────────────────────────

    def _publish_lowstate(self) -> None:
        with self._state_lock:
            state = self._latest_state
        if state is None:
            return

        msg = LowState()
        q   = state.get("q",  [0.0] * N_UPPER)
        dq  = state.get("dq", [0.0] * N_UPPER)

        for i in range(N_UPPER):
            ms = MotorState()
            ms.mode = 1
            ms.q   = float(q[i])  if i < len(q)  else 0.0
            ms.dq  = float(dq[i]) if i < len(dq) else 0.0
            ms.ddq = 0.0
            ms.tau_est = 0.0
            msg.motor_state[i] = ms

        imu = IMUState()
        imu.quaternion    = [float(x) for x in state.get("quat", [1, 0, 0, 0])]
        imu.gyroscope     = [float(x) for x in state.get("gyro", [0, 0, 0])]
        imu.accelerometer = [float(x) for x in state.get("acc",  [0, 0, 9.81])]
        imu.rpy           = [0.0, 0.0, 0.0]
        msg.imu_state = imu

        self._lowstate_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = H1IsaacBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
