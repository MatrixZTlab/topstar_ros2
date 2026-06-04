#!/usr/bin/env python3
"""h2_isaac_ros2_bridge.py — ROS2 bridge between Isaac Sim and H2 topics.

Subscribes:
  /lowcmd    topstar_hg/LowCmd   → ZMQ "lowcmd" to Isaac Sim

Publishes:
  /lowstate  topstar_hg/LowState ← ZMQ state from Isaac Sim (50 Hz)

Run AFTER h2_isaac_sim.py is started:
  source /opt/ros/humble/setup.bash
  source ~/topstar_ros2/cyclonedds_ws/install/local_setup.bash
  python3 ~/topstar_ros2/example/isaac_bridge/h2_isaac_ros2_bridge.py

ZMQ sockets (localhost only):
  :15557  Isaac Sim → bridge  (state,    PUSH/PULL)
  :15558  bridge → Isaac Sim  (commands, PUSH/PULL)
"""
from __future__ import annotations
import json
import threading
import time

import zmq

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

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

N_JOINTS = 29


class H2IsaacBridge(Node):
    """ROS2 node that bridges /lowcmd and /lowstate to/from Isaac Sim via ZMQ."""

    def __init__(self) -> None:
        super().__init__("h2_isaac_ros2_bridge")

        self.declare_parameter("state_hz", 50)
        state_hz = float(self.get_parameter("state_hz").value)

        ctx = zmq.Context()

        self._state_sock = ctx.socket(zmq.PULL)
        self._state_sock.setsockopt(zmq.RCVHWM, 2)
        self._state_sock.connect("tcp://127.0.0.1:15557")

        self._cmd_sock = ctx.socket(zmq.PUSH)
        self._cmd_sock.setsockopt(zmq.SNDHWM, 2)
        self._cmd_sock.connect("tcp://127.0.0.1:15558")

        self._state_lock = threading.Lock()
        self._latest_state: dict | None = None

        self._recv_thread = threading.Thread(target=self._recv_loop, daemon=True)
        self._recv_thread.start()

        self._lowcmd_sub = self.create_subscription(
            LowCmd, "/lowcmd", self._on_lowcmd, _CMD_QOS
        )

        self._lowstate_pub = self.create_publisher(LowState, "/lowstate", _SENSOR_QOS)
        self._state_timer = self.create_timer(1.0 / state_hz, self._publish_lowstate)

        self.get_logger().info(
            f"H2IsaacBridge ready — /lowstate at {state_hz:.0f} Hz"
        )

    def _recv_loop(self) -> None:
        while rclpy.ok():
            try:
                raw = self._state_sock.recv(flags=0)
                state = json.loads(raw)
                with self._state_lock:
                    self._latest_state = state
            except Exception as exc:
                self.get_logger().warn(f"ZMQ recv error: {exc}", throttle_duration_sec=5)
                time.sleep(0.01)

    def _on_lowcmd(self, msg: LowCmd) -> None:
        q = [0.0] * N_JOINTS
        modes = [0] * N_JOINTS
        for i in range(min(N_JOINTS, len(msg.motor_cmd))):
            q[i] = float(msg.motor_cmd[i].q)
            modes[i] = int(msg.motor_cmd[i].mode)
        payload = json.dumps({"type": "lowcmd", "q": q, "mode": modes}).encode()
        try:
            self._cmd_sock.send(payload, zmq.NOBLOCK)
        except zmq.Again:
            pass
        self.get_logger().info(
            "LowCmd → sim | L_knee={:.3f} R_knee={:.3f}".format(q[3], q[9]),
            throttle_duration_sec=2.0,
        )

    def _publish_lowstate(self) -> None:
        with self._state_lock:
            state = self._latest_state
        if state is None:
            return

        msg = LowState()
        q  = state.get("q",  [0.0] * N_JOINTS)
        dq = state.get("dq", [0.0] * N_JOINTS)

        for i in range(N_JOINTS):
            ms = MotorState()
            ms.mode = 1
            ms.q       = float(q[i])  if i < len(q)  else 0.0
            ms.dq      = float(dq[i]) if i < len(dq) else 0.0
            ms.ddq     = 0.0
            ms.tau_est = 0.0
            msg.motor_state[i] = ms

        msg.mode_machine = 0

        imu = IMUState()
        imu.quaternion    = [float(x) for x in state.get("quat", [1, 0, 0, 0])]
        imu.gyroscope     = [float(x) for x in state.get("gyro", [0, 0, 0])]
        imu.accelerometer = [float(x) for x in state.get("acc",  [0, 0, 9.81])]
        imu.rpy           = [0.0, 0.0, 0.0]
        msg.imu_state = imu

        self._lowstate_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = H2IsaacBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
