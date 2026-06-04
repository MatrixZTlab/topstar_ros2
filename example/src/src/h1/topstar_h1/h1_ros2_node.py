#!/usr/bin/env python3

"""h1_ros2_node.py — ROS2 bridge node for the H1 wheeled humanoid.

Arm API (topstar_api request/response on /api/arm/request → /api/arm/response)
-------------------------------------------------------------------------------
API ID 1001 — move_joints_timed
  Request  parameter JSON: {"joints": [<18 floats, hw rad/m>], "duration": <float s>}
  Response data JSON:      {} (empty on success)
  Response status.code:    0 = accepted, non-zero = error (see ARM_API_* constants)
  The motion runs asynchronously; the response is sent as soon as the command is
  validated, not when the motion completes.  Regular /lowcmd joint commands are
  suppressed for the duration of the move so they cannot override the trajectory.

Topics
------
Subscribed:
  /lowcmd           topstar_hg/msg/LowCmd       — joint position commands (slots 0–17)
  /base_cmd         geometry_msgs/msg/Twist     — base velocity (vx, vy, omega)
  /hand/right/cmd   topstar_hg/msg/GripperCmd   — right gripper command
  /hand/left/cmd    topstar_hg/msg/GripperCmd   — left gripper command

Published:
  /lowstate         topstar_hg/msg/LowState     — joint + IMU state (slots 0–17)
  /hand/right/state topstar_hg/msg/GripperState — right gripper state
  /hand/left/state  topstar_hg/msg/GripperState — left gripper state

The node shares an H1Bridge instance with the MuJoCo simulation loop.
It can be instantiated two ways:

  Standalone (simulation):
    bridge = H1Bridge(...)  # already started by caller
    node = H1Ros2Node(bridge)
    rclpy.spin(node)

  Via launch file (recommended):
    ros2 launch topstar_ros2_example h1_sim.launch.py
"""
from __future__ import annotations

import json
import sys
import os
import threading
import time
from typing import Any

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist
from topstar_hg.msg import LowCmd, LowState, MotorCmd, MotorState, IMUState, GripperCmd, GripperState
from topstar_api.msg import Request as ArmRequest, Response as ArmResponse

from topstar_h1.backends import create_backend
from topstar_h1.joint_defs import H1JointIndex, H1_NUM_JOINTS, H1_MOTOR_SLOTS

# Arm API identifiers (topstar_api request/response style)
ARM_API_ID_MOVE_JOINTS_TIMED = 1001

ARM_ERR_OK = 0
ARM_ERR_INVALID_PARAMS = 1001
ARM_ERR_INTERNAL = 1002


# QoS matching the topstar_hg convention (sensor data)
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


class H1Ros2Node(Node):
    """ROS2 node that bridges H1 topics and H1Bridge.

    Parameters
    ----------
    bridge:
        A started H1Bridge instance (shared with the simulation thread).
    state_hz:
        Rate at which /lowstate is published (default 50 Hz).
    """

    def __init__(self, bridge: Any) -> None:
        super().__init__("h1_ros2_node")
        self._bridge = bridge

        self.declare_parameter("state_hz", 50)
        state_hz = float(self.get_parameter("state_hz").value)

        # ── Subscribers ───────────────────────────────────────────────────
        # /lowcmd is a high-rate position stream: only the *latest* pose
        # matters.  BEST_EFFORT + KEEP_LAST(1) prevents the DDS layer from
        # buffering stale messages that would burst into the callback when the
        # executor is momentarily slow, flooding the InterpController queue.
        self._lowcmd_sub = self.create_subscription(
            LowCmd,
            "/lowcmd",
            self._on_lowcmd,
            _SENSOR_QOS,  # BEST_EFFORT + KEEP_LAST(1) — latest-wins
        )
        self._base_cmd_sub = self.create_subscription(
            Twist,
            "/base_cmd",
            self._on_base_cmd,
            _CMD_QOS,
        )
        self._gripper_right_sub = self.create_subscription(
            GripperCmd, "/hand/right/cmd",
            lambda m: self._on_gripper_cmd(0, m), _CMD_QOS,
        )
        self._gripper_left_sub = self.create_subscription(
            GripperCmd, "/hand/left/cmd",
            lambda m: self._on_gripper_cmd(1, m), _CMD_QOS,
        )

        # ── Publishers ────────────────────────────────────────────────────
        self._lowstate_pub = self.create_publisher(
            LowState,
            "/lowstate",
            _SENSOR_QOS,
        )
        self._gripper_right_pub = self.create_publisher(
            GripperState, "/hand/right/state", _SENSOR_QOS,
        )
        self._gripper_left_pub = self.create_publisher(
            GripperState, "/hand/left/state", _SENSOR_QOS,
        )

        # ── Arm API (topstar_api request/response) ────────────────────────
        self._arm_req_sub = self.create_subscription(
            ArmRequest, "/api/arm/request", self._on_arm_request, _CMD_QOS
        )
        self._arm_resp_pub = self.create_publisher(ArmResponse, "/api/arm/response", _CMD_QOS)

        # ── State publish timer ───────────────────────────────────────────
        period = 1.0 / state_hz
        self._state_timer = self.create_timer(period, self._publish_lowstate)

        self.get_logger().info(
            f"H1Ros2Node started — publishing /lowstate at {state_hz:.0f} Hz"
        )

    # ── Callbacks ─────────────────────────────────────────────────────────

    def _on_lowcmd(self, msg: LowCmd) -> None:
        """Receive a LowCmd and forward joint targets to H1Bridge (slots 0–17 only)."""
        try:
            joints = np.array(self._bridge.get_state()["joints_hw"], dtype=np.float64)
        except Exception:
            joints = np.zeros(H1_NUM_JOINTS, dtype=np.float64)

        for i in range(min(H1_MOTOR_SLOTS, len(msg.motor_cmd))):
            if msg.motor_cmd[i].mode == 1:
                joints[i] = float(msg.motor_cmd[i].q)
        self._bridge.set_upper_body_joints(joints)
        self.get_logger().info(
            "LowCmd received — elbow R={:.3f} L={:.3f} rad".format(
                joints[7], joints[14]),
            throttle_duration_sec=2.0,
        )

    def _on_gripper_cmd(self, arm_idx: int, msg: GripperCmd) -> None:
        """Receive a GripperCmd for one arm and forward to H1Bridge."""
        try:
            gripper = np.array(
                self._bridge.get_state().get("gripper", [0.0, 0.0]), dtype=np.float64
            )
        except Exception:
            gripper = np.zeros(2, dtype=np.float64)
        gripper[arm_idx] = float(msg.position)
        self._bridge.set_gripper(gripper)

    def _on_arm_request(self, msg: ArmRequest) -> None:
        api_id = msg.header.identity.api_id
        req_id = msg.header.identity.id

        resp = ArmResponse()
        resp.header.identity.api_id = api_id
        resp.header.identity.id = req_id

        if api_id == ARM_API_ID_MOVE_JOINTS_TIMED:
            try:
                params = json.loads(msg.parameter)
                joints = np.array(params["joints"], dtype=np.float64)
                duration = float(params["duration"])
                if joints.shape != (H1_NUM_JOINTS,):
                    raise ValueError(f"expected {H1_NUM_JOINTS} joints, got {joints.shape[0]}")
                if duration <= 0:
                    raise ValueError(f"duration must be positive, got {duration}")
                self._bridge.set_upper_body_joints_timed(joints, duration)
                resp.header.status.code = ARM_ERR_OK
                self.get_logger().info(
                    f"arm move_joints_timed: duration={duration:.2f}s accepted",
                    throttle_duration_sec=1.0,
                )
            except Exception as exc:
                resp.header.status.code = ARM_ERR_INVALID_PARAMS
                resp.data = str(exc)
                self.get_logger().error(f"arm move_joints_timed failed: {exc}")
        else:
            resp.header.status.code = ARM_ERR_INVALID_PARAMS
            resp.data = f"unknown api_id {api_id}"
            self.get_logger().warn(f"arm request: unknown api_id {api_id}")

        self._arm_resp_pub.publish(resp)

    def _on_base_cmd(self, msg: Twist) -> None:
        """Receive a Twist and forward to the wheel base."""
        self._bridge.set_base_velocity(
            vx=msg.linear.x,
            vy=msg.linear.y,
            omega=msg.angular.z,
        )
        # self.get_logger().info(
        #     "BasCmd received — vx={:.3f} vy={:.3f} omega={:.3f}".format(
        #         msg.linear.x, msg.linear.y, msg.angular.z),
        #     throttle_duration_sec=2.0,
        # )

    def _publish_lowstate(self) -> None:
        """Read current robot state and publish LowState."""
        try:
            state = self._bridge.get_state()
        except Exception as exc:
            self.get_logger().warn(f"get_state failed: {exc}")
            return

        msg = LowState()
        joints_hw = state["joints_hw"]

        for i in range(H1_MOTOR_SLOTS):
            ms = MotorState()
            ms.mode = 1
            ms.q = float(joints_hw[i]) if i < len(joints_hw) else 0.0
            ms.dq = 0.0
            ms.ddq = 0.0
            ms.tau_est = 0.0
            msg.motor_state[i] = ms

        # IMU — read from MuJoCo sensordata if bridge exposes it
        imu = IMUState()
        imu_data = state.get("imu", None)
        if imu_data is not None:
            imu.quaternion = [float(x) for x in imu_data.get("quat", [1, 0, 0, 0])]
            imu.gyroscope   = [float(x) for x in imu_data.get("gyro", [0, 0, 0])]
            imu.accelerometer = [float(x) for x in imu_data.get("acc",  [0, 0, 0])]
            imu.rpy         = [float(x) for x in imu_data.get("rpy",  [0, 0, 0])]
        else:
            imu.quaternion = [1.0, 0.0, 0.0, 0.0]
            imu.gyroscope = [0.0, 0.0, 0.0]
            imu.accelerometer = [0.0, 0.0, 0.0]
            imu.rpy = [0.0, 0.0, 0.0]
        msg.imu_state = imu

        self._lowstate_pub.publish(msg)

        gripper_fb = state.get("gripper")
        for arm_idx, pub in enumerate([self._gripper_right_pub, self._gripper_left_pub]):
            gs = GripperState()
            if gripper_fb is not None and arm_idx < len(gripper_fb):
                gs.position = float(gripper_fb[arm_idx])
            pub.publish(gs)


def main(args=None) -> None:
    """Entry point for ros2 run / launch."""
    import logging
    import os

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)-8s %(name)s: %(message)s",
        datefmt="%H:%M:%S",
    )

    sim_path = os.path.expanduser(os.environ.get(
        "TOPSTAR_SIM_PATH",
        "~/topstar_mujoco/simulate_python",
    ))
    backend_kind = os.environ.get("TOPSTAR_H1_BACKEND", "mujoco")

    upper_body_config = None
    cfg_file = os.environ.get("TOPSTAR_H1_ROBOT_CFG_FILE", "")
    if cfg_file:
        import json
        with open(cfg_file) as f:
            upper_body_config = json.load(f)

    rclpy.init(args=args)

    bridge = create_backend(
        backend_kind,
        sim_path=sim_path,
        upper_body_config=upper_body_config,
        use_mock_upper_body=(backend_kind != "xapi"),
        frequency=50,
    )
    bridge.start()

    node = H1Ros2Node(bridge)
    node.get_logger().info(f"H1 backend selected: {backend_kind}")
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        bridge.stop()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
