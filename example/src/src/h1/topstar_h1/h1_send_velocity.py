#!/usr/bin/env python3
"""Send H1 base velocity commands over ROS2 (/base_cmd).

Usage:
  ros2 run topstar_ros2_example h1_send_velocity --vx 0.3 --duration 3.0
  ros2 run topstar_ros2_example h1_send_velocity --vx 0.2 --omega 0.5 --hold
  ros2 run topstar_ros2_example h1_send_velocity --omega 1.0 --duration 2.0
"""
from __future__ import annotations

import argparse
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist

_CMD_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)


def main() -> None:
    parser = argparse.ArgumentParser(description="Send a cmd_vel command to the H1 base over ROS2.")
    parser.add_argument("--vx",       type=float, default=0.0, help="forward velocity in m/s")
    parser.add_argument("--vy",       type=float, default=0.0, help="lateral velocity in m/s")
    parser.add_argument("--omega",    type=float, default=0.0, help="yaw rate in rad/s")
    parser.add_argument("--duration", type=float, default=1.0, help="how long to publish (seconds); ignored with --hold")
    parser.add_argument("--hz",       type=float, default=10.0, help="publish rate in Hz")
    parser.add_argument("--hold",     action="store_true", help="keep publishing until Ctrl+C")
    args = parser.parse_args()

    rclpy.init()
    node = Node("h1_vel_sender")
    pub = node.create_publisher(Twist, "/base_cmd", _CMD_QOS)

    cmd = Twist()
    cmd.linear.x  = args.vx
    cmd.linear.y  = args.vy
    cmd.angular.z = args.omega

    period = 1.0 / args.hz
    end = time.time() + args.duration

    try:
        while rclpy.ok():
            pub.publish(cmd)
            rclpy.spin_once(node, timeout_sec=0)
            if not args.hold and time.time() >= end:
                break
            time.sleep(period)
    except KeyboardInterrupt:
        pass
    finally:
        pub.publish(Twist())
        rclpy.spin_once(node, timeout_sec=0.05)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
