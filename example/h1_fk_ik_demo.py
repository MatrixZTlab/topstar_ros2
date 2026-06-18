#!/usr/bin/env python3
"""h1_fk_ik_demo.py — FK / IK service demo and round-trip test for the H1 robot.

Base frame
----------
All FK outputs and IK inputs use **Robot_Body_Rotation_Link** as the reference frame.
That link is the torso upper-body frame — the child of the TORSO_PITCH revolute joint.
Its position in the kinematic chain (at zero config):

  base_link
    └─ Robot_Body_Movement_Joint (TORSO_LIFT, prismatic, idx 0)
         xyz=[-0.0800, 0, 0.4155]  rpy=[-π, 0, π]
       Robot_Body_Movement_Link
    └─ Robot_Body_Rotation_Joint (TORSO_PITCH, revolute, idx 1)
         xyz=[0.2360, 0, 0]  rpy=[-π/2, 0, 0]
       **Robot_Body_Rotation_Link**  ← base frame origin

In Robot_Body_Rotation_Link coordinates (torso_lift=0, torso_pitch=0):
  +x  ≈ forward / ventral direction
  +y  ≈ vertical upward
  +z  ≈ robot's right lateral direction

Static transforms to arm mount links (from URDF joint origins)
--------------------------------------------------------------
Right arm (Robot_Right_Hand_base_Joint):
  origin  xyz=[-0.015, 0.5643,  0.1205]  rpy=[0, 0, 0]
  → T_body_right: identity rotation, translation [-0.015, 0.5643,  0.1205]

Left arm (Robot_Left_Hand_base_Joint):
  origin  xyz=[-0.015, 0.5643, -0.1205]  rpy=[π, 0, π]
  → rotation Rx(π)@Rz(π) = diag(-1, 1, -1); translation [-0.015, 0.5643, -0.1205]

At these mount points, little_top.urdf's base_link is placed.
little_top joints 1–7 map to H1 hw indices:
  right arm → [4, 5, 6, 7, 8, 9, 10]  (shoulder_base … wrist_roll)
  left  arm → [11,12,13,14,15,16,17]

Usage
-----
  # With the H1 node already running:
  python3 h1_fk_ik_demo.py

  # Geometry-only test (no ROS2 required):
  python3 h1_fk_ik_demo.py --dry-run
"""
from __future__ import annotations

import argparse
import sys
import time

import numpy as np
from scipy.spatial.transform import Rotation

try:
    import rclpy
except ImportError:
    rclpy = None


# ── Static frame data (mirrors h1_ros2_node.py) ──────────────────────────────

T_BODY_TO_RIGHT_ARM: np.ndarray = np.array([
    [ 1.,  0.,  0., -0.015],
    [ 0.,  1.,  0.,  0.5643],
    [ 0.,  0.,  1.,  0.1205],
    [ 0.,  0.,  0.,  1.],
], dtype=np.float64)

T_BODY_TO_LEFT_ARM: np.ndarray = np.array([
    [-1.,  0.,  0., -0.015],
    [ 0.,  1.,  0.,  0.5643],
    [ 0.,  0., -1., -0.1205],
    [ 0.,  0.,  0.,  1.],
], dtype=np.float64)

RIGHT_ARM_HW_IDX = list(range(4, 11))   # H1 hw indices for right arm
LEFT_ARM_HW_IDX  = list(range(11, 18))  # H1 hw indices for left arm


def print_frame_summary() -> None:
    print("=" * 64)
    print("H1 FK/IK frame geometry summary")
    print("=" * 64)
    print()
    print("Base frame : Robot_Body_Rotation_Link")
    print("  Origin   : torso upper-body; child of TORSO_PITCH joint")
    print()
    print("Right arm mount (in base frame):")
    print(f"  translation : [-0.015, 0.5643,  0.1205] m")
    print(f"  rotation    : identity (rpy = [0, 0, 0])")
    print()
    print("Left arm mount (in base frame):")
    print(f"  translation : [-0.015, 0.5643, -0.1205] m")
    print(f"  rotation    : Rx(π)·Rz(π) = diag(-1, 1, -1)  (rpy = [π, 0, π])")
    print()
    print("FK/IK joint ordering (7-DOF, H1 hw convention):")
    print(f"  right arm hw indices : {RIGHT_ARM_HW_IDX}")
    print(f"  left  arm hw indices : {LEFT_ARM_HW_IDX}")
    print()
    print("  little_top joint1 = shoulder_base,  joint7 = wrist_roll")
    print()


# ── Geometry-only dry-run ─────────────────────────────────────────────────────

def dry_run() -> None:
    """Test FK/IK math locally without ROS2, using IIWAIK directly."""
    import sys, os
    # Make the vendored package importable when running outside the ROS2 install tree.
    _pkg_root = os.path.join(os.path.dirname(__file__), 'src', 'src', 'h1')
    if _pkg_root not in sys.path:
        sys.path.insert(0, _pkg_root)
    from topstar_h1.vendor.topstar.dls_ik import IIWAIK
    ik = IIWAIK()

    print("── Dry-run: local IIWAIK (no ROS2) ──────────────────────────────")
    print()

    tests = [
        ("right", "zero config",
         np.zeros(7),
         None),
        ("right", "bent elbow",
         np.array([0.2, -0.5, 0.1, -0.8, 0.0, 0.3, 0.0]),
         None),
        ("left", "bent elbow",
         np.array([0.2, -0.5, 0.1, -0.8, 0.0, 0.3, 0.0]),
         None),
    ]

    all_ok = True
    for arm, label, q7, _ in tests:
        T_body_to_arm = T_BODY_TO_RIGHT_ARM if arm == 'right' else T_BODY_TO_LEFT_ARM
        T_ee_arm  = ik.forward_kinematics(q7)
        T_ee_body = T_body_to_arm @ T_ee_arm
        pos = T_ee_body[:3, 3]
        rpy = Rotation.from_matrix(T_ee_body[:3, :3]).as_euler('xyz', degrees=True)
        print(f"FK  [{arm:5s} | {label}]")
        print(f"    EE in body frame: pos={np.round(pos,4)}  rpy_deg={np.round(rpy,1)}")

        # IK round-trip: target = FK result displaced slightly
        T_target_body = T_ee_body.copy()
        T_target_body[0, 3] += 0.03   # 3 cm along body +x
        T_target_arm  = np.linalg.inv(T_body_to_arm) @ T_target_body
        q7_ik, err = ik.inverse_kinematics(
            T_target_arm, initial_angles=q7, max_iterations=200
        )
        T_check_arm  = ik.forward_kinematics(q7_ik)
        T_check_body = T_body_to_arm @ T_check_arm
        pos_err = np.linalg.norm(T_check_body[:3, 3] - T_target_body[:3, 3])
        ok = pos_err < 5e-3   # 5 mm — DLS numeric solver, not analytic
        if not ok:
            all_ok = False
        print(f"IK  [{arm:5s} | {label}] err={pos_err*1000:.3f} mm  {'OK' if ok else 'FAIL (>5 mm)'}")
        print()

    print("Dry-run result:", "PASS" if all_ok else "FAIL")
    return all_ok


# ── Live ROS2 service tests ────────────────────────────────────────────────────

def call_fk(node, client_fk, arm: str, q7: np.ndarray) -> tuple[bool, np.ndarray, str]:
    """Call get_arm_fk and return (success, 4x4 T, message)."""
    from topstar_hg.srv import GetArmFK
    req = GetArmFK.Request()
    req.arm = arm
    req.joint_angles = list(q7.astype(float))
    future = client_fk.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
    if not future.done():
        return False, np.eye(4), "timeout"
    resp = future.result()
    T = np.array(resp.transform, dtype=np.float64).reshape(4, 4)
    return resp.success, T, resp.message


def call_ik(node, client_ik, arm: str, T_body: np.ndarray,
            method: str = 'placo', seed: np.ndarray | None = None,
            ) -> tuple[bool, np.ndarray, float, str]:
    """Call get_arm_ik and return (success, q7, error_norm, message)."""
    from topstar_hg.srv import GetArmIK
    req = GetArmIK.Request()
    req.arm = arm
    req.transform = list(T_body.flatten().astype(float))
    req.method = method
    if seed is not None:
        req.seed_joints = list(seed.astype(float))
        req.use_seed = True
    else:
        req.use_seed = False
    future = client_ik.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
    if not future.done():
        return False, np.zeros(7), 0.0, "timeout"
    resp = future.result()
    q7 = np.array(resp.joint_angles, dtype=np.float64)
    return resp.success, q7, resp.error_norm, resp.message


def live_test(method: str) -> bool:
    from rclpy.node import Node
    from topstar_hg.srv import GetArmFK, GetArmIK

    rclpy.init()
    node = Node('h1_fk_ik_demo')

    client_fk = node.create_client(GetArmFK, 'get_arm_fk')
    client_ik = node.create_client(GetArmIK, 'get_arm_ik')

    print(f"Waiting for FK/IK services (method={method}) ...")
    if not client_fk.wait_for_service(timeout_sec=10.0):
        print("ERROR: get_arm_fk service not available")
        node.destroy_node(); rclpy.shutdown()
        return False
    if not client_ik.wait_for_service(timeout_sec=10.0):
        print("ERROR: get_arm_ik service not available")
        node.destroy_node(); rclpy.shutdown()
        return False
    print("Services ready.")
    print()

    # Test cases: (arm, label, q7_seed)
    test_cases = [
        ("right", "zero config",
         np.zeros(7)),
        ("right", "bent arm",
         np.array([0.2, -0.5, 0.1, -0.8, 0.0, 0.3, 0.0])),
        ("left",  "bent arm",
         np.array([0.2, -0.5, 0.1, -0.8, 0.0, 0.3, 0.0])),
    ]

    all_ok = True
    for arm, label, q7 in test_cases:
        print(f"{'─'*60}")
        print(f"Arm: {arm}   Config: {label}")
        print(f"  Input q7 (hw rad): {np.round(q7, 4)}")

        # ── FK ──
        ok_fk, T_fk, msg_fk = call_fk(node, client_fk, arm, q7)
        if not ok_fk:
            print(f"  FK FAILED: {msg_fk}")
            all_ok = False
            continue
        pos = T_fk[:3, 3]
        rpy = Rotation.from_matrix(T_fk[:3, :3]).as_euler('xyz', degrees=True)
        print(f"  FK EE in body frame:")
        print(f"    pos (m)   : {np.round(pos, 4)}")
        print(f"    rpy (deg) : {np.round(rpy, 2)}")

        # ── IK round-trip: add 3 cm offset along body +x ──
        T_target = T_fk.copy()
        T_target[0, 3] += 0.03
        print(f"  IK target (+3 cm along body x): pos={np.round(T_target[:3,3],4)}")

        ok_ik, q7_ik, err, msg_ik = call_ik(
            node, client_ik, arm, T_target, method=method, seed=q7,
        )
        if not ok_ik:
            print(f"  IK FAILED: {msg_ik}")
            all_ok = False
            continue
        print(f"  IK solution q7 : {np.round(q7_ik, 4)}")
        print(f"  IK error_norm  : {err*1000:.3f} mm  ({msg_ik})")

        # Verify FK(IK result) ≈ target
        ok_fk2, T_check, _ = call_fk(node, client_fk, arm, q7_ik)
        if ok_fk2:
            pos_err = np.linalg.norm(T_check[:3, 3] - T_target[:3, 3])
            ok_rt = pos_err < 5e-3   # 5 mm tolerance
            print(f"  Round-trip FK pos error : {pos_err*1000:.3f} mm  "
                  f"{'OK' if ok_rt else 'FAIL (> 5 mm)'}")
            if not ok_rt:
                all_ok = False
        print()

    node.destroy_node()
    rclpy.shutdown()
    print("Live test result:", "PASS" if all_ok else "FAIL")
    return all_ok


# ── Main ──────────────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--dry-run', action='store_true',
                        help='Run geometry test locally without ROS2')
    parser.add_argument('--method', default='placo', choices=['placo', 'iiwa_ik'],
                        help='IK method to use in live test (default: placo)')
    args = parser.parse_args()

    print_frame_summary()

    if args.dry_run:
        ok = dry_run()
    else:
        import rclpy
        ok = live_test(args.method)

    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
