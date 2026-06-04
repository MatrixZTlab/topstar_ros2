"""H1 joint index definitions, limits, and unit conversions.

This module is the local source of truth for the H1 ROS2 package.
"""
from __future__ import annotations

import enum

import numpy as np


class H1JointIndex(enum.IntEnum):
    TORSO_LIFT = 0
    TORSO_PITCH = 1
    HEAD_YAW = 2
    HEAD_PITCH = 3
    RIGHT_SHOULDER_BASE = 4
    RIGHT_SHOULDER = 5
    RIGHT_ELBOW_YAW = 6
    RIGHT_ELBOW = 7
    RIGHT_WRIST_YAW = 8
    RIGHT_WRIST_PITCH = 9
    RIGHT_WRIST_ROLL = 10
    LEFT_SHOULDER_BASE = 11
    LEFT_SHOULDER = 12
    LEFT_ELBOW_YAW = 13
    LEFT_ELBOW = 14
    LEFT_WRIST_YAW = 15
    LEFT_WRIST_PITCH = 16
    LEFT_WRIST_ROLL = 17


H1_NUM_JOINTS: int = 18
H1_MOTOR_SLOTS: int = 18

H1_URDF_JOINT_NAMES: list[str] = [
    "Robot_Body_Movement_Joint",
    "Robot_Body_Rotation_Joint",
    "Robot_Head_Rotation_Joint",
    "Robot_Head_Tonod_Joint",
    "Robot_Right_Hand_base_Joint",
    "Robot_Right_Hand_1_Joint",
    "Robot_Right_Hand_2_Joint",
    "Robot_Right_Hand_3_Joint",
    "Robot_Right_Hand_4_Joint",
    "Robot_Right_Hand_5_Joint",
    "Robot_Right_Hand_6_Joint",
    "Robot_Left_Hand_base_Joint",
    "Robot_Left_Hand_1_Joint",
    "Robot_Left_Hand_2_Joint",
    "Robot_Left_Hand_3_Joint",
    "Robot_Left_Hand_4_Joint",
    "Robot_Left_Hand_5_Joint",
    "Robot_Left_Hand_6_Joint",
]

H1_HW_TO_MJ_SCALE: np.ndarray = np.array([
    -1.0,
    -1.0,
    -1.0,
    -1.0,
    1.0,
    1.0,
    1.0,
    1.0,
    1.0,
    1.0,
    1.0,
    1.0,
    1.0,
    1.0,
    1.0,
    1.0,
    1.0,
    1.0,
], dtype=np.float64)
H1_MJ_TO_HW_SCALE: np.ndarray = H1_HW_TO_MJ_SCALE.copy()

# Limits in MuJoCo (mj) convention.  mj = -hw for indices 0-3, mj = hw for 4-17.
# Source of truth for hw limits: JOINT_SPECS in h1_upper_body_jog.py.
H1_JOINT_LIMITS_MJ: np.ndarray = np.array([
    # idx  joint                hw limits → mj limits (sign-flip for 0-3)
    [-0.4500,  0.0100],        # 0  TORSO_LIFT          hw[-0.01, 0.45] m
    [-1.65806279,  0.0000],    # 1  TORSO_PITCH         hw[0.0, 1.658] rad
    [-1.5708,  1.5708],        # 2  HEAD_YAW            hw[-1.5708, 1.5708] rad
    [-0.48869219,  0.6981317], # 3  HEAD_PITCH          hw[-0.698, 0.489] rad
    [-2.61799388,  2.61799388],# 4  R_SHOULDER_BASE
    [-1.57079633,  0.43633231],# 5  R_SHOULDER
    [-2.61799388,  2.61799388],# 6  R_ELBOW_YAW
    [-1.79768913,  0.43633231],# 7  R_ELBOW
    [-2.87979327,  2.87979327],# 8  R_WRIST_YAW
    [-1.53588974,  0.43633231],# 9  R_WRIST_PITCH
    [-2.96705973,  2.96705973],# 10 R_WRIST_ROLL
    [-2.61799388,  2.61799388],# 11 L_SHOULDER_BASE
    [-1.57079633,  0.43633231],# 12 L_SHOULDER
    [-2.61799388,  2.61799388],# 13 L_ELBOW_YAW
    [-1.79768913,  0.43633231],# 14 L_ELBOW
    [-2.87979327,  2.87979327],# 15 L_WRIST_YAW
    [-1.53588974,  0.43633231],# 16 L_WRIST_PITCH
    [-2.96705973,  2.96705973],# 17 L_WRIST_ROLL
], dtype=np.float64)

ROBOT0_H1_INDICES: list[int] = [4, 5, 6, 7, 8, 9, 10, 1, 0]
ROBOT1_H1_INDICES: list[int] = [11, 12, 13, 14, 15, 16, 17, 2, 3]

H1_DEFAULT_KP = np.array([
    5000, 3000, 500, 500,
    800, 800, 800, 800, 600, 600, 600,
    800, 800, 800, 800, 600, 600, 600,
], dtype=np.float32)

H1_DEFAULT_KD = np.array([
    100, 80, 20, 20,
    30, 30, 30, 30, 20, 20, 20,
    30, 30, 30, 30, 20, 20, 20,
], dtype=np.float32)


def hw_to_mj(q_hw: np.ndarray) -> np.ndarray:
    return H1_HW_TO_MJ_SCALE * q_hw


def mj_to_hw(q_mj: np.ndarray) -> np.ndarray:
    return H1_MJ_TO_HW_SCALE * q_mj
