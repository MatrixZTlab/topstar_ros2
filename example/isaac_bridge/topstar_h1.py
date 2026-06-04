"""Configuration for the Topstar H1 wheeled upper-body robot (18 DOF + 4-wheel swerve base).

Joint layout (H1JointIndex, slots 0–17 in LowCmd/LowState):
  0:  Robot_Body_Movement_Joint    TORSO_LIFT   (prismatic, +z, 0–0.45 m)
  1:  Robot_Body_Rotation_Joint    TORSO_PITCH  (revolute)
  2:  Robot_Head_Rotation_Joint    HEAD_YAW     (revolute)
  3:  Robot_Head_Tonod_Joint       HEAD_PITCH   (revolute)
  4–10: Right arm (7 DOF): base, shoulder, elbow_yaw, elbow, wrist_yaw, wrist_pitch, wrist_roll
 11–17: Left arm  (7 DOF): mirror

Wheel base: 4-wheel swerve drive (Wheel_Rotation_{1-4}_{1-2}_Joint).
  _1_ = steer (position), _2_ = drive (velocity).
  Controlled via /base_cmd (geometry_msgs/Twist), not LowCmd slots.

Phase 1 (fix_base=True): upper-body control only; base is fixed to world.
Phase 2 (fix_base=False): full mobile-base simulation via wheel contacts.
"""

import os

from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg
from isaaclab.sim.converters.urdf_converter_cfg import UrdfConverterCfg
from isaaclab.sim.spawners.from_files import UrdfFileCfg

TOPSTAR_H1_URDF = os.environ.get(
    "H1_URDF_PATH",
    os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                  "..", "src", "urdf", "h1", "h1_abs.urdf")),
)

TOPSTAR_H1_CFG = ArticulationCfg(
    spawn=UrdfFileCfg(
        asset_path=TOPSTAR_H1_URDF,
        fix_base=True,   # set False to enable swerve-drive mobile base
        joint_drive=UrdfConverterCfg.JointDriveCfg(
            gains=UrdfConverterCfg.JointDriveCfg.PDGainsCfg(stiffness=0.0, damping=0.0)
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.0),
        joint_pos={
            "Robot_Body_Movement_Joint":   0.0,
            "Robot_Body_Rotation_Joint":   0.0,
            "Robot_Head_Rotation_Joint":   0.0,
            "Robot_Head_Tonod_Joint":      0.0,
            ".*_Hand_base_Joint":          0.0,
            ".*_Hand_1_Joint":             0.0,
            ".*_Hand_2_Joint":             0.0,
            ".*_Hand_3_Joint":             0.0,
            ".*_Hand_4_Joint":             0.0,
            ".*_Hand_5_Joint":             0.0,
            ".*_Hand_6_Joint":             0.0,
            "Wheel_Rotation_.*":           0.0,
        },
    ),
    actuators={
        # Torso lift: prismatic, m_eff≈21.24 kg
        # kp=5000 N/m, kd=100 N·s/m (critical damping: 2√(kp·m)=652; 100 is underdamped)
        # Note: MuJoCo uses kd=652 + 208 N gravity bias. Isaac Sim omits bias → slight sag.
        "torso_lift": ImplicitActuatorCfg(
            joint_names_expr=["Robot_Body_Movement_Joint"],
            effort_limit_sim=10000.0,
            velocity_limit_sim=1.0,
            stiffness=5000.0,
            damping=652.0,
        ),
        "torso_pitch": ImplicitActuatorCfg(
            joint_names_expr=["Robot_Body_Rotation_Joint"],
            effort_limit_sim=10000.0,
            velocity_limit_sim=2.0,
            stiffness=3000.0,
            damping=257.0,
        ),
        "head": ImplicitActuatorCfg(
            joint_names_expr=["Robot_Head_Rotation_Joint", "Robot_Head_Tonod_Joint"],
            effort_limit_sim=500.0,
            velocity_limit_sim=3.14,
            stiffness=500.0,
            damping=20.0,
        ),
        "shoulder": ImplicitActuatorCfg(
            joint_names_expr=["Robot_Right_Hand_base_Joint", "Robot_Left_Hand_base_Joint",
                               "Robot_Right_Hand_1_Joint",   "Robot_Left_Hand_1_Joint"],
            effort_limit_sim=3000.0,
            velocity_limit_sim=3.14,
            stiffness=2000.0,
            damping=72.0,
        ),
        "elbow": ImplicitActuatorCfg(
            joint_names_expr=["Robot_Right_Hand_2_Joint", "Robot_Left_Hand_2_Joint",
                               "Robot_Right_Hand_3_Joint", "Robot_Left_Hand_3_Joint"],
            effort_limit_sim=3000.0,
            velocity_limit_sim=3.14,
            stiffness=2000.0,
            damping=30.0,
        ),
        "wrist": ImplicitActuatorCfg(
            joint_names_expr=["Robot_Right_Hand_4_Joint", "Robot_Left_Hand_4_Joint",
                               "Robot_Right_Hand_5_Joint", "Robot_Left_Hand_5_Joint",
                               "Robot_Right_Hand_6_Joint", "Robot_Left_Hand_6_Joint"],
            effort_limit_sim=1000.0,
            velocity_limit_sim=5.0,
            stiffness=600.0,
            damping=20.0,
        ),
        "wheel_steer": ImplicitActuatorCfg(
            joint_names_expr=["Wheel_Rotation_.*_1_Joint"],
            effort_limit_sim=200.0,
            velocity_limit_sim=5.0,
            stiffness=200.0,
            damping=10.0,
        ),
        # Drive joints: velocity control (stiffness=0, damping acts as speed regulator)
        "wheel_drive": ImplicitActuatorCfg(
            joint_names_expr=["Wheel_Rotation_.*_2_Joint"],
            effort_limit_sim=1200.0,
            velocity_limit_sim=100.0,
            stiffness=0.0,
            damping=5.0,
        ),
    },
    # SDK joint ordering matches H1JointIndex (slots 0–17)
    # fmt: off
    joint_sdk_names=[
        "Robot_Body_Movement_Joint",    # 0  TORSO_LIFT
        "Robot_Body_Rotation_Joint",    # 1  TORSO_PITCH
        "Robot_Head_Rotation_Joint",    # 2  HEAD_YAW
        "Robot_Head_Tonod_Joint",       # 3  HEAD_PITCH
        "Robot_Right_Hand_base_Joint",  # 4  RIGHT_SHOULDER_BASE
        "Robot_Right_Hand_1_Joint",     # 5  RIGHT_SHOULDER
        "Robot_Right_Hand_2_Joint",     # 6  RIGHT_ELBOW_YAW
        "Robot_Right_Hand_3_Joint",     # 7  RIGHT_ELBOW
        "Robot_Right_Hand_4_Joint",     # 8  RIGHT_WRIST_YAW
        "Robot_Right_Hand_5_Joint",     # 9  RIGHT_WRIST_PITCH
        "Robot_Right_Hand_6_Joint",     # 10 RIGHT_WRIST_ROLL
        "Robot_Left_Hand_base_Joint",   # 11 LEFT_SHOULDER_BASE
        "Robot_Left_Hand_1_Joint",      # 12 LEFT_SHOULDER
        "Robot_Left_Hand_2_Joint",      # 13 LEFT_ELBOW_YAW
        "Robot_Left_Hand_3_Joint",      # 14 LEFT_ELBOW
        "Robot_Left_Hand_4_Joint",      # 15 LEFT_WRIST_YAW
        "Robot_Left_Hand_5_Joint",      # 16 LEFT_WRIST_PITCH
        "Robot_Left_Hand_6_Joint",      # 17 LEFT_WRIST_ROLL
    ],
    # fmt: on
)
