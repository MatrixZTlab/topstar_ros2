"""h1_isaac_sim.py — Isaac Sim headless simulation for H1 upper-body robot.

Publishes robot state over ZMQ and receives joint commands.
Pair with h1_isaac_ros2_bridge.py to expose the H1 ROS2 topics.

Phase 1: fix_base=True (upper body control only).
         Set fix_base=False in H1_ROBOT_CFG to enable mobile base.

Usage (from ~/topstar_rl_lab):
  conda run -n env_isaaclab \\
    python scripts/h1_isaac_bridge/h1_isaac_sim.py --headless
"""

"""Launch Isaac Sim first — must be before all other imports."""
import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser("H1 Isaac Sim Bridge")
AppLauncher.add_app_launcher_args(parser)
args_cli, _ = parser.parse_known_args()
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Post-launch imports."""
import json
import math
import os
import time

import numpy as np
import torch
import zmq

import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation, ArticulationCfg
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.sim.spawners.from_files import UrdfFileCfg
from isaaclab.sim.converters.urdf_converter_cfg import UrdfConverterCfg

# ── Joint ordering: index i ↔ LowCmd motor_cmd[i] ─────────────────────────
H1_JOINT_NAMES = [
    "Robot_Body_Movement_Joint",    # slot 0  TORSO_LIFT
    "Robot_Body_Rotation_Joint",    # slot 1  TORSO_PITCH
    "Robot_Head_Rotation_Joint",    # slot 2  HEAD_YAW
    "Robot_Head_Tonod_Joint",       # slot 3  HEAD_PITCH
    "Robot_Right_Hand_base_Joint",  # slot 4  RIGHT_SHOULDER_BASE
    "Robot_Right_Hand_1_Joint",     # slot 5  RIGHT_SHOULDER
    "Robot_Right_Hand_2_Joint",     # slot 6  RIGHT_ELBOW_YAW
    "Robot_Right_Hand_3_Joint",     # slot 7  RIGHT_ELBOW
    "Robot_Right_Hand_4_Joint",     # slot 8  RIGHT_WRIST_YAW
    "Robot_Right_Hand_5_Joint",     # slot 9  RIGHT_WRIST_PITCH
    "Robot_Right_Hand_6_Joint",     # slot 10 RIGHT_WRIST_ROLL
    "Robot_Left_Hand_base_Joint",   # slot 11 LEFT_SHOULDER_BASE
    "Robot_Left_Hand_1_Joint",      # slot 12 LEFT_SHOULDER
    "Robot_Left_Hand_2_Joint",      # slot 13 LEFT_ELBOW_YAW
    "Robot_Left_Hand_3_Joint",      # slot 14 LEFT_ELBOW
    "Robot_Left_Hand_4_Joint",      # slot 15 LEFT_WRIST_YAW
    "Robot_Left_Hand_5_Joint",      # slot 16 LEFT_WRIST_PITCH
    "Robot_Left_Hand_6_Joint",      # slot 17 LEFT_WRIST_ROLL
]
N_UPPER = len(H1_JOINT_NAMES)

# Slots whose URDF axis points opposite to the hardware SDK convention.
# hw_q = -urdf_q  for these joints:
#   0 TORSO_LIFT  (mj range [-0.45, 0]  ↔ hw [0, 0.45] m)
#   1 TORSO_PITCH (mj range [-1.658, 0] ↔ hw [0, 1.658] rad)
#   3 HEAD_PITCH  (mj range [-0.489, 0.559] ↔ hw [-0.559, 0.489] rad)
HW_TO_URDF_SIGN = np.ones(N_UPPER, dtype=np.float64)
HW_TO_URDF_SIGN[[0, 1, 3]] = -1.0

STEER_JOINT_NAMES = [
    "Wheel_Rotation_1_1_Joint",
    "Wheel_Rotation_2_1_Joint",
    "Wheel_Rotation_3_1_Joint",
    "Wheel_Rotation_4_1_Joint",
]
DRIVE_JOINT_NAMES = [
    "Wheel_Rotation_1_2_Joint",
    "Wheel_Rotation_2_2_Joint",
    "Wheel_Rotation_3_2_Joint",
    "Wheel_Rotation_4_2_Joint",
]

# Wheel positions (x, y) in base frame [m]
WHEEL_XY = np.array([
    [-0.22, -0.165],
    [-0.22,  0.165],
    [ 0.22, -0.165],
    [ 0.22,  0.165],
], dtype=np.float64)
WHEEL_R = 0.0625  # m

URDF_PATH = os.environ.get(
    "H1_URDF_PATH",
    os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                  "..", "src", "urdf", "h1", "h1_abs.urdf")),
)

# ── ArticulationCfg — gains mirror MuJoCo h1.xml actuators ────────────────
H1_ROBOT_CFG = ArticulationCfg(
    prim_path="/World/H1",
    spawn=UrdfFileCfg(
        asset_path=URDF_PATH,
        fix_base=True,   # Phase 1: pinned; change to False for mobile base
        force_usd_conversion=True,  # re-convert on every launch; set False once stable
        # joint_drive gains are placeholders; ArticulationCfg.actuators overrides them
        joint_drive=UrdfConverterCfg.JointDriveCfg(
            gains=UrdfConverterCfg.JointDriveCfg.PDGainsCfg(stiffness=0.0, damping=0.0)
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(pos=(0.0, 0.0, 0.0)),
    actuators={
        "torso_lift": ImplicitActuatorCfg(
            joint_names_expr=["Robot_Body_Movement_Joint"],
            effort_limit_sim=10000.0, velocity_limit_sim=1.0,
            stiffness=5000.0, damping=100.0,
        ),
        "torso_pitch": ImplicitActuatorCfg(
            joint_names_expr=["Robot_Body_Rotation_Joint"],
            effort_limit_sim=10000.0, velocity_limit_sim=2.0,
            stiffness=3000.0, damping=80.0,
        ),
        "head": ImplicitActuatorCfg(
            joint_names_expr=["Robot_Head_Rotation_Joint", "Robot_Head_Tonod_Joint"],
            effort_limit_sim=500.0, velocity_limit_sim=3.14,
            stiffness=500.0, damping=20.0,
        ),
        "shoulder": ImplicitActuatorCfg(
            joint_names_expr=["Robot_Right_Hand_base_Joint", "Robot_Left_Hand_base_Joint",
                               "Robot_Right_Hand_1_Joint",   "Robot_Left_Hand_1_Joint"],
            effort_limit_sim=3000.0, velocity_limit_sim=3.14,
            stiffness=2000.0, damping=30.0,
        ),
        "elbow": ImplicitActuatorCfg(
            joint_names_expr=["Robot_Right_Hand_2_Joint", "Robot_Left_Hand_2_Joint",
                               "Robot_Right_Hand_3_Joint", "Robot_Left_Hand_3_Joint"],
            effort_limit_sim=3000.0, velocity_limit_sim=3.14,
            stiffness=2000.0, damping=30.0,
        ),
        "wrist": ImplicitActuatorCfg(
            joint_names_expr=["Robot_Right_Hand_4_Joint", "Robot_Left_Hand_4_Joint",
                               "Robot_Right_Hand_5_Joint", "Robot_Left_Hand_5_Joint",
                               "Robot_Right_Hand_6_Joint", "Robot_Left_Hand_6_Joint"],
            effort_limit_sim=1000.0, velocity_limit_sim=5.0,
            stiffness=600.0, damping=20.0,
        ),
        "wheel_steer": ImplicitActuatorCfg(
            joint_names_expr=["Wheel_Rotation_.*_1_Joint"],
            effort_limit_sim=200.0, velocity_limit_sim=5.0,
            stiffness=200.0, damping=10.0,
        ),
        # Drive joints: velocity control (stiffness=0)
        "wheel_drive": ImplicitActuatorCfg(
            joint_names_expr=["Wheel_Rotation_.*_2_Joint"],
            effort_limit_sim=1200.0, velocity_limit_sim=100.0,
            stiffness=0.0, damping=5.0,
        ),
    },
)


def swerve_ik(vx: float, vy: float, omega: float):
    """Swerve drive IK → (steer_angles[4], drive_ang_vels[4])."""
    steer = np.empty(4)
    drive = np.empty(4)
    for i, (rx, ry) in enumerate(WHEEL_XY):
        wx = vx - omega * ry
        wy = vy + omega * rx
        steer[i] = math.atan2(wy, wx)
        drive[i] = math.hypot(wx, wy) / WHEEL_R
    return steer, drive


def main():
    # ZMQ sockets
    ctx = zmq.Context()

    state_sock = ctx.socket(zmq.PUSH)
    state_sock.setsockopt(zmq.SNDHWM, 2)
    state_sock.bind("tcp://127.0.0.1:15555")

    cmd_sock = ctx.socket(zmq.PULL)
    cmd_sock.setsockopt(zmq.RCVHWM, 4)
    cmd_sock.bind("tcp://127.0.0.1:15556")

    # Scene
    sim = sim_utils.SimulationContext(
        sim_utils.SimulationCfg(dt=1.0 / 200.0, gravity=(0.0, 0.0, -9.81))
    )
    sim_utils.GroundPlaneCfg().func("/World/ground", sim_utils.GroundPlaneCfg())
    sim_utils.DomeLightCfg(intensity=2000.0, color=(0.75, 0.75, 0.75)).func(
        "/World/skyLight", sim_utils.DomeLightCfg(intensity=2000.0, color=(0.75, 0.75, 0.75))
    )

    robot = Articulation(H1_ROBOT_CFG)
    sim.reset()
    robot.reset()

    # Build index lists once
    h1_ids, _ = robot.find_joints(H1_JOINT_NAMES, preserve_order=True)
    steer_ids, _ = robot.find_joints(STEER_JOINT_NAMES, preserve_order=True)
    drive_ids, _ = robot.find_joints(DRIVE_JOINT_NAMES, preserve_order=True)

    device = sim.device

    # Initial control targets = default joint positions
    joint_pos_target = torch.zeros(1, N_UPPER, device=device)
    steer_target     = torch.zeros(1, 4, device=device)
    drive_target     = torch.zeros(1, 4, device=device)

    print("[H1 Isaac] Ready — state on :15555, commands on :15556")
    step = 0

    while simulation_app.is_running():
        # ── Drain command queue (non-blocking) ─────────────────────────
        while True:
            try:
                msg = json.loads(cmd_sock.recv(zmq.NOBLOCK))
                t = msg["type"]
                if t == "lowcmd":
                    q_in = msg["q"]
                    modes = msg["mode"]
                    for i in range(min(N_UPPER, len(q_in))):
                        if modes[i] == 1:
                            joint_pos_target[0, i] = float(q_in[i]) * HW_TO_URDF_SIGN[i]
                elif t == "basecmd":
                    sa, dv = swerve_ik(msg["vx"], msg["vy"], msg["omega"])
                    steer_target[0] = torch.from_numpy(sa.astype(np.float32)).to(device)
                    drive_target[0] = torch.from_numpy(dv.astype(np.float32)).to(device)
            except zmq.Again:
                break

        # ── Apply controls ──────────────────────────────────────────────
        robot.set_joint_position_target(joint_pos_target, joint_ids=h1_ids)
        robot.set_joint_position_target(steer_target, joint_ids=steer_ids)
        robot.set_joint_velocity_target(drive_target, joint_ids=drive_ids)
        robot.write_data_to_sim()

        # ── Physics step ────────────────────────────────────────────────
        sim.step()
        robot.update(1.0 / 200.0)

        # ── Publish state at 50 Hz (every 4 steps) ──────────────────────
        if step % 4 == 0:
            q_urdf  = robot.data.joint_pos[0, h1_ids].cpu().numpy()
            dq_urdf = robot.data.joint_vel[0, h1_ids].cpu().numpy()
            q_out  = (q_urdf  * HW_TO_URDF_SIGN).tolist()
            dq_out = (dq_urdf * HW_TO_URDF_SIGN).tolist()

            root_quat    = robot.data.root_quat_w[0].cpu().numpy().tolist()
            root_ang_vel = robot.data.root_ang_vel_b[0].cpu().numpy().tolist()

            try:
                state_sock.send(
                    json.dumps({
                        "t":    time.monotonic(),
                        "q":    q_out,
                        "dq":   dq_out,
                        "quat": root_quat,
                        "gyro": root_ang_vel,
                        "acc":  [0.0, 0.0, 9.81],  # gravity-aligned constant
                    }).encode(),
                    zmq.NOBLOCK,
                )
            except zmq.Again:
                pass

        step += 1

    state_sock.close()
    cmd_sock.close()
    ctx.term()
    simulation_app.close()


if __name__ == "__main__":
    main()
