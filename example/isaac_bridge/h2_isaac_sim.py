"""h2_isaac_sim.py — Isaac Sim headless simulation for the H2 bipedal robot.

Publishes robot state over ZMQ and receives joint commands.
Pair with h2_isaac_ros2_bridge.py to expose the /lowcmd and /lowstate ROS2 topics.

Phase 1: fix_base=True (upper body + leg control, base pinned at world origin).
         Set fix_base=False in H2_ROBOT_CFG to enable free-floating base (Phase 2).

Usage:
  conda run -n env_isaaclab \\
    python ~/topstar_ros2/example/isaac_bridge/h2_isaac_sim.py --headless
"""

"""Launch Isaac Sim first — must be before all other imports."""
import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser("H2 Isaac Sim Bridge")
AppLauncher.add_app_launcher_args(parser)
args_cli, _ = parser.parse_known_args()
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Post-launch imports."""
import json
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

# ── Joint ordering: slot i ↔ LowCmd motor_cmd[i] ──────────────────────────
# Must match JOINT_NAMES order in h2_lowcmd_gui.py and h2_isaac_jog.py.
H2_JOINT_NAMES = [
    "left_hip_pitch_joint",       # slot 0  L_HipPitch
    "left_hip_roll_joint",        # slot 1  L_HipRoll
    "left_hip_yaw_joint",         # slot 2  L_HipYaw
    "left_knee_joint",            # slot 3  L_Knee
    "left_ankle_pitch_joint",     # slot 4  L_AnklePitch
    "left_ankle_roll_joint",      # slot 5  L_AnkleRoll
    "right_hip_pitch_joint",      # slot 6  R_HipPitch
    "right_hip_roll_joint",       # slot 7  R_HipRoll
    "right_hip_yaw_joint",        # slot 8  R_HipYaw
    "right_knee_joint",           # slot 9  R_Knee
    "right_ankle_pitch_joint",    # slot 10 R_AnklePitch
    "right_ankle_roll_joint",     # slot 11 R_AnkleRoll
    "waist_yaw_joint",            # slot 12 WaistYaw
    "head_yaw_joint",             # slot 13 HeadYaw
    "head_pitch_joint",           # slot 14 HeadPitch
    "left_shoulder_pitch_joint",  # slot 15 L_ShoulderPitch
    "left_shoulder_roll_joint",   # slot 16 L_ShoulderRoll
    "left_shoulder_yaw_joint",    # slot 17 L_ShoulderYaw
    "left_elbow_joint",           # slot 18 L_Elbow
    "left_wrist_yaw_joint",       # slot 19 L_WristYaw
    "left_wrist_pitch_joint",     # slot 20 L_WristPitch
    "left_wrist_roll_joint",      # slot 21 L_WristRoll
    "right_shoulder_pitch_joint", # slot 22 R_ShoulderPitch
    "right_shoulder_roll_joint",  # slot 23 R_ShoulderRoll
    "right_shoulder_yaw_joint",   # slot 24 R_ShoulderYaw
    "right_elbow_joint",          # slot 25 R_Elbow
    "right_wrist_yaw_joint",      # slot 26 R_WristYaw
    "right_wrist_pitch_joint",    # slot 27 R_WristPitch
    "right_wrist_roll_joint",     # slot 28 R_WristRoll
]
N_JOINTS = len(H2_JOINT_NAMES)

# Phase 1 assumption: all signs positive. Verify in sim; if a joint moves in the
# wrong direction, negate the corresponding index here (e.g. HW_TO_URDF_SIGN[3] = -1).
HW_TO_URDF_SIGN = np.ones(N_JOINTS, dtype=np.float64)

URDF_PATH = os.environ.get(
    "H2_URDF_PATH",
    os.path.expanduser("~/topstar_h2/h2_model/urdf/h2_abs.urdf"),
)

# ── Elastic-band configuration ───────────────────────────────────────────────
# Mirrors topstar_mujoco simulate_python/topstar_bridge.py ElasticBand.
# A spring-damper force is applied to the pelvis (base_hip_link) every physics
# step via robot.set_external_force_and_torque().
# Has no effect in Phase 1 (fix_base=True) because the base is pinned.
# Toggle ELASTIC_BAND_ENABLE=False to disable, or adjust LENGTH to change how
# much of gravity is countered (0 = full pull toward anchor, ~2.1 m ≈ 420 N up).
ELASTIC_BAND_ENABLE    = True
ELASTIC_BAND_STIFFNESS = 200.0          # N/m — spring constant
ELASTIC_BAND_DAMPING   = 100.0          # N·s/m — velocity damping
ELASTIC_BAND_ANCHOR    = np.array([0.0, 0.0, 3.0])  # world-frame attach point (m)
ELASTIC_BAND_LENGTH    = 0.0            # rest length (m); 0 = always pulls toward anchor

# ── ArticulationCfg — gains tuned for Phase 1 fixed-base operation ─────────
H2_ROBOT_CFG = ArticulationCfg(
    prim_path="/World/H2",
    spawn=UrdfFileCfg(
        asset_path=URDF_PATH,
        fix_base=True,   # Phase 1: base pinned; set False for free-floating base
        force_usd_conversion=True,
        joint_drive=UrdfConverterCfg.JointDriveCfg(
            gains=UrdfConverterCfg.JointDriveCfg.PDGainsCfg(stiffness=0.0, damping=0.0)
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(pos=(0.0, 0.0, 0.9)),
    actuators={
        # Gains: stiffness = Kp (N·m/rad), damping = Kd (N·m·s/rad).
        # Unlike MuJoCo's torque actuators (Kp/Kd applied explicitly in Python at
        # policy rate ~50 Hz), Isaac Sim ImplicitActuator feeds stiffness/damping
        # directly into PhysX's implicit PD solver at the physics rate (200 Hz).
        # Critical damping condition: damping_crit = 2 * sqrt(stiffness * I_eff).
        # With stiffness=100 and I_eff≈0.5–2 kg·m², damping_crit≈14–28.
        # The MuJoCo deploy value damping=2 is appropriate for a 1 kHz servo loop;
        # here it gives ζ≈0.07–0.14 (severely underdamped). Values below target
        # ζ≈0.5–0.7 for well-damped position tracking.
        "hip": ImplicitActuatorCfg(
            joint_names_expr=[
                "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
                "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
            ],
            effort_limit_sim=240.0, velocity_limit_sim=5.55,
            stiffness=100.0, damping=10.0,
        ),
        "knee": ImplicitActuatorCfg(
            joint_names_expr=["left_knee_joint", "right_knee_joint"],
            effort_limit_sim=240.0, velocity_limit_sim=5.55,
            stiffness=100.0, damping=10.0,
        ),
        "ankle": ImplicitActuatorCfg(
            joint_names_expr=[
                "left_ankle_pitch_joint", "left_ankle_roll_joint",
                "right_ankle_pitch_joint", "right_ankle_roll_joint",
            ],
            effort_limit_sim=104.0, velocity_limit_sim=10.47,
            stiffness=100.0, damping=10.0,
        ),
        "waist": ImplicitActuatorCfg(
            joint_names_expr=["waist_yaw_joint"],
            effort_limit_sim=102.0, velocity_limit_sim=3.77,
            stiffness=100.0, damping=10.0,
        ),
        "head": ImplicitActuatorCfg(
            joint_names_expr=["head_yaw_joint", "head_pitch_joint"],
            effort_limit_sim=11.0, velocity_limit_sim=3.87,
            stiffness=50.0, damping=5.0,
        ),
        "shoulder": ImplicitActuatorCfg(
            joint_names_expr=[
                "left_shoulder_pitch_joint", "left_shoulder_roll_joint",
                "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
            ],
            effort_limit_sim=66.0, velocity_limit_sim=3.78,
            stiffness=50.0, damping=5.0,
        ),
        "shoulder_yaw_elbow": ImplicitActuatorCfg(
            joint_names_expr=[
                "left_shoulder_yaw_joint", "left_elbow_joint",
                "right_shoulder_yaw_joint", "right_elbow_joint",
            ],
            effort_limit_sim=34.0, velocity_limit_sim=5.45,
            stiffness=50.0, damping=5.0,
        ),
        "wrist": ImplicitActuatorCfg(
            joint_names_expr=[
                "left_wrist_yaw_joint", "left_wrist_pitch_joint", "left_wrist_roll_joint",
                "right_wrist_yaw_joint", "right_wrist_pitch_joint", "right_wrist_roll_joint",
            ],
            effort_limit_sim=11.0, velocity_limit_sim=3.87,
            stiffness=30.0, damping=3.0,
        ),
    },
)


def main():
    ctx = zmq.Context()

    state_sock = ctx.socket(zmq.PUSH)
    state_sock.setsockopt(zmq.SNDHWM, 2)
    state_sock.bind("tcp://127.0.0.1:15557")

    cmd_sock = ctx.socket(zmq.PULL)
    cmd_sock.setsockopt(zmq.RCVHWM, 4)
    cmd_sock.bind("tcp://127.0.0.1:15558")

    sim = sim_utils.SimulationContext(
        sim_utils.SimulationCfg(dt=1.0 / 200.0, gravity=(0.0, 0.0, -9.81))
    )
    sim_utils.GroundPlaneCfg().func("/World/ground", sim_utils.GroundPlaneCfg())
    sim_utils.DomeLightCfg(intensity=2000.0, color=(0.75, 0.75, 0.75)).func(
        "/World/skyLight", sim_utils.DomeLightCfg(intensity=2000.0, color=(0.75, 0.75, 0.75))
    )

    robot = Articulation(H2_ROBOT_CFG)
    sim.reset()
    robot.reset()

    joint_ids, _ = robot.find_joints(H2_JOINT_NAMES, preserve_order=True)
    device = sim.device

    joint_pos_target = torch.zeros(1, N_JOINTS, device=device)

    # Elastic band: find pelvis body index, pre-allocate force tensors.
    eb_body_ids, _ = robot.find_bodies(["base_hip_link"])
    eb_forces  = torch.zeros(1, 1, 3, device=device)
    eb_torques = torch.zeros(1, 1, 3, device=device)

    print("[H2 Isaac] Ready — state on :15557, commands on :15558")
    if ELASTIC_BAND_ENABLE:
        print(f"[H2 Isaac] Elastic band ON  stiffness={ELASTIC_BAND_STIFFNESS} "
              f"damping={ELASTIC_BAND_DAMPING} anchor={ELASTIC_BAND_ANCHOR.tolist()} "
              f"length={ELASTIC_BAND_LENGTH}")
    step = 0

    while simulation_app.is_running():
        # ── Drain command queue (non-blocking) ─────────────────────────
        while True:
            try:
                msg = json.loads(cmd_sock.recv(zmq.NOBLOCK))
                if msg.get("type") == "lowcmd":
                    q_in = msg["q"]
                    modes = msg["mode"]
                    for i in range(min(N_JOINTS, len(q_in))):
                        if modes[i] == 1:
                            joint_pos_target[0, i] = float(q_in[i]) * HW_TO_URDF_SIGN[i]
            except zmq.Again:
                break

        # ── Apply controls ──────────────────────────────────────────────
        robot.set_joint_position_target(joint_pos_target, joint_ids=joint_ids)

        # ── Elastic band ────────────────────────────────────────────────
        if ELASTIC_BAND_ENABLE:
            pos = robot.data.root_pos_w[0].cpu().numpy()
            vel = robot.data.root_lin_vel_w[0].cpu().numpy()
            delta = ELASTIC_BAND_ANCHOR - pos
            dist  = float(np.linalg.norm(delta))
            if dist > 1e-6:
                direction = delta / dist
                v_along   = float(np.dot(vel, direction))
                f = ((ELASTIC_BAND_STIFFNESS * (dist - ELASTIC_BAND_LENGTH)
                      - ELASTIC_BAND_DAMPING * v_along) * direction)
                eb_forces[0, 0] = torch.as_tensor(f, dtype=torch.float32, device=device)
            else:
                eb_forces.zero_()
            robot.set_external_force_and_torque(eb_forces, eb_torques,
                                                body_ids=eb_body_ids)

        robot.write_data_to_sim()

        # ── Physics step ────────────────────────────────────────────────
        sim.step()
        robot.update(1.0 / 200.0)

        # ── Publish state at 50 Hz (every 4 steps) ──────────────────────
        if step % 4 == 0:
            q_urdf  = robot.data.joint_pos[0, joint_ids].cpu().numpy()
            dq_urdf = robot.data.joint_vel[0, joint_ids].cpu().numpy()
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
                        "acc":  [0.0, 0.0, 9.81],
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
