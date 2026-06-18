"""h1_sim.launch.py — Launch the H1 simulation + ROS2 bridge node.

Usage:
  ros2 launch topstar_ros2_example h1_sim.launch.py
  ros2 launch topstar_ros2_example h1_sim.launch.py state_hz:=100
  ros2 launch topstar_ros2_example h1_sim.launch.py backend:=isaac
  ros2 launch topstar_ros2_example h1_sim.launch.py backend:=xapi
  ros2 launch topstar_ros2_example h1_sim.launch.py sim_path:=/path/to/simulate_python
  ros2 launch topstar_ros2_example h1_sim.launch.py viewer:=true
  ros2 launch topstar_ros2_example h1_sim.launch.py config_file:=/path/to/robot_config.json

Environment variables honoured by the node:
  TOPSTAR_ROBOT               — must be "h1" (set automatically here)
  TOPSTAR_SIM_PATH            — path to topstar_mujoco/simulate_python
  TOPSTAR_H1_BACKEND          — mujoco | isaac | xapi
  TOPSTAR_H1_ROBOT_CFG_FILE   — path to robot JSON config (set from config_file arg)
"""
from __future__ import annotations

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _placo_env() -> dict:
    """Discover cmeel lib/site-packages dirs needed for placo and return env overrides.

    placo (and its pinocchio/eigenpy dependencies) are installed via pip into
    cmeel.prefix subtrees inside Python's site-packages.  The exact location
    depends on whether the install was system-wide (sudo pip) or per-user
    (pip --user).  We probe the known candidate roots in preference order —
    newest / most-specific first — and collect every cmeel.prefix that actually
    exists on disk.

    Both PYTHONPATH and LD_LIBRARY_PATH must point at the *same* cmeel tree so
    that the placo.so and its native shared libraries are ABI-compatible.
    Mixing trees (e.g. new placo.so with old boost libs) causes a SIGFPE on
    import.
    """
    import sys

    minor = sys.version_info.minor
    # Candidate site-packages roots, newest/most-specific first.
    sp_roots = [
        f"/usr/local/lib/python3.{minor}/dist-packages",
        os.path.expanduser(f"~/.local/lib/python3.{minor}/site-packages"),
        os.path.expanduser(
            f"~/miniforge3/envs/topstar-mujoco/lib/python3.{minor}/site-packages"
        ),
    ]

    lib_dirs: list[str] = []
    py_dirs:  list[str] = []
    for sp in sp_roots:
        cmeel = os.path.join(sp, "cmeel.prefix")
        lib = os.path.join(cmeel, "lib")
        pys = os.path.join(lib, f"python3.{minor}", "site-packages")
        if os.path.isdir(lib):
            lib_dirs.append(lib)
        if os.path.isdir(pys):
            py_dirs.append(pys)

    if not lib_dirs:
        return {}

    existing_ld = os.environ.get("LD_LIBRARY_PATH", "")
    existing_py = os.environ.get("PYTHONPATH", "")
    ld = ":".join(lib_dirs + ([existing_ld] if existing_ld else []))
    py = ":".join(py_dirs + ([existing_py] if existing_py else []))
    return {"LD_LIBRARY_PATH": ld, "PYTHONPATH": py}


def generate_launch_description() -> LaunchDescription:
    sim_path_default = os.path.expanduser(
        "~/topstar_mujoco/simulate_python"
    )
    config_file_default = os.path.join(
        get_package_share_directory("topstar_ros2_example"),
        "config", "h1", "robot_config.json",
    )

    return LaunchDescription([
        # ── Environment ──────────────────────────────────────────────────
        SetEnvironmentVariable("TOPSTAR_ROBOT", "h1"),

        # ── Arguments ────────────────────────────────────────────────────
        DeclareLaunchArgument(
            "backend",
            default_value="mujoco",
            description="H1 backend: mujoco, isaac, or xapi",
        ),
        DeclareLaunchArgument(
            "sim_path",
            default_value=sim_path_default,
            description="Path to topstar_mujoco/simulate_python",
        ),
        DeclareLaunchArgument(
            "state_hz",
            default_value="50",
            description="Rate (Hz) for /lowstate publication",
        ),
        DeclareLaunchArgument(
            "viewer",
            default_value="false",
            description="Launch MuJoCo viewer window via topstar_mujoco.py",
        ),
        DeclareLaunchArgument(
            "config_file",
            default_value=config_file_default,
            description="Path to robot JSON config file (robot IPs, gripper settings, etc.)",
        ),

        # ── Optional MuJoCo viewer process ──────────────────────────────
        ExecuteProcess(
            cmd=[
                "python3",
                "topstar_mujoco.py",
                "--ros-args",
                "-p",
                ["state_hz:=", LaunchConfiguration("state_hz")],
            ],
            cwd=LaunchConfiguration("sim_path"),
            output="screen",
            additional_env={
                "TOPSTAR_ROBOT": "h1",
                "TOPSTAR_SIM_PATH": LaunchConfiguration("sim_path"),
                "TOPSTAR_H1_BACKEND": LaunchConfiguration("backend"),
                "TOPSTAR_H1_ROBOT_CFG_FILE": LaunchConfiguration("config_file"),
                **_placo_env(),
            },
            condition=IfCondition(LaunchConfiguration("viewer")),
        ),

        # ── Headless H1 ROS2 node ───────────────────────────────────────
        Node(
            package="topstar_ros2_example",
            executable="h1_ros2_node",
            name="h1_ros2_node",
            output="screen",
            additional_env={
                "TOPSTAR_ROBOT": "h1",
                "TOPSTAR_SIM_PATH": LaunchConfiguration("sim_path"),
                "TOPSTAR_H1_BACKEND": LaunchConfiguration("backend"),
                "TOPSTAR_H1_ROBOT_CFG_FILE": LaunchConfiguration("config_file"),
                **_placo_env(),
            },
            parameters=[{
                "state_hz": LaunchConfiguration("state_hz"),
            }],
            condition=UnlessCondition(LaunchConfiguration("viewer")),
        ),
    ])
