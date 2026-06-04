"""H1 upper-body controller with local mock and optional hardware backends."""
from __future__ import annotations

import os
import sys
import threading
import time
from multiprocessing.managers import SharedMemoryManager
from queue import Full
from typing import Any

import numpy as np

from topstar_h1.joint_defs import (
    H1_JOINT_LIMITS_MJ,
    H1_NUM_JOINTS,
    H1_URDF_JOINT_NAMES,
    ROBOT0_H1_INDICES,
    ROBOT1_H1_INDICES,
    hw_to_mj,
    mj_to_hw,
)


_DEFAULT_CONFIG = {
    "robot0": {
        "ip": "192.168.1.10",
        "num_joints": 7,
        "init_joints": None,
        "init_duration": 2.0,
        "obs_latency": 0.01,
        "action_latency": 0.01,
        "gripper_enabled": False,
        "gripper_out_addr": None,
        "gripper_in_addr": None,
    },
    "robot1": {
        "ip": "192.168.1.9",
        "num_joints": 7,
        "init_joints": None,
        "init_duration": 2.0,
        "obs_latency": 0.01,
        "action_latency": 0.01,
        "gripper_enabled": False,
        "gripper_out_addr": None,
        "gripper_in_addr": None,
    },
}


def _load_interp_controller():
    try:
        from topstar_h1.vendor.topstar.topstar_xapi import InterpController
        return InterpController
    except ImportError:
        pass

    env_path = os.path.expanduser(os.environ.get("TOPSTAR_XAPI_CONTROLLER_PATH", ""))
    if env_path:
        if env_path not in sys.path:
            sys.path.insert(0, env_path)
    try:
        from topstar_xapi import InterpController
    except ImportError as exc:
        raise ImportError(
            "Unable to import InterpController. Preferred path is "
            "topstar_h1.vendor.topstar.topstar_xapi (vendored). If using a custom "
            "wrapper, set TOPSTAR_XAPI_CONTROLLER_PATH. Ensure vendor xapi Python "
            "package is installed in this environment."
        ) from exc
    return InterpController


class H1UpperBodyController:
    def __init__(
        self,
        config: dict | None = None,
        use_mock: bool = True,
        mj_model=None,
        mj_data=None,
        lock: threading.Lock | None = None,
        frequency: int = 50,
    ) -> None:
        self._use_mock = use_mock
        self._dt = 1.0 / frequency

        cfg = {}
        for key, value in _DEFAULT_CONFIG.items():
            cfg[key] = {**value, **(config or {}).get(key, {})}
        self._cfg = cfg

        if use_mock:
            if mj_model is None or mj_data is None:
                raise ValueError("mj_model and mj_data are required in mock mode")
            from topstar_h1.mock_xapi import MockInterpController

            r0_joint_names = [H1_URDF_JOINT_NAMES[i] for i in ROBOT0_H1_INDICES]
            r1_joint_names = [H1_URDF_JOINT_NAMES[i] for i in ROBOT1_H1_INDICES]
            r0_init = (
                np.array(cfg["robot0"]["init_joints"], dtype=np.float64)
                if cfg["robot0"]["init_joints"] is not None
                else None
            )
            r1_init = (
                np.array(cfg["robot1"]["init_joints"], dtype=np.float64)
                if cfg["robot1"]["init_joints"] is not None
                else None
            )
            self._robot0 = MockInterpController(
                mj_model=mj_model,
                mj_data=mj_data,
                joint_names=r0_joint_names,
                h1_indices=ROBOT0_H1_INDICES,
                joints_init=r0_init,
                joints_init_duration=cfg["robot0"]["init_duration"],
                frequency=frequency,
                lock=lock,
                gripper_enabled=cfg["robot0"]["gripper_enabled"],
            )
            self._robot1 = MockInterpController(
                mj_model=mj_model,
                mj_data=mj_data,
                joint_names=r1_joint_names,
                h1_indices=ROBOT1_H1_INDICES,
                joints_init=r1_init,
                joints_init_duration=cfg["robot1"]["init_duration"],
                frequency=frequency,
                lock=lock,
                gripper_enabled=cfg["robot1"]["gripper_enabled"],
            )
        else:
            InterpController = _load_interp_controller()
            shm_manager = SharedMemoryManager()
            shm_manager.start()
            scale9 = np.ones(9)
            self._robot0 = InterpController(
                shm_manager=shm_manager,
                robot_ip=cfg["robot0"]["ip"],
                num_joints=cfg["robot0"]["num_joints"],
                joints_init=cfg["robot0"]["init_joints"],
                joints_init_duration=cfg["robot0"]["init_duration"],
                frequency=frequency,
                scale=scale9,
                gripper_enabled=cfg["robot0"]["gripper_enabled"],
                gripper_addr=[
                    cfg["robot0"]["gripper_out_addr"],
                    cfg["robot0"]["gripper_in_addr"],
                ],
                max_gripper_speed=cfg["robot0"].get("gripper_speed", 100.0),
            )
            self._robot1 = InterpController(
                shm_manager=shm_manager,
                robot_ip=cfg["robot1"]["ip"],
                num_joints=cfg["robot1"]["num_joints"],
                joints_init=cfg["robot1"]["init_joints"],
                joints_init_duration=cfg["robot1"]["init_duration"],
                frequency=frequency,
                scale=scale9,
                gripper_enabled=cfg["robot1"]["gripper_enabled"],
                gripper_addr=[
                    cfg["robot1"]["gripper_out_addr"],
                    cfg["robot1"]["gripper_in_addr"],
                ],
                max_gripper_speed=cfg["robot1"].get("gripper_speed", 100.0),
            )

    def start(self) -> None:
        # Spawn both InterpController processes in parallel (non-blocking) before
        # waiting for either.  Sequential start() delays robot1's spawn until
        # robot0 is fully ready (potentially several seconds if joints_init is
        # configured), creating a permanent t_start phase offset between their
        # 50 Hz servo loops.  That phase offset causes both robots to evaluate
        # joint_interp(t_now) at different wall-clock times, making the left arm
        # consistently trail the right arm through every motion even though both
        # converge to the same target_time.  Parallel spawn keeps their t_start
        # values within milliseconds of each other.
        self._robot0.start(wait=False)
        self._robot1.start(wait=False)
        self._robot0.start_wait()
        self._robot1.start_wait()

    def stop(self) -> None:
        self._robot0.stop()
        self._robot1.stop()

    @property
    def is_ready(self) -> bool:
        return self._robot0.is_ready and self._robot1.is_ready

    @property
    def gripper_enabled(self) -> dict[str, bool]:
        return {
            "robot0": self._cfg["robot0"]["gripper_enabled"],
            "robot1": self._cfg["robot1"]["gripper_enabled"],
        }

    def get_state(self) -> dict[str, Any]:
        s0 = self._robot0.get_state()
        s1 = self._robot1.get_state()
        q0_xapi = np.asarray(s0["ActualQ"][:9], dtype=np.float64)
        q1_xapi = np.asarray(s1["ActualQ"][:9], dtype=np.float64)

        # xapi reports in hw convention: TORSO_LIFT in mm, all others in degrees.
        # hw→mj: sign flip for h1_idx < 4 (TORSO_LIFT, TORSO_PITCH, HEAD_YAW, HEAD_PITCH).
        joints_mj = np.zeros(H1_NUM_JOINTS, dtype=np.float64)
        for xapi_idx, h1_idx in enumerate(ROBOT0_H1_INDICES):
            v = q0_xapi[xapi_idx]
            unit = v / 1000.0 if h1_idx == 0 else np.radians(v)
            joints_mj[h1_idx] = -unit if h1_idx < 4 else unit
        for xapi_idx, h1_idx in enumerate(ROBOT1_H1_INDICES):
            v = q1_xapi[xapi_idx]
            joints_mj[h1_idx] = -np.radians(v) if h1_idx < 4 else np.radians(v)

        return {
            "joints_hw": mj_to_hw(joints_mj),
            "joints_mj": joints_mj,
            "gripper": np.array([
                s0.get("TargetGripper", 0.0),
                s1.get("TargetGripper", 0.0),
            ]),
        }

    def set_joints(
        self,
        joints_hw: np.ndarray,
        target_time: float | None = None,
        gripper_pos: np.ndarray | None = None,
    ) -> None:
        joints_hw = np.asarray(joints_hw, dtype=np.float64)
        if joints_hw.shape != (H1_NUM_JOINTS,):
            raise ValueError(f"Expected (18,), got {joints_hw.shape}")
        if target_time is None:
            target_time = time.monotonic() + self._dt

        joints_mj = hw_to_mj(joints_hw)
        joints_mj = np.clip(
            joints_mj,
            H1_JOINT_LIMITS_MJ[:, 0],
            H1_JOINT_LIMITS_MJ[:, 1],
        )

        # mj→hw sign flip for h1_idx < 4, then convert to xapi units (mm / degrees).
        pose0 = np.zeros(9, dtype=np.float64)
        pose1 = np.zeros(9, dtype=np.float64)
        for xapi_idx, h1_idx in enumerate(ROBOT0_H1_INDICES):
            v = joints_mj[h1_idx]
            hw = -v if h1_idx < 4 else v
            pose0[xapi_idx] = hw * 1000.0 if h1_idx == 0 else np.degrees(hw)
        for xapi_idx, h1_idx in enumerate(ROBOT1_H1_INDICES):
            v = joints_mj[h1_idx]
            hw = -v if h1_idx < 4 else v
            pose1[xapi_idx] = np.degrees(hw)

        g0 = float(gripper_pos[0]) if gripper_pos is not None else 0.0
        g1 = float(gripper_pos[1]) if gripper_pos is not None else 0.0
        try:
            self._robot0.schedule_waypoint(pose=pose0, gripper_pos=g0, target_time=target_time)
            self._robot1.schedule_waypoint(pose=pose1, gripper_pos=g1, target_time=target_time)
        except Full:
            # The InterpController input queue overflowed with stale commands
            # (consumer loop running slower than sender rate).  Clear both
            # queues to discard the backlog and immediately re-post the current
            # command so the robot resumes responding without waiting for the
            # full 256-entry backlog to drain.
            print(
                "[H1UpperBodyController] WARNING: InterpController input queue full — "
                "clearing stale backlog and retrying.",
                file=sys.stderr,
            )
            self._robot0.input_queue.clear()
            self._robot1.input_queue.clear()
            self._robot0.schedule_waypoint(pose=pose0, gripper_pos=g0, target_time=target_time)
            self._robot1.schedule_waypoint(pose=pose1, gripper_pos=g1, target_time=target_time)

    def step(self) -> None:
        if self._use_mock:
            self._robot0.step()
            self._robot1.step()
