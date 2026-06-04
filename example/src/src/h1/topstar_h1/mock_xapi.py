"""MuJoCo-backed drop-in replacement for the hardware interpolation controller."""
from __future__ import annotations

import threading
import time
from typing import Sequence

import mujoco
import numpy as np

from topstar_h1.joint_defs import H1_JOINT_LIMITS_MJ


class MockInterpController:
    """Synchronous MuJoCo-backed replacement for InterpController."""

    def __init__(
        self,
        mj_model: mujoco.MjModel,
        mj_data: mujoco.MjData,
        joint_names: Sequence[str],
        h1_indices: Sequence[int],
        actuator_names: Sequence[str] | None = None,
        joints_init: np.ndarray | None = None,
        joints_init_duration: float = 2.0,
        frequency: int = 50,
        lock: threading.Lock | None = None,
        gripper_enabled: bool = False,
        **_kwargs,
    ) -> None:
        self._mj_model = mj_model
        self._mj_data = mj_data
        self._lock = lock or threading.Lock()
        self._dt = 1.0 / frequency
        self._ready = False

        if len(joint_names) != 9:
            raise ValueError(
                f"MockInterpController expects 9 joints, got {len(joint_names)}"
            )
        if len(h1_indices) != 9:
            raise ValueError(
                f"MockInterpController expects 9 H1 indices, got {len(h1_indices)}"
            )
        self._joint_names = list(joint_names)
        self._h1_indices = list(h1_indices)
        self._joint_limits_mj = H1_JOINT_LIMITS_MJ[self._h1_indices].copy()

        self._joint_ids: list[int] = []
        for name in self._joint_names:
            joint_id = mujoco.mj_name2id(
                mj_model, mujoco._enums.mjtObj.mjOBJ_JOINT, name
            )
            if joint_id < 0:
                raise ValueError(f"Joint '{name}' not found in MuJoCo model")
            self._joint_ids.append(joint_id)

        if actuator_names is None:
            actuator_names = [f"{name}_act" for name in joint_names]
        self._actuator_ids: list[int] = []
        for index, name in enumerate(actuator_names):
            actuator_id = mujoco.mj_name2id(
                mj_model, mujoco._enums.mjtObj.mjOBJ_ACTUATOR, name
            )
            if actuator_id < 0:
                raise ValueError(f"Actuator '{name}' not found in MuJoCo model")
            self._actuator_ids.append(actuator_id)

        # Push hardware joint limits into the runtime model so MuJoCo enforces
        # the same limits as the real hardware.  The XML was generated with axes
        # aligned to the hardware convention, so no sign conversion is needed.
        for index, (joint_id, actuator_id) in enumerate(
            zip(self._joint_ids, self._actuator_ids)
        ):
            self._mj_model.jnt_range[joint_id, :] = self._joint_limits_mj[index]
            self._mj_model.actuator_ctrlrange[actuator_id, :] = self._joint_limits_mj[index]

        self._gripper_enabled = gripper_enabled
        self._current_target = self._read_joint_positions()
        self._waypoint_start = self._current_target.copy()
        self._waypoint_target = self._current_target.copy()
        self._waypoint_start_time = 0.0
        self._waypoint_end_time = 0.0
        self._gripper_target = 0.0
        self._joints_init = (
            self._xapi_to_mj(np.array(joints_init, dtype=np.float64))
            if joints_init is not None else None
        )
        self._joints_init_duration = max(joints_init_duration, 0.01)

    @property
    def is_ready(self) -> bool:
        return self._ready

    def start(self, wait: bool = True) -> None:
        if self._joints_init is not None:
            now = time.perf_counter()
            self._waypoint_start = self._read_joint_positions()
            self._waypoint_target = self._joints_init.copy()
            self._waypoint_start_time = now
            self._waypoint_end_time = now + self._joints_init_duration
        self._ready = True
        if wait:
            self.start_wait()

    def start_wait(self) -> None:
        return None

    def stop(self) -> None:
        self._ready = False

    def get_state(self) -> dict:
        q_mj = self._read_joint_positions()
        qd_mj = self._read_joint_velocities()
        q = self._mj_to_xapi(q_mj)
        qd = self._mj_to_xapi_velocity(qd_mj)
        tcp_pose = np.zeros(9, dtype=np.float64)
        tcp_pose[7:9] = q[7:9]
        return {
            "ActualTCPPose": tcp_pose,
            "ActualQ": q,
            "ActualQd": qd,
            "TargetTCPPose": tcp_pose.copy(),
            "TargetQ": self._mj_to_xapi(self._current_target),
            "TargetGripper": float(self._gripper_target),
            "gripper_position": float(self._gripper_target),
            "DI": 0,
        }

    def schedule_waypoint(
        self,
        pose: np.ndarray,
        gripper_pos: float,
        target_time: float,
    ) -> None:
        pose = np.asarray(pose, dtype=np.float64)
        if pose.shape != (9,):
            raise ValueError(f"Expected pose shape (9,), got {pose.shape}")
        now = time.perf_counter()
        self._waypoint_start = self._read_joint_positions()
        self._waypoint_target = self._xapi_to_mj(pose)
        self._waypoint_start_time = now
        self._waypoint_end_time = max(target_time, now + self._dt)
        self._gripper_target = float(gripper_pos)

    def step(self) -> None:
        now = time.perf_counter()
        duration = self._waypoint_end_time - self._waypoint_start_time
        if duration <= 0:
            alpha = 1.0
        else:
            alpha = np.clip((now - self._waypoint_start_time) / duration, 0.0, 1.0)
        self._current_target = (
            (1.0 - alpha) * self._waypoint_start + alpha * self._waypoint_target
        )
        self._write_actuator_commands(self._current_target)

    def _read_joint_positions(self) -> np.ndarray:
        q = np.zeros(9, dtype=np.float64)
        for index, joint_id in enumerate(self._joint_ids):
            q[index] = self._mj_data.qpos[self._mj_model.jnt_qposadr[joint_id]]
        return q

    def _read_joint_velocities(self) -> np.ndarray:
        qd = np.zeros(9, dtype=np.float64)
        for index, joint_id in enumerate(self._joint_ids):
            qd[index] = self._mj_data.qvel[self._mj_model.jnt_dofadr[joint_id]]
        return qd

    def _xapi_to_mj(self, q_xapi: np.ndarray) -> np.ndarray:
        q_mj = np.zeros(9, dtype=np.float64)
        for index, h1_idx in enumerate(self._h1_indices):
            value = float(q_xapi[index])
            if h1_idx == 0:
                unit = value / 1000.0
            else:
                unit = np.radians(value)
            # Torso/head (h1_idx < 4): hardware and MuJoCo have opposite sign
            # conventions.  Arm joints (h1_idx >= 4): same sign convention after
            # the axis corrections applied in prepare_h1_model.py.
            q_mj[index] = -unit if h1_idx < 4 else unit
        return q_mj

    def _mj_to_xapi(self, q_mj: np.ndarray) -> np.ndarray:
        q_xapi = np.zeros(9, dtype=np.float64)
        for index, h1_idx in enumerate(self._h1_indices):
            value = -float(q_mj[index]) if h1_idx < 4 else float(q_mj[index])
            q_xapi[index] = value * 1000.0 if h1_idx == 0 else np.degrees(value)
        return q_xapi

    def _mj_to_xapi_velocity(self, qd_mj: np.ndarray) -> np.ndarray:
        qd_xapi = np.zeros(9, dtype=np.float64)
        for index, h1_idx in enumerate(self._h1_indices):
            value = -float(qd_mj[index]) if h1_idx < 4 else float(qd_mj[index])
            qd_xapi[index] = value * 1000.0 if h1_idx == 0 else np.degrees(value)
        return qd_xapi

    def _write_actuator_commands(self, targets: np.ndarray) -> None:
        with self._lock:
            for index, actuator_id in enumerate(self._actuator_ids):
                low = float(self._joint_limits_mj[index, 0])
                high = float(self._joint_limits_mj[index, 1])
                self._mj_data.ctrl[actuator_id] = float(np.clip(targets[index], low, high))
