"""Unified H1 bridge composed from local base and upper-body controllers."""
from __future__ import annotations

import threading
import time
from typing import Any

import numpy as np

from topstar_h1.h1_base import H1BaseController
from topstar_h1.h1_upper_body import H1UpperBodyController
from topstar_h1.joint_defs import H1_NUM_JOINTS


class H1Bridge:
    def __init__(
        self,
        mj_model,
        mj_data,
        upper_body_config: dict | None = None,
        use_mock_upper_body: bool = True,
        lock: threading.Lock | None = None,
        frequency: int = 50,
    ) -> None:
        self._lock = lock or threading.Lock()
        self._frequency = frequency
        self.base_ctrl = H1BaseController(mj_model, mj_data)
        self.upper_body = H1UpperBodyController(
            config=upper_body_config,
            use_mock=use_mock_upper_body,
            mj_model=mj_model,
            mj_data=mj_data,
            lock=self._lock,
            frequency=frequency,
        )
        self._cmd_joints = np.zeros(H1_NUM_JOINTS, dtype=np.float64)
        self._cmd_gripper = np.zeros(2, dtype=np.float64)
        self._cmd_joints_lock = threading.Lock()
        self._cmd_joints_updated = False
        self._timed_move_end_time: float = 0.0

    def start(self) -> None:
        self.upper_body.start()

    def stop(self) -> None:
        self.upper_body.stop()

    @property
    def is_ready(self) -> bool:
        return self.upper_body.is_ready

    def step(self) -> None:
        with self._cmd_joints_lock:
            if self._cmd_joints_updated:
                self.upper_body.set_joints(
                    self._cmd_joints,
                    target_time=time.monotonic() + 1.0 / self._frequency,
                    gripper_pos=self._cmd_gripper,
                )
                self._cmd_joints_updated = False
        self.upper_body.step()
        self.base_ctrl.step()

    def get_state(self) -> dict[str, Any]:
        ub_state = self.upper_body.get_state()
        return {
            "joints_hw": ub_state["joints_hw"],
            "joints_mj": ub_state["joints_mj"],
            "gripper": ub_state["gripper"],
        }

    def set_upper_body_joints(self, joints_hw: np.ndarray) -> None:
        with self._cmd_joints_lock:
            if time.monotonic() < self._timed_move_end_time:
                return  # timed move in progress; ignore regular LowCmd joints
            self._cmd_joints = np.asarray(joints_hw, dtype=np.float64).copy()
            self._cmd_joints_updated = True

    def set_gripper(self, gripper: np.ndarray) -> None:
        with self._cmd_joints_lock:
            self._cmd_gripper = np.asarray(gripper, dtype=np.float64).copy()

    def set_upper_body_joints_timed(self, joints_hw: np.ndarray, duration: float) -> None:
        target_time = time.monotonic() + max(duration, 1.0 / self._frequency)
        with self._cmd_joints_lock:
            self._timed_move_end_time = target_time
            self._cmd_joints_updated = False  # discard any LowCmd that beat us here
        self.upper_body.set_joints(np.asarray(joints_hw, dtype=np.float64), target_time=target_time)

    def set_base_velocity(self, vx: float, vy: float, omega: float) -> None:
        self.base_ctrl.set_command(vx=float(vx), vy=float(vy), omega=float(omega))
