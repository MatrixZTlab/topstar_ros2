from __future__ import annotations

import threading
import time
from typing import Any
# Note: target_time values are passed to InterpController (a subprocess) via
# SharedMemoryQueue.  The subprocess evaluates joint_interp(time.monotonic()),
# so all target_time values must be in the time.monotonic() domain.  Using
# time.perf_counter() here would require a per-command clock-domain conversion
# inside the subprocess; using time.monotonic() directly removes that step.

import numpy as np

from topstar_h1.h1_upper_body import H1UpperBodyController


class H1XapiBackend:
    """Hardware upper-body backend using the vendor xapi transport.

    The current H1 ROS2 stack publishes only the 18 upper-body joints in LowState,
    so this backend leaves base velocity commands as a no-op.
    """

    def __init__(self, upper_body_config: dict | None = None, frequency: int = 50) -> None:
        self._frequency = frequency
        self._upper_body = H1UpperBodyController(
            config=upper_body_config,
            use_mock=False,
            frequency=frequency,
        )
        self._cmd_gripper = np.zeros(2, dtype=np.float64)
        self._gripper_lock = threading.Lock()

    def start(self) -> None:
        self._upper_body.start()

    def stop(self) -> None:
        self._upper_body.stop()

    def get_state(self) -> dict[str, Any]:
        return self._upper_body.get_state()

    def set_upper_body_joints(self, joints_hw: np.ndarray) -> None:
        # Use 5-cycle lookahead so target_time survives IPC latency and is still
        # in the future when InterpController.run() processes the waypoint.
        # (A 1-cycle lookahead arrives at curr_time and gets silently dropped.)
        with self._gripper_lock:
            gripper = self._cmd_gripper.copy()
        self._upper_body.set_joints(
            np.asarray(joints_hw, dtype=np.float64),
            target_time=time.monotonic() + 5.0 / self._frequency,
            gripper_pos=gripper,
        )

    def set_upper_body_joints_timed(self, joints_hw: np.ndarray, duration: float) -> None:
        with self._gripper_lock:
            gripper = self._cmd_gripper.copy()
        self._upper_body.set_joints(
            np.asarray(joints_hw, dtype=np.float64),
            target_time=time.monotonic() + max(duration, 1.0 / self._frequency),
            gripper_pos=gripper,
        )

    def set_gripper(self, gripper: np.ndarray) -> None:
        with self._gripper_lock:
            self._cmd_gripper = np.asarray(gripper, dtype=np.float64).copy()

    def set_base_velocity(self, vx: float, vy: float, omega: float) -> None:
        _ = (vx, vy, omega)
