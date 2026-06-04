from __future__ import annotations

import logging
import os
import sys
import threading
import time
from typing import Any

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)-8s %(name)s: %(message)s",
    datefmt="%H:%M:%S",
)

import numpy as np

from topstar_h1.h1_bridge import H1Bridge


class H1MujocoBackend:
    def __init__(
        self,
        sim_path: str,
        upper_body_config: dict | None = None,
        use_mock_upper_body: bool = True,
        frequency: int = 50,
    ) -> None:
        self._sim_path = os.path.expanduser(sim_path)
        self._upper_body_config = upper_body_config
        self._use_mock_upper_body = use_mock_upper_body
        self._frequency = frequency
        self._running = False
        self._sim_thread: threading.Thread | None = None

        if self._sim_path not in sys.path:
            sys.path.insert(0, self._sim_path)

        import mujoco
        import config

        self._mujoco = mujoco
        self._config = config

        robot_scene = config.ROBOT_SCENE
        if not os.path.isabs(robot_scene):
            robot_scene = os.path.normpath(os.path.join(self._sim_path, robot_scene))
        self._mj_model = mujoco.MjModel.from_xml_path(robot_scene)
        self._mj_data = mujoco.MjData(self._mj_model)
        self._lock = threading.RLock()
        self._bridge = H1Bridge(
            self._mj_model,
            self._mj_data,
            upper_body_config=upper_body_config or config.H1_UPPER_BODY_CONFIG,
            use_mock_upper_body=use_mock_upper_body,
            lock=self._lock,
            frequency=frequency,
        )

    def start(self) -> None:
        self._bridge.start()
        self._running = True

        def _sim_loop() -> None:
            dt = self._mj_model.opt.timestep
            while self._running:
                t0 = time.perf_counter()
                with self._lock:
                    self._bridge.step()
                    self._mujoco.mj_step(self._mj_model, self._mj_data)
                elapsed = time.perf_counter() - t0
                if dt - elapsed > 0:
                    time.sleep(dt - elapsed)

        self._sim_thread = threading.Thread(target=_sim_loop, daemon=True)
        self._sim_thread.start()

    def stop(self) -> None:
        self._running = False
        self._bridge.stop()

    def get_state(self) -> dict[str, Any]:
        state = self._bridge.get_state()
        return state

    def set_upper_body_joints(self, joints_hw: np.ndarray) -> None:
        self._bridge.set_upper_body_joints(joints_hw)

    def set_upper_body_joints_timed(self, joints_hw: np.ndarray, duration: float) -> None:
        self._bridge.set_upper_body_joints_timed(joints_hw, duration)

    def set_base_velocity(self, vx: float, vy: float, omega: float) -> None:
        self._bridge.set_base_velocity(vx=vx, vy=vy, omega=omega)

    def set_gripper(self, gripper: np.ndarray) -> None:
        self._bridge.set_gripper(gripper)