from __future__ import annotations

import json
import threading
import time
from typing import Any

import numpy as np
import zmq

from topstar_h1.joint_defs import H1_NUM_JOINTS, hw_to_mj


class H1IsaacBackend:
    def __init__(
        self,
        state_endpoint: str = "tcp://127.0.0.1:15555",
        command_endpoint: str = "tcp://127.0.0.1:15556",
    ) -> None:
        self._context = zmq.Context()
        self._state_sock = self._context.socket(zmq.PULL)
        self._state_sock.setsockopt(zmq.RCVHWM, 2)
        self._state_sock.connect(state_endpoint)
        self._cmd_sock = self._context.socket(zmq.PUSH)
        self._cmd_sock.setsockopt(zmq.SNDHWM, 2)
        self._cmd_sock.connect(command_endpoint)
        self._state_lock = threading.Lock()
        self._latest_state: dict[str, Any] | None = None
        self._running = False
        self._recv_thread: threading.Thread | None = None

    def start(self) -> None:
        self._running = True

        def _recv_loop() -> None:
            while self._running:
                try:
                    raw = self._state_sock.recv(flags=0)
                    state = json.loads(raw)
                    with self._state_lock:
                        self._latest_state = state
                except Exception:
                    time.sleep(0.01)

        self._recv_thread = threading.Thread(target=_recv_loop, daemon=True)
        self._recv_thread.start()

    def stop(self) -> None:
        self._running = False

    def get_state(self) -> dict[str, Any]:
        with self._state_lock:
            state = dict(self._latest_state or {})
        q_hw = np.asarray(state.get("q", [0.0] * H1_NUM_JOINTS), dtype=np.float64)
        return {
            "joints_hw": q_hw,
            "joints_mj": hw_to_mj(q_hw),
            "gripper": np.zeros(2, dtype=np.float64),
            "imu": {
                "quat": state.get("quat", [1.0, 0.0, 0.0, 0.0]),
                "gyro": state.get("gyro", [0.0, 0.0, 0.0]),
                "acc": state.get("acc", [0.0, 0.0, 9.81]),
            },
        }

    def set_upper_body_joints(self, joints_hw: np.ndarray) -> None:
        payload = json.dumps({
            "type": "lowcmd",
            "q": np.asarray(joints_hw, dtype=np.float64).tolist(),
            "mode": [1] * H1_NUM_JOINTS,
        }).encode()
        try:
            self._cmd_sock.send(payload, zmq.NOBLOCK)
        except zmq.Again:
            pass

    def set_base_velocity(self, vx: float, vy: float, omega: float) -> None:
        payload = json.dumps({
            "type": "basecmd",
            "vx": float(vx),
            "vy": float(vy),
            "omega": float(omega),
        }).encode()
        try:
            self._cmd_sock.send(payload, zmq.NOBLOCK)
        except zmq.Again:
            pass