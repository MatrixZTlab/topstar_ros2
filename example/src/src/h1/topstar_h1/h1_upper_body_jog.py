#!/usr/bin/env python3
"""H1 upper-body joint jog panel (PySide6).

Run after sourcing all workspaces and launching h1_ros2_node:
    ros2 run topstar_ros2_example h1_upper_body_jog
"""
from __future__ import annotations

import os
import sys
import json
import time
import threading
import math
from dataclasses import dataclass, field
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from topstar_hg.msg import LowCmd, LowState, GripperCmd, GripperState
from topstar_api.msg import Request as ArmRequest

from topstar_h1.joint_defs import H1JointIndex
from topstar_h1.h1_drive_example import make_lowcmd

ARM_API_ID_MOVE_JOINTS_TIMED = 1001  # mirrors h1_ros2_node.ARM_API_ID_MOVE_JOINTS_TIMED


# h1_ros2_node publishes and receives hw rad/m on /lowstate and /lowcmd.
# H1UpperBodyController handles the deg/mm ↔ rad/m conversion at the xapi boundary.
# No unit conversion is needed here.

def _wire_to_hw(_slot: int, v: float) -> float:
    return v


def _hw_to_wire(_slot: int, v: float) -> float:
    return v

from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton, QDoubleSpinBox, QTabWidget, QFrame,
    QScrollArea, QSlider, QSizePolicy, QSpacerItem,
    QDialog, QDialogButtonBox, QLineEdit, QFileDialog, QMessageBox,
)
from PySide6.QtCore import Qt, Signal, QObject, QTimer, Slot
from PySide6.QtGui import QFont


# ── Joint specifications ──────────────────────────────────────────────────────

@dataclass
class JointSpec:
    display_name: str
    index: int          # H1JointIndex value → motor_cmd/motor_state slot
    low: float
    high: float
    unit: str = "rad"


# Hardware-native limits, taken directly from the MuJoCo model joint ranges.
# Torso/head: sign-flipped from model (hw+ = physical extension).
#   TORSO_LIFT:  mj[-0.45, 0]   → hw[0, 0.45] m
#   TORSO_PITCH: mj[-1.658, 0]  → hw[0, 1.658] rad
#   HEAD_PITCH:  mj[-0.489, 0.559] → hw[-0.559, 0.489] rad
# Arm joints: hw == mj (no sign flip) — limits from Robot_*_Hand_*_Joint ranges.
# Note the left/right elbow asymmetry: R_Elbow[-1.7977, 0.4363], L_Elbow[-0.4363, 1.7977].
JOINT_SPECS: list[JointSpec] = [
    JointSpec("Torso Lift",      H1JointIndex.TORSO_LIFT,          -0.0100,      0.4500, "m"),
    JointSpec("Torso Pitch",     H1JointIndex.TORSO_PITCH,          0.0000,      1.65806279, "rad"),
    JointSpec("Head Yaw",        H1JointIndex.HEAD_YAW,            -1.5708,      1.5708, "rad"),
    JointSpec("Head Pitch",      H1JointIndex.HEAD_PITCH,          -0.6981317,   0.48869219, "rad"),

    JointSpec("R Shoulder Base", H1JointIndex.RIGHT_SHOULDER_BASE, -2.61799388,  2.61799388, "rad"),    
    JointSpec("R Shoulder",      H1JointIndex.RIGHT_SHOULDER,      -1.57079633,  0.43633231, "rad"),
    JointSpec("R Elbow Yaw",     H1JointIndex.RIGHT_ELBOW_YAW,     -2.61799388,  2.61799388, "rad"),
    JointSpec("R Elbow",         H1JointIndex.RIGHT_ELBOW,         -1.79768913,  0.43633231, "rad"),
    JointSpec("R Wrist Yaw",     H1JointIndex.RIGHT_WRIST_YAW,     -2.87979327,  2.87979327, "rad"),
    JointSpec("R Wrist Pitch",   H1JointIndex.RIGHT_WRIST_PITCH,   -1.53588974,  0.43633231, "rad"),
    JointSpec("R Wrist Roll",    H1JointIndex.RIGHT_WRIST_ROLL,    -2.96705973,  2.96705973, "rad"),

    JointSpec("L Shoulder Base", H1JointIndex.LEFT_SHOULDER_BASE,  -2.61799388,  2.61799388, "rad"),
    JointSpec("L Shoulder",      H1JointIndex.LEFT_SHOULDER,       -1.57079633,  0.43633231, "rad"),
    JointSpec("L Elbow Yaw",     H1JointIndex.LEFT_ELBOW_YAW,      -2.61799388,  2.61799388, "rad"),
    JointSpec("L Elbow",         H1JointIndex.LEFT_ELBOW,          -1.79768913,  0.43633231, "rad"),
    JointSpec("L Wrist Yaw",     H1JointIndex.LEFT_WRIST_YAW,      -2.87979327,  2.87979327, "rad"),
    JointSpec("L Wrist Pitch",   H1JointIndex.LEFT_WRIST_PITCH,    -1.53588974,  0.43633231, "rad"),
    JointSpec("L Wrist Roll",    H1JointIndex.LEFT_WRIST_ROLL,     -2.96705973,  2.96705973, "rad"),
]

@dataclass
class JointPoint:
    name: str
    values: list[float]  # 18 elements indexed by motor/joint index (same layout as bridge targets)
    gripper_pos: list[float] = field(default_factory=lambda: [0.0, 0.0])  # [robot0, robot1]


TAB_GROUPS: list[tuple[str, list[int]]] = [
    ("Torso & Head", list(range(0, 4))),
    ("Right Arm",    list(range(4, 11))),
    ("Left Arm",     list(range(11, 18))),
]

_SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)
_CMD_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)


# ── ROS2 bridge (lives in GUI thread; node spins in daemon thread) ─────────────

class RosBridge(QObject):
    """Thread-safe link between the GUI and the ROS2 spin thread.

    Signals are emitted from the ROS2 thread; Qt's queued-connection mechanism
    marshals them safely to the GUI thread.
    """
    state_updated = Signal(list)    # list[float] of 18 motor positions
    gripper_updated = Signal(list)  # list[float] of 2 gripper positions [robot0, robot1]

    def __init__(self) -> None:
        super().__init__()
        self._lock = threading.Lock()
        self._targets: list[float] = [0.0] * 18
        self._gripper_pos: list[float] = [0.0, 0.0]  # [robot0 (right), robot1 (left)]
        self._enabled: bool = False
        self._state_received: bool = False
        self._base_vx: float = 0.0
        self._base_vy: float = 0.0
        self._base_wz: float = 0.0
        self._last_state_t: float = 0.0
        self._joint_hold_until: float = 0.0   # suppress LowCmd joints during timed move
        self._node: Optional[_JogNode] = None

    def start(self) -> None:
        rclpy.init()
        self._node = _JogNode(self)
        threading.Thread(target=rclpy.spin, args=(self._node,), daemon=True).start()

    def stop(self) -> None:
        if self._node:
            try:
                self._node.destroy_node()
            except Exception:
                pass
        try:
            rclpy.shutdown()
        except Exception:
            pass

    # ── Target management (GUI thread → ROS2 thread) ──────────────────────

    def set_target(self, idx: int, value: float) -> None:
        with self._lock:
            self._targets[idx] = value

    def set_targets(self, values: list[float]) -> None:
        with self._lock:
            self._targets = list(values)

    def get_targets(self) -> list[float]:
        with self._lock:
            return list(self._targets)

    def set_gripper(self, idx: int, value: float) -> None:
        with self._lock:
            self._gripper_pos[idx] = value

    def get_gripper_pos(self) -> list[float]:
        with self._lock:
            return list(self._gripper_pos)

    def set_enabled(self, enabled: bool) -> None:
        with self._lock:
            self._enabled = enabled

    def is_enabled(self) -> bool:
        with self._lock:
            return self._enabled

    def set_base_cmd(self, vx: float, vy: float, wz: float) -> None:
        with self._lock:
            self._base_vx = float(vx)
            self._base_vy = float(vy)
            self._base_wz = float(wz)

    def get_base_cmd(self) -> tuple[float, float, float]:
        with self._lock:
            return self._base_vx, self._base_vy, self._base_wz

    def last_state_age(self) -> float:
        """Seconds since last LowState was received; inf if never."""
        with self._lock:
            if self._last_state_t == 0.0:
                return math.inf
            return time.monotonic() - self._last_state_t

    def is_joint_hold_active(self) -> bool:
        with self._lock:
            return time.monotonic() < self._joint_hold_until

    def move_joints_timed(self, joints: list[float], duration: float) -> None:
        """Suppress LowCmd joints and publish a timed-move arm API request."""
        with self._lock:
            self._joint_hold_until = time.monotonic() + duration
        if self._node is not None:
            self._node.publish_arm_move_timed(joints, duration)

    # ── Called from ROS2 thread ────────────────────────────────────────────

    def _record_state(self, positions: list[float]) -> None:
        with self._lock:
            self._last_state_t = time.monotonic()
            self._state_received = True
        self.state_updated.emit(positions)  # Qt queues this to GUI thread

    def _record_gripper_state(self, gripper: list[float]) -> None:
        self.gripper_updated.emit(gripper)  # Qt queues this to GUI thread

    def is_state_received(self) -> bool:
        with self._lock:
            return self._state_received


class _JogNode(Node):
    _HZ = 50
    _LIN_ACCEL = 0.8   # m/s^2
    _LIN_DECEL = 1.2   # m/s^2
    _ANG_ACCEL = 1.5   # rad/s^2
    _ANG_DECEL = 2.0   # rad/s^2

    def __init__(self, bridge: RosBridge) -> None:
        super().__init__('h1_upper_body_jog')
        self._bridge = bridge
        self._cmd_vx = 0.0
        self._cmd_vy = 0.0
        self._cmd_wz = 0.0
        self._req_counter: int = 0
        self._pub = self.create_publisher(LowCmd, '/lowcmd', _CMD_QOS)
        self._base_pub = self.create_publisher(Twist, '/base_cmd', _CMD_QOS)
        self._arm_req_pub = self.create_publisher(ArmRequest, '/api/arm/request', _CMD_QOS)
        self._gripper_right_pub = self.create_publisher(GripperCmd, '/hand/right/cmd', _CMD_QOS)
        self._gripper_left_pub = self.create_publisher(GripperCmd, '/hand/left/cmd', _CMD_QOS)
        self._sub = self.create_subscription(
            LowState, '/lowstate', self._on_state, _SENSOR_QOS)
        self._gripper_right_sub = self.create_subscription(
            GripperState, '/hand/right/state',
            lambda m: self._on_gripper_state(0, m), _SENSOR_QOS)
        self._gripper_left_sub = self.create_subscription(
            GripperState, '/hand/left/state',
            lambda m: self._on_gripper_state(1, m), _SENSOR_QOS)
        self._timer = self.create_timer(1.0 / self._HZ, self._publish)
        self.get_logger().info('h1_upper_body_jog ready.')

    def publish_arm_move_timed(self, joints: list[float], duration: float) -> None:
        self._req_counter += 1
        msg = ArmRequest()
        msg.header.identity.api_id = ARM_API_ID_MOVE_JOINTS_TIMED
        msg.header.identity.id = self._req_counter
        msg.parameter = json.dumps({"joints": joints, "duration": duration})
        self._arm_req_pub.publish(msg)

    @staticmethod
    def _step_toward(current: float, target: float, accel_step: float, decel_step: float) -> float:
        if current == target:
            return current

        same_direction = (current == 0.0) or (target == 0.0) or ((current > 0.0) == (target > 0.0))
        speeding_up = same_direction and (abs(target) > abs(current))
        step = accel_step if speeding_up else decel_step

        delta = target - current
        if delta > step:
            return current + step
        if delta < -step:
            return current - step
        return target

    def _on_state(self, msg: LowState) -> None:
        positions = [_wire_to_hw(i, msg.motor_state[i].q) for i in range(18)]
        self._bridge._record_state(positions)

    def _on_gripper_state(self, arm_idx: int, msg: GripperState) -> None:
        gripper = list(self._bridge.get_gripper_pos())
        gripper[arm_idx] = float(msg.position)
        self._bridge._record_gripper_state(gripper)

    def _publish(self) -> None:
        target_vx, target_vy, target_wz = self._bridge.get_base_cmd()
        lin_accel_step = self._LIN_ACCEL / self._HZ
        lin_decel_step = self._LIN_DECEL / self._HZ
        ang_accel_step = self._ANG_ACCEL / self._HZ
        ang_decel_step = self._ANG_DECEL / self._HZ

        self._cmd_vx = self._step_toward(self._cmd_vx, target_vx, lin_accel_step, lin_decel_step)
        self._cmd_vy = self._step_toward(self._cmd_vy, target_vy, lin_accel_step, lin_decel_step)
        self._cmd_wz = self._step_toward(self._cmd_wz, target_wz, ang_accel_step, ang_decel_step)

        twist = Twist()
        twist.linear.x = self._cmd_vx
        twist.linear.y = self._cmd_vy
        twist.angular.z = self._cmd_wz
        self._base_pub.publish(twist)

        gripper = self._bridge.get_gripper_pos()
        for arm_idx, pub in enumerate([self._gripper_right_pub, self._gripper_left_pub]):
            gc = GripperCmd()
            gc.position = float(gripper[arm_idx])
            gc.mode = 1
            pub.publish(gc)

        if (self._bridge.is_state_received()
                and self._bridge.is_enabled()
                and not self._bridge.is_joint_hold_active()):
            wire = [_hw_to_wire(i, v) for i, v in enumerate(self._bridge.get_targets())]
            self._pub.publish(make_lowcmd(wire))


# ── Per-joint row widget ──────────────────────────────────────────────────────

class JointRow(QWidget):
    """One row: name | current | unit | slider | spinbox | − | + """

    target_changed = Signal(int, float)  # joint index, new target value

    _SLIDER_RES = 2000  # integer steps for full slider range

    def __init__(self, spec: JointSpec, step_getter, parent=None) -> None:
        super().__init__(parent)
        self._spec = spec
        self._step_getter = step_getter  # callable → float
        self._busy = False

        lo = QHBoxLayout(self)
        lo.setContentsMargins(6, 3, 6, 3)
        lo.setSpacing(8)

        name = QLabel(spec.display_name)
        name.setFixedWidth(130)
        name.setFont(QFont("Monospace", 9))

        self._cur = QLabel("  ---   ")
        self._cur.setFixedWidth(72)
        self._cur.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self._cur.setFont(QFont("Monospace", 9))
        self._cur.setStyleSheet("color: #1565c0;")

        unit = QLabel(spec.unit)
        unit.setFixedWidth(28)

        self._slider = QSlider(Qt.Horizontal)
        self._slider.setMinimum(0)
        self._slider.setMaximum(self._SLIDER_RES)
        self._slider.setValue(self._to_slider(0.0))
        self._slider.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)

        self._spin = QDoubleSpinBox()
        self._spin.setRange(spec.low, spec.high)
        self._spin.setSingleStep(0.05)
        self._spin.setDecimals(3)
        self._spin.setValue(0.0)
        self._spin.setFixedWidth(90)

        btn_m = QPushButton("−")
        btn_p = QPushButton("+")
        for b in (btn_m, btn_p):
            b.setFixedWidth(28)
            b.setFixedHeight(24)

        for w in (name, self._cur, unit, self._slider, self._spin, btn_m, btn_p):
            lo.addWidget(w)

        self._slider.valueChanged.connect(self._slider_moved)
        self._spin.valueChanged.connect(self._spin_moved)
        btn_m.clicked.connect(lambda: self._jog(-self._step_getter()))
        btn_p.clicked.connect(lambda: self._jog(+self._step_getter()))

    # ── Conversions ────────────────────────────────────────────────────────

    def _to_slider(self, v: float) -> int:
        frac = (v - self._spec.low) / (self._spec.high - self._spec.low)
        return int(max(0.0, min(1.0, frac)) * self._SLIDER_RES)

    def _from_slider(self, s: int) -> float:
        return self._spec.low + (s / self._SLIDER_RES) * (self._spec.high - self._spec.low)

    # ── Internal sync ──────────────────────────────────────────────────────

    def _slider_moved(self, s: int) -> None:
        if self._busy:
            return
        v = self._from_slider(s)
        self._busy = True
        self._spin.setValue(v)
        self._busy = False
        self.target_changed.emit(self._spec.index, v)

    def _spin_moved(self, v: float) -> None:
        if self._busy:
            return
        self._busy = True
        self._slider.setValue(self._to_slider(v))
        self._busy = False
        self.target_changed.emit(self._spec.index, v)

    def _jog(self, delta: float) -> None:
        clamped = max(self._spec.low, min(self._spec.high, self._spin.value() + delta))
        self._spin.setValue(clamped)

    # ── Public API ─────────────────────────────────────────────────────────

    def update_current(self, v: float) -> None:
        self._cur.setText(f"{v:+.3f}")

    def set_target(self, v: float) -> None:
        self._spin.setValue(max(self._spec.low, min(self._spec.high, v)))

    def get_target(self) -> float:
        return self._spin.value()


# ── Point list ────────────────────────────────────────────────────────────────

class PointEditDialog(QDialog):
    """Dialog for renaming a point and fine-tuning its joint values."""

    def __init__(self, point: JointPoint, gripper_cfg: dict | None = None, parent=None) -> None:
        super().__init__(parent)
        self.setWindowTitle("Edit Point")
        self.setMinimumWidth(540)
        self._gripper_cfg = gripper_cfg or {}

        layout = QVBoxLayout(self)
        layout.setSpacing(8)

        name_row = QHBoxLayout()
        name_row.addWidget(QLabel("Name:"))
        self._name_edit = QLineEdit(point.name)
        name_row.addWidget(self._name_edit)
        layout.addLayout(name_row)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFixedHeight(380)
        scroll.setFrameShape(QFrame.StyledPanel)
        container = QWidget()
        vbox = QVBoxLayout(container)
        vbox.setSpacing(2)

        self._spins: list[QDoubleSpinBox] = []
        for spec in JOINT_SPECS:
            row = QHBoxLayout()
            lbl = QLabel(f"{spec.display_name} ({spec.unit})")
            lbl.setFixedWidth(190)
            spin = QDoubleSpinBox()
            spin.setRange(spec.low, spec.high)
            spin.setDecimals(3)
            spin.setSingleStep(0.01)
            spin.setValue(point.values[spec.index])
            spin.setFixedWidth(100)
            row.addWidget(lbl)
            row.addWidget(spin)
            row.addStretch()
            self._spins.append(spin)
            vbox.addLayout(row)

        self._gripper_spins: dict[str, QDoubleSpinBox] = {}
        for arm_idx, (key, label) in enumerate([("robot0", "Right Gripper (0=OFF, 1=ON)"),
                                                ("robot1", "Left Gripper (0=OFF, 1=ON)")]):
            if self._gripper_cfg.get(key, False):
                row = QHBoxLayout()
                lbl = QLabel(label)
                lbl.setFixedWidth(190)
                spin = QDoubleSpinBox()
                spin.setRange(0.0, 1.0)
                spin.setDecimals(1)
                spin.setSingleStep(1.0)
                spin.setValue(point.gripper_pos[arm_idx])
                spin.setFixedWidth(100)
                row.addWidget(lbl)
                row.addWidget(spin)
                row.addStretch()
                self._gripper_spins[key] = spin
                vbox.addLayout(row)

        vbox.addStretch()
        scroll.setWidget(container)
        layout.addWidget(scroll)

        buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)

    def result_point(self, original: JointPoint) -> JointPoint:
        values = list(original.values)
        for i, spec in enumerate(JOINT_SPECS):
            values[spec.index] = self._spins[i].value()
        gripper_pos = list(original.gripper_pos)
        for arm_idx, key in enumerate(["robot0", "robot1"]):
            if key in self._gripper_spins:
                gripper_pos[arm_idx] = self._gripper_spins[key].value()
        return JointPoint(
            name=self._name_edit.text().strip() or original.name,
            values=values,
            gripper_pos=gripper_pos,
        )


class PointItemWidget(QWidget):
    """One row in the point list: index | name | [Replay] [Edit] [Delete]"""

    replay_requested = Signal(int)
    edit_requested = Signal(int)
    delete_requested = Signal(int)

    def __init__(self, index: int, point: JointPoint, parent=None) -> None:
        super().__init__(parent)
        lo = QHBoxLayout(self)
        lo.setContentsMargins(6, 3, 6, 3)
        lo.setSpacing(8)

        num_lbl = QLabel(f"{index + 1}.")
        num_lbl.setFixedWidth(32)
        num_lbl.setAlignment(Qt.AlignRight | Qt.AlignVCenter)

        name_lbl = QLabel(point.name)
        name_lbl.setFont(QFont("Monospace", 9))

        gripper_lbl = QLabel(f"G:{point.gripper_pos[0]:.0f}/{point.gripper_pos[1]:.0f}")
        gripper_lbl.setFixedWidth(40)
        gripper_lbl.setFont(QFont("Monospace", 8))
        gripper_lbl.setStyleSheet("color: #777;")

        btn_replay = QPushButton("Replay")
        btn_edit = QPushButton("Edit")
        btn_del = QPushButton("Delete")
        for b in (btn_replay, btn_edit, btn_del):
            b.setFixedHeight(24)
            b.setFixedWidth(70)
        btn_replay.setStyleSheet("QPushButton{background:#a5d6a7;border-radius:3px;}")
        btn_del.setStyleSheet("QPushButton{background:#ef9a9a;border-radius:3px;}")

        lo.addWidget(num_lbl)
        lo.addWidget(name_lbl, stretch=1)
        lo.addWidget(gripper_lbl)
        lo.addWidget(btn_replay)
        lo.addWidget(btn_edit)
        lo.addWidget(btn_del)

        btn_replay.clicked.connect(lambda: self.replay_requested.emit(index))
        btn_edit.clicked.connect(lambda: self.edit_requested.emit(index))
        btn_del.clicked.connect(lambda: self.delete_requested.emit(index))


class PointListPanel(QWidget):
    """Tab for recording, editing, replaying, and deleting named joint poses."""

    def __init__(
        self,
        bridge: RosBridge,
        joint_rows: list,
        gripper_cfg: dict | None = None,
        gripper_btns: list | None = None,
        parent=None,
    ) -> None:
        super().__init__(parent)
        self._bridge = bridge
        self._joint_rows = joint_rows  # list[Optional[JointRow]] shared with JogWindow
        self._gripper_cfg = gripper_cfg or {}
        self._gripper_btns = gripper_btns or [None, None]
        self._points: list[JointPoint] = []
        self._counter = 0

        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)

        toolbar = QHBoxLayout()
        record_btn = QPushButton("Record Current Pose")
        record_btn.setFixedHeight(32)
        record_btn.setStyleSheet(
            "QPushButton{background:#bbdefb;border-radius:4px;font-weight:bold;}"
        )
        record_btn.clicked.connect(self._record_current)
        toolbar.addWidget(record_btn)

        save_btn = QPushButton("Save…")
        save_btn.setFixedHeight(32)
        save_btn.setFixedWidth(70)
        save_btn.setStyleSheet("QPushButton{border-radius:4px;}")
        save_btn.clicked.connect(self._save_points)
        toolbar.addWidget(save_btn)

        load_btn = QPushButton("Load…")
        load_btn.setFixedHeight(32)
        load_btn.setFixedWidth(70)
        load_btn.setStyleSheet("QPushButton{border-radius:4px;}")
        load_btn.clicked.connect(self._load_points)
        toolbar.addWidget(load_btn)

        toolbar.addWidget(QLabel("  Move time:"))
        self._duration_spin = QDoubleSpinBox()
        self._duration_spin.setRange(0.5, 30.0)
        self._duration_spin.setSingleStep(0.5)
        self._duration_spin.setDecimals(1)
        self._duration_spin.setValue(3.0)
        self._duration_spin.setSuffix(" s")
        self._duration_spin.setFixedWidth(80)
        toolbar.addWidget(self._duration_spin)

        toolbar.addStretch()
        clear_btn = QPushButton("Clear All")
        clear_btn.setFixedHeight(32)
        clear_btn.setFixedWidth(90)
        clear_btn.setStyleSheet("QPushButton{background:#ffcdd2;border-radius:4px;}")
        clear_btn.clicked.connect(self._clear_all)
        toolbar.addWidget(clear_btn)
        layout.addLayout(toolbar)

        hdr = QWidget()
        hdr.setStyleSheet("background:#e0e0e0;")
        hl = QHBoxLayout(hdr)
        hl.setContentsMargins(6, 2, 6, 2)
        hl.setSpacing(8)
        bold = QFont("Arial", 8, QFont.Bold)
        for text, width in [("#", 32), ("Name", -1), ("Actions", 228)]:
            lbl = QLabel(text)
            lbl.setFont(bold)
            if width > 0:
                lbl.setFixedWidth(width)
            hl.addWidget(lbl, 0 if width > 0 else 1)
        layout.addWidget(hdr)

        self._scroll = QScrollArea()
        self._scroll.setWidgetResizable(True)
        self._scroll.setFrameShape(QFrame.NoFrame)
        self._list_container = QWidget()
        self._list_layout = QVBoxLayout(self._list_container)
        self._list_layout.setSpacing(1)
        self._list_layout.setContentsMargins(0, 0, 0, 0)
        self._scroll.setWidget(self._list_container)
        layout.addWidget(self._scroll)

        self._rebuild_list()

    def _record_current(self) -> None:
        self._counter += 1
        self._points.append(JointPoint(
            name=f"Point {self._counter}",
            values=self._bridge.get_targets(),
            gripper_pos=self._bridge.get_gripper_pos(),
        ))
        self._rebuild_list()

    def _clear_all(self) -> None:
        self._points.clear()
        self._rebuild_list()

    def _save_points(self) -> None:
        if not self._points:
            QMessageBox.information(self, "Save Points", "No points to save.")
            return
        path, _ = QFileDialog.getSaveFileName(
            self, "Save Point List", "points.json", "JSON files (*.json)"
        )
        if not path:
            return
        data = {
            "version": 1,
            "points": [
                {"name": p.name, "values": p.values, "gripper_pos": p.gripper_pos}
                for p in self._points
            ],
        }
        try:
            with open(path, "w") as f:
                json.dump(data, f, indent=2)
        except OSError as e:
            QMessageBox.critical(self, "Save Failed", str(e))

    def _load_points(self) -> None:
        path, _ = QFileDialog.getOpenFileName(
            self, "Load Point List", "", "JSON files (*.json)"
        )
        if not path:
            return
        try:
            with open(path) as f:
                data = json.load(f)
            raw = data["points"]
            points = []
            for entry in raw:
                vals = [float(v) for v in entry["values"]]
                if len(vals) != 18:
                    raise ValueError(f"Expected 18 values per point, got {len(vals)}")
                raw_g = entry.get("gripper_pos", [0.0, 0.0])
                gripper_pos = [float(v) for v in raw_g]
                if len(gripper_pos) != 2:
                    gripper_pos = [0.0, 0.0]
                points.append(JointPoint(name=str(entry["name"]), values=vals, gripper_pos=gripper_pos))
        except (OSError, KeyError, ValueError, TypeError) as e:
            QMessageBox.critical(self, "Load Failed", f"Could not load file:\n{e}")
            return
        self._points = points
        self._counter = len(self._points)
        self._rebuild_list()

    def _replay(self, idx: int) -> None:
        point = self._points[idx]
        duration = self._duration_spin.value()
        # Send timed move via topstar_api arm request to h1_ros2_node.
        # H1Bridge will suppress regular LowCmd joints for `duration` seconds so
        # the jog stream cannot override the interpolated trajectory.
        self._bridge.move_joints_timed(point.values, duration)
        # Prime the jog-stream target so it holds the final pose once the
        # timed move expires and H1Bridge resumes accepting LowCmd joints.
        self._bridge.set_targets(point.values)
        for spec in JOINT_SPECS:
            if self._joint_rows[spec.index] is not None:
                self._joint_rows[spec.index].set_target(point.values[spec.index])
        # Apply gripper state from the recorded point; flows to hardware via /hand/*/cmd.
        for i, val in enumerate(point.gripper_pos):
            self._bridge.set_gripper(i, val)
        # Sync gripper toggle buttons to reflect the replayed state.
        for i, btn in enumerate(self._gripper_btns):
            if btn is not None:
                checked = point.gripper_pos[i] > 0.5
                btn.blockSignals(True)
                btn.setChecked(checked)
                btn.setText(f"Gripper: {'ON' if checked else 'OFF'}")
                btn.blockSignals(False)

    def _edit(self, idx: int) -> None:
        dlg = PointEditDialog(self._points[idx], gripper_cfg=self._gripper_cfg, parent=self)
        if dlg.exec() == QDialog.Accepted:
            self._points[idx] = dlg.result_point(self._points[idx])
            self._rebuild_list()

    def _delete(self, idx: int) -> None:
        self._points.pop(idx)
        self._rebuild_list()

    def _rebuild_list(self) -> None:
        while self._list_layout.count():
            item = self._list_layout.takeAt(0)
            if item.widget():
                item.widget().deleteLater()

        if not self._points:
            empty = QLabel("No points recorded. Click 'Record Current Pose' to add one.")
            empty.setAlignment(Qt.AlignCenter)
            empty.setStyleSheet("color: gray; font-style: italic; padding: 20px;")
            self._list_layout.addWidget(empty)
        else:
            for i, point in enumerate(self._points):
                item_w = PointItemWidget(i, point)
                item_w.setStyleSheet("background:#fafafa;" if i % 2 == 0 else "background:#ffffff;")
                item_w.replay_requested.connect(self._replay)
                item_w.edit_requested.connect(self._edit)
                item_w.delete_requested.connect(self._delete)
                self._list_layout.addWidget(item_w)

        self._list_layout.addStretch()


# ── Main window ───────────────────────────────────────────────────────────────

class JogWindow(QMainWindow):
    def __init__(
        self,
        bridge: RosBridge,
        gripper_cfg: dict | None = None,
    ) -> None:
        super().__init__()
        self._bridge = bridge
        self._gripper_cfg = gripper_cfg or {}
        self._rows: list[Optional[JointRow]] = [None] * 18
        self._gripper_btns: list[Optional[QPushButton]] = [None, None]
        self._gripper_user_interacted: list[bool] = [False, False]  # True once user explicitly toggles
        self._active_base_jog_key: Optional[str] = None
        self._active_base_dir: tuple[float, float, float] = (0.0, 0.0, 0.0)
        self._state_initialized = False

        self.setWindowTitle("H1 Upper Body Joint Jog")
        self.setMinimumSize(860, 500)

        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setSpacing(6)
        root.setContentsMargins(8, 8, 8, 8)

        root.addWidget(self._build_toolbar())
        root.addWidget(self._build_tabs())

        bridge.state_updated.connect(self._on_state)
        bridge.gripper_updated.connect(self._on_gripper_state)

        self._poll = QTimer()
        self._poll.timeout.connect(self._update_status)
        self._poll.start(400)
        self._update_status()

    # ── Builders ───────────────────────────────────────────────────────────

    def _build_toolbar(self) -> QWidget:
        bar = QWidget()
        bar.setStyleSheet("background:#f0f0f0; border-radius:4px;")
        lo = QHBoxLayout(bar)
        lo.setContentsMargins(8, 6, 8, 6)
        lo.setSpacing(10)

        self._enable_btn = QPushButton("  Disabled — click to enable publishing")
        self._enable_btn.setCheckable(True)
        self._enable_btn.setFixedHeight(32)
        self._enable_btn.setMinimumWidth(260)
        self._enable_btn.setStyleSheet(
            "QPushButton{background:#ef9a9a;border-radius:4px;font-weight:bold;}"
            "QPushButton:checked{background:#a5d6a7;}"
        )
        self._enable_btn.toggled.connect(self._on_enable_toggled)

        home_btn = QPushButton("Home All  (→ 0)")
        home_btn.setFixedHeight(32)
        home_btn.setFixedWidth(140)
        home_btn.setStyleSheet("QPushButton{border-radius:4px;}")
        home_btn.clicked.connect(self._home_all)

        lo.addWidget(self._enable_btn)
        lo.addWidget(home_btn)
        lo.addSpacerItem(QSpacerItem(1, 1, QSizePolicy.Expanding, QSizePolicy.Fixed))

        lo.addWidget(QLabel("Jog step:"))
        self._step_spin = QDoubleSpinBox()
        self._step_spin.setRange(0.001, 0.500)
        self._step_spin.setSingleStep(0.01)
        self._step_spin.setDecimals(3)
        self._step_spin.setValue(0.05)
        self._step_spin.setFixedWidth(80)
        lo.addWidget(self._step_spin)

        lo.addWidget(QLabel("  "))
        self._state_lbl = QLabel("● no state")
        self._state_lbl.setFixedWidth(150)
        self._state_lbl.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        lo.addWidget(self._state_lbl)

        return bar

    def _build_tabs(self) -> QTabWidget:
        _ARM_IDX = {"Right Arm": 0, "Left Arm": 1}
        tabs = QTabWidget()
        tabs.addTab(self._build_base_jog_panel(), "Base Jog")
        for name, indices in TAB_GROUPS:
            tabs.addTab(
                self._build_joint_tab([JOINT_SPECS[i] for i in indices], arm_idx=_ARM_IDX.get(name)),
                name,
            )
        self._point_panel = PointListPanel(
            self._bridge, self._rows,
            gripper_cfg=self._gripper_cfg,
            gripper_btns=self._gripper_btns,
        )
        tabs.addTab(self._point_panel, "Point List")
        return tabs

    def _build_base_jog_panel(self) -> QWidget:
        panel = QWidget()
        panel.setStyleSheet("background:#f7fbff; border:1px solid #d0e3f2; border-radius:4px;")

        root = QVBoxLayout(panel)
        root.setContentsMargins(8, 6, 8, 6)
        root.setSpacing(4)
        root.setAlignment(Qt.AlignTop)

        title = QLabel("Base Jog (press and hold)")
        title.setFont(QFont("Arial", 8, QFont.Bold))
        title.setFixedHeight(18)
        root.addWidget(title)

        speed_row = QHBoxLayout()
        speed_row.setSpacing(6)
        speed_row.setContentsMargins(0, 0, 0, 0)
        speed_row.addStretch()
        speed_row.addWidget(QLabel("Speed:"))

        self._base_speed_slider = QSlider(Qt.Horizontal)
        self._base_speed_slider.setRange(5, 100)
        self._base_speed_slider.setValue(30)
        self._base_speed_slider.setSingleStep(1)
        self._base_speed_slider.setPageStep(5)
        self._base_speed_slider.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)
        self._base_speed_slider.setFixedWidth(260)
        self._base_speed_slider.setFixedHeight(16)
        self._base_speed_slider.valueChanged.connect(self._on_base_speed_changed)
        speed_row.addWidget(self._base_speed_slider)

        self._base_speed_lbl = QLabel()
        self._base_speed_lbl.setFixedWidth(96)
        self._base_speed_lbl.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        speed_row.addWidget(self._base_speed_lbl)
        speed_row.addStretch()

        root.addLayout(speed_row)

        grid = QVBoxLayout()
        grid.setSpacing(6)

        row1 = QHBoxLayout()
        row1.addStretch()
        row1.addWidget(self._make_hold_btn("Forward", "fwd", (+1.0, 0.0, 0.0)))
        row1.addStretch()

        row2 = QHBoxLayout()
        row2.addWidget(self._make_hold_btn("Left", "left", (0.0, +1.0, 0.0)))
        row2.addWidget(self._make_hold_btn("Stop", "stop", (0.0, 0.0, 0.0), hold=False))
        row2.addWidget(self._make_hold_btn("Right", "right", (0.0, -1.0, 0.0)))

        row3 = QHBoxLayout()
        row3.addStretch()
        row3.addWidget(self._make_hold_btn("Backward", "back", (-1.0, 0.0, 0.0)))
        row3.addStretch()

        row4 = QHBoxLayout()
        row4.addWidget(self._make_hold_btn("Turn Left", "turn_l", (0.0, 0.0, +1.0)))
        row4.addWidget(self._make_hold_btn("Turn Right", "turn_r", (0.0, 0.0, -1.0)))

        grid.addLayout(row1)
        grid.addLayout(row2)
        grid.addLayout(row3)
        grid.addLayout(row4)

        root.addLayout(grid)
        root.addStretch()
        self._refresh_base_speed_label()
        return panel

    def _make_hold_btn(self, text: str, key: str, direction: tuple[float, float, float], hold: bool = True) -> QPushButton:
        btn = QPushButton(text)
        btn.setFixedWidth(120)
        btn.setFixedHeight(28)
        btn.setStyleSheet("QPushButton{border-radius:4px;}")
        if hold:
            btn.pressed.connect(lambda k=key, d=direction: self._on_base_hold_pressed(k, d))
            btn.released.connect(lambda k=key: self._on_base_hold_released(k))
        else:
            btn.clicked.connect(lambda: self._set_base_jog(None, (0.0, 0.0, 0.0)))
        return btn

    def _base_speed(self) -> float:
        return self._base_speed_slider.value() / 100.0

    def _refresh_base_speed_label(self) -> None:
        speed = self._base_speed()
        self._base_speed_lbl.setText(f"{speed:.2f} m/s, rad/s")

    def _on_base_speed_changed(self, _value: int) -> None:
        self._refresh_base_speed_label()
        if self._active_base_jog_key is not None:
            self._apply_base_cmd(self._active_base_dir)

    def _apply_base_cmd(self, direction: tuple[float, float, float]) -> None:
        speed = self._base_speed()
        self._bridge.set_base_cmd(direction[0] * speed, direction[1] * speed, direction[2] * speed)

    def _set_base_jog(self, key: Optional[str], direction: tuple[float, float, float]) -> None:
        self._active_base_jog_key = key
        self._active_base_dir = direction
        self._apply_base_cmd(direction)

    def _on_base_hold_pressed(self, key: str, direction: tuple[float, float, float]) -> None:
        self._set_base_jog(key, direction)

    def _on_base_hold_released(self, key: str) -> None:
        if self._active_base_jog_key == key:
            self._set_base_jog(None, (0.0, 0.0, 0.0))

    def _on_gripper_toggled(self, arm_idx: int, checked: bool) -> None:
        self._gripper_user_interacted[arm_idx] = True  # user has explicit control; stop feedback override
        value = 1.0 if checked else 0.0
        self._bridge.set_gripper(arm_idx, value)
        btn = self._gripper_btns[arm_idx]
        if btn is not None:
            btn.setText(f"Gripper: {'ON' if checked else 'OFF'}")

    def _build_joint_tab(self, specs: list[JointSpec], arm_idx: int | None = None) -> QWidget:
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)

        container = QWidget()
        vbox = QVBoxLayout(container)
        vbox.setSpacing(1)
        vbox.setContentsMargins(4, 4, 4, 4)

        # Column header
        hdr = QWidget()
        hdr.setStyleSheet("background:#e0e0e0;")
        hl = QHBoxLayout(hdr)
        hl.setContentsMargins(6, 2, 6, 2)
        hl.setSpacing(8)
        bold = QFont("Arial", 8, QFont.Bold)
        for text, width in [
            ("Joint", 130), ("Current", 72), ("", 28),
            ("Target (drag slider or edit value)", -1), ("Jog ±", 64),
        ]:
            lbl = QLabel(text)
            lbl.setFont(bold)
            if width > 0:
                lbl.setFixedWidth(width)
            hl.addWidget(lbl)
        vbox.addWidget(hdr)

        sep = QFrame()
        sep.setFrameShape(QFrame.HLine)
        sep.setFrameShadow(QFrame.Sunken)
        vbox.addWidget(sep)

        for i, spec in enumerate(specs):
            row = JointRow(spec, step_getter=self._step_spin.value)
            row.setStyleSheet("background:#fafafa;" if i % 2 == 0 else "background:#ffffff;")
            row.target_changed.connect(self._on_target_changed)
            self._rows[spec.index] = row
            vbox.addWidget(row)

        if arm_idx is not None:
            arm_key = "robot0" if arm_idx == 0 else "robot1"
            if self._gripper_cfg.get(arm_key, False):
                sep2 = QFrame()
                sep2.setFrameShape(QFrame.HLine)
                sep2.setFrameShadow(QFrame.Sunken)
                vbox.addWidget(sep2)

                gripper_row = QHBoxLayout()
                gripper_row.setContentsMargins(6, 6, 6, 6)
                gripper_lbl = QLabel("Vacuum Gripper:")
                gripper_lbl.setFixedWidth(130)
                gripper_lbl.setFont(QFont("Monospace", 9))
                btn = QPushButton("Gripper: OFF")
                btn.setCheckable(True)
                btn.setChecked(False)
                btn.setFixedWidth(130)
                btn.setFixedHeight(28)
                btn.setStyleSheet(
                    "QPushButton{border-radius:4px;background:#ef9a9a;}"
                    "QPushButton:checked{background:#a5d6a7;}"
                )
                btn.toggled.connect(lambda checked, idx=arm_idx: self._on_gripper_toggled(idx, checked))
                self._gripper_btns[arm_idx] = btn
                gripper_row.addWidget(gripper_lbl)
                gripper_row.addWidget(btn)
                gripper_row.addStretch()
                gripper_widget = QWidget()
                gripper_widget.setLayout(gripper_row)
                vbox.addWidget(gripper_widget)

        vbox.addStretch()
        scroll.setWidget(container)
        return scroll

    # ── Slots ──────────────────────────────────────────────────────────────

    @Slot(list)
    def _on_state(self, positions: list[float]) -> None:
        if not self._state_initialized:
            self._state_initialized = True
            self._bridge.set_targets(list(positions))
            for i, pos in enumerate(positions):
                if self._rows[i] is not None:
                    self._rows[i].set_target(pos)
        for i, pos in enumerate(positions):
            if self._rows[i] is not None:
                self._rows[i].update_current(pos)

    def _on_gripper_state(self, gripper: list[float]) -> None:
        """Initialise gripper button from lowstate feedback — only before the user
        has explicitly clicked that arm's button.  Once the user interacts,
        hardware feedback no longer overrides the commanded state."""
        for arm_idx, value in enumerate(gripper):
            if self._gripper_user_interacted[arm_idx]:
                continue  # user has explicit control; ignore feedback
            btn = self._gripper_btns[arm_idx]
            if btn is None:
                continue
            checked = value >= 0.5
            if btn.isChecked() != checked:
                btn.blockSignals(True)
                btn.setChecked(checked)
                btn.setText(f"Gripper: {'ON' if checked else 'OFF'}")
                btn.blockSignals(False)
                self._bridge.set_gripper(arm_idx, 1.0 if checked else 0.0)

    @Slot(int, float)
    def _on_target_changed(self, idx: int, value: float) -> None:
        self._bridge.set_target(idx, value)

    def _on_enable_toggled(self, checked: bool) -> None:
        self._bridge.set_enabled(checked)
        if checked:
            self._enable_btn.setText("  Publishing ENABLED — click to disable")
        else:
            self._enable_btn.setText("  Disabled — click to enable publishing")

    def _home_all(self) -> None:
        for row in self._rows:
            if row is not None:
                row.set_target(0.0)
        self._bridge.set_targets([0.0] * 18)

    def _update_status(self) -> None:
        age = self._bridge.last_state_age()
        if age < 0.5:
            self._state_lbl.setText("● receiving state")
            self._state_lbl.setStyleSheet("color:green;")
        elif age < 2.0:
            self._state_lbl.setText("● state stale")
            self._state_lbl.setStyleSheet("color:darkorange;")
        else:
            self._state_lbl.setText("● no state received")
            self._state_lbl.setStyleSheet("color:red;")

    def closeEvent(self, event) -> None:
        self._bridge.set_enabled(False)
        self._bridge.set_base_cmd(0.0, 0.0, 0.0)
        self._bridge.stop()
        event.accept()


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None) -> None:
    app = QApplication(sys.argv)
    app.setApplicationName("H1 Upper Body Jog")

    bridge = RosBridge()
    bridge.start()

    # Resolve config: explicit env var → fallback relative path → installed package path.
    robot_config = None
    cfg_file = os.environ.get("TOPSTAR_H1_ROBOT_CFG_FILE", "")
    if not cfg_file:
        _here = os.path.dirname(os.path.abspath(__file__))
        _fallbacks = [
            os.path.join(_here, "..", "..", "..", "config", "h1", "robot_config.json"),
        ]
        try:
            from ament_index_python.packages import get_package_share_directory
            _fallbacks.append(os.path.join(
                get_package_share_directory("topstar_ros2_example"),
                "config", "h1", "robot_config.json",
            ))
        except Exception:
            pass
        for _fp in _fallbacks:
            if os.path.isfile(_fp):
                cfg_file = os.path.abspath(_fp)
                print(f"[jog] TOPSTAR_H1_ROBOT_CFG_FILE not set; using fallback: {cfg_file!r}")
                break
        else:
            print("[jog] No robot config found; gripper controls will be hidden.")

    if cfg_file:
        try:
            with open(cfg_file) as f:
                robot_config = json.load(f)
        except OSError as e:
            print(f"[jog] Could not load config file {cfg_file!r}: {e}")
        except json.JSONDecodeError as e:
            print(f"[jog] Config file {cfg_file!r} is not valid JSON: {e}")

    # Derive gripper availability directly from config.
    # The jog GUI communicates with the remote robot exclusively via ROS2 topics;
    # no local H1UpperBodyController is instantiated.
    gripper_cfg: dict = {
        "robot0": bool((robot_config or {}).get("robot0", {}).get("gripper_enabled", False)),
        "robot1": bool((robot_config or {}).get("robot1", {}).get("gripper_enabled", False)),
    }

    window = JogWindow(bridge, gripper_cfg=gripper_cfg)
    window.show()

    sys.exit(app.exec())


if __name__ == "__main__":
    main()
