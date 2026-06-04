#!/usr/bin/env python3
"""H2 Isaac Sim joint jog panel (PySide6).

Slider-style GUI for jogging H2 joints in Isaac Sim.
Publishes topstar_hg/LowCmd to /lowcmd at 50 Hz and
displays live joint state from /lowstate.

Usage:
  source ~/topstar_ros2/setup.sh
  python3 ~/topstar_ros2/example/h2_isaac_jog.py
"""
from __future__ import annotations

import json
import math
import struct
import sys
import threading
import time
from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

try:
    from topstar_hg.msg import LowCmd, LowState, MotorCmd
except ImportError:
    print("ERROR: topstar_hg messages not found. Did you source setup.sh?")
    raise

try:
    from topstar_api.msg import Request as SportRequest
    _HAS_SPORT_API = True
except ImportError:
    _HAS_SPORT_API = False
    print("WARNING: topstar_api not found — FSM switching disabled.")

from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton, QDoubleSpinBox, QTabWidget, QFrame,
    QScrollArea, QSlider, QSizePolicy, QSpacerItem, QComboBox,
    QDialog, QDialogButtonBox, QLineEdit, QFileDialog, QMessageBox,
    QButtonGroup,
)
from PySide6.QtCore import Qt, Signal, QObject, QTimer, Slot
from PySide6.QtGui import QFont

# ── CRC (ported from example/src/src/common/motor_crc_hg.cpp) ─────────────

def _crc32_core(data: bytes) -> int:
    words = struct.unpack(f'{len(data) // 4}I', data)
    crc = 0xFFFFFFFF
    poly = 0x04C11DB7
    for word in words:
        xbit = 1 << 31
        for _ in range(32):
            if crc & 0x80000000:
                crc = ((crc << 1) & 0xFFFFFFFF) ^ poly
            else:
                crc = (crc << 1) & 0xFFFFFFFF
            if word & xbit:
                crc ^= poly
            xbit >>= 1
    return crc


def compute_crc(msg: LowCmd) -> None:
    buf = bytearray()
    buf += struct.pack('BB2x', msg.mode_pr, msg.mode_machine)
    for m in msg.motor_cmd:
        buf += struct.pack('=B3xfffffI', m.mode, m.q, m.dq, m.tau, m.kp, m.kd, m.reserve)
    buf += struct.pack('4I', *list(msg.reserve))
    msg.crc = _crc32_core(bytes(buf))


# ── FSM state table ───────────────────────────────────────────────────────

# api_id for SetFsmId (ROBOT_API_ID_LOCO_SET_FSM_ID)
_SPORT_API_SET_FSM_ID = 7101

# (fsm_id, button_label) — matches h2_loco_client.hpp High Level API
FSM_STATES: list[tuple[int, str]] = [
    (0, "Zero Torque"),
    (1, "Damp"),
    (9, "Manual"),
]

# ── Joint metadata ─────────────────────────────────────────────────────────

N_JOINTS = 29

# kp[0-12] = 100 (legs + waist), kp[13-28] = 50 (head + arms)
_DEFAULT_KP = [100.0] * 13 + [50.0] * 16
_DEFAULT_KD = [1.0] * 29


@dataclass
class JointSpec:
    display_name: str
    index: int
    low: float
    high: float
    unit: str = "rad"


JOINT_SPECS: list[JointSpec] = [
    # Left Leg
    JointSpec("L HipPitch",       0, -2.094,   1.4486),
    JointSpec("L HipRoll",        1, -1.222,   1.536),
    JointSpec("L HipYaw",         2, -0.8378,  0.8378),
    JointSpec("L Knee",           3, -0.0873,  2.0944),
    JointSpec("L AnklePitch",     4, -0.65,    0.42),
    JointSpec("L AnkleRoll",      5, -0.1,     0.1),
    # Right Leg
    JointSpec("R HipPitch",       6, -2.094,   1.4486),
    JointSpec("R HipRoll",        7, -1.536,   1.222),
    JointSpec("R HipYaw",         8, -0.8378,  0.8378),
    JointSpec("R Knee",           9, -0.0873,  2.0944),
    JointSpec("R AnklePitch",    10, -0.65,    0.42),
    JointSpec("R AnkleRoll",     11, -0.1,     0.1),
    # Torso/Head
    JointSpec("WaistYaw",        12, -2.8797,  2.8797),
    JointSpec("HeadYaw",         13, -1.6057,  1.6057),
    JointSpec("HeadPitch",       14, -0.2967,  0.384),
    # Left Arm
    JointSpec("L ShoulderPitch", 15, -3.927,   1.8326),
    JointSpec("L ShoulderRoll",  16, -0.15708, 3.0194),
    JointSpec("L ShoulderYaw",   17, -2.9671,  2.9671),
    JointSpec("L Elbow",         18, -2.2515,  1.3614),
    JointSpec("L WristYaw",      19, -2.9671,  2.9671),
    JointSpec("L WristPitch",    20, -1.6581,  1.6581),
    JointSpec("L WristRoll",     21, -1.7453,  1.7453),
    # Right Arm
    JointSpec("R ShoulderPitch", 22, -3.927,   1.8326),
    JointSpec("R ShoulderRoll",  23, -3.0194,  0.15708),
    JointSpec("R ShoulderYaw",   24, -2.9671,  2.9671),
    JointSpec("R Elbow",         25, -1.3614,  2.2515),
    JointSpec("R WristYaw",      26, -2.9671,  2.9671),
    JointSpec("R WristPitch",    27, -1.6581,  1.6581),
    JointSpec("R WristRoll",     28, -1.7453,  1.7453),
]

TAB_GROUPS: list[tuple[str, list[int]]] = [
    ("Left Leg",   list(range(0, 6))),
    ("Right Leg",  list(range(6, 12))),
    ("Torso/Head", list(range(12, 15))),
    ("Left Arm",   list(range(15, 22))),
    ("Right Arm",  list(range(22, 29))),
]

_SPEC_BY_INDEX: dict[int, JointSpec] = {s.index: s for s in JOINT_SPECS}


@dataclass
class JointPoint:
    name: str
    values: list[float]  # N_JOINTS floats, indexed by motor slot


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


# ── ROS2 bridge ────────────────────────────────────────────────────────────

class RosBridge(QObject):
    """Thread-safe link between the GUI and the ROS2 spin thread."""
    state_updated = Signal(list)   # list[float] of N_JOINTS positions

    def __init__(self) -> None:
        super().__init__()
        self._lock = threading.Lock()
        self._targets: list[float] = [0.0] * N_JOINTS
        self._enabled: bool = False
        self._mode_pr: int = 0
        self._mode_machine: int = 0
        self._last_state_t: float = 0.0
        self._state_received: bool = False
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

    def set_target(self, idx: int, value: float) -> None:
        with self._lock:
            self._targets[idx] = value

    def set_targets(self, values: list[float]) -> None:
        with self._lock:
            self._targets = list(values)

    def get_targets(self) -> list[float]:
        with self._lock:
            return list(self._targets)

    def request_fsm(self, fsm_id: int) -> None:
        """Publish a SetFsmId request to /api/sport/request."""
        with self._lock:
            node = self._node
        if node is not None and _HAS_SPORT_API:
            node.send_fsm_request(fsm_id)

    def set_enabled(self, enabled: bool) -> None:
        with self._lock:
            self._enabled = enabled

    def is_enabled(self) -> bool:
        with self._lock:
            return self._enabled

    def set_mode_pr(self, v: int) -> None:
        with self._lock:
            self._mode_pr = v

    def get_mode_pr(self) -> int:
        with self._lock:
            return self._mode_pr

    def get_mode_machine(self) -> int:
        with self._lock:
            return self._mode_machine

    def last_state_age(self) -> float:
        with self._lock:
            if self._last_state_t == 0.0:
                return math.inf
            return time.monotonic() - self._last_state_t

    def is_state_received(self) -> bool:
        with self._lock:
            return self._state_received

    def _record_state(self, positions: list[float], mode_machine: int) -> None:
        with self._lock:
            self._last_state_t = time.monotonic()
            self._state_received = True
            self._mode_machine = mode_machine
        self.state_updated.emit(positions)


class _JogNode(Node):
    _HZ = 50

    def __init__(self, bridge: RosBridge) -> None:
        super().__init__('h2_isaac_jog')
        self._bridge = bridge
        self._pub = self.create_publisher(LowCmd, '/lowcmd', _CMD_QOS)
        self._sub = self.create_subscription(
            LowState, '/lowstate', self._on_state, _SENSOR_QOS)
        self._timer = self.create_timer(1.0 / self._HZ, self._publish)
        if _HAS_SPORT_API:
            self._sport_pub = self.create_publisher(
                SportRequest, '/api/sport/request', _CMD_QOS)
        else:
            self._sport_pub = None
        self.get_logger().info('h2_isaac_jog ready.')

    def send_fsm_request(self, fsm_id: int) -> None:
        if self._sport_pub is None:
            return
        msg = SportRequest()
        msg.header.identity.api_id = _SPORT_API_SET_FSM_ID
        msg.parameter = json.dumps({"data": fsm_id})
        self._sport_pub.publish(msg)
        self.get_logger().info(f'FSM request: fsm_id={fsm_id}')

    def _on_state(self, msg: LowState) -> None:
        positions = [float(msg.motor_state[i].q) for i in range(N_JOINTS)]
        self._bridge._record_state(positions, int(msg.mode_machine))

    def _publish(self) -> None:
        if not self._bridge.is_enabled() or not self._bridge.is_state_received():
            return
        targets = self._bridge.get_targets()
        mode_pr = self._bridge.get_mode_pr()
        mode_machine = self._bridge.get_mode_machine()

        cmd = LowCmd()
        cmd.mode_pr = mode_pr
        cmd.mode_machine = mode_machine
        for i in range(N_JOINTS):
            cmd.motor_cmd[i] = MotorCmd(
                mode=1,
                q=float(targets[i]),
                dq=0.0,
                tau=0.0,
                kp=float(_DEFAULT_KP[i]),
                kd=float(_DEFAULT_KD[i]),
                reserve=0,
            )
        compute_crc(cmd)
        self._pub.publish(cmd)


# ── Per-joint row widget ────────────────────────────────────────────────────

class JointRow(QWidget):
    """One row: name | current | unit | slider | spinbox | − | + """

    target_changed = Signal(int, float)

    _SLIDER_RES = 2000

    def __init__(self, spec: JointSpec, step_getter, parent=None) -> None:
        super().__init__(parent)
        self._spec = spec
        self._step_getter = step_getter
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

    def _to_slider(self, v: float) -> int:
        frac = (v - self._spec.low) / (self._spec.high - self._spec.low)
        return int(max(0.0, min(1.0, frac)) * self._SLIDER_RES)

    def _from_slider(self, s: int) -> float:
        return self._spec.low + (s / self._SLIDER_RES) * (self._spec.high - self._spec.low)

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

    def update_current(self, v: float) -> None:
        self._cur.setText(f"{v:+.3f}")

    def set_target(self, v: float) -> None:
        self._spin.setValue(max(self._spec.low, min(self._spec.high, v)))

    def get_target(self) -> float:
        return self._spin.value()


# ── Point list ─────────────────────────────────────────────────────────────

class PointEditDialog(QDialog):
    def __init__(self, point: JointPoint, parent=None) -> None:
        super().__init__(parent)
        self.setWindowTitle("Edit Point")
        self.setMinimumWidth(540)

        layout = QVBoxLayout(self)
        layout.setSpacing(8)

        name_row = QHBoxLayout()
        name_row.addWidget(QLabel("Name:"))
        self._name_edit = QLineEdit(point.name)
        name_row.addWidget(self._name_edit)
        layout.addLayout(name_row)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFixedHeight(420)
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
        return JointPoint(
            name=self._name_edit.text().strip() or original.name,
            values=values,
        )


class PointItemWidget(QWidget):
    replay_requested = Signal(int)
    edit_requested   = Signal(int)
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

        btn_replay = QPushButton("Replay")
        btn_edit   = QPushButton("Edit")
        btn_del    = QPushButton("Delete")
        for b in (btn_replay, btn_edit, btn_del):
            b.setFixedHeight(24)
            b.setFixedWidth(70)
        btn_replay.setStyleSheet("QPushButton{background:#a5d6a7;border-radius:3px;}")
        btn_del.setStyleSheet("QPushButton{background:#ef9a9a;border-radius:3px;}")

        lo.addWidget(num_lbl)
        lo.addWidget(name_lbl, stretch=1)
        lo.addWidget(btn_replay)
        lo.addWidget(btn_edit)
        lo.addWidget(btn_del)

        btn_replay.clicked.connect(lambda: self.replay_requested.emit(index))
        btn_edit.clicked.connect(lambda: self.edit_requested.emit(index))
        btn_del.clicked.connect(lambda: self.delete_requested.emit(index))


class PointListPanel(QWidget):
    """Tab for recording, editing, replaying, and saving named joint poses."""

    def __init__(self, bridge: RosBridge, joint_rows: list, parent=None) -> None:
        super().__init__(parent)
        self._bridge = bridge
        self._joint_rows = joint_rows   # list[Optional[JointRow]], indexed by slot
        self._points: list[JointPoint] = []
        self._counter = 0
        self._replay_active = False

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
        save_btn.clicked.connect(self._save_points)
        toolbar.addWidget(save_btn)

        load_btn = QPushButton("Load…")
        load_btn.setFixedHeight(32)
        load_btn.setFixedWidth(70)
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
            self, "Save Point List", "h2_points.json", "JSON files (*.json)"
        )
        if not path:
            return
        data = {
            "version": 1,
            "robot": "H2",
            "n_joints": N_JOINTS,
            "points": [{"name": p.name, "values": p.values} for p in self._points],
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
                if len(vals) != N_JOINTS:
                    raise ValueError(f"Expected {N_JOINTS} values per point, got {len(vals)}")
                points.append(JointPoint(name=str(entry["name"]), values=vals))
        except (OSError, KeyError, ValueError, TypeError) as e:
            QMessageBox.critical(self, "Load Failed", f"Could not load file:\n{e}")
            return
        self._points = points
        self._counter = len(self._points)
        self._rebuild_list()

    def _replay(self, idx: int) -> None:
        if self._replay_active:
            return
        target_vals = list(self._points[idx].values)
        duration = self._duration_spin.value()
        start_vals = self._bridge.get_targets()
        t_start = time.monotonic()
        self._replay_active = True

        def step():
            elapsed = time.monotonic() - t_start
            if elapsed >= duration:
                self._bridge.set_targets(target_vals)
                self._update_sliders(target_vals)
                self._replay_active = False
                return
            alpha = elapsed / duration
            current = [
                start_vals[i] + alpha * (target_vals[i] - start_vals[i])
                for i in range(N_JOINTS)
            ]
            self._bridge.set_targets(current)
            self._update_sliders(current)
            QTimer.singleShot(20, step)  # ~50 Hz updates

        step()

    def _update_sliders(self, values: list[float]) -> None:
        for i, v in enumerate(values):
            if i < len(self._joint_rows) and self._joint_rows[i] is not None:
                self._joint_rows[i].set_target(v)

    def _edit(self, idx: int) -> None:
        dlg = PointEditDialog(self._points[idx], parent=self)
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


# ── Main window ────────────────────────────────────────────────────────────

class JogWindow(QMainWindow):
    def __init__(self, bridge: RosBridge) -> None:
        super().__init__()
        self._bridge = bridge
        self._rows: list[Optional[JointRow]] = [None] * N_JOINTS
        self._state_initialized = False
        self._active_fsm_id: int = -1  # -1 = unknown until user clicks

        self.setWindowTitle("H2 Isaac Sim Joint Jog")
        self.setMinimumSize(900, 560)

        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setSpacing(6)
        root.setContentsMargins(8, 8, 8, 8)

        root.addWidget(self._build_toolbar())
        root.addWidget(self._build_fsm_bar())
        root.addWidget(self._build_tabs())

        bridge.state_updated.connect(self._on_state)

        self._poll = QTimer()
        self._poll.timeout.connect(self._update_status)
        self._poll.start(400)
        self._update_status()

    def _build_fsm_bar(self) -> QWidget:
        bar = QWidget()
        bar.setStyleSheet("background:#e8eaf6; border-radius:4px;")
        lo = QHBoxLayout(bar)
        lo.setContentsMargins(8, 4, 8, 4)
        lo.setSpacing(6)

        lbl = QLabel("FSM State:")
        lbl.setFont(QFont("Arial", 9, QFont.Bold))
        lo.addWidget(lbl)

        self._fsm_btn_group = QButtonGroup(self)
        self._fsm_btn_group.setExclusive(True)
        self._fsm_buttons: dict[int, QPushButton] = {}

        # (inactive style, active style) per fsm_id
        _FSM_STYLES: dict[int, tuple[str, str]] = {
            0: (
                "QPushButton{border-radius:4px;background:#e0e0e0;color:#555;font-size:11px;}",
                "QPushButton{border-radius:4px;background:#e65100;color:white;"
                "font-weight:bold;font-size:11px;border:2px solid #bf360c;}",
            ),
            1: (
                "QPushButton{border-radius:4px;background:#e0e0e0;color:#555;font-size:11px;}",
                "QPushButton{border-radius:4px;background:#f9a825;color:#212121;"
                "font-weight:bold;font-size:11px;border:2px solid #f57f17;}",
            ),
            9: (
                "QPushButton{border-radius:4px;background:#e0e0e0;color:#555;font-size:11px;}",
                "QPushButton{border-radius:4px;background:#1565c0;color:white;"
                "font-weight:bold;font-size:11px;border:2px solid #0d47a1;}",
            ),
        }
        self._fsm_styles = _FSM_STYLES

        for fsm_id, label in FSM_STATES:
            btn = QPushButton(label)
            btn.setCheckable(False)  # we manage appearance manually
            btn.setFixedHeight(32)
            btn.setMinimumWidth(90)
            inactive, _ = _FSM_STYLES.get(fsm_id, (_FSM_STYLES[0][0], _FSM_STYLES[0][1]))
            btn.setStyleSheet(inactive)
            btn.setEnabled(_HAS_SPORT_API)
            self._fsm_btn_group.addButton(btn, fsm_id)
            self._fsm_buttons[fsm_id] = btn
            lo.addWidget(btn)
            btn.clicked.connect(lambda _checked, fid=fsm_id: self._on_fsm_clicked(fid))

        lo.addStretch()

        if not _HAS_SPORT_API:
            warn = QLabel("(topstar_api unavailable — FSM disabled)")
            warn.setStyleSheet("color: gray; font-style: italic;")
            lo.addWidget(warn)

        self._fsm_id_lbl = QLabel("mode_machine: ---")
        self._fsm_id_lbl.setFont(QFont("Monospace", 9))
        self._fsm_id_lbl.setFixedWidth(170)
        self._fsm_id_lbl.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        lo.addWidget(self._fsm_id_lbl)

        return bar

    def _build_toolbar(self) -> QWidget:
        bar = QWidget()
        bar.setStyleSheet("background:#f0f0f0; border-radius:4px;")
        lo = QHBoxLayout(bar)
        lo.setContentsMargins(8, 6, 8, 6)
        lo.setSpacing(10)

        self._enable_btn = QPushButton("  Disabled — click to enable publishing")
        self._enable_btn.setCheckable(True)
        self._enable_btn.setFixedHeight(32)
        self._enable_btn.setMinimumWidth(270)
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

        estop_btn = QPushButton("E-STOP")
        estop_btn.setFixedHeight(32)
        estop_btn.setFixedWidth(90)
        estop_btn.setStyleSheet(
            "QPushButton{background:#b82010;color:white;font-weight:bold;border-radius:4px;}"
        )
        estop_btn.clicked.connect(self._estop)

        lo.addWidget(self._enable_btn)
        lo.addWidget(home_btn)
        lo.addWidget(estop_btn)
        lo.addSpacerItem(QSpacerItem(1, 1, QSizePolicy.Expanding, QSizePolicy.Fixed))

        lo.addWidget(QLabel("mode_pr:"))
        self._mode_pr_cb = QComboBox()
        self._mode_pr_cb.addItem("PR (0)", 0)
        self._mode_pr_cb.addItem("AB (1)", 1)
        self._mode_pr_cb.setFixedWidth(75)
        self._mode_pr_cb.setToolTip("PR=pitch-roll virtual ankle joints, AB=physical ankle joints")
        self._mode_pr_cb.currentIndexChanged.connect(self._on_mode_pr_changed)
        lo.addWidget(self._mode_pr_cb)

        lo.addWidget(QLabel("  "))
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
        self._state_lbl.setFixedWidth(160)
        self._state_lbl.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        lo.addWidget(self._state_lbl)

        return bar

    def _build_tabs(self) -> QTabWidget:
        tabs = QTabWidget()
        for name, indices in TAB_GROUPS:
            tabs.addTab(self._build_joint_tab([JOINT_SPECS[i] for i in indices]), name)
        self._point_panel = PointListPanel(self._bridge, self._rows)
        tabs.addTab(self._point_panel, "Point List")
        return tabs

    def _build_joint_tab(self, specs: list[JointSpec]) -> QWidget:
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)

        container = QWidget()
        vbox = QVBoxLayout(container)
        vbox.setSpacing(1)
        vbox.setContentsMargins(4, 4, 4, 4)

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

        vbox.addStretch()
        scroll.setWidget(container)
        return scroll

    @Slot(list)
    def _on_state(self, positions: list[float]) -> None:
        if not self._state_initialized:
            self._state_initialized = True
            self._bridge.set_targets(list(positions))
            for i, pos in enumerate(positions):
                if i < len(self._rows) and self._rows[i] is not None:
                    self._rows[i].set_target(pos)
        for i, pos in enumerate(positions):
            if i < len(self._rows) and self._rows[i] is not None:
                self._rows[i].update_current(pos)

    @Slot(int, float)
    def _on_target_changed(self, idx: int, value: float) -> None:
        self._bridge.set_target(idx, value)

    def _on_enable_toggled(self, checked: bool) -> None:
        self._bridge.set_enabled(checked)
        if checked:
            self._enable_btn.setText("  Publishing ENABLED — click to disable")
        else:
            self._enable_btn.setText("  Disabled — click to enable publishing")

    def _on_fsm_clicked(self, fsm_id: int) -> None:
        self._bridge.request_fsm(fsm_id)
        self._active_fsm_id = fsm_id
        self._apply_fsm_styles()

    def _apply_fsm_styles(self) -> None:
        for fid, btn in self._fsm_buttons.items():
            inactive, active = self._fsm_styles.get(fid, (self._fsm_styles[0][0], self._fsm_styles[0][1]))
            btn.setStyleSheet(active if fid == self._active_fsm_id else inactive)

    def _on_mode_pr_changed(self, _idx: int) -> None:
        self._bridge.set_mode_pr(self._mode_pr_cb.currentData())

    def _home_all(self) -> None:
        for row in self._rows:
            if row is not None:
                row.set_target(0.0)
        self._bridge.set_targets([0.0] * N_JOINTS)

    def _estop(self) -> None:
        self._bridge.set_enabled(False)
        self._enable_btn.setChecked(False)
        self._enable_btn.setText("  Disabled — click to enable publishing")
        self._home_all()

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

        mm = self._bridge.get_mode_machine()
        self._fsm_id_lbl.setText(f"mode_machine: {mm}" if self._bridge.is_state_received() else "mode_machine: ---")

    def closeEvent(self, event) -> None:
        self._bridge.set_enabled(False)
        self._bridge.stop()
        event.accept()


# ── Entry point ────────────────────────────────────────────────────────────

def main(args=None) -> None:
    app = QApplication(sys.argv)
    app.setApplicationName("H2 Isaac Sim Joint Jog")
    app.setStyle("Fusion")

    bridge = RosBridge()
    bridge.start()

    window = JogWindow(bridge)
    window.show()

    sys.exit(app.exec())


if __name__ == "__main__":
    main()
