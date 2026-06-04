#!/usr/bin/env python3
"""
H2 LowCmd Control Panel — PySide6 GUI for interactive motor testing.

Publishes topstar_hg/msg/LowCmd to /lowcmd at a configurable rate and
displays live motor state (q, dq, τ_est) from the lowstate topic.

Usage:
    source ~/topstar_ros2/setup.sh
    python3 h2_lowcmd_gui.py
"""
import struct
import sys
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

try:
    from topstar_hg.msg import LowCmd, LowState, MotorCmd
except ImportError:
    print("ERROR: topstar_hg messages not found. Did you source setup.sh?")
    raise

from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton, QDoubleSpinBox, QSpinBox, QCheckBox,
    QTabWidget, QGroupBox, QScrollArea, QStatusBar, QComboBox, QFrame,
)
from PySide6.QtCore import Qt, QTimer, Signal, QObject
from PySide6.QtGui import QFont

# ── Joint metadata ─────────────────────────────────────────────────────────────

H2_NUM_MOTOR = 29

JOINT_NAMES = [
    # Left leg  0-5
    'L_HipPitch', 'L_HipRoll', 'L_HipYaw', 'L_Knee', 'L_AnklePitch', 'L_AnkleRoll',
    # Right leg 6-11
    'R_HipPitch', 'R_HipRoll', 'R_HipYaw', 'R_Knee', 'R_AnklePitch', 'R_AnkleRoll',
    # Torso/Head 12-14
    'WaistYaw', 'HeadYaw', 'HeadPitch',
    # Left arm  15-21
    'L_ShoulderPitch', 'L_ShoulderRoll', 'L_ShoulderYaw', 'L_Elbow',
    'L_WristYaw', 'L_WristPitch', 'L_WristRoll',
    # Right arm 22-28
    'R_ShoulderPitch', 'R_ShoulderRoll', 'R_ShoulderYaw', 'R_Elbow',
    'R_WristYaw', 'R_WristPitch', 'R_WristRoll',
]

JOINT_GROUPS = {
    'Left Leg':   list(range(0, 6)),
    'Right Leg':  list(range(6, 12)),
    'Torso/Head': list(range(12, 15)),
    'Left Arm':   list(range(15, 22)),
    'Right Arm':  list(range(22, 29)),
}

# kp: 100 for legs+waist (0-12), 50 for head+arms (13-28)
_DEFAULT_KP = [100.0] * 13 + [50.0] * 16
_DEFAULT_KD = [1.0] * 29

# ── CRC (ported from example/src/src/common/motor_crc_hg.cpp) ─────────────────

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
    """Fill msg.crc in-place using the same algorithm as motor_crc_hg.cpp."""
    buf = bytearray()
    buf += struct.pack('BB2x', msg.mode_pr, msg.mode_machine)
    for m in msg.motor_cmd:
        buf += struct.pack('=B3xfffffI', m.mode, m.q, m.dq, m.tau, m.kp, m.kd, m.reserve)
    buf += struct.pack('4I', *list(msg.reserve))
    msg.crc = _crc32_core(bytes(buf))


# ── ROS2 bridge ────────────────────────────────────────────────────────────────

class RosBridge(QObject):
    """Owns the rclpy node; emits Qt signals for received messages."""
    state_received = Signal(object)  # LowState

    def __init__(self):
        super().__init__()
        rclpy.init()
        self._node = _BridgeNode(self)
        self._thread = threading.Thread(target=rclpy.spin, args=(self._node,), daemon=True)
        self._thread.start()

    @property
    def mode_machine(self) -> int:
        return self._node.last_mode_machine

    def publish(self, msg: LowCmd) -> None:
        self._node.publisher.publish(msg)

    def shutdown(self) -> None:
        self._node.destroy_node()
        try:
            rclpy.try_shutdown()
        except Exception:
            pass


class _BridgeNode(Node):
    def __init__(self, bridge: RosBridge):
        super().__init__('h2_lowcmd_gui')
        self._bridge = bridge
        self.last_mode_machine: int = 0
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(LowState, 'lowstate', self._on_state, qos)
        self.publisher = self.create_publisher(LowCmd, '/lowcmd', 10)

    def _on_state(self, msg: LowState) -> None:
        self.last_mode_machine = int(msg.mode_machine)
        self._bridge.state_received.emit(msg)


# ── Layout dimensions (shared between header and rows) ─────────────────────────

_W = {
    'name': 130, 'en': 34,
    'q': 82, 'dq': 74, 'tau': 74, 'kp': 74, 'kd': 66,
    'sep': 8,
    'qs': 70, 'dqs': 70, 'taus': 70,
}
_MONO8 = QFont('Monospace', 8)


def _header_row() -> QWidget:
    cols = [
        ('Joint',    _W['name']),
        ('En',       _W['en']),
        ('q_cmd',    _W['q']),
        ('dq_cmd',   _W['dq']),
        ('τ_ff',     _W['tau']),
        ('kp',       _W['kp']),
        ('kd',       _W['kd']),
        ('',         _W['sep']),
        ('q_state',  _W['qs']),
        ('dq_state', _W['dqs']),
        ('τ_est',    _W['taus']),
    ]
    w = QWidget()
    w.setStyleSheet('background:#dde4ee;')
    lay = QHBoxLayout(w)
    lay.setContentsMargins(2, 1, 2, 1)
    lay.setSpacing(4)
    for text, width in cols:
        lbl = QLabel(f'<b>{text}</b>' if text else '')
        lbl.setFixedWidth(width)
        lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        lbl.setFont(_MONO8)
        lay.addWidget(lbl)
    lay.addStretch()
    return w


# ── Per-motor row widget ───────────────────────────────────────────────────────

class MotorRow(QWidget):
    def __init__(self, joint_idx: int, parent=None):
        super().__init__(parent)
        self.joint_idx = joint_idx

        lay = QHBoxLayout(self)
        lay.setContentsMargins(2, 1, 2, 1)
        lay.setSpacing(4)

        # Joint name
        name = QLabel(JOINT_NAMES[joint_idx])
        name.setFixedWidth(_W['name'])
        name.setFont(_MONO8)
        lay.addWidget(name)

        # Enable checkbox
        self.en = QCheckBox()
        self.en.setChecked(True)
        self.en.setFixedWidth(_W['en'])
        self.en.setToolTip('Enable motor (mode=1/0)')
        lay.addWidget(self.en)

        # Command spin boxes
        self.q_spin   = self._spin(-6.28,  6.28,   0.0,                    4, _W['q'],   'Target position [rad]')
        self.dq_spin  = self._spin(-20.0,  20.0,   0.0,                    3, _W['dq'],  'Target velocity [rad/s]')
        self.tau_spin = self._spin(-200.0, 200.0,  0.0,                    2, _W['tau'], 'Feedforward torque [Nm]')
        self.kp_spin  = self._spin(0.0,    1000.0, _DEFAULT_KP[joint_idx], 1, _W['kp'],  'Position gain')
        self.kd_spin  = self._spin(0.0,    100.0,  _DEFAULT_KD[joint_idx], 2, _W['kd'],  'Velocity gain')
        for w in (self.q_spin, self.dq_spin, self.tau_spin, self.kp_spin, self.kd_spin):
            lay.addWidget(w)

        # Visual separator between cmd and state
        sep = QFrame()
        sep.setFrameShape(QFrame.Shape.VLine)
        sep.setFrameShadow(QFrame.Shadow.Sunken)
        sep.setFixedWidth(_W['sep'])
        lay.addWidget(sep)

        # State display labels (read-only)
        self.q_s   = self._state_lbl(_W['qs'])
        self.dq_s  = self._state_lbl(_W['dqs'])
        self.tau_s = self._state_lbl(_W['taus'])
        for w in (self.q_s, self.dq_s, self.tau_s):
            lay.addWidget(w)

        lay.addStretch()

    def _spin(self, lo, hi, val, decimals, width, tip) -> QDoubleSpinBox:
        s = QDoubleSpinBox()
        s.setRange(lo, hi)
        s.setValue(val)
        s.setDecimals(decimals)
        s.setSingleStep(0.01)
        s.setFixedWidth(width)
        s.setToolTip(tip)
        return s

    def _state_lbl(self, width) -> QLabel:
        lbl = QLabel('--')
        lbl.setFixedWidth(width)
        lbl.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        lbl.setFont(_MONO8)
        lbl.setStyleSheet('color:#1a4a99;')
        return lbl

    def get_motor_cmd(self) -> MotorCmd:
        return MotorCmd(
            mode=1 if self.en.isChecked() else 0,
            q=float(self.q_spin.value()),
            dq=float(self.dq_spin.value()),
            tau=float(self.tau_spin.value()),
            kp=float(self.kp_spin.value()),
            kd=float(self.kd_spin.value()),
            reserve=0,
        )

    def update_state(self, q: float, dq: float, tau_est: float) -> None:
        self.q_s.setText(f'{q:+.4f}')
        self.dq_s.setText(f'{dq:+.3f}')
        self.tau_s.setText(f'{tau_est:+.2f}')

    def zero(self):
        """Set q, dq, τ_ff to 0 while preserving kp/kd."""
        self.q_spin.setValue(0.0)
        self.dq_spin.setValue(0.0)
        self.tau_spin.setValue(0.0)

    def set_enable(self, v: bool):
        self.en.setChecked(v)

    def reset_gains(self):
        self.kp_spin.setValue(_DEFAULT_KP[self.joint_idx])
        self.kd_spin.setValue(_DEFAULT_KD[self.joint_idx])


# ── Joint group tab ────────────────────────────────────────────────────────────

class GroupTab(QWidget):
    def __init__(self, joint_indices: list, parent=None):
        super().__init__(parent)
        self.rows: dict[int, MotorRow] = {}

        outer = QVBoxLayout(self)
        outer.setContentsMargins(4, 4, 4, 4)
        outer.setSpacing(4)

        # Per-group action buttons
        bar = QHBoxLayout()
        bar.setSpacing(6)
        actions = [
            ('Zero q/dq/τ',    self._zero_all),
            ('Enable All',     lambda: self._set_enable(True)),
            ('Disable All',    lambda: self._set_enable(False)),
            ('Reset Gains',    self._reset_gains),
        ]
        for label, fn in actions:
            btn = QPushButton(label)
            btn.setFixedHeight(26)
            btn.clicked.connect(fn)
            bar.addWidget(btn)
        bar.addStretch()
        outer.addLayout(bar)

        # Scroll area: header + motor rows
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        content = QWidget()
        vbox = QVBoxLayout(content)
        vbox.setContentsMargins(0, 0, 0, 0)
        vbox.setSpacing(1)

        vbox.addWidget(_header_row())

        for idx in joint_indices:
            row = MotorRow(idx)
            self.rows[idx] = row
            # Alternate row background for readability
            if joint_indices.index(idx) % 2 == 1:
                row.setStyleSheet('background:#f4f7fb;')
            vbox.addWidget(row)

        vbox.addStretch()
        scroll.setWidget(content)
        outer.addWidget(scroll)

    def _zero_all(self):
        for r in self.rows.values():
            r.zero()

    def _set_enable(self, v: bool):
        for r in self.rows.values():
            r.set_enable(v)

    def _reset_gains(self):
        for r in self.rows.values():
            r.reset_gains()


# ── Main window ────────────────────────────────────────────────────────────────

class H2ControlPanel(QMainWindow):
    def __init__(self, bridge: RosBridge):
        super().__init__()
        self.bridge = bridge
        self._pub_count = 0
        self._mode_machine = 0

        self.setWindowTitle('H2 LowCmd Control Panel')
        self.setMinimumSize(1040, 660)

        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setContentsMargins(6, 6, 6, 4)
        root.setSpacing(6)

        # ── Global controls ──────────────────────────────────────────────
        gb = QGroupBox('Global Controls')
        hb = QHBoxLayout(gb)
        hb.setSpacing(10)

        # mode_pr
        hb.addWidget(QLabel('mode_pr:'))
        self.mode_pr = QComboBox()
        self.mode_pr.addItem('PR (0)', 0)
        self.mode_pr.addItem('AB (1)', 1)
        self.mode_pr.setFixedWidth(80)
        self.mode_pr.setToolTip('PR=pitch-roll virtual joints, AB=physical ankle joints')
        hb.addWidget(self.mode_pr)

        # mode_machine (read-only — echoed from lowstate)
        hb.addWidget(QLabel('mode_machine:'))
        self.mach_lbl = QLabel('--')
        self.mach_lbl.setFont(QFont('Monospace', 9))
        self.mach_lbl.setFixedWidth(36)
        self.mach_lbl.setToolTip('Current FSM mode from lowstate (read-only)')
        hb.addWidget(self.mach_lbl)

        # Publish rate
        hb.addWidget(QLabel('Rate (Hz):'))
        self.rate = QSpinBox()
        self.rate.setRange(1, 500)
        self.rate.setValue(10)
        self.rate.setFixedWidth(60)
        hb.addWidget(self.rate)

        # Start/stop publish toggle
        self.pub_btn = QPushButton('▶ Start Publishing')
        self.pub_btn.setCheckable(True)
        self.pub_btn.setFixedWidth(155)
        self.pub_btn.clicked.connect(self._toggle_pub)
        hb.addWidget(self.pub_btn)

        # Send once button
        once_btn = QPushButton('Send Once')
        once_btn.setFixedWidth(80)
        once_btn.clicked.connect(self._send_once)
        hb.addWidget(once_btn)

        # E-STOP
        estop = QPushButton('⛔ E-STOP')
        estop.setStyleSheet('background:#b82010;color:white;font-weight:bold;')
        estop.setFixedWidth(100)
        estop.setToolTip('Stop publishing and send a zero/disable command immediately')
        estop.clicked.connect(self._estop)
        hb.addWidget(estop)

        hb.addStretch()
        root.addWidget(gb)

        # ── Joint group tabs ─────────────────────────────────────────────
        self.tabs = QTabWidget()
        self.group_tabs: dict[str, GroupTab] = {}
        for name, indices in JOINT_GROUPS.items():
            tab = GroupTab(indices)
            self.group_tabs[name] = tab
            self.tabs.addTab(tab, name)
        root.addWidget(self.tabs, stretch=1)

        # ── Status bar ───────────────────────────────────────────────────
        sb = QStatusBar()
        self.setStatusBar(sb)
        self._lbl_ros = QLabel('ROS: waiting…')
        self._lbl_pub = QLabel('Pub: idle')
        sb.addPermanentWidget(self._lbl_ros)
        sb.addPermanentWidget(QLabel('  '))
        sb.addPermanentWidget(self._lbl_pub)

        # ── Publish timer ────────────────────────────────────────────────
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._send)
        self.rate.valueChanged.connect(self._on_rate_changed)

        # Wire ROS2 signal
        bridge.state_received.connect(self._on_state)

    # ── Helpers ──────────────────────────────────────────────────────────────

    def _build_lowcmd(self) -> LowCmd:
        msg = LowCmd()
        msg.mode_pr = self.mode_pr.currentData()
        msg.mode_machine = self._mode_machine
        for tab in self.group_tabs.values():
            for idx, row in tab.rows.items():
                msg.motor_cmd[idx] = row.get_motor_cmd()
        compute_crc(msg)
        return msg

    def _send(self):
        self.bridge.publish(self._build_lowcmd())
        self._pub_count += 1
        self._lbl_pub.setText(f'Pub: #{self._pub_count}  @ {self.rate.value()} Hz')

    def _send_once(self):
        self.bridge.publish(self._build_lowcmd())
        self._pub_count += 1
        self._lbl_pub.setText(f'Pub: #{self._pub_count} (once)')

    # ── Slots ─────────────────────────────────────────────────────────────────

    def _on_rate_changed(self, hz: int):
        if self._timer.isActive():
            self._timer.setInterval(max(2, 1000 // hz))

    def _toggle_pub(self, checked: bool):
        if checked:
            self._timer.start(max(2, 1000 // self.rate.value()))
            self.pub_btn.setText('⏹ Stop Publishing')
            self._lbl_pub.setText('Pub: running…')
        else:
            self._timer.stop()
            self.pub_btn.setText('▶ Start Publishing')
            self._lbl_pub.setText('Pub: idle')

    def _estop(self):
        self._timer.stop()
        self.pub_btn.setChecked(False)
        self.pub_btn.setText('▶ Start Publishing')

        for tab in self.group_tabs.values():
            for row in tab.rows.values():
                row.zero()
                row.set_enable(False)

        # Publish one zero/disabled command immediately
        msg = LowCmd()
        msg.mode_pr = 0
        msg.mode_machine = self._mode_machine
        for i in range(H2_NUM_MOTOR):
            msg.motor_cmd[i] = MotorCmd(mode=0, q=0., dq=0., tau=0., kp=0., kd=0., reserve=0)
        compute_crc(msg)
        self.bridge.publish(msg)

        self._lbl_pub.setText('Pub: STOPPED')
        self.statusBar().showMessage('E-STOP: all motors disabled', 5000)

    def _on_state(self, msg: LowState):
        self._mode_machine = int(msg.mode_machine)
        self.mach_lbl.setText(str(self._mode_machine))
        self._lbl_ros.setText('ROS: ✓ connected')
        for tab in self.group_tabs.values():
            for idx, row in tab.rows.items():
                if idx < len(msg.motor_state):
                    ms = msg.motor_state[idx]
                    row.update_state(float(ms.q), float(ms.dq), float(ms.tau_est))

    def closeEvent(self, event):
        self._timer.stop()
        self.bridge.shutdown()
        event.accept()


# ── Entry point ────────────────────────────────────────────────────────────────

def main():
    app = QApplication(sys.argv)
    app.setStyle('Fusion')
    bridge = RosBridge()
    win = H2ControlPanel(bridge)
    win.show()
    sys.exit(app.exec())


if __name__ == '__main__':
    main()
