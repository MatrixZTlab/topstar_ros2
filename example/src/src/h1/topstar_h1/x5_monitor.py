#!/usr/bin/env python3
"""Dual X5 arm monitor and jog GUI (PySide6).

Connects to up to two X5 robot arms via the xapi vendor library, displays
live joint angles, allows per-joint jogging, and records/replays joint
angle snapshots.

Usage (after sourcing workspaces):
    ros2 run topstar_ros2_example x5_monitor --robot_ip 192.168.1.10 192.168.1.11
"""
import argparse
import json
import sys
import numpy as np
import time
from datetime import datetime

from PySide6.QtGui import QKeySequence, QShortcut
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout,
    QHBoxLayout, QPushButton, QLabel, QTextEdit, QSizePolicy,
    QGroupBox, QGridLayout, QComboBox, QTableWidget, QTableWidgetItem, QHeaderView, QSpacerItem, QSlider, QFileDialog,
    QMessageBox, QDialog
)
from PySide6.QtCore import (
    Qt, QDateTime, QThread, Signal, Slot, QMutex, QMutexLocker
)
import xapi.api as x5
from topstar_h1.vendor.topstar.topstar_xapi import TC
from topstar_h1.vendor.topstar.precise_sleep import precise_wait


Mode = {x5.SYSTEM_MODE_MANUAL: "Manual",
        x5.SYSTEM_MODE_AUTO: "Auto",
        x5.SYSTEM_MODE_DEBUG: "Debug",
        x5.SYSTEM_MODE_AUTOCMD: "AutoCmd"}

joint_limit = np.array([[[-150, 150], [-90, 25], [-150, 150], [-103, 25], [-165, 165],
                         [-88, 25], [-170, 170], [0, 95], [-10, 450]],
                        [[-150, 150], [-90, 25], [-150, 150], [-103, 25], [-165, 165],
                         [-88, 25], [-170, 170], [-90, 90], [-40, 28]]])

speed_ratio = 0.1

class RobotConnection(QThread):
    """Background thread for robot connection and status monitoring"""
    connected = Signal()
    disconnected = Signal()
    alarm_changed = Signal(bool)
    servo_changed = Signal(bool)
    mode_changed = Signal(str)
    remote_changed = Signal(bool)
    log_message = Signal(str)
    joint_changed = Signal(list)

    def __init__(self, robot_id: int, robot_ip: str, parent=None):
        super().__init__(parent)
        self.robot_id = robot_id
        self.robot_ip = robot_ip
        self.is_connected = False
        self.is_alarm = False
        self.is_servo_enabled = False
        self.current_mode = x5.SYSTEM_MODE_AUTOCMD
        self.is_remote = False
        self._stop_monitoring = False
        self.handle = None
        self.stop = False
        self.current_joint = [0.0 for _ in range(9)]
        self.delta_joint = [0.0 for _ in range(9)]
        self.target_joint = [0.0 for _ in range(9)]
        self.jog_step = 1.0  # Degrees per jog press
        self.max_speed = 10.0
        self.cmd_dt = 0.020
        self.gain: int = 5
        self.vel = 100
        self.acc = 100

        # Motion abort flag (protected by mutex for thread safety)
        self._abort_motion = False
        self._motion_mutex = QMutex()

    def connect_robot(self):
        """Initiate robot connection (simulated)"""
        if not self.is_connected:
            try:
                self.log_message.emit(f"Robot {self.robot_id}: Attempting connection...")
                self.handle = x5.connect(self.robot_ip)
                self.log_message.emit(f"Robot {self.robot_id}: 检查系统状态...")
                if not self.check_system_state():
                    return
                self.log_message.emit(f"Robot {self.robot_id}: 初始化系统...")
                if not self.initialize_system():
                    return
                self.is_connected = True
                self._stop_monitoring = False
                self.connected.emit()
                self.log_message.emit(f"Robot {self.robot_id}: Connected successfully")
            except Exception as e:
                error_msg = f"Robot {self.robot_id}: 连接失败: {str(e)}"
                self.log_message.emit(error_msg)

            # Initial status updates
            self.alarm_changed.emit(self.is_alarm)
            self.servo_changed.emit(self.is_servo_enabled)
            self.mode_changed.emit(Mode[self.current_mode])
            self.remote_changed.emit(self.is_remote)
            self.joint_changed.emit(self.current_joint)

    def disconnect_robot(self):
        if self.is_connected:
            self.log_message.emit(f"Robot {self.robot_id}: Attempting disconnection...")
            x5.enable_servo(self.handle, False)
            self.msleep(500)
            self._stop_monitoring = True
            self.is_connected = False
            x5.disconnect(self.handle)
            self.handle = None
            self.disconnected.emit()
            self.log_message.emit(f"Robot {self.robot_id}: Disconnected successfully")
            self.current_joint = [0.0 for _ in range(9)]
            self.joint_changed.emit(self.current_joint)

    def run(self):
        """Thread loop for status monitoring"""
        count = 0
        dt = 0.1
        while not self.stop:
            if self.is_connected and not self._stop_monitoring:
                self.msleep(int(1000*dt))
                system_state = x5.get_system_state(self.handle)
                if system_state.alarm != self.is_alarm:
                    self.is_alarm = system_state.alarm
                    self.alarm_changed.emit(self.is_alarm)
                    errors = x5.get_system_alarm_info(self.handle)
                    for e in errors:
                        self.log_message.emit(f"Robot {self.robot_id}: {e['content']}")
                if system_state.mode != self.current_mode:
                    self.current_mode = system_state.mode
                    self.mode_changed.emit(Mode[self.current_mode])
                if system_state.enable != self.is_servo_enabled:
                    self.is_servo_enabled = bool(system_state.enable)
                    self.servo_changed.emit(self.is_servo_enabled)
                if system_state.enable:
                    count = 0
                else:
                    if count >= 5:
                        count = 0
                        self.disconnect_robot()
                        continue
                    else:
                        count += 1
                if system_state.remote != self.is_remote:
                    self.is_remote = system_state.remote
                    self.remote_changed.emit(self.is_remote)

                c_joint = x5.get_cjoint(self.handle)
                self.current_joint = c_joint.tolist()
                self.joint_changed.emit(self.current_joint)

                delta = np.array(self.delta_joint)
                if np.any(delta != 0):
                    with QMutexLocker(self._motion_mutex):
                        if self._abort_motion:
                            self.delta_joint = [0.0]*9
                            continue

                    self.target_joint += delta * self.max_speed * dt
                    self.target_joint = np.clip(self.target_joint, joint_limit[0, :, 0], joint_limit[0, :, 1])
                    joint = x5.Joint(*self.target_joint[:9])
                    x5.servoj(self.handle, joint, self.cmd_dt, 0, self.gain, self.vel, self.acc)
            else:
                self.msleep(500)  # Idle wait when disconnected

    def check_system_state(self):
        """检查系统状态"""
        try:
            system_state = x5.get_system_state(self.handle)
            if system_state.alarm:
                errors = x5.get_system_alarm_info(self.handle)
                for e in errors:
                    self.log_message.emit(f"Robot {self.robot_id}: {e['content']}")
                self.log_message.emit(f"Robot {self.robot_id}: 检测到报警，尝试复位...")
                x5.reset(self.handle)
                self.msleep(500)

                system_state = x5.get_system_state(self.handle)
                if system_state.alarm:
                    self.log_message.emit(f"Robot {self.robot_id}: 机器人报警未消除")
                    return False

            return True
        except Exception as e:
            self.log_message.emit(f"Robot {self.robot_id}: 状态检查失败: {str(e)}")
            return False

    def initialize_system(self):
        """初始化系统"""
        try:
            system_state = x5.get_system_state(self.handle)
            if system_state.remote:
                x5.set_remote(self.handle, False)
                self.msleep(500)
                self.is_remote = False
            if system_state.mode != x5.SYSTEM_MODE_AUTOCMD:
                x5.set_system_mode(self.handle, x5.SYSTEM_MODE_AUTOCMD)
                self.current_mode = x5.SYSTEM_MODE_AUTOCMD
            if not system_state.enable:
                x5.enable_servo(self.handle, True)
                self.msleep(500)
                self.is_servo_enabled = True
            return True
        except Exception as e:
            self.log_message.emit(f"Robot {self.robot_id}: 系统初始化失败: {str(e)}")
            return False

    def move_to_joint_position(self, positions, time_to_go=2.0):
        """Move robot to target position with abort support"""
        with QMutexLocker(self._motion_mutex):
            self._abort_motion = False

        dest = np.array(positions)
        curr = np.array(self.current_joint)
        dist = np.fabs(curr - dest)
        k = 1.0 / 5.0
        max_dist = np.max(dist)
        v = max_dist / (time_to_go - k)
        if v > self.max_speed * speed_ratio:
            v = self.max_speed * speed_ratio
        print(f"v={v}")
        tc = TC(max_dist, v, v / k)
        t_start = time.monotonic()
        iter_idx = 0

        try:
            while max_dist - tc.progress > 1e-6:
                with QMutexLocker(self._motion_mutex):
                    if self._abort_motion:
                        self.log_message.emit(f"Robot {self.robot_id}: Motion aborted by user")
                        x5.abort(self.handle)
                        return -1  # Return error code for abort

                t_cycle_end = t_start + (iter_idx + 1) * 0.1
                tc.run_cycle()
                ratio = tc.progress / max_dist
                value = (1 - ratio) * curr + ratio * dest
                joint = x5.Joint(*value[:9])
                try:
                    x5.servoj(self.handle, joint, self.cmd_dt, 0, self.gain, self.vel, self.acc)
                except x5.RobException as exc:
                    print(f"错误代码：{exc.error_code}")
                    print(f"错误信息：{exc.error_message}")
                    self.log_message.emit(f"Robot {self.robot_id}: Motion error - {exc.error_message}")
                    raise Exception()

                precise_wait(t_cycle_end)
                iter_idx += 1

            self.log_message.emit(f"Robot {self.robot_id}: Motion completed successfully")
            return 0
        finally:
            with QMutexLocker(self._motion_mutex):
                self._abort_motion = False

    def abort_motion(self):
        """Abort all ongoing motion (thread-safe)"""
        with QMutexLocker(self._motion_mutex):
            self._abort_motion = True
        self.log_message.emit(f"Robot {self.robot_id}: Abort signal sent - stopping motion")


class RecordedDataWindow(QDialog):
    """Window to view, edit, save, load, replay, and delete recorded joint angle snapshots"""
    def __init__(self, parent=None, recorded_data=None, robot_conn=None):
        super().__init__(parent)
        self.setWindowTitle("Recorded Joint Snapshots")
        self.setMinimumSize(1000, 650)
        self.setModal(True)

        self.recorded_data = recorded_data or []
        self.robot_conn = robot_conn
        self.is_replaying = False

        self._setup_ui()
        self._connect_signals()
        self._populate_table()
        self._update_button_states()

    def _setup_ui(self):
        layout = QVBoxLayout(self)

        btn_layout = QHBoxLayout()

        self.save_btn = QPushButton("Save to File")
        self.save_btn.setStyleSheet("""
            QPushButton { background-color: #2196F3; color: white; padding: 8px; }
            QPushButton:hover { background-color: #1976D2; }
        """)
        btn_layout.addWidget(self.save_btn)

        self.load_btn = QPushButton("Load from File")
        self.load_btn.setStyleSheet("""
            QPushButton { background-color: #4CAF50; color: white; padding: 8px; }
            QPushButton:hover { background-color: #45a049; }
        """)
        btn_layout.addWidget(self.load_btn)

        self.replay_btn = QPushButton("Replay Selected Snapshot")
        self.replay_btn.setStyleSheet("""
            QPushButton { background-color: #FF9800; color: white; padding: 8px; }
            QPushButton:hover { background-color: #F57C00; }
            QPushButton:disabled { background-color: #cccccc; }
        """)
        btn_layout.addWidget(self.replay_btn)

        self.abort_btn = QPushButton("Abort Motion")
        self.abort_btn.setStyleSheet("""
            QPushButton { background-color: #DC143C; color: white; padding: 8px; font-weight: bold; }
            QPushButton:hover { background-color: #B22222; }
            QPushButton:disabled { background-color: #cccccc; }
        """)
        self.abort_btn.setEnabled(False)
        btn_layout.addWidget(self.abort_btn)

        self.delete_btn = QPushButton("Delete Selected Row")
        self.delete_btn.setStyleSheet("""
            QPushButton { background-color: #f44336; color: white; padding: 8px; }
            QPushButton:hover { background-color: #d32f2f; }
            QPushButton:disabled { background-color: #cccccc; }
        """)
        btn_layout.addWidget(self.delete_btn)

        self.clear_btn = QPushButton("Clear All Snapshots")
        self.clear_btn.setStyleSheet("""
            QPushButton { background-color: #9C27B0; color: white; padding: 8px; }
            QPushButton:hover { background-color: #7B1FA2; }
            QPushButton:disabled { background-color: #cccccc; }
        """)
        btn_layout.addWidget(self.clear_btn)

        btn_layout.addStretch()
        layout.addLayout(btn_layout)

        self.replay_status = QLabel("Replay Status: Idle")
        self.replay_status.setStyleSheet("""
            QLabel { color: #333; font-style: italic; padding: 4px; }
        """)
        layout.addWidget(self.replay_status)

        self.table = QTableWidget()
        self.table.setColumnCount(11)  # Timestamp + 9 joints + Notes
        headers = ["Timestamp", "J1", "J2", "J3", "J4", "J5", "J6", "J7", "J8", "J9", "Notes"]
        self.table.setHorizontalHeaderLabels(headers)

        self.table.setStyleSheet("""
            QTableWidget { border: 1px solid #ccc; padding: 5px; }
            QHeaderView::section { background-color: #f0f0f0; font-weight: bold; }
            QTableWidgetItem { text-align: center; }
            QTableWidget::item:selected { background-color: #2196F3; color: white; }
            QTableWidget::item:selected:disabled { background-color: #888888; color: white; }
        """)

        self.table.setEditTriggers(QTableWidget.DoubleClicked | QTableWidget.EditKeyPressed)

        self.table.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeToContents)
        for col in range(1, 10):
            self.table.setColumnWidth(col, 70)
        self.table.horizontalHeader().setSectionResizeMode(10, QHeaderView.Stretch)

        layout.addWidget(self.table)

    def _connect_signals(self):
        self.save_btn.clicked.connect(self._save_to_file)
        self.load_btn.clicked.connect(self._load_from_file)
        self.replay_btn.clicked.connect(self._replay_selected_snapshot)
        self.abort_btn.clicked.connect(self._abort_motion)
        self.delete_btn.clicked.connect(self._delete_selected_row)
        self.clear_btn.clicked.connect(self._clear_all_snapshots)
        self.table.itemSelectionChanged.connect(self._update_button_states)
        self.table.itemChanged.connect(self._handle_table_edit)

    def _populate_table(self):
        """Fill table with recorded snapshots (make timestamp non-editable)"""
        self.table.setRowCount(0)
        self.table.blockSignals(True)

        for idx, entry in enumerate(self.recorded_data):
            self.table.insertRow(idx)

            ts_item = QTableWidgetItem(entry["timestamp"])
            ts_item.setFlags(ts_item.flags() & ~Qt.ItemIsEditable)
            self.table.setItem(idx, 0, ts_item)

            for j in range(9):
                angle_item = QTableWidgetItem(f"{entry['angles'][j]:.2f}")
                angle_item.setData(Qt.UserRole, j)
                self.table.setItem(idx, j+1, angle_item)

            notes_item = QTableWidgetItem(entry["notes"])
            self.table.setItem(idx, 10, notes_item)

        self.table.blockSignals(False)

    def _handle_table_edit(self, item: QTableWidgetItem):
        """Update recorded data when table is edited"""
        row = item.row()
        col = item.column()

        if 0 <= row < len(self.recorded_data):
            if 1 <= col <= 9:
                joint_idx = col - 1
                try:
                    new_angle = float(item.text())
                    clamped_angle = max(-180.0, min(180.0, new_angle))

                    self.recorded_data[row]["angles"][joint_idx] = clamped_angle

                    if new_angle != clamped_angle:
                        self.table.blockSignals(True)
                        item.setText(f"{clamped_angle:.2f}")
                        self.table.blockSignals(False)

                    self.parent().add_log_message(f"Updated snapshot {row+1}, Joint {joint_idx+1} to {clamped_angle:.2f}°")

                except ValueError:
                    original_angle = self.recorded_data[row]["angles"][joint_idx]
                    self.table.blockSignals(True)
                    item.setText(f"{original_angle:.2f}")
                    self.table.blockSignals(False)
                    QMessageBox.warning(self, "Invalid Value", "Please enter a valid numeric angle!")

            elif col == 10:
                self.recorded_data[row]["notes"] = item.text()
                self.parent().add_log_message(f"Updated snapshot {row+1} notes: {item.text()}")

    def _update_button_states(self):
        """Enable/disable buttons based on selection and data"""
        has_data = len(self.recorded_data) > 0
        has_selection = len(self.table.selectedItems()) > 0

        self.replay_btn.setEnabled(has_selection and self.robot_conn and self.robot_conn.is_connected and not self.is_replaying)
        self.abort_btn.setEnabled(self.is_replaying)
        self.delete_btn.setEnabled(has_selection and not self.is_replaying)
        self.clear_btn.setEnabled(has_data and not self.is_replaying)
        self.save_btn.setEnabled(has_data and not self.is_replaying)
        self.load_btn.setEnabled(not self.is_replaying)

    def _save_to_file(self):
        """Save recorded snapshots to JSON file"""
        if not self.recorded_data:
            QMessageBox.warning(self, "Warning", "No recorded snapshots to save!")
            return

        file_path, _ = QFileDialog.getSaveFileName(
            self, "Save Recorded Snapshots",
            f"robot_snapshots_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json",
            "JSON Files (*.json);;All Files (*.*)"
        )

        if file_path:
            try:
                with open(file_path, 'w') as f:
                    json.dump(self.recorded_data, f, indent=4)
                QMessageBox.information(self, "Success", f"Saved {len(self.recorded_data)} snapshots to:\n{file_path}")
                self.parent().add_log_message(f"Recorded snapshots saved to {file_path}")
            except Exception as e:
                QMessageBox.critical(self, "Error", f"Failed to save file:\n{str(e)}")

    def _load_from_file(self):
        """Load recorded snapshots from JSON file"""
        file_path, _ = QFileDialog.getOpenFileName(
            self, "Load Recorded Snapshots", "",
            "JSON Files (*.json);;All Files (*.*)"
        )

        if file_path:
            try:
                with open(file_path, 'r') as f:
                    loaded_data = json.load(f)

                if isinstance(loaded_data, list) and all(isinstance(item, dict) and
                   "timestamp" in item and "angles" in item and len(item["angles"]) == 9
                   for item in loaded_data):
                    self.recorded_data = loaded_data
                    self._populate_table()
                    self._update_button_states()
                    QMessageBox.information(self, "Success", f"Loaded {len(loaded_data)} snapshots from:\n{file_path}")
                    self.parent().add_log_message(f"Recorded snapshots loaded from {file_path}")
                else:
                    QMessageBox.warning(self, "Invalid File", "File contains invalid snapshot data format!")
            except Exception as e:
                QMessageBox.critical(self, "Error", f"Failed to load file:\n{str(e)}")

    def _replay_selected_snapshot(self):
        """Move robot to the selected snapshot's joint angles with abort support"""
        selected_rows = self.table.selectionModel().selectedRows()
        if not selected_rows:
            return

        row = selected_rows[0].row()
        snapshot = self.recorded_data[row]

        if not self.robot_conn or not self.robot_conn.is_connected:
            QMessageBox.warning(self, "Warning", "Robot is not connected!")
            return

        if "robot_id" in snapshot and snapshot["robot_id"] != self.robot_conn.robot_id:
            QMessageBox.question(self, "Mismatch",
                                 f"Snapshot was recorded for Robot {snapshot['robot_id']}, "
                                 f"but connected to Robot {self.robot_conn.robot_id}.")
            return

        self.is_replaying = True
        self._update_button_states()
        self.replay_status.setText(f"Replay Status: Active (Snapshot {row+1})")
        self.replay_status.setStyleSheet("color: #FF9800; font-style: italic; font-weight: bold; padding: 4px;")

        class MotionThread(QThread):
            finished = Signal(int)  # 0 = success, -1 = aborted, 1 = error

            def __init__(self, robot_conn, target_angles):
                super().__init__()
                self.robot_conn = robot_conn
                self.target_angles = target_angles

            def run(self):
                try:
                    result = self.robot_conn.move_to_joint_position(self.target_angles)
                    self.finished.emit(result)
                except Exception:
                    self.finished.emit(1)

        self.motion_thread = MotionThread(self.robot_conn, snapshot["angles"])
        self.motion_thread.finished.connect(self._on_replay_finished)
        self.motion_thread.start()

        self.parent().add_log_message(f"Started replay of snapshot {row+1}: Moving to recorded joint angles")

    def _abort_motion(self):
        """Abort the current replay motion"""
        if self.is_replaying and self.robot_conn:
            self.robot_conn.abort_motion()
            self.replay_status.setText("Replay Status: Aborting...")
            self.replay_status.setStyleSheet("color: #DC143C; font-style: italic; font-weight: bold; padding: 4px;")
            self.parent().add_log_message("User requested motion abort")

    def _on_replay_finished(self, result):
        """Handle replay completion or abortion"""
        self.is_replaying = False
        self._update_button_states()

        if result == 0:
            self.replay_status.setText("Replay Status: Completed Successfully")
            self.replay_status.setStyleSheet("color: #4CAF50; font-style: italic; padding: 4px;")
        elif result == -1:
            self.replay_status.setText("Replay Status: Aborted by User")
            self.replay_status.setStyleSheet("color: #DC143C; font-style: italic; padding: 4px;")
        else:
            self.replay_status.setText("Replay Status: Error Occurred")
            self.replay_status.setStyleSheet("color: #f44336; font-style: italic; font-weight: bold; padding: 4px;")

        self.motion_thread.deleteLater()

    def _delete_selected_row(self):
        """Delete the selected row from recorded data"""
        selected_rows = self.table.selectionModel().selectedRows()
        if not selected_rows:
            return

        row = selected_rows[0].row()
        if QMessageBox.question(self, "Confirm", f"Delete snapshot {row + 1}?") == QMessageBox.Yes:
            del self.recorded_data[row]
            self._populate_table()
            self._update_button_states()
            self.parent().add_log_message(f"Deleted snapshot {row + 1}")

    def _clear_all_snapshots(self):
        """Clear all recorded snapshots"""
        if not self.recorded_data:
            return

        if QMessageBox.question(self, "Confirm", "Delete all recorded snapshots?") == QMessageBox.Yes:
            self.recorded_data.clear()
            self._populate_table()
            self._update_button_states()
            self.parent().add_log_message("Cleared all recorded snapshots")


class RobotControlMainWindow(QMainWindow):
    def __init__(self, robot_ips):
        super().__init__()
        self.setWindowTitle("Dual Robot Control Center")
        self.setMinimumSize(800, 700)

        self.robot0_conn = RobotConnection(0, robot_ips[0], self)
        self.robot1_conn = RobotConnection(1, robot_ips[1], self)

        self.joint_angles = {
            0: [0.0] * 9,
            1: [0.0] * 9
        }
        self.selected_robot = 0
        self.recorded_snapshots = []

        self.status_labels = {
            0: {
                "connection": QLabel("Disconnected"),
                "alarm": QLabel("None"),
                "servo": QLabel("Disabled"),
                "mode": QLabel("AutoCmd"),
                "remote": QLabel("False")
            },
            1: {
                "connection": QLabel("Disconnected"),
                "alarm": QLabel("None"),
                "servo": QLabel("Disabled"),
                "mode": QLabel("AutoCmd"),
                "remote": QLabel("False")
            }
        }

        self.jog_buttons = []

        self._setup_ui()
        self._connect_signals()

        self.add_log_message("Application started - ready to connect to robots")
        self.add_log_message("Press F9 or click 'Take Snapshot' to record joint angles")

        self.robot0_conn.start()
        self.robot1_conn.start()

    def _setup_ui(self):
        """Main layout: top (three panels) + bottom (log window)"""
        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        main_layout = QVBoxLayout(central_widget)
        main_layout.setContentsMargins(15, 15, 15, 15)
        main_layout.setSpacing(15)

        # ---------------------- TOP CONTROLS BAR ----------------------
        top_bar_layout = QHBoxLayout()

        self.snapshot_btn = QPushButton("Take Snapshot (F9)")
        self.snapshot_btn.setStyleSheet("""
            QPushButton { background-color: #4CAF50; color: white; padding: 8px 16px; font-weight: bold; }
            QPushButton:hover { background-color: #45a049; }
            QPushButton:pressed { background-color: #388E3C; }
        """)
        top_bar_layout.addWidget(self.snapshot_btn)

        self.view_snapshots_btn = QPushButton("View Snapshots")
        self.view_snapshots_btn.setStyleSheet("""
            QPushButton { background-color: #2196F3; color: white; padding: 8px 16px; }
            QPushButton:hover { background-color: #1976D2; }
        """)
        top_bar_layout.addWidget(self.view_snapshots_btn)

        top_bar_layout.addStretch()
        main_layout.addLayout(top_bar_layout)

        # ---------------------- TOP SECTION: Three-Panel Layout ----------------------
        top_layout = QHBoxLayout()
        top_layout.setSpacing(15)

        # --- Left Panel: Robot Controls ---
        left_panel = QWidget()
        left_layout = QVBoxLayout(left_panel)
        left_layout.setSpacing(15)

        robot0_group = QGroupBox("Robot 0 Control")
        robot0_layout = QVBoxLayout(robot0_group)
        robot0_layout.setSpacing(10)

        robot0_btn_layout = QHBoxLayout()
        self.robot0_connect_btn = QPushButton("Connect")
        self.robot0_connect_btn.setStyleSheet("""
            QPushButton { background-color: #4CAF50; color: white; padding: 8px; }
            QPushButton:hover { background-color: #45a049; }
            QPushButton:disabled { background-color: #cccccc; }
        """)
        self.robot0_disconnect_btn = QPushButton("Disconnect")
        self.robot0_disconnect_btn.setStyleSheet("""
            QPushButton { background-color: #f44336; color: white; padding: 8px; }
            QPushButton:hover { background-color: #d32f2f; }
            QPushButton:disabled { background-color: #cccccc; }
        """)
        self.robot0_disconnect_btn.setEnabled(False)

        robot0_btn_layout.addWidget(self.robot0_connect_btn)
        robot0_btn_layout.addWidget(self.robot0_disconnect_btn)
        robot0_layout.addLayout(robot0_btn_layout)

        robot0_status_grid = QGridLayout()
        robot0_status_grid.setSpacing(5)
        status_labels = [
            ("Connection:", "connection"),
            ("Alarm:", "alarm"),
            ("Servo:", "servo"),
            ("Mode:", "mode"),
            ("Remote:", "remote"),
        ]

        for row, (label_text, key) in enumerate(status_labels):
            label = QLabel(label_text)
            label.setStyleSheet("font-weight: bold;")
            robot0_status_grid.addWidget(label, row, 0)
            status_label = self.status_labels[0][key]
            status_label.setStyleSheet("padding: 2px 8px; border-radius: 4px;")
            robot0_status_grid.addWidget(status_label, row, 1)

        robot0_layout.addLayout(robot0_status_grid)
        left_layout.addWidget(robot0_group)

        robot1_group = QGroupBox("Robot 1 Control")
        robot1_layout = QVBoxLayout(robot1_group)
        robot1_layout.setSpacing(10)

        robot1_btn_layout = QHBoxLayout()
        self.robot1_connect_btn = QPushButton("Connect")
        self.robot1_connect_btn.setStyleSheet("""
            QPushButton { background-color: #4CAF50; color: white; padding: 8px; }
            QPushButton:hover { background-color: #45a049; }
            QPushButton:disabled { background-color: #cccccc; }
        """)
        self.robot1_disconnect_btn = QPushButton("Disconnect")
        self.robot1_disconnect_btn.setStyleSheet("""
            QPushButton { background-color: #f44336; color: white; padding: 8px; }
            QPushButton:hover { background-color: #d32f2f; }
            QPushButton:disabled { background-color: #cccccc; }
        """)
        self.robot1_disconnect_btn.setEnabled(False)

        robot1_btn_layout.addWidget(self.robot1_connect_btn)
        robot1_btn_layout.addWidget(self.robot1_disconnect_btn)
        robot1_layout.addLayout(robot1_btn_layout)

        robot1_status_grid = QGridLayout()
        robot1_status_grid.setSpacing(5)

        for row, (label_text, key) in enumerate(status_labels):
            label = QLabel(label_text)
            label.setStyleSheet("font-weight: bold;")
            robot1_status_grid.addWidget(label, row, 0)
            status_label = self.status_labels[1][key]
            status_label.setStyleSheet("padding: 2px 8px; border-radius: 4px;")
            robot1_status_grid.addWidget(status_label, row, 1)
        robot1_layout.addLayout(robot1_status_grid)
        left_layout.addWidget(robot1_group)

        left_layout.addStretch()
        top_layout.addWidget(left_panel, stretch=2)

        # --- Middle Panel: Joint Table ---
        middle_panel = QWidget()
        middle_layout = QVBoxLayout(middle_panel)
        middle_layout.setSpacing(10)
        middle_panel.setMinimumWidth(250)

        combo_layout = QHBoxLayout()
        combo_label = QLabel("Display Joints for:")
        combo_label.setStyleSheet("font-weight: bold;")
        self.robot_combo = QComboBox()
        self.robot_combo.addItems(["Robot 0", "Robot 1"])
        self.robot_combo.setStyleSheet("padding: 5px; font-size: 12px;")
        combo_layout.addWidget(combo_label)
        combo_layout.addWidget(self.robot_combo)
        combo_layout.addStretch()
        middle_layout.addLayout(combo_layout)

        self.joint_table = QTableWidget()
        self.joint_table.setRowCount(9)
        self.joint_table.setColumnCount(2)
        self.joint_table.setHorizontalHeaderLabels(["Joint", "Angle (°)"])

        self.joint_table.verticalHeader().setVisible(False)

        row_height = 28
        self.joint_table.setRowHeight(-1, row_height)

        min_table_height = (row_height * 9) + 60
        self.joint_table.setMinimumHeight(min_table_height)
        self.joint_table.setMaximumHeight(min_table_height)

        self.joint_table.setStyleSheet("""
                    QTableWidget { border: 1px solid #ccc; padding: 5px; }
                    QHeaderView::section { background-color: #f0f0f0; font-weight: bold; height: 30px; }
                    QTableWidgetItem { text-align: center; font-size: 12px; }
                """)

        for i in range(9):
            joint_item = QTableWidgetItem(f"J{i + 1}")
            joint_item.setFlags(Qt.ItemIsSelectable | Qt.ItemIsEnabled)
            joint_item.setTextAlignment(Qt.AlignHCenter | Qt.AlignVCenter)
            self.joint_table.setItem(i, 0, joint_item)
            angle_item = QTableWidgetItem(f"{0.0:.3f}")
            angle_item.setFlags(Qt.ItemIsSelectable | Qt.ItemIsEnabled)
            self.joint_table.setItem(i, 1, angle_item)

        self.joint_table.setColumnWidth(0, 70)
        self.joint_table.horizontalHeader().setSectionResizeMode(1, QHeaderView.Stretch)
        self.joint_table.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)

        middle_layout.addWidget(self.joint_table)
        top_layout.addWidget(middle_panel, stretch=1)

        # --- Right Panel: Jog Controls ---
        right_panel = QWidget()
        right_layout = QVBoxLayout(right_panel)
        right_layout.setSpacing(8)
        right_layout.setAlignment(Qt.AlignTop)
        right_panel.setMinimumWidth(200)

        jog_header = QLabel("Jog Controls")
        jog_header.setStyleSheet("font-weight: bold; font-size: 14px;")
        jog_header.setAlignment(Qt.AlignCenter)
        right_layout.addWidget(jog_header)
        right_layout.addSpacerItem(QSpacerItem(20, 8, QSizePolicy.Minimum, QSizePolicy.Fixed))

        speed_container = QWidget()
        speed_layout = QVBoxLayout(speed_container)
        speed_layout.setSpacing(5)

        speed_label = QLabel("Jog Speed:")
        speed_label.setStyleSheet("font-size: 12px; font-weight: bold;")
        speed_label.setAlignment(Qt.AlignCenter)
        speed_layout.addWidget(speed_label)

        self.jog_speed_slider = QSlider(Qt.Horizontal)
        self.jog_speed_slider.setRange(1, 100)
        self.jog_speed_slider.setValue(10)
        self.jog_speed_slider.setStyleSheet("""
                    QSlider::groove:horizontal {
                        border: 1px solid #bbb;
                        background: white;
                        height: 8px;
                        border-radius: 4px;
                    }
                    QSlider::handle:horizontal {
                        background: #4CAF50;
                        border: 1px solid #5c5c5c;
                        width: 18px;
                        margin: -5px 0;
                        border-radius: 9px;
                    }
                """)
        speed_layout.addWidget(self.jog_speed_slider)

        self.jog_speed_display = QLabel("10%")
        self.jog_speed_display.setStyleSheet("""
                    QLabel { background-color: #f0f0f0; padding: 4px; border-radius: 4px;
                             font-size: 12px; font-weight: bold; }
                """)
        self.jog_speed_display.setAlignment(Qt.AlignCenter)
        speed_layout.addWidget(self.jog_speed_display)

        right_layout.addWidget(speed_container)
        right_layout.addSpacerItem(QSpacerItem(20, 10, QSizePolicy.Minimum, QSizePolicy.Fixed))

        for joint_idx in range(9):
            joint_layout = QHBoxLayout()
            joint_layout.setSpacing(5)

            dec_btn = QPushButton("-")
            dec_btn.setFixedSize(50, 30)
            dec_btn.setStyleSheet("""
                        QPushButton { background-color: #f44336; color: white; font-weight: bold; }
                        QPushButton:hover { background-color: #d32f2f; }
                        QPushButton:disabled { background-color: #cccccc; }
                    """)
            dec_btn.setEnabled(False)
            dec_btn.pressed.connect(lambda idx=joint_idx: self._on_jog_decrement(idx))
            dec_btn.released.connect(lambda idx=joint_idx: self._on_jog_release(idx))

            joint_label = QLabel(f"J{joint_idx + 1}")
            joint_label.setFixedWidth(30)
            joint_label.setAlignment(Qt.AlignCenter)

            inc_btn = QPushButton("+")
            inc_btn.setFixedSize(50, 30)
            inc_btn.setStyleSheet("""
                        QPushButton { background-color: #4CAF50; color: white; font-weight: bold; }
                        QPushButton:hover { background-color: #45a049; }
                        QPushButton:disabled { background-color: #cccccc; }
                    """)
            inc_btn.setEnabled(False)
            inc_btn.pressed.connect(lambda idx=joint_idx: self._on_jog_increment(idx))
            inc_btn.released.connect(lambda idx=joint_idx: self._on_jog_release(idx))

            joint_layout.addWidget(dec_btn)
            joint_layout.addWidget(joint_label)
            joint_layout.addWidget(inc_btn)
            right_layout.addLayout(joint_layout)

            self.jog_buttons.append((dec_btn, inc_btn))

        right_layout.addStretch()
        top_layout.addWidget(right_panel, stretch=0)

        main_layout.addLayout(top_layout, stretch=3)

        # ---------------------- BOTTOM SECTION: Log Window ----------------------
        log_container = QWidget()
        log_layout = QVBoxLayout(log_container)
        log_layout.setSpacing(5)

        log_header_layout = QHBoxLayout()
        log_label = QLabel("System Log:")
        log_label.setStyleSheet("font-size: 12px; font-weight: bold;")
        log_header_layout.addWidget(log_label)
        log_header_layout.addStretch()
        log_layout.addLayout(log_header_layout)

        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setStyleSheet("""
                    QTextEdit { border: 1px solid #ccc; padding: 8px;
                                background-color: #f8f8f8; font-family: monospace; }
                """)
        self.log_text.setMinimumHeight(150)
        log_layout.addWidget(self.log_text)

        main_layout.addWidget(log_container, stretch=1)

        # ---------------------- Keyboard Shortcuts ----------------------
        self.snapshot_shortcut = QShortcut(QKeySequence("F9"), self)

    def _connect_signals(self):
        """Connect robot thread signals to UI update slots"""
        self.robot0_connect_btn.clicked.connect(self.robot0_conn.connect_robot)
        self.robot0_disconnect_btn.clicked.connect(self.robot0_conn.disconnect_robot)
        self.robot0_conn.connected.connect(lambda: self._update_connection_status(0, True))
        self.robot0_conn.disconnected.connect(lambda: self._update_connection_status(0, False))
        self.robot0_conn.alarm_changed.connect(lambda state: self._update_alarm_status(0, state))
        self.robot0_conn.servo_changed.connect(lambda state: self._update_servo_status(0, state))
        self.robot0_conn.mode_changed.connect(lambda mode: self._update_mode_status(0, mode))
        self.robot0_conn.remote_changed.connect(lambda state: self._update_remote_status(0, state))
        self.robot0_conn.log_message.connect(self.add_log_message)
        self.robot0_conn.joint_changed.connect(lambda angles: self._update_joint(0, angles))

        self.robot1_connect_btn.clicked.connect(self.robot1_conn.connect_robot)
        self.robot1_disconnect_btn.clicked.connect(self.robot1_conn.disconnect_robot)
        self.robot1_conn.connected.connect(lambda: self._update_connection_status(1, True))
        self.robot1_conn.disconnected.connect(lambda: self._update_connection_status(1, False))
        self.robot1_conn.alarm_changed.connect(lambda state: self._update_alarm_status(1, state))
        self.robot1_conn.servo_changed.connect(lambda state: self._update_servo_status(1, state))
        self.robot1_conn.mode_changed.connect(lambda mode: self._update_mode_status(1, mode))
        self.robot1_conn.remote_changed.connect(lambda state: self._update_remote_status(1, state))
        self.robot1_conn.log_message.connect(self.add_log_message)
        self.robot1_conn.joint_changed.connect(lambda angles: self._update_joint(1, angles))

        self.robot_combo.currentIndexChanged.connect(self._on_robot_selection_changed)
        self.jog_speed_slider.valueChanged.connect(self._on_jog_speed_changed)
        self.snapshot_btn.clicked.connect(self._take_snapshot)
        self.snapshot_shortcut.activated.connect(self._take_snapshot)
        self.view_snapshots_btn.clicked.connect(self._open_snapshots_window)

    def _take_snapshot(self):
        """Take a single snapshot of current joint angles"""
        if not ((self.selected_robot == 0 and self.robot0_conn.is_connected) or
                (self.selected_robot == 1 and self.robot1_conn.is_connected)):
            QMessageBox.warning(self, "Warning", "Selected robot is not connected!")
            return

        timestamp = QDateTime.currentDateTime().toString("yyyy-MM-dd HH:mm:ss.zzz")
        angles = self.joint_angles[self.selected_robot].copy()
        snapshot = {
            "timestamp": timestamp,
            "robot_id": self.selected_robot,
            "angles": angles,
            "notes": f"Snapshot {len(self.recorded_snapshots) + 1}"
        }
        self.recorded_snapshots.append(snapshot)
        self.add_log_message(f"Recorded snapshot {len(self.recorded_snapshots)} for Robot {self.selected_robot}")
        QMessageBox.information(self, "Snapshot Taken",
                                f"Successfully recorded snapshot {len(self.recorded_snapshots)}!")

    def _open_snapshots_window(self):
        """Open window to manage recorded snapshots"""
        robot_conn = self.robot0_conn if self.selected_robot == 0 else self.robot1_conn
        win = RecordedDataWindow(self, self.recorded_snapshots, robot_conn)
        win.exec()
        self.recorded_snapshots = win.recorded_data

    @Slot(int)
    def _on_jog_speed_changed(self, value: int):
        global speed_ratio
        speed_ratio = value * 0.01
        self.jog_speed_display.setText(f"{value:.1f}%")

    @Slot(int)
    def _on_jog_increment(self, joint_idx: int):
        """Handle jog increment button press"""
        global speed_ratio
        if self.selected_robot == 0:
            self.robot0_conn.target_joint = self.robot0_conn.current_joint.copy()
            self.robot0_conn.delta_joint[joint_idx] = speed_ratio
        else:
            self.robot1_conn.target_joint = self.robot1_conn.current_joint.copy()
            self.robot1_conn.delta_joint[joint_idx] = speed_ratio

    @Slot(int)
    def _on_jog_decrement(self, joint_idx: int):
        """Handle jog decrement button press"""
        global speed_ratio
        if self.selected_robot == 0:
            self.robot0_conn.target_joint = self.robot0_conn.current_joint.copy()
            self.robot0_conn.delta_joint[joint_idx] = -speed_ratio
        else:
            self.robot1_conn.target_joint = self.robot1_conn.current_joint.copy()
            self.robot1_conn.delta_joint[joint_idx] = -speed_ratio

    @Slot(int)
    def _on_jog_release(self, joint_idx: int):
        """Handle jog button release"""
        if self.selected_robot == 0:
            self.robot0_conn.delta_joint[joint_idx] = 0.0
        else:
            self.robot1_conn.delta_joint[joint_idx] = 0.0

    def _update_jog_buttons_state(self):
        """Enable/disable jog buttons based on selected robot's connection status"""
        is_connected = (self.selected_robot == 0 and self.robot0_conn.is_connected) or \
                       (self.selected_robot == 1 and self.robot1_conn.is_connected)
        for dec_btn, inc_btn in self.jog_buttons:
            dec_btn.setEnabled(is_connected)
            inc_btn.setEnabled(is_connected)
        self.jog_speed_slider.setEnabled(is_connected)

    @Slot(int, list)
    def _update_joint(self, robot_id: int, angles: list):
        """Update stored joint angles and refresh table if robot is selected"""
        self.joint_angles[robot_id] = angles
        if robot_id == self.selected_robot:
            self._refresh_joint_table()

    @Slot(int)
    def _on_robot_selection_changed(self, index: int):
        """Handle robot selection change"""
        self.selected_robot = index
        self._refresh_joint_table()
        self._update_jog_buttons_state()
        self.add_log_message(f"Displaying joint angles for Robot {self.selected_robot}")

    def _refresh_joint_table(self):
        """Update joint table with current selected robot's angles"""
        angles = self.joint_angles[self.selected_robot]
        is_connected = (self.selected_robot == 0 and self.robot0_conn.is_connected) or \
                       (self.selected_robot == 1 and self.robot1_conn.is_connected)
        for i in range(9):
            angle_item = QTableWidgetItem(f"{angles[i]:.3f}")
            angle_item.setFlags(Qt.ItemIsSelectable | Qt.ItemIsEnabled)
            angle_item.setBackground(Qt.lightGray if not is_connected else Qt.white)
            self.joint_table.setItem(i, 1, angle_item)

    @Slot(int, bool)
    def _update_connection_status(self, robot_id: int, is_connected: bool):
        """Update connection status display for a robot"""
        label = self.status_labels[robot_id]["connection"]
        if is_connected:
            label.setText("Connected")
            label.setStyleSheet("color: white; background-color: #4CAF50; padding: 2px 8px; border-radius: 4px;")
        else:
            label.setText("Disconnected")
            label.setStyleSheet("color: white; background-color: #f44336; padding: 2px 8px; border-radius: 4px;")

        if robot_id == 0:
            self.robot0_connect_btn.setEnabled(not is_connected)
            self.robot0_disconnect_btn.setEnabled(is_connected)
        else:
            self.robot1_connect_btn.setEnabled(not is_connected)
            self.robot1_disconnect_btn.setEnabled(is_connected)

        if robot_id == self.selected_robot:
            self._update_jog_buttons_state()
            self._refresh_joint_table()

    @Slot(int, bool)
    def _update_alarm_status(self, robot_id: int, is_alarm: bool):
        """Update alarm status display for a robot"""
        label = self.status_labels[robot_id]["alarm"]
        if is_alarm:
            label.setText("Yes")
            label.setStyleSheet("color: white; background-color: #f44336; padding: 2px 8px; border-radius: 4px;")
        else:
            label.setText("None")
            label.setStyleSheet("color: white; background-color: #4CAF50; padding: 2px 8px; border-radius: 4px;")

    @Slot(int, bool)
    def _update_servo_status(self, robot_id: int, is_enabled: bool):
        """Update servo status display for a robot"""
        label = self.status_labels[robot_id]["servo"]
        if is_enabled:
            label.setText("Enabled")
            label.setStyleSheet("color: white; background-color: #4CAF50; padding: 2px 8px; border-radius: 4px;")
        else:
            label.setText("Disabled")
            label.setStyleSheet("color: white; background-color: #f44336; padding: 2px 8px; border-radius: 4px;")

    @Slot(int, bool)
    def _update_remote_status(self, robot_id: int, is_enabled: bool):
        """Update remote status display for a robot"""
        label = self.status_labels[robot_id]["remote"]
        if is_enabled:
            label.setText("True")
            label.setStyleSheet("color: white; background-color: #f44336; padding: 2px 8px; border-radius: 4px;")
        else:
            label.setText("False")
            label.setStyleSheet("color: white; background-color: #4CAF50; padding: 2px 8px; border-radius: 4px;")

    @Slot(int, str)
    def _update_mode_status(self, robot_id: int, mode: str):
        """Update operation mode display for a robot"""
        label = self.status_labels[robot_id]["mode"]
        label.setText(mode)
        label.setStyleSheet("color: black; background-color: #e0e0e0; padding: 2px 8px; border-radius: 4px;")

    @Slot(str)
    def add_log_message(self, message: str):
        """Add timestamped message to log window"""
        timestamp = QDateTime.currentDateTime().toString("yyyy-MM-dd HH:mm:ss")
        log_entry = f"[{timestamp}] {message}\n"
        self.log_text.append(log_entry)
        scroll_bar = self.log_text.verticalScrollBar()
        scroll_bar.setValue(scroll_bar.maximum())

    def closeEvent(self, event):
        """Clean up threads before closing window"""
        self.robot0_conn.disconnect_robot()
        self.robot1_conn.disconnect_robot()
        self.robot0_conn.stop = True
        self.robot1_conn.stop = True
        time.sleep(0.5)
        self.robot0_conn.quit()
        self.robot1_conn.quit()
        self.robot0_conn.wait(500)
        self.robot1_conn.wait(500)
        event.accept()


def main():
    parser = argparse.ArgumentParser(description="Dual X5 arm monitor and jog GUI")
    parser.add_argument('--robot_ip', required=True, nargs=2,
                        metavar=('IP0', 'IP1'),
                        help="IP addresses for Robot 0 and Robot 1")
    args = parser.parse_args()

    app = QApplication(sys.argv)
    window = RobotControlMainWindow(args.robot_ip)
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
