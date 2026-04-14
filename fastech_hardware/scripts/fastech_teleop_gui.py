#!/usr/bin/env python3
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray

try:
    from PyQt5.QtCore import Qt, QTimer
    from PyQt5.QtWidgets import (
        QApplication, QWidget, QLabel, QPushButton, QVBoxLayout, QHBoxLayout,
        QGridLayout, QGroupBox, QDoubleSpinBox, QButtonGroup
    )
    QT_VER = 5
except Exception:
    from PyQt6.QtCore import Qt, QTimer
    from PyQt6.QtWidgets import (
        QApplication, QWidget, QLabel, QPushButton, QVBoxLayout, QHBoxLayout,
        QGridLayout, QGroupBox, QDoubleSpinBox, QButtonGroup
    )
    QT_VER = 6


def qt_align_center():
    return Qt.AlignmentFlag.AlignCenter if QT_VER == 6 else Qt.AlignCenter


def app_exec(app):
    return app.exec() if QT_VER == 6 else app.exec_()


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class SimpleFastechGUI(Node, QWidget):
    def __init__(self):
        Node.__init__(self, "simple_fastech_gui")
        QWidget.__init__(self)

        self.declare_parameter("left_cmd_topic", "/left_wheel_controller/commands")
        self.declare_parameter("right_cmd_topic", "/right_wheel_controller/commands")
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("command_message_type", "multiarray")

        self.declare_parameter("max_left_cmd", 1.0)
        self.declare_parameter("max_right_cmd", 1.0)
        self.declare_parameter("default_left_cmd", 0.20)
        self.declare_parameter("default_right_cmd", 0.20)
        self.declare_parameter("cmd_step", 0.01)

        self.declare_parameter("left_forward_sign", 1.0)
        self.declare_parameter("right_forward_sign", -1.0)
        self.declare_parameter("left_cw_sign", 1.0)
        self.declare_parameter("right_cw_sign", 1.0)

        self.left_cmd_topic = str(self.get_parameter("left_cmd_topic").value)
        self.right_cmd_topic = str(self.get_parameter("right_cmd_topic").value)
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.command_message_type = str(self.get_parameter("command_message_type").value).strip().lower()

        self.max_left_cmd = float(self.get_parameter("max_left_cmd").value)
        self.max_right_cmd = float(self.get_parameter("max_right_cmd").value)
        self.default_left_cmd = float(self.get_parameter("default_left_cmd").value)
        self.default_right_cmd = float(self.get_parameter("default_right_cmd").value)
        self.cmd_step = float(self.get_parameter("cmd_step").value)

        self.left_forward_sign = float(self.get_parameter("left_forward_sign").value)
        self.right_forward_sign = float(self.get_parameter("right_forward_sign").value)
        self.left_cw_sign = float(self.get_parameter("left_cw_sign").value)
        self.right_cw_sign = float(self.get_parameter("right_cw_sign").value)

        if self.command_message_type == "float64":
            self.left_pub = self.create_publisher(Float64, self.left_cmd_topic, 10)
            self.right_pub = self.create_publisher(Float64, self.right_cmd_topic, 10)
        else:
            self.command_message_type = "multiarray"
            self.left_pub = self.create_publisher(Float64MultiArray, self.left_cmd_topic, 10)
            self.right_pub = self.create_publisher(Float64MultiArray, self.right_cmd_topic, 10)

        self.enabled = True
        self.motion_mode = "stop"
        self.left_motor_mode = 0
        self.right_motor_mode = 0
        self.left_last_cmd = 0.0
        self.right_last_cmd = 0.0

        self._build_ui()
        self._apply_theme()
        self._apply_button_colors()
        self._refresh_status()

        self.timer = QTimer(self)
        self.timer.timeout.connect(self._tick)
        self.timer.start(int(1000.0 / max(1.0, self.publish_rate_hz)))

    def _build_ui(self):
        self.setWindowTitle("Simple FASTECH GUI - Individual Motor Speeds")
        root = QVBoxLayout()
        root.setContentsMargins(10, 10, 10, 10)
        root.setSpacing(10)

        motion_group = QGroupBox("ROBOT CONTROL")
        motion_layout = QVBoxLayout()

        pad = QGridLayout()
        pad.setHorizontalSpacing(8)
        pad.setVerticalSpacing(8)

        self.btn_forward = QPushButton("FORWARD")
        self.btn_reverse = QPushButton("REVERSE")
        self.btn_left = QPushButton("LEFT")
        self.btn_right = QPushButton("RIGHT")
        self.btn_stop = QPushButton("STOP")

        for b in [self.btn_forward, self.btn_reverse, self.btn_left, self.btn_right, self.btn_stop]:
            b.setCheckable(True)
            b.setFixedSize(130, 50)

        self.motion_group = QButtonGroup(self)
        self.motion_group.setExclusive(True)
        for b in [self.btn_forward, self.btn_reverse, self.btn_left, self.btn_right, self.btn_stop]:
            self.motion_group.addButton(b)

        self.btn_forward.clicked.connect(lambda: self._set_motion_mode("forward"))
        self.btn_reverse.clicked.connect(lambda: self._set_motion_mode("reverse"))
        self.btn_left.clicked.connect(lambda: self._set_motion_mode("left"))
        self.btn_right.clicked.connect(lambda: self._set_motion_mode("right"))
        self.btn_stop.clicked.connect(lambda: self._set_motion_mode("stop"))
        self.btn_stop.setChecked(True)

        pad.addWidget(self.btn_forward, 0, 1)
        pad.addWidget(self.btn_left, 1, 0)
        pad.addWidget(self.btn_stop, 1, 1)
        pad.addWidget(self.btn_right, 1, 2)
        pad.addWidget(self.btn_reverse, 2, 1)
        motion_layout.addLayout(pad)
        motion_group.setLayout(motion_layout)
        root.addWidget(motion_group)

        speed_group = QGroupBox("INDIVIDUAL SPEED SETTING")
        speed_layout = QGridLayout()

        self.sp_left_cmd = QDoubleSpinBox()
        self.sp_left_cmd.setDecimals(4)
        self.sp_left_cmd.setRange(0.0, self.max_left_cmd)
        self.sp_left_cmd.setSingleStep(self.cmd_step)
        self.sp_left_cmd.setValue(clamp(self.default_left_cmd, 0.0, self.max_left_cmd))
        self.sp_left_cmd.setSuffix(" cmd")

        self.sp_right_cmd = QDoubleSpinBox()
        self.sp_right_cmd.setDecimals(4)
        self.sp_right_cmd.setRange(0.0, self.max_right_cmd)
        self.sp_right_cmd.setSingleStep(self.cmd_step)
        self.sp_right_cmd.setValue(clamp(self.default_right_cmd, 0.0, self.max_right_cmd))
        self.sp_right_cmd.setSuffix(" cmd")

        speed_layout.addWidget(QLabel("Left Motor Speed"), 0, 0)
        speed_layout.addWidget(self.sp_left_cmd, 0, 1)
        speed_layout.addWidget(QLabel("Right Motor Speed"), 1, 0)
        speed_layout.addWidget(self.sp_right_cmd, 1, 1)
        speed_group.setLayout(speed_layout)
        root.addWidget(speed_group)

        motor_row = QHBoxLayout()

        left_group = QGroupBox("LEFT MOTOR")
        left_layout = QVBoxLayout()
        left_btn_row = QHBoxLayout()

        self.btn_left_cw = QPushButton("CW")
        self.btn_left_motor_stop = QPushButton("STOP")
        self.btn_left_ccw = QPushButton("CCW")
        for b in [self.btn_left_cw, self.btn_left_motor_stop, self.btn_left_ccw]:
            b.setCheckable(True)
            b.setFixedSize(100, 45)

        self.left_motor_group = QButtonGroup(self)
        self.left_motor_group.setExclusive(True)
        for b in [self.btn_left_cw, self.btn_left_motor_stop, self.btn_left_ccw]:
            self.left_motor_group.addButton(b)

        self.btn_left_cw.clicked.connect(lambda: self._set_left_motor_mode(+1))
        self.btn_left_motor_stop.clicked.connect(lambda: self._set_left_motor_mode(0))
        self.btn_left_ccw.clicked.connect(lambda: self._set_left_motor_mode(-1))
        self.btn_left_motor_stop.setChecked(True)

        left_btn_row.addWidget(self.btn_left_cw)
        left_btn_row.addWidget(self.btn_left_motor_stop)
        left_btn_row.addWidget(self.btn_left_ccw)
        left_layout.addLayout(left_btn_row)
        left_group.setLayout(left_layout)

        right_group = QGroupBox("RIGHT MOTOR")
        right_layout = QVBoxLayout()
        right_btn_row = QHBoxLayout()

        self.btn_right_cw = QPushButton("CW")
        self.btn_right_motor_stop = QPushButton("STOP")
        self.btn_right_ccw = QPushButton("CCW")
        for b in [self.btn_right_cw, self.btn_right_motor_stop, self.btn_right_ccw]:
            b.setCheckable(True)
            b.setFixedSize(100, 45)

        self.right_motor_group = QButtonGroup(self)
        self.right_motor_group.setExclusive(True)
        for b in [self.btn_right_cw, self.btn_right_motor_stop, self.btn_right_ccw]:
            self.right_motor_group.addButton(b)

        self.btn_right_cw.clicked.connect(lambda: self._set_right_motor_mode(+1))
        self.btn_right_motor_stop.clicked.connect(lambda: self._set_right_motor_mode(0))
        self.btn_right_ccw.clicked.connect(lambda: self._set_right_motor_mode(-1))
        self.btn_right_motor_stop.setChecked(True)

        right_btn_row.addWidget(self.btn_right_cw)
        right_btn_row.addWidget(self.btn_right_motor_stop)
        right_btn_row.addWidget(self.btn_right_ccw)
        right_layout.addLayout(right_btn_row)
        right_group.setLayout(right_layout)

        motor_row.addWidget(left_group)
        motor_row.addWidget(right_group)
        root.addLayout(motor_row)

        status_group = QGroupBox("ROBOT STATUS")
        status_layout = QGridLayout()

        self.lbl_status = self._make_box("STOP")
        self.lbl_control_mode = self._make_box("VEHICLE")
        self.lbl_left_set = self._make_box("+0.0000")
        self.lbl_right_set = self._make_box("+0.0000")
        self.lbl_left_cmd = self._make_box("+0.0000")
        self.lbl_right_cmd = self._make_box("+0.0000")
        self.lbl_enabled = self._make_box("ENABLED")

        status_layout.addWidget(QLabel("Robot State"), 0, 0)
        status_layout.addWidget(self.lbl_status, 0, 1)
        status_layout.addWidget(QLabel("Control Mode"), 1, 0)
        status_layout.addWidget(self.lbl_control_mode, 1, 1)
        status_layout.addWidget(QLabel("Left Speed Set"), 2, 0)
        status_layout.addWidget(self.lbl_left_set, 2, 1)
        status_layout.addWidget(QLabel("Right Speed Set"), 3, 0)
        status_layout.addWidget(self.lbl_right_set, 3, 1)
        status_layout.addWidget(QLabel("Left Command"), 4, 0)
        status_layout.addWidget(self.lbl_left_cmd, 4, 1)
        status_layout.addWidget(QLabel("Right Command"), 5, 0)
        status_layout.addWidget(self.lbl_right_cmd, 5, 1)
        status_layout.addWidget(QLabel("System"), 6, 0)
        status_layout.addWidget(self.lbl_enabled, 6, 1)
        status_group.setLayout(status_layout)
        root.addWidget(status_group)

        bottom_row = QHBoxLayout()
        self.btn_enable = QPushButton("Disable")
        self.btn_enable.setFixedHeight(42)
        self.btn_enable.clicked.connect(self._toggle_enable)

        self.btn_stop_now = QPushButton("STOP NOW")
        self.btn_stop_now.setFixedHeight(42)
        self.btn_stop_now.clicked.connect(self._hard_stop)

        bottom_row.addWidget(self.btn_enable)
        bottom_row.addWidget(self.btn_stop_now)
        bottom_row.addStretch(1)
        root.addLayout(bottom_row)

        self.setLayout(root)

    def _make_box(self, text):
        lbl = QLabel(text)
        lbl.setAlignment(qt_align_center())
        lbl.setStyleSheet(
            "QLabel { background: white; border: 1px solid #cbd5e1; border-radius: 8px; padding: 8px; font-weight: 800; min-height: 30px; }"
        )
        return lbl

    def _apply_theme(self):
        self.setStyleSheet(
            "QWidget { background: #f4f7fb; color: #0f172a; font-size: 12px; }"
            "QGroupBox { border: 1px solid #cbd5e1; border-radius: 10px; margin-top: 12px; padding-top: 10px; background: white; font-weight: 900; }"
            "QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 6px; }"
            "QPushButton { background: #e2e8f0; border: 1px solid #cbd5e1; border-radius: 8px; font-weight: 800; }"
            "QDoubleSpinBox { background: white; border: 1px solid #cbd5e1; border-radius: 6px; min-height: 28px; padding: 2px 6px; }"
        )

    def _apply_button_colors(self):
        self.btn_forward.setStyleSheet("QPushButton:checked { background: #22c55e; color: white; font-weight: 900; }")
        self.btn_reverse.setStyleSheet("QPushButton:checked { background: #f97316; color: white; font-weight: 900; }")
        self.btn_left.setStyleSheet("QPushButton:checked { background: #3b82f6; color: white; font-weight: 900; }")
        self.btn_right.setStyleSheet("QPushButton:checked { background: #3b82f6; color: white; font-weight: 900; }")
        self.btn_stop.setStyleSheet("QPushButton:checked { background: #ef4444; color: white; font-weight: 900; }")
        self.btn_left_cw.setStyleSheet("QPushButton:checked { background: #8b5cf6; color: white; font-weight: 900; }")
        self.btn_left_motor_stop.setStyleSheet("QPushButton:checked { background: #64748b; color: white; font-weight: 900; }")
        self.btn_left_ccw.setStyleSheet("QPushButton:checked { background: #a78bfa; color: white; font-weight: 900; }")
        self.btn_right_cw.setStyleSheet("QPushButton:checked { background: #10b981; color: white; font-weight: 900; }")
        self.btn_right_motor_stop.setStyleSheet("QPushButton:checked { background: #64748b; color: white; font-weight: 900; }")
        self.btn_right_ccw.setStyleSheet("QPushButton:checked { background: #34d399; color: white; font-weight: 900; }")
        self.btn_stop_now.setStyleSheet("QPushButton { background: #ef4444; color: white; font-weight: 900; }")

    def _set_checked_safely(self, button, checked):
        if button.isChecked() != checked:
            button.blockSignals(True)
            button.setChecked(checked)
            button.blockSignals(False)

    def _clear_vehicle_mode(self):
        self.motion_mode = "stop"
        self._set_checked_safely(self.btn_forward, False)
        self._set_checked_safely(self.btn_reverse, False)
        self._set_checked_safely(self.btn_left, False)
        self._set_checked_safely(self.btn_right, False)
        self._set_checked_safely(self.btn_stop, True)

    def _clear_individual_mode(self):
        self.left_motor_mode = 0
        self.right_motor_mode = 0
        self._set_checked_safely(self.btn_left_cw, False)
        self._set_checked_safely(self.btn_left_motor_stop, True)
        self._set_checked_safely(self.btn_left_ccw, False)
        self._set_checked_safely(self.btn_right_cw, False)
        self._set_checked_safely(self.btn_right_motor_stop, True)
        self._set_checked_safely(self.btn_right_ccw, False)

    def _set_motion_mode(self, mode):
        self._clear_individual_mode()
        self.motion_mode = mode
        self._set_checked_safely(self.btn_forward, mode == "forward")
        self._set_checked_safely(self.btn_reverse, mode == "reverse")
        self._set_checked_safely(self.btn_left, mode == "left")
        self._set_checked_safely(self.btn_right, mode == "right")
        self._set_checked_safely(self.btn_stop, mode == "stop")
        self._refresh_status()

    def _set_left_motor_mode(self, mode):
        self._clear_vehicle_mode()
        self.left_motor_mode = int(mode)
        self._set_checked_safely(self.btn_left_cw, mode > 0)
        self._set_checked_safely(self.btn_left_motor_stop, mode == 0)
        self._set_checked_safely(self.btn_left_ccw, mode < 0)
        self._refresh_status()

    def _set_right_motor_mode(self, mode):
        self._clear_vehicle_mode()
        self.right_motor_mode = int(mode)
        self._set_checked_safely(self.btn_right_cw, mode > 0)
        self._set_checked_safely(self.btn_right_motor_stop, mode == 0)
        self._set_checked_safely(self.btn_right_ccw, mode < 0)
        self._refresh_status()

    def _toggle_enable(self):
        self.enabled = not self.enabled
        self.btn_enable.setText("Disable" if self.enabled else "Enable")
        if not self.enabled:
            self._hard_stop()
        self._refresh_status()

    def _hard_stop(self):
        self._clear_vehicle_mode()
        self._clear_individual_mode()
        self.left_last_cmd = 0.0
        self.right_last_cmd = 0.0
        self._publish_commands(0.0, 0.0)
        self._refresh_status()

    def _compute_commands(self):
        if not self.enabled:
            return 0.0, 0.0

        left_set = clamp(float(self.sp_left_cmd.value()), 0.0, self.max_left_cmd)
        right_set = clamp(float(self.sp_right_cmd.value()), 0.0, self.max_right_cmd)

        if self.left_motor_mode != 0 or self.right_motor_mode != 0:
            left_cmd = self.left_motor_mode * self.left_cw_sign * left_set
            right_cmd = self.right_motor_mode * self.right_cw_sign * right_set
            return left_cmd, right_cmd

        if self.motion_mode == "forward":
            left_cmd = self.left_forward_sign * left_set
            right_cmd = self.right_forward_sign * right_set
        elif self.motion_mode == "reverse":
            left_cmd = -self.left_forward_sign * left_set
            right_cmd = -self.right_forward_sign * right_set
        elif self.motion_mode == "left":
            left_cmd = -self.left_forward_sign * left_set
            right_cmd = self.right_forward_sign * right_set
        elif self.motion_mode == "right":
            left_cmd = self.left_forward_sign * left_set
            right_cmd = -self.right_forward_sign * right_set
        else:
            left_cmd = 0.0
            right_cmd = 0.0

        return left_cmd, right_cmd

    def _publish_one(self, pub, value):
        if self.command_message_type == "float64":
            msg = Float64()
            msg.data = float(value)
        else:
            msg = Float64MultiArray()
            msg.data = [float(value)]
        pub.publish(msg)

    def _publish_commands(self, left_cmd, right_cmd):
        self._publish_one(self.left_pub, left_cmd)
        self._publish_one(self.right_pub, right_cmd)

    def _refresh_status(self):
        if self.left_motor_mode != 0 or self.right_motor_mode != 0:
            control_mode = "INDIVIDUAL"
            if self.left_motor_mode != 0 and self.right_motor_mode == 0:
                state = "LEFT MOTOR"
            elif self.left_motor_mode == 0 and self.right_motor_mode != 0:
                state = "RIGHT MOTOR"
            else:
                state = "BOTH MOTORS"
        else:
            control_mode = "VEHICLE"
            state = self.motion_mode.upper()

        self.lbl_status.setText(state)
        self.lbl_control_mode.setText(control_mode)
        self.lbl_left_set.setText(f"{self.sp_left_cmd.value():+.4f}")
        self.lbl_right_set.setText(f"{self.sp_right_cmd.value():+.4f}")
        self.lbl_left_cmd.setText(f"{self.left_last_cmd:+.4f}")
        self.lbl_right_cmd.setText(f"{self.right_last_cmd:+.4f}")
        self.lbl_enabled.setText("ENABLED" if self.enabled else "DISABLED")

    def _tick(self):
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.0)

        left_cmd, right_cmd = self._compute_commands()
        self.left_last_cmd = left_cmd
        self.right_last_cmd = right_cmd
        self._publish_commands(left_cmd, right_cmd)
        self._refresh_status()


def main():
    rclpy.init(args=sys.argv)
    app = QApplication(sys.argv)
    w = SimpleFastechGUI()
    w.show()
    try:
        sys.exit(app_exec(app))
    finally:
        try:
            w.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()