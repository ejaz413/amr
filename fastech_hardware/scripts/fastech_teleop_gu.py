#!/usr/bin/env python3
import sys
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float64, Float64MultiArray
from sensor_msgs.msg import JointState
from lvs_driver.msg import LvsProfile

try:
    from PyQt5.QtCore import Qt, QTimer, QPointF
    from PyQt5.QtGui import QPainter, QPen, QFont, QColor, QPolygonF
    from PyQt5.QtWidgets import (
        QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
        QSlider, QDoubleSpinBox, QGroupBox, QGridLayout, QCheckBox,
        QButtonGroup, QSizePolicy, QScrollArea
    )
    QT_VER = 5
except Exception:
    from PyQt6.QtCore import Qt, QTimer, QPointF
    from PyQt6.QtGui import QPainter, QPen, QFont, QColor, QPolygonF
    from PyQt6.QtWidgets import (
        QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
        QSlider, QDoubleSpinBox, QGroupBox, QGridLayout, QCheckBox,
        QButtonGroup, QSizePolicy, QScrollArea
    )
    QT_VER = 6


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def is_finite(v):
    return not math.isnan(v) and not math.isinf(v)


def _qt_align_center():
    return Qt.AlignmentFlag.AlignCenter if QT_VER == 6 else Qt.AlignCenter


def _qt_horizontal():
    return Qt.Orientation.Horizontal if QT_VER == 6 else Qt.Horizontal


def _qt_strong_focus():
    return Qt.FocusPolicy.StrongFocus if QT_VER == 6 else Qt.StrongFocus


def _qpainter_antialiasing():
    return QPainter.RenderHint.Antialiasing if QT_VER == 6 else QPainter.Antialiasing


def _no_brush():
    return Qt.BrushStyle.NoBrush if QT_VER == 6 else Qt.NoBrush


def _app_exec(app):
    return app.exec() if QT_VER == 6 else app.exec_()


class LVSProfileViewer(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.xs = []
        self.zs = []
        self.intensities = []
        self.precision = None
        self.valid = True
        self.unit = "mm"
        self.status_text = "Waiting for profile ..."
        self.setMinimumHeight(340)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def clear(self, status_text="Waiting for profile ..."):
        self.xs = []
        self.zs = []
        self.intensities = []
        self.precision = None
        self.valid = True
        self.status_text = status_text
        self.update()

    def update_profile(
        self,
        xs,
        zs,
        intensity_list=None,
        precision=None,
        valid=True,
        unit="mm",
        status_text=""
    ):
        self.xs = list(xs) if xs is not None else []
        self.zs = list(zs) if zs is not None else []
        self.intensities = list(intensity_list) if intensity_list is not None else []
        self.precision = precision
        self.valid = bool(valid)
        self.unit = str(unit)
        self.status_text = status_text
        self.update()

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(_qpainter_antialiasing(), True)

        w = self.width()
        h = self.height()

        bg_color = QColor(8, 15, 28)
        grid_color = QColor(49, 63, 84)
        axis_color = QColor(100, 116, 139)
        text_color = QColor(226, 232, 240)
        line_color = QColor(16, 185, 129)
        point_color = QColor(250, 204, 21)
        min_outer_color = QColor(239, 68, 68)
        min_inner_color = QColor(253, 224, 71)

        p.fillRect(0, 0, w, h, bg_color)

        left = 54
        right = 18
        top = 66
        bottom = 40

        plot_x = left
        plot_y = top
        plot_w = max(10, w - left - right)
        plot_h = max(10, h - top - bottom)

        p.setPen(QPen(axis_color, 1))
        p.drawRect(plot_x, plot_y, plot_w, plot_h)

        p.setPen(QPen(grid_color, 1))
        for i in range(1, 5):
            gx = plot_x + i * plot_w / 5.0
            gy = plot_y + i * plot_h / 5.0
            p.drawLine(int(gx), plot_y, int(gx), plot_y + plot_h)
            p.drawLine(plot_x, int(gy), plot_x + plot_w, int(gy))

        n = min(len(self.xs), len(self.zs))
        if n < 2:
            p.setPen(text_color)
            font = QFont()
            font.setPointSize(11)
            font.setBold(True)
            p.setFont(font)
            p.drawText(plot_x, plot_y, plot_w, plot_h, _qt_align_center(), self.status_text)
            return

        xs = self.xs[:n]
        zs = self.zs[:n]

        finite_indices = [i for i in range(n) if is_finite(xs[i]) and is_finite(zs[i])]
        if len(finite_indices) < 2:
            p.setPen(text_color)
            font = QFont()
            font.setPointSize(11)
            font.setBold(True)
            p.setFont(font)
            p.drawText(plot_x, plot_y, plot_w, plot_h, _qt_align_center(), self.status_text)
            return

        finite_xs = [xs[i] for i in finite_indices]
        finite_zs = [zs[i] for i in finite_indices]

        x_min = min(finite_xs)
        x_max = max(finite_xs)
        z_min = min(finite_zs)
        z_max = max(finite_zs)

        if abs(x_max - x_min) < 1e-12:
            x_max = x_min + 1.0
        if abs(z_max - z_min) < 1e-12:
            z_max = z_min + 1.0

        z_pad = (z_max - z_min) * 0.08 if (z_max - z_min) > 0 else 1.0
        z_draw_min = z_min - z_pad
        z_draw_max = z_max + z_pad

        def map_x(xv):
            return plot_x + (xv - x_min) / (x_max - x_min) * plot_w

        def map_z(zv):
            return plot_y + (z_draw_max - zv) / (z_draw_max - z_draw_min) * plot_h

        p.setPen(QPen(line_color, 2))
        current_segment = QPolygonF()
        for xv, zv in zip(xs, zs):
            if is_finite(xv) and is_finite(zv):
                current_segment.append(QPointF(map_x(xv), map_z(zv)))
            else:
                if len(current_segment) >= 2:
                    p.drawPolyline(current_segment)
                current_segment = QPolygonF()
        if len(current_segment) >= 2:
            p.drawPolyline(current_segment)

        p.setPen(QPen(point_color, 1))
        p.setBrush(point_color)
        for i in finite_indices:
            p.drawEllipse(QPointF(map_x(xs[i]), map_z(zs[i])), 1.7, 1.7)

        min_idx = min(finite_indices, key=lambda i: zs[i])
        min_x = xs[min_idx]
        min_z = zs[min_idx]
        min_px = map_x(min_x)
        min_py = map_z(min_z)

        p.setPen(QPen(min_outer_color, 2))
        p.setBrush(_no_brush())
        p.drawEllipse(QPointF(min_px, min_py), 5.5, 5.5)

        p.setPen(QPen(min_inner_color, 1))
        p.setBrush(min_inner_color)
        p.drawEllipse(QPointF(min_px, min_py), 2.4, 2.4)

        precision_text = "N/A" if self.precision is None else str(self.precision)
        valid_text = "True" if self.valid else "False"

        if self.intensities and len(self.intensities) >= n:
            finite_ints = [self.intensities[i] for i in finite_indices if is_finite(self.intensities[i])]
            if finite_ints:
                i_avg = sum(finite_ints) / len(finite_ints)
                i_min = min(finite_ints)
                i_max = max(finite_ints)
                intensity_text = f"Intensity avg/min/max: {i_avg:.1f} / {i_min:.0f} / {i_max:.0f}"
            else:
                intensity_text = "Intensity avg/min/max: N/A"
        else:
            intensity_text = "Intensity avg/min/max: N/A"

        info_lines = [
            f"Min Z: x={min_x:.2f} {self.unit}, z={min_z:.2f} {self.unit}",
            f"Z range: {z_min:.2f} ~ {z_max:.2f} {self.unit}",
            f"Points: {len(finite_indices)} | Precision: {precision_text} | Valid: {valid_text}",
            intensity_text,
        ]

        p.setPen(text_color)
        font = QFont("Sans", 9)
        p.setFont(font)

        info_y = 8
        for line in info_lines:
            p.drawText(12, info_y + 16, line)
            info_y += 20

        p.drawText(12, h - 10, f"X range: {x_min:.2f} ~ {x_max:.2f} {self.unit}")


class CombinedTeleopGUI(Node, QWidget):
    def __init__(self):
        Node.__init__(self, "combined_teleop_gui")
        QWidget.__init__(self)

        # --------------------------------------------------
        # Parameters
        # --------------------------------------------------
        self.declare_parameter("cmd_topic", "/diff_drive_controller/cmd_vel")
        self.declare_parameter("max_speed", 0.20)
        self.declare_parameter("max_yaw_rate", 1.0)
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("speed_step", 0.0001)
        self.declare_parameter("turn_step", 0.01)
        self.declare_parameter("turn_rate_default", 0.10)

        self.declare_parameter("use_individual_axis_topics", True)
        self.declare_parameter("joint_cmd_topic", "/position_controller/commands")
        self.declare_parameter("x_cmd_topic", "/x_axis_controller/commands")
        self.declare_parameter("y_cmd_topic", "/y_axis_controller/commands")
        self.declare_parameter("z_cmd_topic", "/z_axis_controller/commands")
        self.declare_parameter("line_error_topic", "/x_line_error_mm")
        self.declare_parameter("joint_states_topic", "/joint_states")

        self.declare_parameter("x_joint_name", "x_joint")
        self.declare_parameter("y_joint_name", "y_joint")
        self.declare_parameter("z_joint_name", "z_joint")

        self.declare_parameter("x_min", -0.50)
        self.declare_parameter("x_max", 0.50)
        self.declare_parameter("y_min", -0.20)
        self.declare_parameter("y_max", 0.20)
        self.declare_parameter("z_min", -6.283)
        self.declare_parameter("z_max", 6.283)

        self.declare_parameter("x_rate_default_mm_s", 5.0)
        self.declare_parameter("y_rate_default_mm_s", 1.0)
        self.declare_parameter("z_rate_default_deg_s", 5.0)

        self.declare_parameter("lvs_topic", "/lvs/profile")
        self.declare_parameter("filtered_lvs_topic", "/lvs/profile_filtered")

        # --------------------------------------------------
        # Read parameters
        # --------------------------------------------------
        self.cmd_topic = str(self.get_parameter("cmd_topic").value)
        self.max_speed = float(self.get_parameter("max_speed").value)
        self.max_yaw = float(self.get_parameter("max_yaw_rate").value)
        self.rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.speed_step = float(self.get_parameter("speed_step").value)
        self.turn_step = float(self.get_parameter("turn_step").value)
        self.turn_rate_mag = float(self.get_parameter("turn_rate_default").value)

        self.use_individual_axis_topics = bool(self.get_parameter("use_individual_axis_topics").value)
        self.joint_cmd_topic = str(self.get_parameter("joint_cmd_topic").value)
        self.x_cmd_topic = str(self.get_parameter("x_cmd_topic").value)
        self.y_cmd_topic = str(self.get_parameter("y_cmd_topic").value)
        self.z_cmd_topic = str(self.get_parameter("z_cmd_topic").value)
        self.line_error_topic = str(self.get_parameter("line_error_topic").value)
        self.joint_states_topic = str(self.get_parameter("joint_states_topic").value)

        self.x_joint_name = str(self.get_parameter("x_joint_name").value)
        self.y_joint_name = str(self.get_parameter("y_joint_name").value)
        self.z_joint_name = str(self.get_parameter("z_joint_name").value)

        self.x_min = float(self.get_parameter("x_min").value)
        self.x_max = float(self.get_parameter("x_max").value)
        self.y_min = float(self.get_parameter("y_min").value)
        self.y_max = float(self.get_parameter("y_max").value)
        self.z_min = float(self.get_parameter("z_min").value)
        self.z_max = float(self.get_parameter("z_max").value)

        self.x_rate_default_mm_s = float(self.get_parameter("x_rate_default_mm_s").value)
        self.y_rate_default_mm_s = float(self.get_parameter("y_rate_default_mm_s").value)
        self.z_rate_default_deg_s = float(self.get_parameter("z_rate_default_deg_s").value)

        self.lvs_topic = str(self.get_parameter("lvs_topic").value)
        self.filtered_lvs_topic = str(self.get_parameter("filtered_lvs_topic").value)

        # --------------------------------------------------
        # ROS I/O
        # --------------------------------------------------
        self.pub_drive = self.create_publisher(TwistStamped, self.cmd_topic, 10)

        if self.use_individual_axis_topics:
            self.x_pub = self.create_publisher(Float64MultiArray, self.x_cmd_topic, 10)
            self.y_pub = self.create_publisher(Float64MultiArray, self.y_cmd_topic, 10)
            self.z_pub = self.create_publisher(Float64MultiArray, self.z_cmd_topic, 10)
        else:
            self.joint_pub = self.create_publisher(Float64MultiArray, self.joint_cmd_topic, 10)

        self.lvs_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.latest_line_error_mm = 0.0

        self.sub_line_error = self.create_subscription(
            Float64, self.line_error_topic, self.line_error_callback, 10
        )
        self.sub_joint_states = self.create_subscription(
            JointState, self.joint_states_topic, self._on_joint_states, 10
        )
        self.sub_lvs_raw = self.create_subscription(
            LvsProfile, self.lvs_topic, self._on_lvs_profile_raw, self.lvs_qos
        )
        self.sub_lvs_filtered = self.create_subscription(
            LvsProfile, self.filtered_lvs_topic, self._on_lvs_profile_filtered, self.lvs_qos
        )

        # --------------------------------------------------
        # Runtime state
        # --------------------------------------------------
        self.enabled = True
        self.estop = False
        self.speed_mag = 0.0
        self.motion_mode = "stop"

        self.current_positions = [0.0, 0.0, 0.0]
        self.actual_positions = {
            self.x_joint_name: 0.0,
            self.y_joint_name: 0.0,
            self.z_joint_name: 0.0,
        }
        self.have_joint_state = False

        self.x_manual_mode = 0
        self.y_manual_mode = 0
        self.z_manual_mode = 0

        self.x_auto_running = False
        self.x_auto_start_m = 0.0
        self.x_auto_end_m = 0.0
        self.x_auto_phase = 0.0
        self.x_auto_direction = +1
        self.x_auto_period = 1.0

        self.x_track_mode = False
        self.x_track_base = 0.0

        self.lvs_unit = "mm"

        self.raw_lvs_msg_count = 0
        self.filtered_lvs_msg_count = 0
        self.raw_lvs_last_points = 0
        self.filtered_lvs_last_points = 0

        self.status_page_index = 0
        self.status_rotate_counter = 0
        self.status_rotate_every_ticks = 20

        self.xyz_initialized = False
        self.warned_waiting_for_joint_state = False

        # --------------------------------------------------
        # Build UI
        # --------------------------------------------------
        self.setWindowTitle("FASTECH Teleop (Crawler + XYZ + LVS)")
        self.setFocusPolicy(_qt_strong_focus())

        self._build_ui()
        self._apply_professional_theme()
        self._apply_button_colors()
        self._refresh()

        self.timer = QTimer(self)
        self.timer.timeout.connect(self._tick)
        self.timer.start(int(1000.0 / max(1.0, self.rate_hz)))

    # ==========================================================
    # UI helpers
    # ==========================================================
    def _box_style(self, bg, fg):
        return (
            f"QLabel {{"
            f"background: {bg};"
            f"color: {fg};"
            f"border: 1px solid #cbd5e1;"
            f"border-radius: 8px;"
            f"padding: 6px 8px;"
            f"font-weight: 800;"
            f"}}"
        )

    def _make_small_button(self, text, checkable=False, width=120, height=34):
        b = QPushButton(text)
        b.setCheckable(checkable)
        b.setFixedSize(width, height)
        return b

    def _make_caption(self, text: str):
        lbl = QLabel(text)
        lbl.setStyleSheet("font-weight: 700; color: #334155;")
        return lbl

    def _make_value_box(self, text="---", bg="#ffffff", fg="#0f172a"):
        lbl = QLabel(text)
        lbl.setAlignment(_qt_align_center())
        lbl.setMinimumHeight(34)
        lbl.setStyleSheet(self._box_style(bg, fg))
        return lbl

    def _apply_professional_theme(self):
        self.setStyleSheet(
            """
            QWidget {
                background: #f4f7fb;
                color: #0f172a;
                font-size: 12px;
            }

            QScrollArea {
                border: none;
            }

            QGroupBox {
                border: 1px solid #cbd5e1;
                border-radius: 10px;
                margin-top: 14px;
                padding-top: 10px;
                background: #ffffff;
                font-weight: 700;
            }

            QGroupBox::title {
                subcontrol-origin: margin;
                left: 12px;
                padding: 0 6px 0 6px;
                color: #0f172a;
                font-weight: 800;
            }

            QPushButton {
                background: #e2e8f0;
                border: 1px solid #cbd5e1;
                border-radius: 8px;
                padding: 6px 10px;
                font-weight: 700;
            }

            QPushButton:hover {
                background: #dbeafe;
            }

            QPushButton:pressed {
                background: #bfdbfe;
            }

            QDoubleSpinBox {
                background: #ffffff;
                border: 1px solid #cbd5e1;
                border-radius: 6px;
                padding: 2px 6px;
                min-height: 28px;
            }

            QCheckBox {
                font-weight: 700;
                color: #334155;
            }
            """
        )

    def _apply_button_colors(self):
        self.btn_forward.setStyleSheet(
            "QPushButton:checked { background-color: #22c55e; color: #052e16; font-weight: 900; }"
        )
        self.btn_reverse.setStyleSheet(
            "QPushButton:checked { background-color: #fb923c; color: #3b1d0a; font-weight: 900; }"
        )
        self.btn_left.setStyleSheet(
            "QPushButton:checked { background-color: #60a5fa; color: #0b1220; font-weight: 900; }"
        )
        self.btn_right.setStyleSheet(
            "QPushButton:checked { background-color: #60a5fa; color: #0b1220; font-weight: 900; }"
        )
        self.btn_stop.setStyleSheet(
            "QPushButton:checked { background-color: #ef4444; color: #2b0b0b; font-weight: 900; }"
        )

        self.btn_x_minus.setStyleSheet(
            "QPushButton:checked { background-color: #c4b5fd; font-weight: 900; }"
        )
        self.btn_x_stop.setStyleSheet(
            "QPushButton:checked { background-color: #e9d5ff; font-weight: 900; }"
        )
        self.btn_x_plus.setStyleSheet(
            "QPushButton:checked { background-color: #a78bfa; font-weight: 900; }"
        )

        self.btn_y_minus.setStyleSheet(
            "QPushButton:checked { background-color: #a7f3d0; font-weight: 900; }"
        )
        self.btn_y_stop.setStyleSheet(
            "QPushButton:checked { background-color: #d1fae5; font-weight: 900; }"
        )
        self.btn_y_plus.setStyleSheet(
            "QPushButton:checked { background-color: #6ee7b7; font-weight: 900; }"
        )

        self.btn_z_minus.setStyleSheet(
            "QPushButton:checked { background-color: #fde68a; font-weight: 900; }"
        )
        self.btn_z_stop.setStyleSheet(
            "QPushButton:checked { background-color: #fef3c7; font-weight: 900; }"
        )
        self.btn_z_plus.setStyleSheet(
            "QPushButton:checked { background-color: #fcd34d; font-weight: 900; }"
        )

        self.btn_x_auto.setStyleSheet(
            "QPushButton:checked { background-color: #22c55e; color: #052e16; font-weight: 900; }"
        )
        self.btn_x_track.setStyleSheet(
            "QPushButton:checked { background-color: #a855f7; color: white; font-weight: 900; }"
        )
        self.btn_hard_stop.setStyleSheet(
            "QPushButton { background-color: #ef4444; color: #1f0a0a; font-weight: 900; }"
        )
        self.btn_estop.setStyleSheet(
            "QPushButton:checked { background-color: #dc2626; color: white; font-weight: 900; }"
        )
        self.btn_reset_xyz.setStyleSheet(
            "QPushButton { background-color: #cbd5e1; font-weight: 800; }"
        )

    # ==========================================================
    # Build UI
    # ==========================================================
    def _build_ui(self):
        outer = QVBoxLayout()
        outer.setContentsMargins(4, 4, 4, 4)
        outer.setSpacing(4)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)

        content = QWidget()
        root = QVBoxLayout(content)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(8)

        # --------------------------------------------------
        # Crawler
        # --------------------------------------------------
        gb_crawler = QGroupBox("CRAWLER CONTROL")
        gc_outer = QVBoxLayout()
        gc_outer.setSpacing(8)

        pad_row = QHBoxLayout()
        pad_row.addStretch(1)

        pad_widget = QWidget()
        pad_grid = QGridLayout(pad_widget)
        pad_grid.setContentsMargins(0, 0, 0, 0)
        pad_grid.setHorizontalSpacing(6)
        pad_grid.setVerticalSpacing(6)

        self.btn_forward = self._make_small_button("FORWARD", checkable=True, width=120, height=36)
        self.btn_reverse = self._make_small_button("REVERSE", checkable=True, width=120, height=36)
        self.btn_left = self._make_small_button("LEFT", checkable=True, width=120, height=36)
        self.btn_right = self._make_small_button("RIGHT", checkable=True, width=120, height=36)
        self.btn_stop = self._make_small_button("STOP", checkable=True, width=120, height=36)

        self.motion_group = QButtonGroup(self)
        self.motion_group.setExclusive(True)
        for b in (self.btn_forward, self.btn_reverse, self.btn_left, self.btn_right, self.btn_stop):
            self.motion_group.addButton(b)

        self.btn_forward.clicked.connect(lambda: self._set_motion_mode("forward"))
        self.btn_reverse.clicked.connect(lambda: self._set_motion_mode("reverse"))
        self.btn_left.clicked.connect(lambda: self._set_motion_mode("left"))
        self.btn_right.clicked.connect(lambda: self._set_motion_mode("right"))
        self.btn_stop.clicked.connect(lambda: self._set_motion_mode("stop"))
        self.btn_stop.setChecked(True)

        pad_grid.addWidget(self.btn_forward, 0, 1)
        pad_grid.addWidget(self.btn_left,    1, 0)
        pad_grid.addWidget(self.btn_stop,    1, 1)
        pad_grid.addWidget(self.btn_right,   1, 2)
        pad_grid.addWidget(self.btn_reverse, 2, 1)

        pad_row.addWidget(pad_widget)
        pad_row.addStretch(1)
        gc_outer.addLayout(pad_row)

        speed_grid = QGridLayout()
        speed_grid.setHorizontalSpacing(8)
        speed_grid.setVerticalSpacing(6)

        self.s_speed = QSlider(_qt_horizontal())
        self.s_speed.setMinimum(0)
        self.s_speed.setMaximum(int(round(self.max_speed * 10000)))
        self.s_speed.setValue(0)
        self.s_speed.valueChanged.connect(lambda v: self._set_speed_mag(float(v) / 10000.0))

        self.sp_speed = QDoubleSpinBox()
        self.sp_speed.setDecimals(4)
        self.sp_speed.setSingleStep(self.speed_step)
        self.sp_speed.setRange(0.0, self.max_speed)
        self.sp_speed.setValue(0.0)
        self.sp_speed.valueChanged.connect(lambda v: self._set_speed_mag(float(v)))

        self.s_turn = QSlider(_qt_horizontal())
        self.s_turn.setMinimum(0)
        self.s_turn.setMaximum(int(round(self.max_yaw * 1000)))
        self.s_turn.setValue(int(round(self.turn_rate_mag * 1000)))
        self.s_turn.valueChanged.connect(lambda v: self._set_turn_rate_mag(float(v) / 1000.0))

        self.sp_turn = QDoubleSpinBox()
        self.sp_turn.setDecimals(3)
        self.sp_turn.setSingleStep(self.turn_step)
        self.sp_turn.setRange(0.0, self.max_yaw)
        self.sp_turn.setValue(self.turn_rate_mag)
        self.sp_turn.valueChanged.connect(lambda v: self._set_turn_rate_mag(float(v)))

        speed_grid.addWidget(self._make_caption("Linear Speed"), 0, 0)
        speed_grid.addWidget(self.s_speed,                      0, 1)
        speed_grid.addWidget(self.sp_speed,                     0, 2)

        speed_grid.addWidget(self._make_caption("Turn Rate"),   1, 0)
        speed_grid.addWidget(self.s_turn,                       1, 1)
        speed_grid.addWidget(self.sp_turn,                      1, 2)

        speed_grid.setColumnStretch(1, 1)
        gc_outer.addLayout(speed_grid)

        crawler_vals = QGridLayout()
        crawler_vals.setHorizontalSpacing(8)
        crawler_vals.setVerticalSpacing(6)

        self.drive_speed_set_box = self._make_value_box("0.0000 m/s", "#eef2ff", "#1e3a8a")
        self.drive_turn_set_box = self._make_value_box(f"{self.turn_rate_mag:.3f} rad/s", "#eef2ff", "#1e3a8a")
        self.drive_linear_cmd_box = self._make_value_box("+0.0000 m/s", "#dcfce7", "#14532d")
        self.drive_angular_cmd_box = self._make_value_box("+0.000 rad/s", "#dbeafe", "#1e3a8a")

        crawler_vals.addWidget(self._make_caption("Speed Set"),   0, 0)
        crawler_vals.addWidget(self._make_caption("Turn Set"),    0, 1)
        crawler_vals.addWidget(self.drive_speed_set_box,          1, 0)
        crawler_vals.addWidget(self.drive_turn_set_box,           1, 1)

        crawler_vals.addWidget(self._make_caption("Linear Cmd"),  2, 0)
        crawler_vals.addWidget(self._make_caption("Angular Cmd"), 2, 1)
        crawler_vals.addWidget(self.drive_linear_cmd_box,         3, 0)
        crawler_vals.addWidget(self.drive_angular_cmd_box,        3, 1)

        gc_outer.addLayout(crawler_vals)
        gb_crawler.setLayout(gc_outer)

        # --------------------------------------------------
        # Active status
        # --------------------------------------------------
        status_group = QGroupBox("ACTIVE STATUS")
        status_layout = QVBoxLayout()
        status_layout.setSpacing(6)

        self.status_screen_title = QLabel("SYSTEM")
        self.status_screen_title.setAlignment(_qt_align_center())
        self.status_screen_title.setStyleSheet("""
            QLabel {
                background: #0f172a;
                color: #e2e8f0;
                border: 1px solid #334155;
                border-radius: 8px;
                padding: 6px;
                font-size: 13px;
                font-weight: 900;
            }
        """)

        self.status_screen_value = QLabel("READY")
        self.status_screen_value.setAlignment(_qt_align_center())
        self.status_screen_value.setMinimumHeight(90)
        self.status_screen_value.setStyleSheet("""
            QLabel {
                background: #020617;
                color: #22c55e;
                border: 2px solid #1e293b;
                border-radius: 10px;
                padding: 12px;
                font-size: 20px;
                font-weight: 900;
            }
        """)

        self.status_screen_detail = QLabel("No active state")
        self.status_screen_detail.setAlignment(_qt_align_center())
        self.status_screen_detail.setWordWrap(True)
        self.status_screen_detail.setStyleSheet("""
            QLabel {
                background: #111827;
                color: #cbd5e1;
                border: 1px solid #374151;
                border-radius: 8px;
                padding: 8px;
                font-size: 12px;
                font-weight: 700;
            }
        """)

        status_btn_row = QHBoxLayout()

        self.btn_status_prev = QPushButton("PREV")
        self.btn_status_prev.setFixedHeight(32)
        self.btn_status_prev.clicked.connect(self._prev_status_page)

        self.btn_status_next = QPushButton("NEXT")
        self.btn_status_next.setFixedHeight(32)
        self.btn_status_next.clicked.connect(self._next_status_page)

        self.cb_status_auto = QCheckBox("Auto Rotate")
        self.cb_status_auto.setChecked(True)

        status_btn_row.addWidget(self.btn_status_prev)
        status_btn_row.addWidget(self.btn_status_next)
        status_btn_row.addWidget(self.cb_status_auto)
        status_btn_row.addStretch(1)

        status_layout.addWidget(self.status_screen_title)
        status_layout.addWidget(self.status_screen_value)
        status_layout.addWidget(self.status_screen_detail)
        status_layout.addLayout(status_btn_row)

        status_group.setLayout(status_layout)

        # --------------------------------------------------
        # XYZ Jog Control
        # --------------------------------------------------
        gb_axes = QGroupBox("XYZ JOG CONTROL")
        ga = QGridLayout()
        ga.setContentsMargins(8, 8, 8, 8)
        ga.setHorizontalSpacing(6)
        ga.setVerticalSpacing(6)

        VALUE_W = 108
        BTN_W = 72
        SPIN_W = 120
        ROW_H = 30
        HEAD_H = 24

        def make_head(text):
            lbl = QLabel(text)
            lbl.setAlignment(_qt_align_center())
            lbl.setFixedHeight(HEAD_H)
            lbl.setStyleSheet("font-weight: 800; color: #334155;")
            return lbl

        def make_axis_name(text):
            lbl = QLabel(text)
            lbl.setAlignment(_qt_align_center())
            lbl.setFixedHeight(ROW_H)
            lbl.setStyleSheet("font-weight: 800; color: #0f172a;")
            return lbl

        def make_compact_box(text, bg, fg):
            lbl = QLabel(text)
            lbl.setAlignment(_qt_align_center())
            lbl.setFixedSize(VALUE_W, ROW_H)
            lbl.setStyleSheet(
                f"""
                QLabel {{
                    background: {bg};
                    color: {fg};
                    border: 1px solid #cbd5e1;
                    border-radius: 6px;
                    padding: 2px 6px;
                    font-weight: 800;
                }}
                """
            )
            return lbl

        def make_axis_btn(text, checkable=False):
            b = QPushButton(text)
            b.setCheckable(checkable)
            b.setFixedSize(BTN_W, ROW_H)
            return b

        def make_axis_spinbox(decimals, min_v, max_v, step, value, suffix):
            sp = QDoubleSpinBox()
            sp.setDecimals(decimals)
            sp.setRange(min_v, max_v)
            sp.setSingleStep(step)
            sp.setValue(value)
            sp.setSuffix(suffix)
            sp.setFixedSize(SPIN_W, ROW_H)
            return sp

        ga.addWidget(make_head("Axis"),   0, 0)
        ga.addWidget(make_head("Actual"), 0, 1)
        ga.addWidget(make_head("Cmd"),    0, 2)
        ga.addWidget(make_head("-"),      0, 3)
        ga.addWidget(make_head("STOP"),   0, 4)
        ga.addWidget(make_head("+"),      0, 5)
        ga.addWidget(make_head("Jog"),    0, 6)

        self.x_actual_box = make_compact_box("+0.0000 m", "#f3e8ff", "#6b21a8")
        self.x_cmd_box = make_compact_box("+0.0000 m", "#ede9fe", "#5b21b6")
        self.rate_x = make_axis_spinbox(3, 0.001, 1000.0, 1.0, self.x_rate_default_mm_s, " mm/s")

        self.btn_x_minus = make_axis_btn("X -", checkable=True)
        self.btn_x_stop = make_axis_btn("STOP", checkable=True)
        self.btn_x_plus = make_axis_btn("X +", checkable=True)

        self.x_group = QButtonGroup(self)
        self.x_group.setExclusive(True)
        self.x_group.addButton(self.btn_x_minus)
        self.x_group.addButton(self.btn_x_stop)
        self.x_group.addButton(self.btn_x_plus)

        self.btn_x_minus.clicked.connect(lambda: self._set_x_manual_mode(-1))
        self.btn_x_stop.clicked.connect(lambda: self._set_x_manual_mode(0))
        self.btn_x_plus.clicked.connect(lambda: self._set_x_manual_mode(+1))
        self.btn_x_stop.setChecked(True)

        ga.addWidget(make_axis_name("X"), 1, 0)
        ga.addWidget(self.x_actual_box,   1, 1)
        ga.addWidget(self.x_cmd_box,      1, 2)
        ga.addWidget(self.btn_x_minus,    1, 3)
        ga.addWidget(self.btn_x_stop,     1, 4)
        ga.addWidget(self.btn_x_plus,     1, 5)
        ga.addWidget(self.rate_x,         1, 6)

        self.y_actual_box = make_compact_box("+0.0000 m", "#dcfce7", "#14532d")
        self.y_cmd_box = make_compact_box("+0.0000 m", "#d1fae5", "#065f46")
        self.rate_y = make_axis_spinbox(3, 0.001, 1000.0, 1.0, self.y_rate_default_mm_s, " mm/s")

        self.btn_y_minus = make_axis_btn("Y -", checkable=True)
        self.btn_y_stop = make_axis_btn("STOP", checkable=True)
        self.btn_y_plus = make_axis_btn("Y +", checkable=True)

        self.y_group = QButtonGroup(self)
        self.y_group.setExclusive(True)
        self.y_group.addButton(self.btn_y_minus)
        self.y_group.addButton(self.btn_y_stop)
        self.y_group.addButton(self.btn_y_plus)

        self.btn_y_minus.clicked.connect(lambda: self._set_y_manual_mode(-1))
        self.btn_y_stop.clicked.connect(lambda: self._set_y_manual_mode(0))
        self.btn_y_plus.clicked.connect(lambda: self._set_y_manual_mode(+1))
        self.btn_y_stop.setChecked(True)

        ga.addWidget(make_axis_name("Y"), 2, 0)
        ga.addWidget(self.y_actual_box,   2, 1)
        ga.addWidget(self.y_cmd_box,      2, 2)
        ga.addWidget(self.btn_y_minus,    2, 3)
        ga.addWidget(self.btn_y_stop,     2, 4)
        ga.addWidget(self.btn_y_plus,     2, 5)
        ga.addWidget(self.rate_y,         2, 6)

        self.z_actual_box = make_compact_box("+0.00 deg", "#fff7ed", "#9a3412")
        self.z_cmd_box = make_compact_box("+0.00 deg", "#fef3c7", "#92400e")
        self.rate_z = make_axis_spinbox(2, 0.01, 360.0, 1.0, self.z_rate_default_deg_s, " deg/s")

        self.btn_z_minus = make_axis_btn("Z -", checkable=True)
        self.btn_z_stop = make_axis_btn("STOP", checkable=True)
        self.btn_z_plus = make_axis_btn("Z +", checkable=True)

        self.z_group = QButtonGroup(self)
        self.z_group.setExclusive(True)
        self.z_group.addButton(self.btn_z_minus)
        self.z_group.addButton(self.btn_z_stop)
        self.z_group.addButton(self.btn_z_plus)

        self.btn_z_minus.clicked.connect(lambda: self._set_z_manual_mode(-1))
        self.btn_z_stop.clicked.connect(lambda: self._set_z_manual_mode(0))
        self.btn_z_plus.clicked.connect(lambda: self._set_z_manual_mode(+1))
        self.btn_z_stop.setChecked(True)

        ga.addWidget(make_axis_name("Z"), 3, 0)
        ga.addWidget(self.z_actual_box,   3, 1)
        ga.addWidget(self.z_cmd_box,      3, 2)
        ga.addWidget(self.btn_z_minus,    3, 3)
        ga.addWidget(self.btn_z_stop,     3, 4)
        ga.addWidget(self.btn_z_plus,     3, 5)
        ga.addWidget(self.rate_z,         3, 6)

        for r in range(4):
            ga.setRowStretch(r, 0)
        ga.setRowStretch(4, 1)

        ga.setColumnMinimumWidth(0, 42)
        ga.setColumnMinimumWidth(1, VALUE_W)
        ga.setColumnMinimumWidth(2, VALUE_W)
        ga.setColumnMinimumWidth(3, BTN_W)
        ga.setColumnMinimumWidth(4, BTN_W)
        ga.setColumnMinimumWidth(5, BTN_W)
        ga.setColumnMinimumWidth(6, SPIN_W)

        gb_axes.setLayout(ga)

        controls_row = QHBoxLayout()
        controls_row.setSpacing(8)
        controls_row.addWidget(gb_crawler, 3)
        controls_row.addWidget(status_group, 2)
        controls_row.addWidget(gb_axes, 5)
        root.addLayout(controls_row)

        # --------------------------------------------------
        # X auto / tracking
        # --------------------------------------------------
        auto_row = QHBoxLayout()
        auto_row.setSpacing(8)

        auto_group = QGroupBox("X AXIS AUTO")
        auto_layout = QGridLayout()
        auto_layout.setHorizontalSpacing(8)
        auto_layout.setVerticalSpacing(6)

        self.x_auto_start = QDoubleSpinBox()
        self.x_auto_start.setDecimals(3)
        self.x_auto_start.setRange(-5000.0, 5000.0)
        self.x_auto_start.setValue(0.0)
        self.x_auto_start.setSuffix(" mm")

        self.x_auto_end = QDoubleSpinBox()
        self.x_auto_end.setDecimals(3)
        self.x_auto_end.setRange(-5000.0, 5000.0)
        self.x_auto_end.setValue(100.0)
        self.x_auto_end.setSuffix(" mm")

        self.x_auto_speed = QDoubleSpinBox()
        self.x_auto_speed.setDecimals(3)
        self.x_auto_speed.setRange(0.1, 10000.0)
        self.x_auto_speed.setValue(10.0)
        self.x_auto_speed.setSuffix(" mm/s")

        self.btn_x_auto = QPushButton("Start X Auto")
        self.btn_x_auto.setCheckable(True)
        self.btn_x_auto.setFixedHeight(36)
        self.btn_x_auto.toggled.connect(self.on_x_auto_toggled)

        auto_layout.addWidget(self._make_caption("Start"), 0, 0)
        auto_layout.addWidget(self.x_auto_start,           0, 1)
        auto_layout.addWidget(self._make_caption("End"),   1, 0)
        auto_layout.addWidget(self.x_auto_end,             1, 1)
        auto_layout.addWidget(self._make_caption("Speed"), 2, 0)
        auto_layout.addWidget(self.x_auto_speed,           2, 1)
        auto_layout.addWidget(self.btn_x_auto,             0, 2, 3, 1)

        auto_group.setLayout(auto_layout)
        auto_row.addWidget(auto_group, 1)

        track_group = QGroupBox("X AXIS TRACKING")
        track_layout = QGridLayout()
        track_layout.setHorizontalSpacing(8)
        track_layout.setVerticalSpacing(6)

        self.x_track_gain = QDoubleSpinBox()
        self.x_track_gain.setDecimals(3)
        self.x_track_gain.setRange(0.0, 10.0)
        self.x_track_gain.setSingleStep(0.1)
        self.x_track_gain.setValue(1.0)
        self.x_track_gain.setSuffix(" mm/mm")

        self.x_track_max_step = QDoubleSpinBox()
        self.x_track_max_step.setDecimals(3)
        self.x_track_max_step.setRange(0.01, 100.0)
        self.x_track_max_step.setSingleStep(0.5)
        self.x_track_max_step.setValue(2.0)
        self.x_track_max_step.setSuffix(" mm/step")

        self.btn_x_track = QPushButton("Start X Track")
        self.btn_x_track.setCheckable(True)
        self.btn_x_track.setFixedHeight(36)
        self.btn_x_track.toggled.connect(self.on_x_track_toggled)

        self.lbl_line_error = self._make_value_box("+0.000 mm", "#fef9c3", "#854d0e")

        track_layout.addWidget(self._make_caption("Gain"),       0, 0)
        track_layout.addWidget(self.x_track_gain,                0, 1)
        track_layout.addWidget(self._make_caption("Max Step"),   1, 0)
        track_layout.addWidget(self.x_track_max_step,            1, 1)
        track_layout.addWidget(self._make_caption("Line Error"), 2, 0)
        track_layout.addWidget(self.lbl_line_error,              2, 1)
        track_layout.addWidget(self.btn_x_track,                 0, 2, 3, 1)

        track_group.setLayout(track_layout)
        auto_row.addWidget(track_group, 1)

        root.addLayout(auto_row)

        # --------------------------------------------------
        # Action bar
        # --------------------------------------------------
        action_group = QGroupBox("ACTION BAR")
        action_layout = QHBoxLayout()
        action_layout.setSpacing(8)

        self.btn_enable = QPushButton("Disable")
        self.btn_enable.setFixedHeight(34)
        self.btn_enable.clicked.connect(self._toggle_enable)

        self.btn_estop = QPushButton("E-STOP")
        self.btn_estop.setFixedHeight(34)
        self.btn_estop.setCheckable(True)
        self.btn_estop.clicked.connect(self._toggle_estop)

        self.btn_hard_stop = QPushButton("STOP NOW")
        self.btn_hard_stop.setFixedHeight(34)
        self.btn_hard_stop.clicked.connect(self._hard_stop)

        self.btn_reset_xyz = QPushButton("Reset XYZ to 0")
        self.btn_reset_xyz.setFixedHeight(34)
        self.btn_reset_xyz.clicked.connect(self.reset_xyz)

        self.cb_publish_drive = QCheckBox("Publish DRIVE")
        self.cb_publish_drive.setChecked(True)

        self.cb_publish_joints = QCheckBox("Publish XYZ")
        self.cb_publish_joints.setChecked(True)

        action_layout.addWidget(self.btn_enable)
        action_layout.addWidget(self.btn_estop)
        action_layout.addWidget(self.btn_hard_stop)
        action_layout.addWidget(self.btn_reset_xyz)
        action_layout.addStretch(1)
        action_layout.addWidget(self.cb_publish_drive)
        action_layout.addWidget(self.cb_publish_joints)

        action_group.setLayout(action_layout)
        root.addWidget(action_group)

        # --------------------------------------------------
        # Plots
        # --------------------------------------------------
        plots_widget = QWidget()
        plots_layout = QHBoxLayout(plots_widget)
        plots_layout.setContentsMargins(0, 0, 0, 0)
        plots_layout.setSpacing(8)

        raw_group = QGroupBox("RAW LVS PROFILE")
        raw_layout = QVBoxLayout()
        raw_layout.setContentsMargins(4, 4, 4, 4)
        raw_layout.setSpacing(4)
        self.lvs_viewer = LVSProfileViewer()
        self.lvs_viewer.clear(f"Waiting for {self.lvs_topic} ...")
        raw_layout.addWidget(self.lvs_viewer)
        raw_group.setLayout(raw_layout)

        filtered_group = QGroupBox("FILTERED LVS PROFILE")
        filtered_layout = QVBoxLayout()
        filtered_layout.setContentsMargins(4, 4, 4, 4)
        filtered_layout.setSpacing(4)
        self.lbl_filtered_status = self._make_caption(f"Topic: {self.filtered_lvs_topic} | waiting ...")
        self.lvs_filtered_viewer = LVSProfileViewer()
        self.lvs_filtered_viewer.clear(f"Waiting for {self.filtered_lvs_topic} ...")
        filtered_layout.addWidget(self.lbl_filtered_status)
        filtered_layout.addWidget(self.lvs_filtered_viewer)
        filtered_group.setLayout(filtered_layout)

        plots_layout.addWidget(raw_group, 1)
        plots_layout.addWidget(filtered_group, 1)
        root.addWidget(plots_widget, 1)

        scroll.setWidget(content)
        outer.addWidget(scroll)
        self.setLayout(outer)

    # ==========================================================
    # Status helpers
    # ==========================================================
    def _status_pages(self):
        linear_x, angular_z = self._current_drive_cmd()
        pages = []

        if self.estop:
            pages.append({
                "title": "SAFETY",
                "value": "E-STOP ACTIVE",
                "detail": "Emergency stop is active. All motion commands are blocked."
            })

        if not self.enabled:
            pages.append({
                "title": "SYSTEM",
                "value": "DISABLED",
                "detail": "System is disabled."
            })

        if self.motion_mode != "stop":
            crawler_state = {
                "forward": "FORWARD",
                "reverse": "REVERSE",
                "left": "LEFT",
                "right": "RIGHT",
            }.get(self.motion_mode, "ACTIVE")
            pages.append({
                "title": "CRAWLER",
                "value": crawler_state,
                "detail": f"Linear={linear_x:+.4f} m/s | Angular={angular_z:+.3f} rad/s"
            })

        if self.x_track_mode:
            pages.append({
                "title": "X AXIS",
                "value": "TRACKING",
                "detail": f"Line error={self.latest_line_error_mm:+.3f} mm"
            })
        elif self.x_auto_running:
            pages.append({
                "title": "X AXIS",
                "value": "AUTO",
                "detail": (
                    f"Start={self.x_auto_start.value():.3f} mm | "
                    f"End={self.x_auto_end.value():.3f} mm | "
                    f"Speed={self.x_auto_speed.value():.3f} mm/s"
                )
            })
        elif self.x_manual_mode != 0:
            pages.append({
                "title": "X AXIS",
                "value": "JOG -" if self.x_manual_mode < 0 else "JOG +",
                "detail": f"Cmd={self.current_positions[0]:+.4f} m"
            })

        if self.y_manual_mode != 0:
            pages.append({
                "title": "Y AXIS",
                "value": "JOG -" if self.y_manual_mode < 0 else "JOG +",
                "detail": f"Cmd={self.current_positions[1]:+.4f} m"
            })

        if self.z_manual_mode != 0:
            pages.append({
                "title": "Z AXIS",
                "value": "JOG -" if self.z_manual_mode < 0 else "JOG +",
                "detail": f"Cmd={math.degrees(self.current_positions[2]):+.2f} deg"
            })

        if abs(self.latest_line_error_mm) > 0.001 and not self.x_track_mode:
            pages.append({
                "title": "LINE ERROR",
                "value": f"{self.latest_line_error_mm:+.3f} mm",
                "detail": "Tracking feedback available"
            })

        if not self.cb_publish_drive.isChecked():
            pages.append({
                "title": "PUBLISH",
                "value": "DRIVE OFF",
                "detail": "Drive command publishing is disabled."
            })

        if not self.cb_publish_joints.isChecked():
            pages.append({
                "title": "PUBLISH",
                "value": "XYZ OFF",
                "detail": "XYZ command publishing is disabled."
            })

        if not pages:
            pages.append({
                "title": "SYSTEM",
                "value": "READY",
                "detail": "No active state. Waiting for operator input."
            })

        return pages

    def _set_status_monitor_color(self, title: str, value: str):
        bg = "#020617"
        border = "#1e293b"
        fg = "#22c55e"

        txt = f"{title} {value}".upper()

        if "E-STOP" in txt:
            fg = "#ef4444"
            border = "#7f1d1d"
        elif "DISABLED" in txt:
            fg = "#f97316"
            border = "#9a3412"
        elif "OFF" in txt:
            fg = "#f59e0b"
            border = "#92400e"
        elif "ERROR" in txt:
            fg = "#f59e0b"
            border = "#92400e"
        elif "STOP" in txt:
            fg = "#eab308"
            border = "#713f12"
        elif "LEFT" in txt or "RIGHT" in txt:
            fg = "#60a5fa"
            border = "#1d4ed8"
        elif "TRACK" in txt or "AUTO" in txt:
            fg = "#c084fc"
            border = "#6b21a8"
        elif "JOG" in txt:
            fg = "#38bdf8"
            border = "#0369a1"
        elif "READY" in txt:
            fg = "#22c55e"
            border = "#166534"

        self.status_screen_value.setStyleSheet(
            f"""
            QLabel {{
                background: {bg};
                color: {fg};
                border: 2px solid {border};
                border-radius: 10px;
                padding: 12px;
                font-size: 20px;
                font-weight: 900;
            }}
            """
        )

    def _update_status_screen(self):
        pages = self._status_pages()
        if not pages:
            return

        self.status_page_index %= len(pages)
        page = pages[self.status_page_index]

        self.status_screen_title.setText(page["title"])
        self.status_screen_value.setText(page["value"])
        self.status_screen_detail.setText(page["detail"])
        self._set_status_monitor_color(page["title"], page["value"])

    def _next_status_page(self):
        pages = self._status_pages()
        if not pages:
            return
        self.status_page_index = (self.status_page_index + 1) % len(pages)
        self._update_status_screen()

    def _prev_status_page(self):
        pages = self._status_pages()
        if not pages:
            return
        self.status_page_index = (self.status_page_index - 1) % len(pages)
        self._update_status_screen()

    # ==========================================================
    # Joint / init helpers
    # ==========================================================
    def _actual_x(self):
        return self.actual_positions[self.x_joint_name]

    def _actual_y(self):
        return self.actual_positions[self.y_joint_name]

    def _actual_z(self):
        return self.actual_positions[self.z_joint_name]

    def _hold_actual_xyz(self):
        if self.have_joint_state:
            self.current_positions[0] = self._actual_x()
            self.current_positions[1] = self._actual_y()
            self.current_positions[2] = self._actual_z()

    def _sync_cmd_to_actual(self):
        if not self.have_joint_state:
            return False

        self.current_positions[0] = self._actual_x()
        self.current_positions[1] = self._actual_y()
        self.current_positions[2] = self._actual_z()
        self.xyz_initialized = True
        self.warned_waiting_for_joint_state = False
        return True

    def _ensure_xyz_initialized(self):
        if self.xyz_initialized:
            return True

        if self.have_joint_state:
            return self._sync_cmd_to_actual()

        if not self.warned_waiting_for_joint_state:
            self.get_logger().warn("Ignoring XYZ jog command until /joint_states is received.")
            self.warned_waiting_for_joint_state = True

        return False

    def _on_joint_states(self, msg: JointState):
        if len(msg.name) != len(msg.position):
            return

        name_to_pos = dict(zip(msg.name, msg.position))
        updated = False

        for joint_name in (self.x_joint_name, self.y_joint_name, self.z_joint_name):
            if joint_name in name_to_pos:
                self.actual_positions[joint_name] = float(name_to_pos[joint_name])
                updated = True

        if updated:
            self.have_joint_state = True

            if not self.xyz_initialized:
                self.current_positions[0] = self._actual_x()
                self.current_positions[1] = self._actual_y()
                self.current_positions[2] = self._actual_z()
                self.xyz_initialized = True
                self.warned_waiting_for_joint_state = False
                self.get_logger().info("XYZ command initialized from joint states.")

    # ==========================================================
    # LVS callbacks
    # ==========================================================
    def _on_lvs_profile_raw(self, msg: LvsProfile):
        self.raw_lvs_msg_count += 1

        xs = list(msg.x_mm)
        zs = list(msg.z_mm)
        intensities = list(msg.intensity)

        if not xs or not zs:
            self.raw_lvs_last_points = 0
            self.lvs_viewer.clear(f"No raw profile data from {self.lvs_topic}")
            return

        n = min(len(xs), len(zs))
        self.raw_lvs_last_points = n

        xs = xs[:n]
        zs = zs[:n]
        intensities = intensities[:n] if len(intensities) >= n else []

        indexed = list(range(n))
        indexed.sort(key=lambda i: xs[i] if is_finite(xs[i]) else float("inf"))

        xs_sorted = [xs[i] for i in indexed]
        zs_sorted = [zs[i] for i in indexed]
        intensities_sorted = [intensities[i] for i in indexed] if len(intensities) == n else []

        self.lvs_viewer.update_profile(
            xs_sorted,
            zs_sorted,
            intensity_list=intensities_sorted,
            precision=msg.precision,
            valid=msg.valid,
            unit=self.lvs_unit,
            status_text=f"RAW {self.lvs_topic} | msgs={self.raw_lvs_msg_count} | points={n}"
        )

    def _on_lvs_profile_filtered(self, msg: LvsProfile):
        self.filtered_lvs_msg_count += 1

        xs = list(msg.x_mm)
        zs = list(msg.z_mm)
        intensities = list(msg.intensity)

        if not xs or not zs:
            self.filtered_lvs_last_points = 0
            self.lbl_filtered_status.setText(
                f"Topic: {self.filtered_lvs_topic} | msgs={self.filtered_lvs_msg_count} | no data"
            )
            self.lvs_filtered_viewer.clear(f"No filtered profile data from {self.filtered_lvs_topic}")
            return

        n = min(len(xs), len(zs))
        self.filtered_lvs_last_points = n

        xs = xs[:n]
        zs = zs[:n]
        intensities = intensities[:n] if len(intensities) >= n else []

        indexed = list(range(n))
        indexed.sort(key=lambda i: xs[i] if is_finite(xs[i]) else float("inf"))

        xs_sorted = [xs[i] for i in indexed]
        zs_sorted = [zs[i] for i in indexed]
        intensities_sorted = [intensities[i] for i in indexed] if len(intensities) == n else []

        self.lbl_filtered_status.setText(
            f"Topic: {self.filtered_lvs_topic} | msgs={self.filtered_lvs_msg_count} | points={n}"
        )

        self.lvs_filtered_viewer.update_profile(
            xs_sorted,
            zs_sorted,
            intensity_list=intensities_sorted,
            precision=msg.precision,
            valid=msg.valid,
            unit=self.lvs_unit,
            status_text=f"FILTERED {self.filtered_lvs_topic} | msgs={self.filtered_lvs_msg_count} | points={n}"
        )

    # ==========================================================
    # Common
    # ==========================================================
    def _set_button_checked_safely(self, btn, checked):
        if btn.isChecked() != checked:
            btn.blockSignals(True)
            btn.setChecked(checked)
            btn.blockSignals(False)

    def _stop_xyz_modes(self):
        self.x_auto_running = False
        self.x_track_mode = False
        self._set_button_checked_safely(self.btn_x_auto, False)
        self._set_button_checked_safely(self.btn_x_track, False)
        self.btn_x_auto.setText("Start X Auto")
        self.btn_x_track.setText("Start X Track")

    # ==========================================================
    # Crawler control
    # ==========================================================
    def _set_motion_mode(self, mode: str):
        self.motion_mode = mode
        self._set_button_checked_safely(self.btn_forward, mode == "forward")
        self._set_button_checked_safely(self.btn_reverse, mode == "reverse")
        self._set_button_checked_safely(self.btn_left, mode == "left")
        self._set_button_checked_safely(self.btn_right, mode == "right")
        self._set_button_checked_safely(self.btn_stop, mode == "stop")
        self._refresh()

    def _set_speed_mag(self, mag: float):
        self.speed_mag = clamp(float(mag), 0.0, self.max_speed)
        sval = int(round(self.speed_mag * 10000))

        if self.s_speed.value() != sval:
            self.s_speed.blockSignals(True)
            self.s_speed.setValue(sval)
            self.s_speed.blockSignals(False)

        if abs(self.sp_speed.value() - self.speed_mag) > 1e-9:
            self.sp_speed.blockSignals(True)
            self.sp_speed.setValue(self.speed_mag)
            self.sp_speed.blockSignals(False)

        self._refresh()

    def _set_turn_rate_mag(self, val: float):
        self.turn_rate_mag = clamp(float(val), 0.0, self.max_yaw)
        sval = int(round(self.turn_rate_mag * 1000))

        if self.s_turn.value() != sval:
            self.s_turn.blockSignals(True)
            self.s_turn.setValue(sval)
            self.s_turn.blockSignals(False)

        if abs(self.sp_turn.value() - self.turn_rate_mag) > 1e-9:
            self.sp_turn.blockSignals(True)
            self.sp_turn.setValue(self.turn_rate_mag)
            self.sp_turn.blockSignals(False)

        self._refresh()

    def _current_drive_cmd(self):
        if not self.enabled or self.estop or self.motion_mode == "stop":
            return 0.0, 0.0

        if self.motion_mode == "forward":
            return self.speed_mag, 0.0
        if self.motion_mode == "reverse":
            return -self.speed_mag, 0.0
        if self.motion_mode == "left":
            return 0.0, self.turn_rate_mag
        if self.motion_mode == "right":
            return 0.0, -self.turn_rate_mag

        return 0.0, 0.0

    def _publish_drive(self, force_stop=False):
        if not self.cb_publish_drive.isChecked():
            return

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()

        if force_stop or (not self.enabled) or self.estop:
            msg.twist.linear.x = 0.0
            msg.twist.angular.z = 0.0
        else:
            linear_x, angular_z = self._current_drive_cmd()
            msg.twist.linear.x = float(linear_x)
            msg.twist.angular.z = float(angular_z)

        self.pub_drive.publish(msg)

    # ==========================================================
    # XYZ control
    # ==========================================================
    def _publish_xyz(self):
        if not self.cb_publish_joints.isChecked():
            return

        if not self.have_joint_state or not self.xyz_initialized:
            return

        x, y, z = self.current_positions

        if self.use_individual_axis_topics:
            mx = Float64MultiArray()
            my = Float64MultiArray()
            mz = Float64MultiArray()
            mx.data = [float(x)]
            my.data = [float(y)]
            mz.data = [float(z)]
            self.x_pub.publish(mx)
            self.y_pub.publish(my)
            self.z_pub.publish(mz)
        else:
            msg = Float64MultiArray()
            msg.data = [float(x), float(y), float(z)]
            self.joint_pub.publish(msg)

    def _set_x_manual_mode(self, mode: int):
        if not self._ensure_xyz_initialized():
            return

        if mode != 0:
            self._stop_xyz_modes()
            self.current_positions[0] = self._actual_x()

        self.x_manual_mode = int(mode)
        self._set_button_checked_safely(self.btn_x_minus, mode < 0)
        self._set_button_checked_safely(self.btn_x_stop, mode == 0)
        self._set_button_checked_safely(self.btn_x_plus, mode > 0)

        if mode == 0:
            self.current_positions[0] = self._actual_x()
            self._publish_xyz()

        self._refresh()

    def _set_y_manual_mode(self, mode: int):
        if not self._ensure_xyz_initialized():
            return

        if mode != 0:
            self.current_positions[1] = self._actual_y()

        self.y_manual_mode = int(mode)
        self._set_button_checked_safely(self.btn_y_minus, mode < 0)
        self._set_button_checked_safely(self.btn_y_stop, mode == 0)
        self._set_button_checked_safely(self.btn_y_plus, mode > 0)

        if mode == 0:
            self.current_positions[1] = self._actual_y()
            self._publish_xyz()

        self._refresh()

    def _set_z_manual_mode(self, mode: int):
        if not self._ensure_xyz_initialized():
            return

        if mode != 0:
            self.current_positions[2] = self._actual_z()

        self.z_manual_mode = int(mode)
        self._set_button_checked_safely(self.btn_z_minus, mode < 0)
        self._set_button_checked_safely(self.btn_z_stop, mode == 0)
        self._set_button_checked_safely(self.btn_z_plus, mode > 0)

        if mode == 0:
            self.current_positions[2] = self._actual_z()
            self._publish_xyz()

        self._refresh()

    def _run_x_manual(self, dt):
        if self.x_manual_mode == 0:
            return False
        rate_m_s = self.rate_x.value() * 0.001
        delta = self.x_manual_mode * rate_m_s * dt
        new_pos = clamp(self.current_positions[0] + delta, self.x_min, self.x_max)
        if abs(new_pos - self.current_positions[0]) < 1e-12:
            return False
        self.current_positions[0] = new_pos
        return True

    def _run_y_manual(self, dt):
        if self.y_manual_mode == 0:
            return False
        rate_m_s = self.rate_y.value() * 0.001
        delta = self.y_manual_mode * rate_m_s * dt
        new_pos = clamp(self.current_positions[1] + delta, self.y_min, self.y_max)
        if abs(new_pos - self.current_positions[1]) < 1e-12:
            return False
        self.current_positions[1] = new_pos
        return True

    def _run_z_manual(self, dt):
        if self.z_manual_mode == 0:
            return False
        rate_rad_s = math.radians(self.rate_z.value())
        delta = self.z_manual_mode * rate_rad_s * dt
        new_pos = clamp(self.current_positions[2] + delta, self.z_min, self.z_max)
        if abs(new_pos - self.current_positions[2]) < 1e-12:
            return False
        self.current_positions[2] = new_pos
        return True

    def reset_xyz(self):
        self._stop_xyz_modes()
        self.x_manual_mode = 0
        self.y_manual_mode = 0
        self.z_manual_mode = 0

        self._set_button_checked_safely(self.btn_x_stop, True)
        self._set_button_checked_safely(self.btn_y_stop, True)
        self._set_button_checked_safely(self.btn_z_stop, True)

        if self.have_joint_state:
            self.current_positions[0] = self._actual_x()
            self.current_positions[1] = self._actual_y()
            self.current_positions[2] = self._actual_z()
            self.xyz_initialized = True
        else:
            self.current_positions = [0.0, 0.0, 0.0]
            self.xyz_initialized = False

        self._refresh()
        self._publish_xyz()

    # ==========================================================
    # X auto / tracking
    # ==========================================================
    def on_x_auto_toggled(self, checked: bool):
        if checked:
            if not self._ensure_xyz_initialized():
                self._set_button_checked_safely(self.btn_x_auto, False)
                return

            self._set_x_manual_mode(0)

            if self.x_track_mode:
                self._set_button_checked_safely(self.btn_x_track, False)
                self.x_track_mode = False
                self.btn_x_track.setText("Start X Track")

            start_m = self.x_auto_start.value() * 0.001
            end_m = self.x_auto_end.value() * 0.001

            if start_m > end_m:
                start_m, end_m = end_m, start_m

            start_m = clamp(start_m, self.x_min, self.x_max)
            end_m = clamp(end_m, self.x_min, self.x_max)

            self.x_auto_start_m = start_m
            self.x_auto_end_m = end_m

            dist_mm = abs(end_m - start_m) * 1000.0
            speed_mm_s = max(self.x_auto_speed.value(), 0.1)
            self.x_auto_period = max(dist_mm / speed_mm_s, 0.1) if dist_mm > 1e-9 else 1.0

            self.x_auto_phase = 0.0
            self.x_auto_direction = +1
            self.current_positions[0] = self.x_auto_start_m
            self.x_auto_running = True
            self.btn_x_auto.setText("Stop X Auto")

            self._refresh()
            self._publish_xyz()
        else:
            self.x_auto_running = False
            self.btn_x_auto.setText("Start X Auto")
            self._refresh()

    def _run_x_auto(self, dt):
        if not self.x_auto_running:
            return False

        span = self.x_auto_end_m - self.x_auto_start_m
        if abs(span) < 1e-12:
            self.current_positions[0] = self.x_auto_start_m
            return True

        dphase = dt / max(self.x_auto_period, 1e-6)

        if self.x_auto_direction > 0:
            self.x_auto_phase += dphase
            if self.x_auto_phase >= 1.0:
                self.x_auto_phase = 1.0
                self.x_auto_direction = -1
        else:
            self.x_auto_phase -= dphase
            if self.x_auto_phase <= 0.0:
                self.x_auto_phase = 0.0
                self.x_auto_direction = +1

        self.current_positions[0] = clamp(
            self.x_auto_start_m + self.x_auto_phase * (self.x_auto_end_m - self.x_auto_start_m),
            self.x_min,
            self.x_max,
        )
        return True

    def line_error_callback(self, msg: Float64):
        self.latest_line_error_mm = float(msg.data)

    def on_x_track_toggled(self, checked: bool):
        if checked:
            if not self._ensure_xyz_initialized():
                self._set_button_checked_safely(self.btn_x_track, False)
                return

            self._set_x_manual_mode(0)

            if self.x_auto_running:
                self._set_button_checked_safely(self.btn_x_auto, False)
                self.x_auto_running = False
                self.btn_x_auto.setText("Start X Auto")

            self.x_track_mode = True
            self.x_track_base = self._actual_x() if self.have_joint_state else self.current_positions[0]
            self.btn_x_track.setText("Stop X Track")
        else:
            self.x_track_mode = False
            self.btn_x_track.setText("Start X Track")

        self._refresh()

    def _run_x_track(self):
        if not self.x_track_mode:
            return False

        err_mm = self.latest_line_error_mm
        gain = self.x_track_gain.value()
        correction_mm = gain * err_mm
        desired_x = clamp(self.x_track_base + correction_mm * 0.001, self.x_min, self.x_max)

        max_step_m = max(self.x_track_max_step.value() * 0.001, 1e-6)
        current_x = self._actual_x() if self.have_joint_state else self.current_positions[0]
        delta = desired_x - current_x

        if delta > max_step_m:
            delta = max_step_m
        elif delta < -max_step_m:
            delta = -max_step_m

        self.current_positions[0] = clamp(current_x + delta, self.x_min, self.x_max)
        return True

    # ==========================================================
    # System actions
    # ==========================================================
    def _toggle_enable(self):
        self.enabled = not self.enabled
        self.btn_enable.setText("Disable" if self.enabled else "Enable")
        if not self.enabled:
            self._set_motion_mode("stop")
            self._stop_xyz_modes()
            self._set_x_manual_mode(0)
            self._set_y_manual_mode(0)
            self._set_z_manual_mode(0)
            self._hold_actual_xyz()
            self._publish_xyz()
        self._refresh()

    def _toggle_estop(self):
        self.estop = self.btn_estop.isChecked()
        if self.estop:
            self._set_motion_mode("stop")
            self._stop_xyz_modes()
            self._set_x_manual_mode(0)
            self._set_y_manual_mode(0)
            self._set_z_manual_mode(0)
            self._hold_actual_xyz()
            self._publish_drive(force_stop=True)
            self._publish_xyz()
        self._refresh()

    def _hard_stop(self):
        self._set_motion_mode("stop")
        self._stop_xyz_modes()
        self._set_x_manual_mode(0)
        self._set_y_manual_mode(0)
        self._set_z_manual_mode(0)
        self._hold_actual_xyz()
        self._publish_drive(force_stop=True)
        self._publish_xyz()
        self._refresh()

    # ==========================================================
    # Refresh / tick
    # ==========================================================
    def _refresh(self):
        linear_x, angular_z = self._current_drive_cmd()

        self.drive_speed_set_box.setText(f"{self.speed_mag:.4f} m/s")
        self.drive_turn_set_box.setText(f"{self.turn_rate_mag:.3f} rad/s")
        self.drive_linear_cmd_box.setText(f"{linear_x:+.4f} m/s")
        self.drive_angular_cmd_box.setText(f"{angular_z:+.3f} rad/s")

        self.x_cmd_box.setText(f"{self.current_positions[0]:+.4f} m")
        self.y_cmd_box.setText(f"{self.current_positions[1]:+.4f} m")
        self.z_cmd_box.setText(f"{math.degrees(self.current_positions[2]):+.2f} deg")

        if self.have_joint_state:
            self.x_actual_box.setText(f"{self._actual_x():+.4f} m")
            self.y_actual_box.setText(f"{self._actual_y():+.4f} m")
            self.z_actual_box.setText(f"{math.degrees(self._actual_z()):+.2f} deg")
        else:
            self.x_actual_box.setText("---")
            self.y_actual_box.setText("---")
            self.z_actual_box.setText("---")

        self.lbl_line_error.setText(f"{self.latest_line_error_mm:+.3f} mm")

        if self.filtered_lvs_msg_count == 0:
            self.lbl_filtered_status.setText(f"Topic: {self.filtered_lvs_topic} | waiting ...")

        self._update_status_screen()

    def _tick(self):
        for _ in range(20):
            rclpy.spin_once(self, timeout_sec=0.0)

        dt = self.timer.interval() / 1000.0
        xyz_changed = False

        if self.enabled and not self.estop:
            if self.x_track_mode:
                xyz_changed |= self._run_x_track()
            elif self.x_auto_running:
                xyz_changed |= self._run_x_auto(dt)
            else:
                xyz_changed |= self._run_x_manual(dt)

            xyz_changed |= self._run_y_manual(dt)
            xyz_changed |= self._run_z_manual(dt)

        if self.cb_status_auto.isChecked():
            pages = self._status_pages()
            if len(pages) > 1:
                self.status_rotate_counter += 1
                if self.status_rotate_counter >= self.status_rotate_every_ticks:
                    self.status_rotate_counter = 0
                    self.status_page_index = (self.status_page_index + 1) % len(pages)
            else:
                self.status_rotate_counter = 0
        else:
            self.status_rotate_counter = 0

        self._refresh()
        self._publish_drive(force_stop=False)

        if xyz_changed:
            self._publish_xyz()


def main():
    rclpy.init(args=sys.argv)
    app = QApplication(sys.argv)
    w = CombinedTeleopGUI()
    w.showMaximized()

    try:
        sys.exit(_app_exec(app))
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