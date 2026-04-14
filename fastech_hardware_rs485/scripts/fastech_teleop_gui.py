#!/usr/bin/env python3
import sys
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from lvs_driver.msg import LvsProfile

try:
    from PyQt5.QtCore import Qt, QTimer, QSize, QPointF
    from PyQt5.QtGui import QPainter, QPen, QFont, QColor, QPolygonF
    from PyQt5.QtWidgets import (
        QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
        QSlider, QDoubleSpinBox, QGroupBox, QGridLayout, QCheckBox,
        QButtonGroup, QSizePolicy
    )
    QT_VER = 5
except Exception:
    from PyQt6.QtCore import Qt, QTimer, QSize, QPointF
    from PyQt6.QtGui import QPainter, QPen, QFont, QColor, QPolygonF
    from PyQt6.QtWidgets import (
        QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
        QSlider, QDoubleSpinBox, QGroupBox, QGridLayout, QCheckBox,
        QButtonGroup, QSizePolicy
    )
    QT_VER = 6


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


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


class SteeringWatch(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.value_deg = 0.0
        self.max_deg = 90.0
        self.setMinimumSize(QSize(220, 220))
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def set_value(self, value_deg: float, max_deg: float):
        self.value_deg = float(value_deg)
        self.max_deg = max(1e-6, float(max_deg))
        self.update()

    def _map_to_theta(self, value_deg):
        v = clamp(value_deg, -self.max_deg, self.max_deg)
        return -90.0 + (v / self.max_deg) * 135.0

    def paintEvent(self, _):
        w, h = self.width(), self.height()
        s = min(w, h)
        cx, cy = w * 0.5, h * 0.5
        r = s * 0.42

        p = QPainter(self)
        p.setRenderHint(_qpainter_antialiasing(), True)

        pen = QPen()
        pen.setWidth(3)
        p.setPen(pen)
        p.drawEllipse(int(cx - r), int(cy - r), int(2 * r), int(2 * r))

        pen2 = QPen()
        pen2.setWidth(2)
        p.setPen(pen2)
        for i in range(-5, 6):
            theta = -90.0 + (i / 5.0) * 135.0
            rad = math.radians(theta)
            x1 = cx + (r * 0.78) * math.cos(rad)
            y1 = cy + (r * 0.78) * math.sin(rad)
            x2 = cx + (r * 0.92) * math.cos(rad)
            y2 = cy + (r * 0.92) * math.sin(rad)
            p.drawLine(int(x1), int(y1), int(x2), int(y2))

        theta = self._map_to_theta(self.value_deg)
        rad = math.radians(theta)
        nx = cx + (r * 0.75) * math.cos(rad)
        ny = cy + (r * 0.75) * math.sin(rad)
        pen3 = QPen()
        pen3.setWidth(4)
        p.setPen(pen3)
        p.drawLine(int(cx), int(cy), int(nx), int(ny))

        font = QFont()
        font.setPointSize(11)
        font.setBold(True)
        p.setFont(font)
        p.drawText(
            int(cx - r * 0.65), int(cy + r * 0.25), int(r * 1.3), 30,
            _qt_align_center(), f"{self.value_deg:+.1f}°/s"
        )


class LVSProfileViewer(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.xs = []
        self.zs = []
        self.intensities = []
        self.precision = None
        self.valid = True
        self.unit = "mm"
        self.status_text = "Waiting for /lvs/profile ..."
        self.setMinimumHeight(330)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def clear(self, status_text="Waiting for /lvs/profile ..."):
        self.xs = []
        self.zs = []
        self.intensities = []
        self.precision = None
        self.valid = True
        self.status_text = status_text
        self.update()

    def update_profile(self, xs, zs, intensity_list=None, precision=None, valid=True, unit="mm", status_text=""):
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

        bg_color = QColor(0, 0, 0)
        grid_color = QColor(55, 55, 55)
        axis_color = QColor(70, 70, 70)
        text_color = QColor(210, 210, 210)
        line_color = QColor(0, 255, 0)
        point_color = QColor(255, 220, 0)
        min_outer_color = QColor(255, 0, 0)
        min_inner_color = QColor(255, 220, 0)

        p.fillRect(0, 0, w, h, bg_color)

        left = 52
        right = 18
        top = 70
        bottom = 42

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

        if len(self.xs) < 2 or len(self.zs) < 2:
            p.setPen(text_color)
            font = QFont()
            font.setPointSize(12)
            font.setBold(True)
            p.setFont(font)
            p.drawText(plot_x, plot_y, plot_w, plot_h, _qt_align_center(), self.status_text)
            return

        n = min(len(self.xs), len(self.zs))
        xs = self.xs[:n]
        zs = self.zs[:n]

        x_min = min(xs)
        x_max = max(xs)
        z_min = min(zs)
        z_max = max(zs)

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

        poly = QPolygonF()
        for xv, zv in zip(xs, zs):
            poly.append(QPointF(map_x(xv), map_z(zv)))

        p.setPen(QPen(line_color, 2))
        p.drawPolyline(poly)

        p.setPen(QPen(point_color, 1))
        p.setBrush(point_color)
        for xv, zv in zip(xs, zs):
            p.drawEllipse(QPointF(map_x(xv), map_z(zv)), 2.0, 2.0)

        min_idx = min(range(n), key=lambda i: zs[i])
        min_x = xs[min_idx]
        min_z = zs[min_idx]
        min_px = map_x(min_x)
        min_py = map_z(min_z)

        p.setPen(QPen(min_outer_color, 2))
        p.setBrush(_no_brush())
        p.drawEllipse(QPointF(min_px, min_py), 6.0, 6.0)

        p.setPen(QPen(min_inner_color, 1))
        p.setBrush(min_inner_color)
        p.drawEllipse(QPointF(min_px, min_py), 2.8, 2.8)

        precision_text = "N/A" if self.precision is None else str(self.precision)
        valid_text = "True" if self.valid else "False"

        if self.intensities:
            ints = self.intensities[:min(len(self.intensities), n)]
            if ints:
                i_avg = sum(ints) / len(ints)
                i_min = min(ints)
                i_max = max(ints)
                intensity_text = f"Intensity avg/min/max: {i_avg:.1f} / {i_min:.0f} / {i_max:.0f}"
            else:
                intensity_text = "Intensity avg/min/max: N/A"
        else:
            intensity_text = "Intensity avg/min/max: N/A"

        info_lines = [
            f"Min Z: x={min_x:.2f} {self.unit}, z={min_z:.2f} {self.unit}",
            f"Z range: {z_min:.2f} ~ {z_max:.2f} {self.unit}",
            f"Points: {n} | Precision: {precision_text} | Valid: {valid_text}",
            intensity_text,
        ]

        p.setPen(text_color)
        font = QFont("Sans", 10)
        p.setFont(font)

        info_y = 10
        for line in info_lines:
            p.drawText(12, info_y + 18, line)
            info_y += 24

        p.drawText(12, h - 10, f"X range: {x_min:.2f} ~ {x_max:.2f} {self.unit}")


class DiffDriveTeleopGUI(Node, QWidget):
    def __init__(self):
        Node.__init__(self, "fastech_teleop_gui")
        QWidget.__init__(self)

        self.declare_parameter("cmd_topic", "/diff_drive_controller/cmd_vel")
        self.declare_parameter("max_speed", 0.20)
        self.declare_parameter("max_yaw_rate", 1.0)
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("speed_step", 0.0001)
        self.declare_parameter("yaw_step", 0.05)

        self.cmd_topic = str(self.get_parameter("cmd_topic").value)
        self.max_speed = float(self.get_parameter("max_speed").value)
        self.max_yaw = float(self.get_parameter("max_yaw_rate").value)
        self.rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.speed_step = float(self.get_parameter("speed_step").value)
        self.yaw_step = float(self.get_parameter("yaw_step").value)

        self.pub = self.create_publisher(TwistStamped, self.cmd_topic, 10)

        self.enabled = True
        self.estop = False
        self.speed_mag = 0.0
        self.dir = 0
        self.yaw = 0.0

        self.lvs_topic = "/lvs/profile"
        self.lvs_unit = "mm"

        self.setWindowTitle("FASTECH Teleop (Diff Drive)")
        self.setMinimumWidth(860)
        self.setMinimumHeight(900)
        self.setFocusPolicy(_qt_strong_focus())

        root = QVBoxLayout()
        main = QHBoxLayout()

        gb_drive = QGroupBox("DRIVE")
        gd = QGridLayout()

        self.btn_forward = QPushButton("FORWARD")
        self.btn_reverse = QPushButton("REVERSE")
        self.btn_stop = QPushButton("STOP")
        for b in (self.btn_forward, self.btn_reverse, self.btn_stop):
            b.setCheckable(True)
            b.setMinimumHeight(70)

        self.dir_group = QButtonGroup(self)
        self.dir_group.setExclusive(True)
        self.dir_group.addButton(self.btn_forward, 1)
        self.dir_group.addButton(self.btn_stop, 0)
        self.dir_group.addButton(self.btn_reverse, -1)

        self.btn_stop.setChecked(True)
        self.btn_forward.clicked.connect(lambda: self._set_dir(+1))
        self.btn_reverse.clicked.connect(lambda: self._set_dir(-1))
        self.btn_stop.clicked.connect(lambda: self._set_dir(0))

        gd.addWidget(self.btn_forward, 0, 0)
        gd.addWidget(self.btn_stop, 0, 1)
        gd.addWidget(self.btn_reverse, 0, 2)

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

        self.lbl_speed = QLabel("Speed magnitude: 0.0000 m/s")
        self.lbl_v = QLabel("Command v: +0.0000 m/s")

        gd.addWidget(QLabel("Speed (m/s)"), 1, 0)
        gd.addWidget(self.s_speed, 1, 1)
        gd.addWidget(self.sp_speed, 1, 2)
        gd.addWidget(self.lbl_speed, 2, 0, 1, 3)
        gd.addWidget(self.lbl_v, 3, 0, 1, 3)

        gb_drive.setLayout(gd)
        main.addWidget(gb_drive, 1)

        gb_turn = QGroupBox("TURN")
        gs = QGridLayout()

        self.watch = SteeringWatch()
        gs.addWidget(self.watch, 0, 0, 4, 2)

        self.btn_left = QPushButton("LEFT")
        self.btn_right = QPushButton("RIGHT")
        self.btn_center = QPushButton("CENTER")
        for b in (self.btn_left, self.btn_right, self.btn_center):
            b.setMinimumHeight(55)
            b.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        self.btn_left.clicked.connect(lambda: self._set_yaw(self.yaw + self.yaw_step))
        self.btn_right.clicked.connect(lambda: self._set_yaw(self.yaw - self.yaw_step))
        self.btn_center.clicked.connect(lambda: self._set_yaw(0.0))

        gs.addWidget(self.btn_left, 4, 0)
        gs.addWidget(self.btn_right, 4, 1)
        gs.addWidget(self.btn_center, 5, 0, 1, 2)

        self.s_yaw = QSlider(_qt_horizontal())
        self.s_yaw.setMinimum(int(-self.max_yaw * 1000))
        self.s_yaw.setMaximum(int(self.max_yaw * 1000))
        self.s_yaw.setValue(0)
        self.s_yaw.valueChanged.connect(lambda v: self._set_yaw(float(v) / 1000.0))

        self.sp_yaw = QDoubleSpinBox()
        self.sp_yaw.setDecimals(3)
        self.sp_yaw.setSingleStep(self.yaw_step)
        self.sp_yaw.setRange(-self.max_yaw, self.max_yaw)
        self.sp_yaw.setValue(0.0)
        self.sp_yaw.valueChanged.connect(lambda v: self._set_yaw(float(v)))

        self.lbl_yaw = QLabel("Yaw rate: +0.000 rad/s")

        gs.addWidget(QLabel("Yaw (rad/s)"), 6, 0)
        gs.addWidget(self.s_yaw, 6, 1)
        gs.addWidget(self.sp_yaw, 7, 1)
        gs.addWidget(self.lbl_yaw, 7, 0)

        gb_turn.setLayout(gs)
        main.addWidget(gb_turn, 1)

        root.addLayout(main)

        bottom = QHBoxLayout()
        self.btn_enable = QPushButton("Disable")
        self.btn_enable.clicked.connect(self._toggle_enable)
        bottom.addWidget(self.btn_enable)

        self.btn_estop = QPushButton("E-STOP")
        self.btn_estop.setCheckable(True)
        self.btn_estop.clicked.connect(self._toggle_estop)
        bottom.addWidget(self.btn_estop)

        self.btn_hard_stop = QPushButton("STOP NOW")
        self.btn_hard_stop.clicked.connect(self._hard_stop)
        bottom.addWidget(self.btn_hard_stop)

        self.cb_publish = QCheckBox("Publish enabled")
        self.cb_publish.setChecked(True)
        bottom.addWidget(self.cb_publish)

        root.addLayout(bottom)

        self.lvs_viewer = LVSProfileViewer()
        root.addWidget(self.lvs_viewer, 1)

        self.setLayout(root)

        self._apply_button_colors()

        self.lvs_sub = self.create_subscription(
            LvsProfile,
            self.lvs_topic,
            self._on_lvs_profile,
            10
        )

        self.timer = QTimer(self)
        self.timer.timeout.connect(self._tick)
        self.timer.start(int(1000.0 / max(1.0, self.rate_hz)))

        self._refresh()

    def _apply_button_colors(self):
        self.btn_forward.setStyleSheet("QPushButton:checked { background-color: #22c55e; color: #052e16; font-weight: 800; }")
        self.btn_reverse.setStyleSheet("QPushButton:checked { background-color: #fb923c; color: #3b1d0a; font-weight: 800; }")
        self.btn_stop.setStyleSheet("QPushButton:checked { background-color: #ef4444; color: #2b0b0b; font-weight: 900; }")
        self.btn_left.setStyleSheet("QPushButton { background-color: #60a5fa; color: #0b1220; font-weight: 800; } QPushButton:hover { background-color: #93c5fd; }")
        self.btn_right.setStyleSheet("QPushButton { background-color: #60a5fa; color: #0b1220; font-weight: 800; } QPushButton:hover { background-color: #93c5fd; }")
        self.btn_center.setStyleSheet("QPushButton { background-color: #cbd5e1; color: #0b1220; font-weight: 800; } QPushButton:hover { background-color: #e2e8f0; }")
        self.btn_hard_stop.setStyleSheet("QPushButton { background-color: #ef4444; color: #1f0a0a; font-weight: 900; } QPushButton:hover { background-color: #f87171; }")
        self.btn_estop.setStyleSheet("QPushButton:checked { background-color: #dc2626; color: #1f0a0a; font-weight: 900; }")

    def _set_dir(self, d: int):
        self.dir = int(d)
        if d == +1:
            self.btn_forward.setChecked(True)
        elif d == -1:
            self.btn_reverse.setChecked(True)
        else:
            self.btn_stop.setChecked(True)
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

    def _set_yaw(self, yaw: float):
        self.yaw = clamp(float(yaw), -self.max_yaw, self.max_yaw)
        sval = int(round(self.yaw * 1000))
        if self.s_yaw.value() != sval:
            self.s_yaw.blockSignals(True)
            self.s_yaw.setValue(sval)
            self.s_yaw.blockSignals(False)
        if abs(self.sp_yaw.value() - self.yaw) > 1e-6:
            self.sp_yaw.blockSignals(True)
            self.sp_yaw.setValue(self.yaw)
            self.sp_yaw.blockSignals(False)
        self._refresh()

    def _toggle_enable(self):
        self.enabled = not self.enabled
        self.btn_enable.setText("Disable" if self.enabled else "Enable")
        if not self.enabled:
            self._set_dir(0)

    def _toggle_estop(self):
        self.estop = self.btn_estop.isChecked()
        if self.estop:
            self._set_dir(0)

    def _hard_stop(self):
        self._set_dir(0)
        self._set_speed_mag(0.0)
        self._set_yaw(0.0)
        self._publish(force_stop=True)

    def _current_v(self):
        if not self.enabled or self.estop or self.dir == 0:
            return 0.0
        return float(self.dir) * float(self.speed_mag)

    def _refresh(self):
        v = self._current_v()
        self.lbl_speed.setText(f"Speed magnitude: {self.speed_mag:.4f} m/s")
        self.lbl_v.setText(f"Command v: {v:+.4f} m/s")
        self.lbl_yaw.setText(f"Yaw rate: {self.yaw:+.3f} rad/s")
        self.watch.set_value(self.yaw * 180.0 / math.pi, self.max_yaw * 180.0 / math.pi)

    def _publish(self, force_stop=False):
        if not self.cb_publish.isChecked():
            return

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()

        if force_stop or (not self.enabled) or self.estop:
            msg.twist.linear.x = 0.0
            msg.twist.angular.z = 0.0
        else:
            msg.twist.linear.x = float(self._current_v())
            msg.twist.angular.z = float(self.yaw)

        self.pub.publish(msg)

    def _on_lvs_profile(self, msg: LvsProfile):
        # exact fields from your standalone LVS viewer node
        xs = list(msg.x_mm)
        zs = list(msg.z_mm)
        intensities = list(msg.intensity)

        if not xs or not zs:
            self.lvs_viewer.clear("No LVS profile data")
            return

        n = min(len(xs), len(zs))
        xs = xs[:n]
        zs = zs[:n]
        intensities = intensities[:n] if len(intensities) >= n else []

        indexed = list(range(n))
        indexed.sort(key=lambda i: xs[i])

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
            status_text=f"Receiving {len(xs_sorted)} points from {self.lvs_topic}"
        )

    def _tick(self):
    # Process multiple ROS callbacks each GUI cycle
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
    w = DiffDriveTeleopGUI()
    w.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()