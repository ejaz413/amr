#!/usr/bin/env python3

import math
import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from lvs_driver.msg import LvsProfile


class LvsProfileViewer(Node):
    def __init__(self):
        super().__init__("lvs_profile_viewer")

        self.declare_parameter("topic_name", "/lvs/profile")
        self.declare_parameter("window_name", "LVS Profile Viewer")
        self.declare_parameter("width", 1000)
        self.declare_parameter("height", 600)
        self.declare_parameter("margin", 50)
        self.declare_parameter("show_intensity", True)
        self.declare_parameter("flip_vertical", True)

        self.topic_name = self.get_parameter("topic_name").value
        self.window_name = self.get_parameter("window_name").value
        self.width = int(self.get_parameter("width").value)
        self.height = int(self.get_parameter("height").value)
        self.margin = int(self.get_parameter("margin").value)
        self.show_intensity = bool(self.get_parameter("show_intensity").value)
        self.flip_vertical = bool(self.get_parameter("flip_vertical").value)

        self.subscription = self.create_subscription(
            LvsProfile,
            self.topic_name,
            self.profile_callback,
            10,
        )

        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, self.width, self.height)

        self.get_logger().info(f"Subscribed to {self.topic_name}")

    def profile_callback(self, msg: LvsProfile):
        x_vals = list(msg.x_mm)
        z_vals = list(msg.z_mm)
        i_vals = list(msg.intensity)

        if len(x_vals) == 0 or len(z_vals) == 0:
            return

        n = min(len(x_vals), len(z_vals))
        x_vals = x_vals[:n]
        z_vals = z_vals[:n]
        i_vals = i_vals[:n] if len(i_vals) >= n else [0] * n

        x_min = min(x_vals)
        x_max = max(x_vals)
        z_min = min(z_vals)
        z_max = max(z_vals)

        if abs(x_max - x_min) < 1e-6:
            x_max = x_min + 1.0
        if abs(z_max - z_min) < 1e-6:
            z_max = z_min + 1.0

        img = np.zeros((self.height, self.width, 3), dtype=np.uint8)

        plot_x0 = self.margin
        plot_y0 = self.margin
        plot_x1 = self.width - self.margin
        plot_y1 = self.height - self.margin

        cv2.rectangle(img, (plot_x0, plot_y0), (plot_x1, plot_y1), (80, 80, 80), 1)

        def map_x(x):
            return int(plot_x0 + (x - x_min) / (x_max - x_min) * (plot_x1 - plot_x0))

        def map_z(z):
            norm = (z - z_min) / (z_max - z_min)
            if self.flip_vertical:
                norm = 1.0 - norm
            return int(plot_y0 + norm * (plot_y1 - plot_y0))

        # grid lines
        for k in range(6):
            gx = int(plot_x0 + k * (plot_x1 - plot_x0) / 5)
            gy = int(plot_y0 + k * (plot_y1 - plot_y0) / 5)
            cv2.line(img, (gx, plot_y0), (gx, plot_y1), (40, 40, 40), 1)
            cv2.line(img, (plot_x0, gy), (plot_x1, gy), (40, 40, 40), 1)

        # polyline points
        pts = []
        for x, z in zip(x_vals, z_vals):
            px = map_x(x)
            py = map_z(z)
            pts.append((px, py))

        if len(pts) >= 2:
            for i in range(len(pts) - 1):
                cv2.line(img, pts[i], pts[i + 1], (0, 255, 0), 2)

        # draw points
        for idx, (px, py) in enumerate(pts):
            cv2.circle(img, (px, py), 2, (0, 200, 255), -1)

        # highlight min z point
        min_idx = int(np.argmin(z_vals))
        min_pt = pts[min_idx]
        cv2.circle(img, min_pt, 6, (0, 0, 255), 2)
        cv2.putText(
            img,
            f"Min Z: x={x_vals[min_idx]:.2f} mm, z={z_vals[min_idx]:.2f} mm",
            (plot_x0, 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )

        # labels
        cv2.putText(
            img,
            f"X range: {x_min:.2f} ~ {x_max:.2f} mm",
            (plot_x0, self.height - 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (200, 200, 200),
            1,
            cv2.LINE_AA,
        )

        cv2.putText(
            img,
            f"Z range: {z_min:.2f} ~ {z_max:.2f} mm",
            (plot_x0, 45),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (200, 200, 200),
            1,
            cv2.LINE_AA,
        )

        cv2.putText(
            img,
            f"Points: {n} | Precision: {msg.precision} | Valid: {msg.valid}",
            (plot_x0, 70),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (200, 200, 200),
            1,
            cv2.LINE_AA,
        )

        if self.show_intensity and len(i_vals) > 0:
            avg_i = float(np.mean(i_vals))
            min_i = int(np.min(i_vals))
            max_i = int(np.max(i_vals))
            cv2.putText(
                img,
                f"Intensity avg/min/max: {avg_i:.1f} / {min_i} / {max_i}",
                (plot_x0, 95),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (200, 200, 200),
                1,
                cv2.LINE_AA,
            )

        cv2.imshow(self.window_name, img)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = LvsProfileViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()