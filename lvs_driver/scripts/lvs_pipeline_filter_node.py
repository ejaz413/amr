#!/usr/bin/env python3
import math
import random
import bisect
from collections import deque

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from lvs_driver.msg import LvsProfile


def is_finite(v: float) -> bool:
    return not math.isnan(v) and not math.isinf(v)


def sanitize_uint32_list(values):
    out = []
    for v in values:
        try:
            if isinstance(v, float):
                if math.isnan(v) or math.isinf(v):
                    iv = 0
                else:
                    iv = int(round(v))
            else:
                iv = int(v)
        except Exception:
            iv = 0

        if iv < 0:
            iv = 0
        elif iv > 4294967295:
            iv = 4294967295

        out.append(iv)
    return out


class MedianWindowFilter:
    def __init__(self):
        self.window = deque()
        self.sorted_vals = []
        self.invalid_count = 0

    def clear(self):
        self.window.clear()
        self.sorted_vals.clear()
        self.invalid_count = 0

    def _insert_sorted(self, v: float):
        idx = bisect.bisect_left(self.sorted_vals, v)
        self.sorted_vals.insert(idx, v)

    def _remove_sorted(self, v: float):
        idx = bisect.bisect_left(self.sorted_vals, v)
        if idx < len(self.sorted_vals) and self.sorted_vals[idx] == v:
            self.sorted_vals.pop(idx)

    def filter(self, value: float, window_size: int) -> float:
        if window_size <= 0:
            raise ValueError("window_size must be > 0")

        if is_finite(value):
            self.invalid_count = 0
            self.window.append(value)
            self._insert_sorted(value)

            while len(self.window) > window_size:
                old = self.window.popleft()
                self._remove_sorted(old)
        else:
            self.invalid_count += 1

        if self.invalid_count >= window_size / 2:
            self.clear()
            return float("nan")

        if len(self.window) > window_size / 2:
            n = len(self.sorted_vals)
            mid = n // 2
            if n % 2 == 1:
                return self.sorted_vals[mid]
            return 0.5 * (self.sorted_vals[mid - 1] + self.sorted_vals[mid])

        return float("nan")


class LvsPipelineFilterNode(Node):
    def __init__(self):
        super().__init__("lvs_pipeline_filter_node")

        self.declare_parameter("input_topic", "/lvs/profile")
        self.declare_parameter("filtered_topic", "/lvs/profile_filtered")
        self.declare_parameter("corners_topic", "/lvs/corners")

        self.declare_parameter("line_thickness_mm", 1.0)
        self.declare_parameter("ransac_iterations", 70)
        self.declare_parameter("segment_gap_mm", 5.0)
        self.declare_parameter("corner_filter_window_size", 60)
        self.declare_parameter("random_seed", 1234)

        self.input_topic = str(self.get_parameter("input_topic").value)
        self.filtered_topic = str(self.get_parameter("filtered_topic").value)
        self.corners_topic = str(self.get_parameter("corners_topic").value)

        self.line_thickness_mm = float(self.get_parameter("line_thickness_mm").value)
        self.ransac_iterations = int(self.get_parameter("ransac_iterations").value)
        self.segment_gap_mm = float(self.get_parameter("segment_gap_mm").value)
        self.corner_filter_window_size = int(self.get_parameter("corner_filter_window_size").value)
        self.random_seed = int(self.get_parameter("random_seed").value)

        random.seed(self.random_seed)

        self.filtered_pub = self.create_publisher(LvsProfile, self.filtered_topic, 10)
        self.corners_pub = self.create_publisher(Float64MultiArray, self.corners_topic, 10)

        self.sub = self.create_subscription(
            LvsProfile,
            self.input_topic,
            self.profile_callback,
            10
        )

        self.corner_filter_x = [MedianWindowFilter(), MedianWindowFilter()]
        self.corner_filter_z = [MedianWindowFilter(), MedianWindowFilter()]
        self.continuous_empty_corner_count = 0

        self.get_logger().info(
            f"Started LVS pipeline filter node: input={self.input_topic}, "
            f"filtered={self.filtered_topic}, corners={self.corners_topic}"
        )

    @staticmethod
    def point_line_distance(x, z, a, b, c):
        denom = math.sqrt(a * a + b * b)
        if denom < 1e-12:
            return float("inf")
        return abs(a * x + b * z + c) / denom

    @staticmethod
    def line_from_points(x1, z1, x2, z2):
        # ax + bz + c = 0
        a = z1 - z2
        b = x2 - x1
        c = x1 * z2 - x2 * z1
        return a, b, c

    def execute_ransac(self, xs, zs, threshold, iterations):
        count = len(xs)
        if count < 2:
            return []

        best_inliers = []

        finite_indices = [i for i in range(count) if is_finite(xs[i]) and is_finite(zs[i])]
        if len(finite_indices) < 2:
            return []

        for _ in range(iterations):
            i1, i2 = random.sample(finite_indices, 2)

            x1, z1 = xs[i1], zs[i1]
            x2, z2 = xs[i2], zs[i2]

            if abs(x1 - x2) < 1e-12 and abs(z1 - z2) < 1e-12:
                continue

            a, b, c = self.line_from_points(x1, z1, x2, z2)

            inliers = []
            for i in finite_indices:
                d = self.point_line_distance(xs[i], zs[i], a, b, c)
                if d <= threshold:
                    inliers.append(i)

            if len(inliers) > len(best_inliers):
                best_inliers = inliers

        best_inliers.sort()
        return best_inliers

    def groove_filter(self, xs, zs, line_thickness_mm, filter_z_to_nan=True):
        count = len(xs)
        if count < 2:
            if filter_z_to_nan:
                return [float("nan")] * count, (-1, -1)
            return list(zs), (-1, -1)

        z_out = list(zs)

        inlier_indices = self.execute_ransac(
            xs,
            z_out,
            threshold=line_thickness_mm / 2.0,
            iterations=self.ransac_iterations
        )

        if len(inlier_indices) < 2:
            if filter_z_to_nan:
                return [float("nan")] * count, (-1, -1)
            return z_out, (-1, -1)

        inlier_longest_interval = 0.0
        inlier_left_end = -1
        for i in range(len(inlier_indices) - 1):
            diff = xs[inlier_indices[i]] - xs[inlier_indices[i + 1]]
            if diff > inlier_longest_interval:
                inlier_longest_interval = diff
                inlier_left_end = i + 1

        if inlier_left_end <= 0:
            if filter_z_to_nan:
                return [float("nan")] * count, (-1, -1)
            return z_out, (-1, -1)

        max_dist_sq = self.segment_gap_mm * self.segment_gap_mm
        segments = []
        segments.append([0, 0])

        for i in range(1, len(inlier_indices)):
            dx = xs[inlier_indices[i - 1]] - xs[inlier_indices[i]]
            dz = z_out[inlier_indices[i - 1]] - z_out[inlier_indices[i]]
            dist_sq = dx * dx + dz * dz

            if dist_sq > max_dist_sq:
                segments[-1][1] = i - segments[-1][0]
                segments.append([i, 0])

        segments[-1][1] = len(inlier_indices) - segments[-1][0]

        if len(segments) < 2:
            if filter_z_to_nan:
                return [float("nan")] * count, (-1, -1)
            return z_out, (-1, -1)

        longest = (-1, -1)
        second_longest = (-1, -1)

        for seg_start, seg_size in segments:
            if seg_size > longest[1]:
                second_longest = longest
                longest = (seg_start, seg_size)
            elif seg_size > second_longest[1]:
                second_longest = (seg_start, seg_size)

        if longest[0] < 0 or second_longest[0] < 0:
            if filter_z_to_nan:
                return [float("nan")] * count, (-1, -1)
            return z_out, (-1, -1)

        best_model_indices = set()

        for i in range(longest[0], longest[0] + longest[1]):
            best_model_indices.add(inlier_indices[i])

        for i in range(second_longest[0], second_longest[0] + second_longest[1]):
            best_model_indices.add(inlier_indices[i])

        if filter_z_to_nan:
            for i in range(count):
                if i not in best_model_indices:
                    z_out[i] = float("nan")

        if longest[0] < second_longest[0]:
            left_corner = inlier_indices[longest[0] + longest[1] - 1]
            right_corner = inlier_indices[second_longest[0]]
            return z_out, (left_corner, right_corner)
        else:
            left_corner = inlier_indices[second_longest[0] + second_longest[1] - 1]
            right_corner = inlier_indices[longest[0]]
            return z_out, (left_corner, right_corner)

    def publish_filtered_profile(self, src_msg, xs, zs, intensities):
        out = LvsProfile()

        if hasattr(out, "x_mm"):
            out.x_mm = [float(v) for v in xs]
        if hasattr(out, "z_mm"):
            out.z_mm = [float(v) for v in zs]
        if hasattr(out, "intensity"):
            out.intensity = sanitize_uint32_list(intensities)

        if hasattr(out, "precision") and hasattr(src_msg, "precision"):
            out.precision = src_msg.precision
        if hasattr(out, "valid") and hasattr(src_msg, "valid"):
            out.valid = src_msg.valid

        self.filtered_pub.publish(out)

    def publish_corners(self, corners):
        msg = Float64MultiArray()
        msg.data = corners
        self.corners_pub.publish(msg)

    def profile_callback(self, msg: LvsProfile):
        xs = list(msg.x_mm)
        zs = list(msg.z_mm)
        intensities = list(msg.intensity)

        if not xs or not zs:
            return

        n = min(len(xs), len(zs))
        xs = xs[:n]
        zs = zs[:n]

        if len(intensities) >= n:
            intensities = intensities[:n]
        else:
            intensities = [0] * n

        indexed = list(range(n))
        indexed.sort(key=lambda i: xs[i], reverse=True)

        xs_sorted = [xs[i] for i in indexed]
        zs_sorted = [zs[i] for i in indexed]
        intensities_sorted = [intensities[i] for i in indexed]

        z_filtered, corner_idx = self.groove_filter(
            xs_sorted,
            zs_sorted,
            self.line_thickness_mm,
            filter_z_to_nan=True
        )

        if corner_idx[0] > -1 and corner_idx[1] > -1:
            x1 = self.corner_filter_x[0].filter(xs_sorted[corner_idx[0]], self.corner_filter_window_size)
            z1 = self.corner_filter_z[0].filter(z_filtered[corner_idx[0]], self.corner_filter_window_size)
            x2 = self.corner_filter_x[1].filter(xs_sorted[corner_idx[1]], self.corner_filter_window_size)
            z2 = self.corner_filter_z[1].filter(z_filtered[corner_idx[1]], self.corner_filter_window_size)

            self.continuous_empty_corner_count = 0
            self.publish_corners([x1, z1, x2, z2])
        else:
            self.continuous_empty_corner_count += 1

            if self.continuous_empty_corner_count > self.corner_filter_window_size / 2:
                self.corner_filter_x[0].clear()
                self.corner_filter_x[1].clear()
                self.corner_filter_z[0].clear()
                self.corner_filter_z[1].clear()
                self.continuous_empty_corner_count = 0

            self.publish_corners([])

        self.publish_filtered_profile(msg, xs_sorted, z_filtered, intensities_sorted)


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = LvsPipelineFilterNode()
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    except Exception as e:
        if node is not None:
            node.get_logger().error(f"Unhandled exception: {e}")
        else:
            print(f"Unhandled exception before node creation: {e}")
        raise

    finally:
        try:
            if node is not None:
                node.destroy_node()
        except Exception:
            pass

        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()