#!/usr/bin/env python3

import math
from typing import List, Tuple

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped


class FollowTheGap(Node):
    """
    Reactive Follow-the-Gap controller for F1TENTH.

    Pipeline:
    1. Read LiDAR scan
    2. Smooth ranges
    3. Find closest obstacle
    4. Create bubble around that obstacle
    5. Find largest free gap
    6. Pick best point inside gap
    7. Convert to steering command
    8. Choose speed based on steering demand
    """

    def __init__(self) -> None:
        super().__init__('follow_the_gap')

        # Topics
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('drive_topic', '/drive')

        # Preprocessing
        self.declare_parameter('max_lidar_range', 10.0)
        self.declare_parameter('smoothing_window_size', 3)

        # Field of view cropping (degrees)
        self.declare_parameter('min_angle_deg', -90.0)
        self.declare_parameter('max_angle_deg', 90.0)

        # Bubble + gap
        self.declare_parameter('bubble_radius_indices', 10)
        self.declare_parameter('min_gap_size', 5)

        # Best point selection
        self.declare_parameter('best_point_method', 'furthest')  # furthest or center

        # Steering + speed
        self.declare_parameter('max_steering_angle', 0.4189)
        self.declare_parameter('straight_speed', 1.5)
        self.declare_parameter('medium_speed', 1.0)
        self.declare_parameter('corner_speed', 0.6)

        # Steering thresholds (radians)
        self.declare_parameter('straight_steering_threshold', 0.10)
        self.declare_parameter('medium_steering_threshold', 0.25)

        # Debug
        self.declare_parameter('debug', True)
        self.declare_parameter('log_every_n', 10)

        self.scan_topic = self.get_parameter('scan_topic').value
        self.drive_topic = self.get_parameter('drive_topic').value

        self.max_lidar_range = float(self.get_parameter('max_lidar_range').value)
        self.smoothing_window_size = int(self.get_parameter('smoothing_window_size').value)

        self.min_angle_deg = float(self.get_parameter('min_angle_deg').value)
        self.max_angle_deg = float(self.get_parameter('max_angle_deg').value)

        self.bubble_radius_indices = int(self.get_parameter('bubble_radius_indices').value)
        self.min_gap_size = int(self.get_parameter('min_gap_size').value)

        self.best_point_method = str(self.get_parameter('best_point_method').value).lower()

        self.max_steering_angle = float(self.get_parameter('max_steering_angle').value)
        self.straight_speed = float(self.get_parameter('straight_speed').value)
        self.medium_speed = float(self.get_parameter('medium_speed').value)
        self.corner_speed = float(self.get_parameter('corner_speed').value)

        self.straight_steering_threshold = float(
            self.get_parameter('straight_steering_threshold').value
        )
        self.medium_steering_threshold = float(
            self.get_parameter('medium_steering_threshold').value
        )

        self.debug = bool(self.get_parameter('debug').value)
        self.log_every_n = int(self.get_parameter('log_every_n').value)
        self.log_counter = 0

        self.scan_sub = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            10
        )

        self.drive_pub = self.create_publisher(
            AckermannDriveStamped,
            self.drive_topic,
            10
        )

        self.get_logger().info('Follow-the-Gap node started.')
        self.get_logger().info(f'Subscribing to: {self.scan_topic}')
        self.get_logger().info(f'Publishing to: {self.drive_topic}')

    def scan_callback(self, msg: LaserScan) -> None:
        ranges = list(msg.ranges)

        if len(ranges) == 0:
            return

        cropped_ranges, cropped_angles = self.crop_scan(msg, ranges)
        if len(cropped_ranges) == 0:
            self.publish_drive(0.0, 0.0)
            return

        proc_ranges = self.preprocess_ranges(cropped_ranges)

        closest_idx = self.find_closest_obstacle(proc_ranges)
        bubble_ranges = self.apply_bubble(proc_ranges, closest_idx)

        gap_start, gap_end = self.find_largest_gap(bubble_ranges)

        if gap_start == -1 or gap_end == -1:
            self.publish_drive(self.corner_speed, 0.0)
            return

        best_idx = self.find_best_point(bubble_ranges, gap_start, gap_end)
        target_angle = cropped_angles[best_idx]

        steering_angle = max(
            -self.max_steering_angle,
            min(self.max_steering_angle, target_angle)
        )

        speed = self.select_speed(steering_angle)

        if self.debug:
            self.log_counter += 1
            if self.log_counter % self.log_every_n == 0:
                self.get_logger().info(
                    f'closest_idx={closest_idx}, gap=({gap_start},{gap_end}), '
                    f'best_idx={best_idx}, target_angle={target_angle:.3f}, '
                    f'steer={steering_angle:.3f}, speed={speed:.2f}'
                )

        self.publish_drive(speed, steering_angle)

    def crop_scan(self, scan: LaserScan, ranges: List[float]) -> Tuple[List[float], List[float]]:
        cropped_ranges = []
        cropped_angles = []

        min_angle_rad = math.radians(self.min_angle_deg)
        max_angle_rad = math.radians(self.max_angle_deg)

        angle = scan.angle_min
        for r in ranges:
            if min_angle_rad <= angle <= max_angle_rad:
                cropped_ranges.append(r)
                cropped_angles.append(angle)
            angle += scan.angle_increment

        return cropped_ranges, cropped_angles

    def preprocess_ranges(self, ranges: List[float]) -> List[float]:
        cleaned = []

        for r in ranges:
            if math.isinf(r) or math.isnan(r):
                cleaned.append(self.max_lidar_range)
            else:
                cleaned.append(min(max(r, 0.0), self.max_lidar_range))

        if self.smoothing_window_size <= 1:
            return cleaned

        smoothed = []
        half = self.smoothing_window_size // 2

        for i in range(len(cleaned)):
            start = max(0, i - half)
            end = min(len(cleaned), i + half + 1)
            window = cleaned[start:end]
            smoothed.append(sum(window) / len(window))

        return smoothed

    def find_closest_obstacle(self, ranges: List[float]) -> int:
        min_val = float('inf')
        min_idx = 0

        for i, r in enumerate(ranges):
            if r > 0.0 and r < min_val:
                min_val = r
                min_idx = i

        return min_idx

    def apply_bubble(self, ranges: List[float], center_idx: int) -> List[float]:
        bubbled = ranges[:]

        start = max(0, center_idx - self.bubble_radius_indices)
        end = min(len(bubbled), center_idx + self.bubble_radius_indices + 1)

        for i in range(start, end):
            bubbled[i] = 0.0

        return bubbled

    def find_largest_gap(self, ranges: List[float]) -> Tuple[int, int]:
        best_start = -1
        best_end = -1
        best_len = 0

        current_start = -1

        for i, r in enumerate(ranges):
            if r > 0.0:
                if current_start == -1:
                    current_start = i
            else:
                if current_start != -1:
                    current_len = i - current_start
                    if current_len > best_len and current_len >= self.min_gap_size:
                        best_len = current_len
                        best_start = current_start
                        best_end = i - 1
                    current_start = -1

        if current_start != -1:
            current_len = len(ranges) - current_start
            if current_len > best_len and current_len >= self.min_gap_size:
                best_start = current_start
                best_end = len(ranges) - 1

        return best_start, best_end

    def find_best_point(self, ranges: List[float], start_idx: int, end_idx: int) -> int:
        if self.best_point_method == 'center':
            return (start_idx + end_idx) // 2

        # Default: furthest point in gap
        best_idx = start_idx
        best_range = ranges[start_idx]

        for i in range(start_idx, end_idx + 1):
            if ranges[i] > best_range:
                best_range = ranges[i]
                best_idx = i

        return best_idx

    def select_speed(self, steering_angle: float) -> float:
        abs_steer = abs(steering_angle)

        if abs_steer < self.straight_steering_threshold:
            return self.straight_speed
        elif abs_steer < self.medium_steering_threshold:
            return self.medium_speed
        else:
            return self.corner_speed

    def publish_drive(self, speed: float, steering_angle: float) -> None:
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.drive.speed = float(speed)
        msg.drive.steering_angle = float(steering_angle)
        self.drive_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FollowTheGap()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.publish_drive(0.0, 0.0)
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()