#!/usr/bin/env python3

import math
import statistics

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped


class SimpleDriver(Node):
    """
    Simple LiDAR-based hallway centering driver.

    Behavior:
    - Uses left and right LiDAR distance sectors.
    - Tries to keep the car centered between the walls.
    - Drives forward slowly.
    - If an obstacle/wall is close in front, slows down and turns toward the more open side.
    """

    def __init__(self):
        super().__init__("simple_driver")

        # Topics
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("drive_topic", "/drive")

        # Driving behavior
        self.declare_parameter("base_speed", 1.0)
        self.declare_parameter("turn_speed", 0.6)
        self.declare_parameter("stop_speed", 0.0)

        # Steering behavior
        self.declare_parameter("kp_centering", 0.8)
        self.declare_parameter("kp_front_avoid", 0.6)
        self.declare_parameter("max_steering_angle", 0.34)

        # LiDAR sectors in degrees
        self.declare_parameter("left_min_angle_deg", 60.0)
        self.declare_parameter("left_max_angle_deg", 100.0)

        self.declare_parameter("right_min_angle_deg", -100.0)
        self.declare_parameter("right_max_angle_deg", -60.0)

        self.declare_parameter("front_min_angle_deg", -15.0)
        self.declare_parameter("front_max_angle_deg", 15.0)

        # Safety distances
        self.declare_parameter("front_slow_distance", 1.2)
        self.declare_parameter("front_stop_distance", 0.45)

        # Filtering
        self.declare_parameter("min_valid_range", 0.05)
        self.declare_parameter("max_valid_range", 10.0)

        scan_topic = self.get_parameter("scan_topic").value
        drive_topic = self.get_parameter("drive_topic").value

        self.scan_sub = self.create_subscription(
            LaserScan,
            scan_topic,
            self.scan_callback,
            10
        )

        self.drive_pub = self.create_publisher(
            AckermannDriveStamped,
            drive_topic,
            10
        )

        self.get_logger().info("simple_driver node started")
        self.get_logger().info(f"Subscribing to: {scan_topic}")
        self.get_logger().info(f"Publishing to: {drive_topic}")

    def scan_callback(self, scan_msg: LaserScan):
        left_dist = self.get_sector_median(
            scan_msg,
            self.get_parameter("left_min_angle_deg").value,
            self.get_parameter("left_max_angle_deg").value
        )

        right_dist = self.get_sector_median(
            scan_msg,
            self.get_parameter("right_min_angle_deg").value,
            self.get_parameter("right_max_angle_deg").value
        )

        front_dist = self.get_sector_median(
            scan_msg,
            self.get_parameter("front_min_angle_deg").value,
            self.get_parameter("front_max_angle_deg").value
        )

        if left_dist is None or right_dist is None or front_dist is None:
            self.publish_drive(0.0, 0.0)
            self.get_logger().warn("Invalid LiDAR sector data. Stopping.")
            return

        base_speed = self.get_parameter("base_speed").value
        turn_speed = self.get_parameter("turn_speed").value
        stop_speed = self.get_parameter("stop_speed").value

        kp_centering = self.get_parameter("kp_centering").value
        kp_front_avoid = self.get_parameter("kp_front_avoid").value
        max_steering_angle = self.get_parameter("max_steering_angle").value

        front_slow_distance = self.get_parameter("front_slow_distance").value
        front_stop_distance = self.get_parameter("front_stop_distance").value

        # ------------------------------------------------------------
        # Centering logic
        #
        # If left wall is closer than right wall:
        #   left_dist < right_dist
        #   error = right_dist - left_dist is positive
        #   steering should go right, so steering is negative.
        #
        # ROS Ackermann convention:
        #   positive steering = left
        #   negative steering = right
        # ------------------------------------------------------------
        center_error = right_dist - left_dist
        steering = -kp_centering * center_error

        speed = base_speed

        # ------------------------------------------------------------
        # Front obstacle logic
        #
        # If front is close, turn toward the more open side.
        # If left side has more room, turn left.
        # If right side has more room, turn right.
        # ------------------------------------------------------------
        if front_dist < front_stop_distance:
            speed = stop_speed

            if left_dist > right_dist:
                steering = max_steering_angle
            else:
                steering = -max_steering_angle

        elif front_dist < front_slow_distance:
            speed = turn_speed

            open_side_error = left_dist - right_dist
            steering += kp_front_avoid * open_side_error

        steering = self.clamp(
            steering,
            -max_steering_angle,
            max_steering_angle
        )

        self.publish_drive(speed, steering)

    def get_sector_median(self, scan_msg: LaserScan, min_angle_deg: float, max_angle_deg: float):
        """
        Returns the median valid LiDAR range in a given angular sector.
        Angles are in degrees, where:
        - 0 degrees is straight ahead
        - positive is left
        - negative is right
        """

        min_angle_rad = math.radians(min_angle_deg)
        max_angle_rad = math.radians(max_angle_deg)

        min_valid_range = self.get_parameter("min_valid_range").value
        max_valid_range = self.get_parameter("max_valid_range").value

        ranges = []

        for i, distance in enumerate(scan_msg.ranges):
            angle = scan_msg.angle_min + i * scan_msg.angle_increment

            if angle < min_angle_rad or angle > max_angle_rad:
                continue

            if math.isnan(distance) or math.isinf(distance):
                continue

            if distance < min_valid_range or distance > max_valid_range:
                continue

            ranges.append(distance)

        if len(ranges) == 0:
            return None

        return statistics.median(ranges)

    def publish_drive(self, speed: float, steering_angle: float):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"

        msg.drive.speed = float(speed)
        msg.drive.steering_angle = float(steering_angle)

        self.drive_pub.publish(msg)

    @staticmethod
    def clamp(value: float, min_value: float, max_value: float):
        return max(min_value, min(value, max_value))


def main(args=None):
    rclpy.init(args=args)

    node = SimpleDriver()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.publish_drive(0.0, 0.0)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()