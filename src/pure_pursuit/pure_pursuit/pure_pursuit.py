#!/usr/bin/env python3

import csv
import math
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped


class PurePursuit(Node):
    """
    Simple pure pursuit controller for F1TENTH.

    Purpose:
    - Read waypoints from CSV
    - Subscribe to odometry
    - Find a lookahead point on the path
    - Compute steering command
    - Publish AckermannDriveStamped to /drive

    CSV format expected:
    x,y
    1.0,2.0
    1.1,2.1
    ...

    Or just:
    1.0,2.0
    1.1,2.1
    ...
    """

    def __init__(self) -> None:
        super().__init__('pure_pursuit')

        # -----------------------------
        # Parameters
        # -----------------------------
        self.declare_parameter('waypoints_csv', '')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('drive_topic', '/drive')
        self.declare_parameter('lookahead_distance', 0.8)
        self.declare_parameter('speed', 0.8)
        self.declare_parameter('wheelbase', 0.33)
        self.declare_parameter('max_steering_angle', 0.4189)  # about 24 degrees
        self.declare_parameter('goal_tolerance', 0.5)
        self.declare_parameter('use_last_found_index', True)

        self.waypoints_csv: str = self.get_parameter('waypoints_csv').value
        self.odom_topic: str = self.get_parameter('odom_topic').value
        self.drive_topic: str = self.get_parameter('drive_topic').value
        self.lookahead_distance: float = float(self.get_parameter('lookahead_distance').value)
        self.speed: float = float(self.get_parameter('speed').value)
        self.wheelbase: float = float(self.get_parameter('wheelbase').value)
        self.max_steering_angle: float = float(self.get_parameter('max_steering_angle').value)
        self.goal_tolerance: float = float(self.get_parameter('goal_tolerance').value)
        self.use_last_found_index: bool = bool(self.get_parameter('use_last_found_index').value)

        # -----------------------------
        # Internal state
        # -----------------------------
        self.waypoints: List[Tuple[float, float]] = self.load_waypoints(self.waypoints_csv)
        self.current_x: float = 0.0
        self.current_y: float = 0.0
        self.current_yaw: float = 0.0
        self.pose_received: bool = False
        self.last_waypoint_index: int = 0
        self.finished: bool = False

        if not self.waypoints:
            self.get_logger().error('No waypoints loaded. Check your CSV file path and contents.')
            raise RuntimeError('No waypoints loaded.')

        self.get_logger().info(f'Loaded {len(self.waypoints)} waypoints from: {self.waypoints_csv}')

        # -----------------------------
        # ROS interfaces
        # -----------------------------
        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10
        )

        self.drive_pub = self.create_publisher(
            AckermannDriveStamped,
            self.drive_topic,
            10
        )

        # Main control loop
        self.timer = self.create_timer(0.02, self.control_loop)  # 50 Hz

    # ---------------------------------------------------------
    # CSV loading
    # ---------------------------------------------------------
    def load_waypoints(self, csv_path: str) -> List[Tuple[float, float]]:
        points: List[Tuple[float, float]] = []

        if not csv_path:
            self.get_logger().error('Parameter "waypoints_csv" is empty.')
            return points

        try:
            with open(csv_path, 'r') as f:
                reader = csv.reader(f)
                for row in reader:
                    if len(row) < 2:
                        continue

                    try:
                        x = float(row[0])
                        y = float(row[1])
                        points.append((x, y))
                    except ValueError:
                        # skip header or malformed rows
                        continue

        except FileNotFoundError:
            self.get_logger().error(f'Waypoint CSV not found: {csv_path}')
        except Exception as e:
            self.get_logger().error(f'Failed to load CSV: {e}')

        return points

    # ---------------------------------------------------------
    # Odometry callback
    # ---------------------------------------------------------
    def odom_callback(self, msg: Odometry) -> None:
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        self.current_yaw = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)

        self.pose_received = True

    # ---------------------------------------------------------
    # Main control loop
    # ---------------------------------------------------------
    def control_loop(self) -> None:
        if not self.pose_received:
            return

        if self.finished:
            self.publish_drive(0.0, 0.0)
            return

        # Stop if close to final waypoint
        goal_x, goal_y = self.waypoints[-1]
        dist_to_goal = self.distance(self.current_x, self.current_y, goal_x, goal_y)

        if dist_to_goal < self.goal_tolerance:
            self.get_logger().info('Reached final waypoint. Stopping.')
            self.publish_drive(0.0, 0.0)
            self.finished = True
            return

        target_index = self.find_lookahead_index()
        target_x, target_y = self.waypoints[target_index]

        # Transform target point into vehicle frame
        local_x, local_y = self.global_to_local(target_x, target_y)

        # If local_x is behind the car, that point is a bad target
        if local_x <= 0.0:
            self.get_logger().warn('Lookahead point is behind vehicle. Sending zero steering.')
            self.publish_drive(self.speed, 0.0)
            return

        # Pure pursuit curvature
        # curvature = 2 * y / L^2
        Ld = math.sqrt(local_x**2 + local_y**2)
        if Ld < 1e-6:
            self.publish_drive(0.0, 0.0)
            return

        curvature = 2.0 * local_y / (Ld ** 2)

        # Ackermann steering angle
        steering_angle = math.atan(self.wheelbase * curvature)

        # Clamp steering
        steering_angle = max(-self.max_steering_angle,
                             min(self.max_steering_angle, steering_angle))

        self.publish_drive(self.speed, steering_angle)

    # ---------------------------------------------------------
    # Find lookahead point
    # ---------------------------------------------------------
    def find_lookahead_index(self) -> int:
        """
        Finds the first waypoint at least lookahead_distance away.
        Optionally starts searching from the last found index to avoid jumping backward.
        """
        start_index = self.last_waypoint_index if self.use_last_found_index else 0

        best_index = start_index

        for i in range(start_index, len(self.waypoints)):
            wx, wy = self.waypoints[i]
            dist = self.distance(self.current_x, self.current_y, wx, wy)

            if dist >= self.lookahead_distance:
                best_index = i
                self.last_waypoint_index = i
                return best_index

        # If nothing farther than lookahead exists, use final waypoint
        self.last_waypoint_index = len(self.waypoints) - 1
        return len(self.waypoints) - 1

    # ---------------------------------------------------------
    # Coordinate transforms
    # ---------------------------------------------------------
    def global_to_local(self, target_x: float, target_y: float) -> Tuple[float, float]:
        dx = target_x - self.current_x
        dy = target_y - self.current_y

        cos_yaw = math.cos(self.current_yaw)
        sin_yaw = math.sin(self.current_yaw)

        local_x = cos_yaw * dx + sin_yaw * dy
        local_y = -sin_yaw * dx + cos_yaw * dy

        return local_x, local_y

    # ---------------------------------------------------------
    # Publish drive command
    # ---------------------------------------------------------
    def publish_drive(self, speed: float, steering_angle: float) -> None:
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.drive.speed = float(speed)
        msg.drive.steering_angle = float(steering_angle)
        self.drive_pub.publish(msg)

    # ---------------------------------------------------------
    # Helpers
    # ---------------------------------------------------------
    @staticmethod
    def distance(x1: float, y1: float, x2: float, y2: float) -> float:
        return math.hypot(x2 - x1, y2 - y1)

    @staticmethod
    def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PurePursuit()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_drive(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()