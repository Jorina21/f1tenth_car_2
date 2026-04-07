#!/usr/bin/env python3

import csv
import math
from typing import List, Tuple

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped
from ackermann_msgs.msg import AckermannDriveStamped


class PurePursuit(Node):
    """
    Improved Pure Pursuit controller for F1TENTH.

    Improvements added:
    - Uses AMCL pose (x, y, yaw)
    - Nearest waypoint -> lookahead waypoint selection
    - Dynamic lookahead based on speed
    - Steering smoothing
    - Curvature-based speed reduction
    - Better debug logging
    """

    def __init__(self) -> None:
        super().__init__('pure_pursuit')

        # -----------------------------
        # Parameters
        # -----------------------------
        self.declare_parameter('waypoints_csv', '')
        self.declare_parameter('amcl_topic', '/amcl_pose')
        self.declare_parameter('drive_topic', '/drive')

        self.declare_parameter('base_lookahead', 0.9)
        self.declare_parameter('lookahead_speed_gain', 0.5)

        self.declare_parameter('min_speed', 0.6)
        self.declare_parameter('max_speed', 1.8)
        self.declare_parameter('curvature_speed_gain', 2.0)

        self.declare_parameter('wheelbase', 0.33)
        self.declare_parameter('max_steering_angle', 0.4189)
        self.declare_parameter('goal_tolerance', 0.40)

        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('loop_path', False)

        self.declare_parameter('steering_smoothing_alpha', 0.75)

        self.declare_parameter('debug', True)
        self.declare_parameter('log_pose_every_n', 20)
        self.declare_parameter('log_control_every_n', 10)

        self.waypoints_csv: str = self.get_parameter('waypoints_csv').value
        self.amcl_topic: str = self.get_parameter('amcl_topic').value
        self.drive_topic: str = self.get_parameter('drive_topic').value

        self.base_lookahead: float = float(self.get_parameter('base_lookahead').value)
        self.lookahead_speed_gain: float = float(self.get_parameter('lookahead_speed_gain').value)

        self.min_speed: float = float(self.get_parameter('min_speed').value)
        self.max_speed: float = float(self.get_parameter('max_speed').value)
        self.curvature_speed_gain: float = float(self.get_parameter('curvature_speed_gain').value)

        self.wheelbase: float = float(self.get_parameter('wheelbase').value)
        self.max_steering_angle: float = float(self.get_parameter('max_steering_angle').value)
        self.goal_tolerance: float = float(self.get_parameter('goal_tolerance').value)

        self.control_rate_hz: float = float(self.get_parameter('control_rate_hz').value)
        self.loop_path: bool = bool(self.get_parameter('loop_path').value)

        self.steering_smoothing_alpha: float = float(
            self.get_parameter('steering_smoothing_alpha').value
        )

        self.debug: bool = bool(self.get_parameter('debug').value)
        self.log_pose_every_n: int = int(self.get_parameter('log_pose_every_n').value)
        self.log_control_every_n: int = int(self.get_parameter('log_control_every_n').value)

        # -----------------------------
        # Internal state
        # -----------------------------
        self.waypoints: List[Tuple[float, float]] = self.load_waypoints(self.waypoints_csv)

        self.current_x: float = 0.0
        self.current_y: float = 0.0
        self.current_yaw: float = 0.0

        self.pose_received: bool = False
        self.finished: bool = False

        self.last_nearest_index: int = 0
        self.prev_steering_angle: float = 0.0

        self.pose_log_counter: int = 0
        self.control_log_counter: int = 0

        if not self.waypoints:
            self.get_logger().error('No waypoints loaded. Check CSV path and contents.')
            raise RuntimeError('No waypoints loaded.')

        self.get_logger().info(f'Loaded {len(self.waypoints)} waypoints from: {self.waypoints_csv}')
        self.get_logger().info(f'First 5 waypoints: {self.waypoints[:5]}')

        # -----------------------------
        # ROS interfaces
        # -----------------------------
        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            self.amcl_topic,
            self.amcl_callback,
            10
        )

        self.drive_pub = self.create_publisher(
            AckermannDriveStamped,
            self.drive_topic,
            10
        )

        timer_period = 1.0 / self.control_rate_hz
        self.timer = self.create_timer(timer_period, self.control_loop)

        self.get_logger().info('Improved Pure Pursuit node started.')
        self.get_logger().info(f'AMCL topic: {self.amcl_topic}')
        self.get_logger().info(f'Drive topic: {self.drive_topic}')

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
                        x = float(row[0].strip())
                        y = float(row[1].strip())
                        points.append((x, y))
                    except ValueError:
                        # Skip header or bad rows
                        continue

        except FileNotFoundError:
            self.get_logger().error(f'Waypoint CSV not found: {csv_path}')
        except Exception as e:
            self.get_logger().error(f'Failed to load waypoint CSV: {e}')

        return points

    # ---------------------------------------------------------
    # AMCL callback
    # ---------------------------------------------------------
    def amcl_callback(self, msg: PoseWithCovarianceStamped) -> None:
        pose = msg.pose.pose

        self.current_x = pose.position.x
        self.current_y = pose.position.y

        q = pose.orientation
        self.current_yaw = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)

        self.pose_received = True

        if self.debug:
            self.pose_log_counter += 1
            if self.pose_log_counter % self.log_pose_every_n == 0:
                self.get_logger().info(
                    f'AMCL pose -> x={self.current_x:.3f}, y={self.current_y:.3f}, yaw={self.current_yaw:.3f}'
                )

    # ---------------------------------------------------------
    # Main control loop
    # ---------------------------------------------------------
    def control_loop(self) -> None:
        if not self.pose_received:
            return

        if self.finished:
            self.publish_drive(0.0, 0.0)
            return

        if len(self.waypoints) < 2:
            self.get_logger().warn('Not enough waypoints to follow.')
            self.publish_drive(0.0, 0.0)
            return

        # Stop only for non-loop paths
        if not self.loop_path:
            goal_x, goal_y = self.waypoints[-1]
            dist_to_goal = self.distance(self.current_x, self.current_y, goal_x, goal_y)

            if dist_to_goal < self.goal_tolerance:
                self.get_logger().info('Reached final waypoint. Stopping.')
                self.publish_drive(0.0, 0.0)
                self.finished = True
                return

        nearest_index = self.find_nearest_waypoint_index()

        # Use max_speed here as design speed for lookahead growth
        dynamic_lookahead = self.base_lookahead + self.lookahead_speed_gain * self.max_speed

        target_index = self.find_lookahead_index_from(nearest_index, dynamic_lookahead)
        target_x, target_y = self.waypoints[target_index]

        local_x, local_y = self.global_to_local(target_x, target_y)

        if local_x <= 0.0:
            if self.debug:
                self.get_logger().warn(
                    f'Target behind vehicle. nearest={nearest_index}, target={target_index}, '
                    f'local_x={local_x:.3f}, local_y={local_y:.3f}'
                )
            self.publish_drive(0.0, 0.0)
            return

        Ld = math.hypot(local_x, local_y)
        if Ld < 1e-6:
            self.publish_drive(0.0, 0.0)
            return

        # Pure pursuit curvature
        curvature = 2.0 * local_y / (Ld * Ld)

        # Raw steering
        raw_steering = math.atan(self.wheelbase * curvature)

        # Clamp raw steering
        raw_steering = max(
            -self.max_steering_angle,
            min(self.max_steering_angle, raw_steering)
        )

        # Steering smoothing
        alpha = self.steering_smoothing_alpha
        steering_angle = alpha * self.prev_steering_angle + (1.0 - alpha) * raw_steering
        self.prev_steering_angle = steering_angle

        # Curvature-based speed control
        speed_cmd = self.max_speed / (1.0 + self.curvature_speed_gain * abs(curvature))
        speed_cmd = max(self.min_speed, min(self.max_speed, speed_cmd))

        if self.debug:
            self.control_log_counter += 1
            if self.control_log_counter % self.log_control_every_n == 0:
                self.get_logger().info(
                    f'nearest={nearest_index}, target={target_index}, '
                    f'lookahead={dynamic_lookahead:.2f}, '
                    f'target_xy=({target_x:.2f},{target_y:.2f}), '
                    f'local_xy=({local_x:.2f},{local_y:.2f}), '
                    f'Ld={Ld:.2f}, curv={curvature:.3f}, '
                    f'raw_steer={raw_steering:.3f}, steer={steering_angle:.3f}, '
                    f'speed={speed_cmd:.2f}'
                )

        self.publish_drive(speed_cmd, steering_angle)

    # ---------------------------------------------------------
    # Find nearest waypoint
    # ---------------------------------------------------------
    def find_nearest_waypoint_index(self) -> int:
        if not self.waypoints:
            return 0

        start = max(0, self.last_nearest_index - 20)
        end = min(len(self.waypoints), self.last_nearest_index + 200)

        nearest_index = start
        nearest_dist = float('inf')

        for i in range(start, end):
            wx, wy = self.waypoints[i]
            dist = self.distance(self.current_x, self.current_y, wx, wy)

            if dist < nearest_dist:
                nearest_dist = dist
                nearest_index = i

        # Fallback global search
        if nearest_dist == float('inf'):
            for i, (wx, wy) in enumerate(self.waypoints):
                dist = self.distance(self.current_x, self.current_y, wx, wy)
                if dist < nearest_dist:
                    nearest_dist = dist
                    nearest_index = i

        self.last_nearest_index = nearest_index
        return nearest_index

    # ---------------------------------------------------------
    # Find lookahead waypoint
    # ---------------------------------------------------------
    def find_lookahead_index_from(self, nearest_index: int, lookahead_distance: float) -> int:
        n = len(self.waypoints)

        if n == 0:
            return 0

        if self.loop_path:
            for step in range(n):
                i = (nearest_index + step) % n
                wx, wy = self.waypoints[i]
                dist = self.distance(self.current_x, self.current_y, wx, wy)
                if dist >= lookahead_distance:
                    return i
            return nearest_index

        for i in range(nearest_index, n):
            wx, wy = self.waypoints[i]
            dist = self.distance(self.current_x, self.current_y, wx, wy)
            if dist >= lookahead_distance:
                return i

        return n - 1

    # ---------------------------------------------------------
    # Coordinate transform
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
        try:
            node.publish_drive(0.0, 0.0)
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()