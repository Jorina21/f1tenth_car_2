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
    Pure Pursuit controller using AMCL pose and CSV waypoints.

    Purpose:
    - Load x,y waypoints from CSV
    - Subscribe to /amcl_pose
    - Find nearest waypoint
    - Find a lookahead waypoint ahead on the path
    - Compute steering with pure pursuit
    - Publish AckermannDriveStamped to /drive

    Expected CSV formats:
    x,y
    1.0,2.0
    1.1,2.1

    or

    1.0,2.0
    1.1,2.1
    """

    def __init__(self) -> None:
        super().__init__('pure_pursuit')

        # -----------------------------
        # Parameters
        # -----------------------------
        self.declare_parameter('waypoints_csv', '')
        self.declare_parameter('amcl_topic', '/amcl_pose')
        self.declare_parameter('drive_topic', '/drive')

        self.declare_parameter('lookahead_distance', 0.8)
        self.declare_parameter('speed', 0.6)
        self.declare_parameter('wheelbase', 0.33)
        self.declare_parameter('max_steering_angle', 0.4189)  # ~24 degrees
        self.declare_parameter('goal_tolerance', 0.40)

        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('loop_path', False)

        self.declare_parameter('debug', True)
        self.declare_parameter('log_pose_every_n', 20)
        self.declare_parameter('log_control_every_n', 10)

        self.waypoints_csv: str = self.get_parameter('waypoints_csv').value
        self.amcl_topic: str = self.get_parameter('amcl_topic').value
        self.drive_topic: str = self.get_parameter('drive_topic').value

        self.lookahead_distance: float = float(self.get_parameter('lookahead_distance').value)
        self.speed: float = float(self.get_parameter('speed').value)
        self.wheelbase: float = float(self.get_parameter('wheelbase').value)
        self.max_steering_angle: float = float(self.get_parameter('max_steering_angle').value)
        self.goal_tolerance: float = float(self.get_parameter('goal_tolerance').value)

        self.control_rate_hz: float = float(self.get_parameter('control_rate_hz').value)
        self.loop_path: bool = bool(self.get_parameter('loop_path').value)

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

        self.get_logger().info('Pure Pursuit node started.')
        self.get_logger().info(f'Subscribing to AMCL: {self.amcl_topic}')
        self.get_logger().info(f'Publishing drive commands to: {self.drive_topic}')

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
                        # Skip header or malformed line
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
        target_index = self.find_lookahead_index_from(nearest_index)

        target_x, target_y = self.waypoints[target_index]
        local_x, local_y = self.global_to_local(target_x, target_y)

        # If target is behind car, do not blindly drive straight.
        if local_x <= 0.0:
            if self.debug:
                self.get_logger().warn(
                    f'Target behind vehicle. nearest={nearest_index}, target={target_index}, '
                    f'local_x={local_x:.3f}, local_y={local_y:.3f}. Stopping.'
                )
            self.publish_drive(0.0, 0.0)
            return

        Ld = math.hypot(local_x, local_y)
        if Ld < 1e-6:
            self.publish_drive(0.0, 0.0)
            return

        # Pure pursuit curvature
        curvature = 2.0 * local_y / (Ld * Ld)

        # Ackermann steering
        steering_angle = math.atan(self.wheelbase * curvature)

        # Clamp steering
        steering_angle = max(
            -self.max_steering_angle,
            min(self.max_steering_angle, steering_angle)
        )

        if self.debug:
            self.control_log_counter += 1
            if self.control_log_counter % self.log_control_every_n == 0:
                self.get_logger().info(
                    f'nearest={nearest_index}, target={target_index}, '
                    f'target_xy=({target_x:.2f},{target_y:.2f}), '
                    f'local_xy=({local_x:.2f},{local_y:.2f}), '
                    f'Ld={Ld:.2f}, curv={curvature:.3f}, steer={steering_angle:.3f}, speed={self.speed:.2f}'
                )

        self.publish_drive(self.speed, steering_angle)

    # ---------------------------------------------------------
    # Find nearest waypoint
    # ---------------------------------------------------------
    def find_nearest_waypoint_index(self) -> int:
        """
        Find the nearest waypoint to the current AMCL pose.
        Uses last_nearest_index as a hint so it doesn't jump around as much.
        """
        if not self.waypoints:
            return 0

        # Search window from last_nearest_index forward, but allow a little backward scan too
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

        # Fallback in case the local window somehow fails
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
    def find_lookahead_index_from(self, nearest_index: int) -> int:
        """
        Starting from the nearest waypoint, move forward until the waypoint
        is at least lookahead_distance away.
        """
        n = len(self.waypoints)

        if n == 0:
            return 0

        if self.loop_path:
            for step in range(n):
                i = (nearest_index + step) % n
                wx, wy = self.waypoints[i]
                dist = self.distance(self.current_x, self.current_y, wx, wy)
                if dist >= self.lookahead_distance:
                    return i
            return nearest_index

        for i in range(nearest_index, n):
            wx, wy = self.waypoints[i]
            dist = self.distance(self.current_x, self.current_y, wx, wy)
            if dist >= self.lookahead_distance:
                return i

        return n - 1

    # ---------------------------------------------------------
    # Coordinate transform
    # ---------------------------------------------------------
    def global_to_local(self, target_x: float, target_y: float) -> Tuple[float, float]:
        """
        Convert a point from map/global frame into vehicle local frame.
        Vehicle frame convention used here:
        - +x forward
        - +y left
        """
        dx = target_x - self.current_x
        dy = target_y - self.current_y

        cos_yaw = math.cos(self.current_yaw)
        sin_yaw = math.sin(self.current_yaw)

        local_x = cos_yaw * dx + sin_yaw * dy
        local_y = -sin_yaw * dx + cos_yaw * dy

        return local_x, local_y

    # ---------------------------------------------------------
    # Drive publisher
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