#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class SimpleDriver(Node):
    def __init__(self):
        super().__init__("simple_driver")

        # Publish high-level drive commands
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.drive_pub = self.create_publisher(AckermannDriveStamped, "/ackermann_cmd", qos)

        # Subscribe to LiDAR
        self.scan_sub = self.create_subscription(LaserScan, "/scan", self.scan_callback, 10)

        # Control loop
        self.dt = 0.05  # 20 Hz
        self.timer = self.create_timer(self.dt, self.drive_loop)

        # --- Safety & behavior tuning ---
        self.target_speed = 0.8          # m/s (SAFE test speed)
        self.max_speed = 1.5             # hard cap (m/s)
        self.max_accel = 0.8             # m/s^2 (smooth accel)
        self.max_decel = 1.5             # m/s^2 (smooth brake)
        self.steering_angle = 0.0

        self.brake_distance = 1.0        # meters (stop if closer than this)
        self.release_distance = 1.2      # meters (hysteresis to prevent chatter)

        # LiDAR sector to check (front cone)
        self.front_half_angle_deg = 10.0 # +/- 10 deg cone
        self.min_distance = float("inf")

        # state
        self.current_cmd_speed = 0.0
        self.is_stopped_for_obstacle = False

    def scan_callback(self, msg: LaserScan):
        if not msg.ranges:
            self.min_distance = float("inf")
            return

        # Robustly treat "front" as angle = 0 rad in LaserScan frame.
        # If your LiDAR frame is rotated, fix TF; this still works if TF is correct.
        front_angle = 0.0
        half = math.radians(self.front_half_angle_deg)

        # Compute index range for [front-half, front+half]
        i0 = int((front_angle - half - msg.angle_min) / msg.angle_increment)
        i1 = int((front_angle + half - msg.angle_min) / msg.angle_increment)

        i0 = max(0, min(len(msg.ranges) - 1, i0))
        i1 = max(0, min(len(msg.ranges) - 1, i1))
        if i1 < i0:
            i0, i1 = i1, i0

        # Filter valid ranges
        best = float("inf")
        for r in msg.ranges[i0:i1 + 1]:
            if r is None:
                continue
            if not math.isfinite(r):
                continue
            if r < msg.range_min or r > msg.range_max:
                continue
            if r < best:
                best = r

        self.min_distance = best

    def ramp_toward(self, current: float, target: float, max_rate: float) -> float:
        """Move current toward target by at most max_rate * dt."""
        step = max_rate * self.dt
        if target > current:
            return min(current + step, target)
        else:
            return max(current - step, target)

    def drive_loop(self):
        # Decide stop/go with hysteresis
        if self.is_stopped_for_obstacle:
            # stay stopped until obstacle clears farther than release_distance
            if self.min_distance > self.release_distance:
                self.is_stopped_for_obstacle = False
        else:
            # stop when obstacle is within brake_distance
            if self.min_distance < self.brake_distance:
                self.is_stopped_for_obstacle = True

        # Target speed selection
        desired_speed = 0.0 if self.is_stopped_for_obstacle else self.target_speed
        desired_speed = max(-self.max_speed, min(self.max_speed, desired_speed))

        # Smooth speed changes (accel/decel limits)
        if desired_speed > self.current_cmd_speed:
            self.current_cmd_speed = self.ramp_toward(self.current_cmd_speed, desired_speed, self.max_accel)
        else:
            self.current_cmd_speed = self.ramp_toward(self.current_cmd_speed, desired_speed, self.max_decel)

        # Publish
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.drive.speed = float(self.current_cmd_speed)
        msg.drive.steering_angle = float(self.steering_angle)

        self.drive_pub.publish(msg)

        # Light logging (not spamming every tick)
        if self.is_stopped_for_obstacle:
            self.get_logger().warn(f"STOP obstacle {self.min_distance:.2f} m (brake<{self.brake_distance}, release>{self.release_distance})")


def main(args=None):
    rclpy.init(args=args)
    node = SimpleDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()
