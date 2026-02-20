#!/usr/bin/env python3
import math
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def finite_range(r: float, rmin: float, rmax: float) -> bool:
    return (r is not None) and math.isfinite(r) and (rmin <= r <= rmax)


class WallFollowing(Node):
    """
    Wall following using 2-beam geometry + PID steering.
    This version is designed so behavior is primarily tuned from YAML.
    """

    def __init__(self):
        super().__init__("wall_following")

        # ---- Topics ----
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("drive_topic", "/ackermann_cmd")

        # ---- Wall following geometry ----
        self.declare_parameter("target_distance", 0.55)  # meters
        self.declare_parameter("look_ahead", 0.9)        # meters

        # Beam indices (in LaserScan.ranges array)
        self.declare_parameter("beam_a_id", 340.0)
        self.declare_parameter("beam_b_id", 300.0)

        # ---- PID steering gains ----
        self.declare_parameter("K_p", 0.9)
        self.declare_parameter("K_i", 0.0)
        self.declare_parameter("K_d", 0.25)

        # ---- Output limits ----
        # Typical F1TENTH steering range is ~0.34-0.45 rad depending on your car.
        self.declare_parameter("steer_limit_rad", 0.40)

        # Integral anti-windup clamp (error*sec)
        self.declare_parameter("i_limit", 1.0)

        # ---- Speed schedule ----
        # Speed is selected based on |steering| and optional front clearance.
        self.declare_parameter("speed_straight", 2.0)
        self.declare_parameter("speed_medium", 1.5)
        self.declare_parameter("speed_turn", 1.0)
        self.declare_parameter("speed_hard_turn", 0.6)

        # Steering thresholds in radians
        self.declare_parameter("th_straight", 0.10)
        self.declare_parameter("th_medium", 0.20)
        self.declare_parameter("th_turn", 0.30)

        # Front clearance gating
        self.declare_parameter("front_beam_id", 248.0)
        self.declare_parameter("front_clearance_m", 5.0)

        # ---- Internal controller state ----
        self.prev_time = None
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_steer = 0.0

        scan_topic = self.get_parameter("scan_topic").value
        drive_topic = self.get_parameter("drive_topic").value

        self.scan_sub = self.create_subscription(LaserScan, scan_topic, self.scan_callback, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, drive_topic, 10)

        self.get_logger().info(
            f"wall_following started.\n"
            f"  scan_topic:  {scan_topic}\n"
            f"  drive_topic: {drive_topic}"
        )

    def scan_callback(self, scan: LaserScan):
        # ---- Read parameters (fast enough for wall follow; can optimize later) ----
        target_distance = float(self.get_parameter("target_distance").value)
        look_ahead = float(self.get_parameter("look_ahead").value)

        beam_a_id = int(float(self.get_parameter("beam_a_id").value))
        beam_b_id = int(float(self.get_parameter("beam_b_id").value))

        Kp = float(self.get_parameter("K_p").value)
        Ki = float(self.get_parameter("K_i").value)
        Kd = float(self.get_parameter("K_d").value)

        steer_limit = float(self.get_parameter("steer_limit_rad").value)
        i_limit = float(self.get_parameter("i_limit").value)

        speed_straight = float(self.get_parameter("speed_straight").value)
        speed_medium = float(self.get_parameter("speed_medium").value)
        speed_turn = float(self.get_parameter("speed_turn").value)
        speed_hard_turn = float(self.get_parameter("speed_hard_turn").value)

        th_straight = float(self.get_parameter("th_straight").value)
        th_medium = float(self.get_parameter("th_medium").value)
        th_turn = float(self.get_parameter("th_turn").value)

        front_beam_id = int(float(self.get_parameter("front_beam_id").value))
        front_clearance = float(self.get_parameter("front_clearance_m").value)

        # ---- Compute dt ----
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.prev_time is None:
            self.prev_time = now
            return
        dt = now - self.prev_time
        self.prev_time = now

        # Protect against bad dt (sim time jumps, startup)
        if dt <= 1e-4 or dt > 0.5:
            dt = 0.02  # assume ~50 Hz-ish as fallback

        # ---- Validate indices ----
        n = len(scan.ranges)
        beam_a_id = clamp(beam_a_id, 0, n - 1)
        beam_b_id = clamp(beam_b_id, 0, n - 1)
        front_beam_id = clamp(front_beam_id, 0, n - 1)
        beam_a_id = int(beam_a_id)
        beam_b_id = int(beam_b_id)
        front_beam_id = int(front_beam_id)

        # ---- Read ranges safely ----
        a = scan.ranges[beam_a_id]
        b = scan.ranges[beam_b_id]
        rmin = scan.range_min
        rmax = scan.range_max

        # If beams are invalid, keep last steer and slow down
        if not finite_range(a, rmin, rmax) or not finite_range(b, rmin, rmax):
            steer = clamp(self.prev_steer, -steer_limit, steer_limit)
            speed = speed_hard_turn
            self.publish_drive(speed, steer)
            return

        # ---- Beam angles ----
        a_ang = scan.angle_min + scan.angle_increment * beam_a_id
        b_ang = scan.angle_min + scan.angle_increment * beam_b_id
        theta = b_ang - a_ang

        # Avoid division-by-zero in geometry
        sin_theta = math.sin(theta)
        if abs(sin_theta) < 1e-6:
            steer = clamp(self.prev_steer, -steer_limit, steer_limit)
            speed = speed_hard_turn
            self.publish_drive(speed, steer)
            return

        # ---- Wall geometry ----
        # alpha = atan( (a*cos(theta)-b) / (a*sin(theta)) )
        num = a * math.cos(theta) - b
        den = a * sin_theta
        alpha = math.atan2(num, den)

        # Distance to wall at current position and projected lookahead
        Dt = b * math.cos(alpha)
        Dt1 = Dt + look_ahead * math.sin(alpha)

        # ---- Control error (use projected distance) ----
        error = target_distance - Dt1  # + => we are too far from wall, steer toward it (depending on side)
        # NOTE: If your steering goes the wrong way (drives away from wall), flip sign here:
        # error = -(target_distance - Dt1)

        # ---- PID ----
        # Integral with anti-windup clamp
        self.integral += error * dt
        self.integral = clamp(self.integral, -i_limit, i_limit)

        # Derivative
        derivative = (error - self.prev_error) / dt
        self.prev_error = error

        steer_cmd = (Kp * error) + (Ki * self.integral) + (Kd * derivative)
        steer_cmd = clamp(steer_cmd, -steer_limit, steer_limit)
        self.prev_steer = steer_cmd

        # ---- Speed selection ----
        # Optional front clearance gating
        front_r = scan.ranges[front_beam_id]
        front_ok = finite_range(front_r, rmin, rmax) and (front_r > front_clearance)

        au = abs(steer_cmd)
        if front_ok and au < th_straight:
            speed = speed_straight
        elif au < th_medium:
            speed = speed_medium
        elif au < th_turn:
            speed = speed_turn
        else:
            speed = speed_hard_turn

        # Debug (lightweight)
        self.get_logger().info(
            f"steer={steer_cmd:+.3f} rad | err={error:+.3f} m | "
            f"I={self.integral:+.3f} | D={derivative:+.3f} | v={speed:.2f}"
        )

        self.publish_drive(speed, steer_cmd)

    def publish_drive(self, speed: float, steer: float):
        msg = AckermannDriveStamped()
        msg.drive.speed = float(speed)
        msg.drive.steering_angle = float(steer)
        self.drive_pub.publish(msg)


def main():
    rclpy.init()
    node = WallFollowing()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()