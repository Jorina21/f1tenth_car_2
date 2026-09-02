#!/usr/bin/env python3
import math
from typing import Optional

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Bool, String, Float64


class SimpleTTCBrake(Node):
    """
    Streamlined TTC emergency brake for F1TENTH.

    Behavior:
      - TTC danger triggers brake latch.
      - Brake latch blocks forward commands.
      - Brake latch cannot clear while forward command is still held.
      - Manual reverse is allowed while brake is active.
      - Manual reverse speed is passed through directly from /teleop.
      - Motor brake current is forced to 0.0 during reverse.

    Important:
      /ackermann_cmd is mux output.
      During safety braking, /ackermann_cmd may be 0.0 because /safety/drive wins.
      Therefore manual reverse must be detected from the pre-mux topic /teleop.
    """

    STATUS_DISABLED = "DISABLED"
    STATUS_READY = "READY"
    STATUS_BRAKING_TTC = "BRAKING_TTC"
    STATUS_BRAKING_STALE = "BRAKING_STALE"
    STATUS_ALLOWING_REVERSE = "ALLOWING_REVERSE"

    def __init__(self):
        super().__init__("simple_ttc_brake")

        # ============================================================
        # SIMPLE YAML PARAMETERS
        # ============================================================

        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("mux_cmd_topic", "/ackermann_cmd")
        self.declare_parameter("manual_cmd_topic", "/teleop")
        self.declare_parameter("auto_cmd_topic", "/drive")
        self.declare_parameter("safety_drive_topic", "/safety/drive")
        self.declare_parameter("motor_brake_topic", "/commands/motor/brake")

        self.declare_parameter("enabled_on_start", True)

        self.declare_parameter("ttc_threshold", 0.70)
        self.declare_parameter("front_clear_distance", 0.70)

        self.declare_parameter("use_motor_brake", True)
        self.declare_parameter("motor_brake_value", 2.0)
        self.declare_parameter("motor_brake_sign", -1.0)

        self.declare_parameter("debug", True)

        # ============================================================
        # STATUS TOPICS
        # ============================================================

        self.declare_parameter("enable_topic", "/brake/enable")
        self.declare_parameter("active_topic", "/brake/active")
        self.declare_parameter("status_topic", "/brake/status")
        self.declare_parameter("min_ttc_topic", "/brake/min_ttc")

        # ============================================================
        # INTERNAL CONSTANTS
        # ============================================================

        self.PUBLISH_RATE_HZ = 50.0

        self.TTC_MIN_SPEED = 0.20
        self.TTC_MIN_RANGE = 0.18
        self.TTC_FRONT_ANGLE_DEG = 25.0

        self.FRONT_CLEAR_ANGLE_DEG = 20.0

        self.TRIGGER_COUNT_REQUIRED = 2
        self.CLEAR_COUNT_REQUIRED = 15
        self.MINIMUM_BRAKE_TIME_SEC = 0.75

        self.RELEASE_SPEED_THRESHOLD = 0.15

        self.SCAN_TIMEOUT_SEC = 0.50
        self.ODOM_TIMEOUT_SEC = 0.50
        self.STOP_ON_STALE_DATA = True

        self.MOTOR_BRAKE_MIN_SPEED = 0.08

        self.REVERSE_SPEED_EPSILON = 0.02
        self.REVERSE_HOLD_TIME_SEC = 0.40

        self.FORWARD_HELD_EPSILON = 0.05

        self.REVERSE_ACCELERATION = 0.0
        self.SAFETY_STOP_ACCELERATION = -3.0

        self.DEBUG_RATE_SEC = 0.50

        # ============================================================
        # LOAD PARAMETERS
        # ============================================================

        self.scan_topic = str(self.get_parameter("scan_topic").value)
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.mux_cmd_topic = str(self.get_parameter("mux_cmd_topic").value)
        self.manual_cmd_topic = str(self.get_parameter("manual_cmd_topic").value)
        self.auto_cmd_topic = str(self.get_parameter("auto_cmd_topic").value)
        self.safety_drive_topic = str(self.get_parameter("safety_drive_topic").value)
        self.motor_brake_topic = str(self.get_parameter("motor_brake_topic").value)

        self.enabled = bool(self.get_parameter("enabled_on_start").value)

        self.ttc_threshold = float(self.get_parameter("ttc_threshold").value)
        self.front_clear_distance = float(self.get_parameter("front_clear_distance").value)

        self.use_motor_brake = bool(self.get_parameter("use_motor_brake").value)
        self.motor_brake_value = float(self.get_parameter("motor_brake_value").value)
        self.motor_brake_sign = float(self.get_parameter("motor_brake_sign").value)

        self.debug = bool(self.get_parameter("debug").value)

        self.enable_topic = str(self.get_parameter("enable_topic").value)
        self.active_topic = str(self.get_parameter("active_topic").value)
        self.status_topic = str(self.get_parameter("status_topic").value)
        self.min_ttc_topic = str(self.get_parameter("min_ttc_topic").value)

        # ============================================================
        # PARAMETER SAFETY CLAMPS
        # ============================================================

        self.ttc_threshold = max(0.10, min(2.00, self.ttc_threshold))
        self.ttc_release_threshold = max(self.ttc_threshold + 0.30, 1.00)

        self.front_clear_distance = max(0.20, min(2.00, self.front_clear_distance))

        self.motor_brake_value = max(0.0, min(20.0, self.motor_brake_value))
        self.motor_brake_sign = -1.0 if self.motor_brake_sign < 0.0 else 1.0

        self.ttc_front_angle_rad = math.radians(self.TTC_FRONT_ANGLE_DEG)
        self.front_clear_angle_rad = math.radians(self.FRONT_CLEAR_ANGLE_DEG)

        # ============================================================
        # RUNTIME STATE
        # ============================================================

        self.last_scan: Optional[LaserScan] = None
        self.last_scan_ns = 0

        self.odom_speed = 0.0
        self.last_odom_ns = 0

        self.mux_cmd_speed = 0.0
        self.mux_cmd_steering = 0.0

        self.manual_cmd_speed = 0.0
        self.manual_cmd_steering = 0.0

        self.auto_cmd_speed = 0.0
        self.auto_cmd_steering = 0.0

        self.brake_active = False
        self.brake_reason = self.STATUS_READY
        self.brake_until_ns = 0

        self.bad_count = 0
        self.clear_count = 0

        self.last_reverse_request_ns = 0
        self.last_reverse_speed = 0.0
        self.last_reverse_steering = 0.0

        self.last_debug_ns = 0
        self.last_logged_status = None

        # ============================================================
        # PUBLISHERS
        # ============================================================

        self.safety_drive_pub = self.create_publisher(
            AckermannDriveStamped,
            self.safety_drive_topic,
            10,
        )

        self.motor_brake_pub = self.create_publisher(
            Float64,
            self.motor_brake_topic,
            10,
        )

        self.active_pub = self.create_publisher(Bool, self.active_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.min_ttc_pub = self.create_publisher(Float64, self.min_ttc_topic, 10)

        # ============================================================
        # SUBSCRIBERS
        # ============================================================

        self.create_subscription(LaserScan, self.scan_topic, self.cb_scan, 10)
        self.create_subscription(Odometry, self.odom_topic, self.cb_odom, 10)

        self.create_subscription(
            AckermannDriveStamped,
            self.mux_cmd_topic,
            self.cb_mux_cmd,
            10,
        )

        self.create_subscription(
            AckermannDriveStamped,
            self.manual_cmd_topic,
            self.cb_manual_cmd,
            10,
        )

        self.create_subscription(
            AckermannDriveStamped,
            self.auto_cmd_topic,
            self.cb_auto_cmd,
            10,
        )

        self.create_subscription(Bool, self.enable_topic, self.cb_enable, 10)

        # ============================================================
        # TIMER
        # ============================================================

        self.timer = self.create_timer(
            1.0 / self.PUBLISH_RATE_HZ,
            self.on_timer,
        )

        self.get_logger().info(
            "simple_ttc_brake streamlined node started.\n"
            f"  enabled_on_start:      {self.enabled}\n"
            f"  scan_topic:            {self.scan_topic}\n"
            f"  odom_topic:            {self.odom_topic}\n"
            f"  mux_cmd_topic:         {self.mux_cmd_topic}\n"
            f"  manual_cmd_topic:      {self.manual_cmd_topic}\n"
            f"  auto_cmd_topic:        {self.auto_cmd_topic}\n"
            f"  safety_drive_topic:    {self.safety_drive_topic}\n"
            f"  motor_brake_topic:     {self.motor_brake_topic}\n"
            f"  ttc_threshold:         {self.ttc_threshold:.2f}s\n"
            f"  front_clear_distance:  {self.front_clear_distance:.2f}m\n"
            f"  motor_brake_value:     {self.motor_brake_value:.2f}\n"
            f"  motor_brake_sign:      {self.motor_brake_sign:.1f}\n"
            f"  reverse_hold_time:     {self.REVERSE_HOLD_TIME_SEC:.2f}s\n"
            "  reverse_speed_limit:   REMOVED - manual /teleop reverse speed passes through directly"
        )

    # ============================================================
    # CALLBACKS
    # ============================================================

    def cb_scan(self, msg: LaserScan):
        self.last_scan = msg
        self.last_scan_ns = self.get_clock().now().nanoseconds

    def cb_odom(self, msg: Odometry):
        self.last_odom_ns = self.get_clock().now().nanoseconds

        vx = float(msg.twist.twist.linear.x)
        vy = float(msg.twist.twist.linear.y)

        self.odom_speed = math.hypot(vx, vy)

    def cb_mux_cmd(self, msg: AckermannDriveStamped):
        self.mux_cmd_speed = float(msg.drive.speed)
        self.mux_cmd_steering = float(msg.drive.steering_angle)

    def cb_manual_cmd(self, msg: AckermannDriveStamped):
        self.manual_cmd_speed = float(msg.drive.speed)
        self.manual_cmd_steering = float(msg.drive.steering_angle)

    def cb_auto_cmd(self, msg: AckermannDriveStamped):
        self.auto_cmd_speed = float(msg.drive.speed)
        self.auto_cmd_steering = float(msg.drive.steering_angle)

    def cb_enable(self, msg: Bool):
        self.enabled = bool(msg.data)

        if self.enabled:
            self.brake_reason = self.STATUS_READY
            self.bad_count = 0
            self.clear_count = 0
            self.get_logger().info("TTC brake ENABLED")
        else:
            self.brake_active = False
            self.brake_reason = self.STATUS_DISABLED
            self.bad_count = 0
            self.clear_count = 0
            self.last_reverse_request_ns = 0
            self.publish_motor_brake(0.0)
            self.get_logger().warn("TTC brake DISABLED")

    # ============================================================
    # COMMAND HELPERS
    # ============================================================

    def forward_command_held(self) -> bool:
        manual_forward = self.manual_cmd_speed > self.FORWARD_HELD_EPSILON
        auto_forward = self.auto_cmd_speed > self.FORWARD_HELD_EPSILON
        return manual_forward or auto_forward

    def manual_reverse_requested_raw(self) -> bool:
        return self.manual_cmd_speed < -self.REVERSE_SPEED_EPSILON

    def update_reverse_memory(self, now_ns: int):
        if self.manual_reverse_requested_raw():
            self.last_reverse_request_ns = now_ns
            self.last_reverse_speed = self.manual_cmd_speed
            self.last_reverse_steering = self.manual_cmd_steering

    def reverse_requested(self, now_ns: int) -> bool:
        if self.last_reverse_request_ns <= 0:
            return False

        age_sec = float(now_ns - self.last_reverse_request_ns) / 1e9
        return age_sec <= self.REVERSE_HOLD_TIME_SEC

    def reverse_command_speed(self) -> float:
        # Pass through manual reverse speed directly.
        # No reverse_speed_limit.
        if self.last_reverse_speed < -self.REVERSE_SPEED_EPSILON:
            return self.last_reverse_speed

        return 0.0

    def reverse_command_steering(self) -> float:
        return self.last_reverse_steering

    # ============================================================
    # SENSOR / TTC HELPERS
    # ============================================================

    def age_sec(self, stamp_ns: int, now_ns: int) -> Optional[float]:
        if stamp_ns <= 0:
            return None

        return float(now_ns - stamp_ns) / 1e9

    def data_is_stale(self, now_ns: int) -> bool:
        scan_age = self.age_sec(self.last_scan_ns, now_ns)
        odom_age = self.age_sec(self.last_odom_ns, now_ns)

        scan_stale = scan_age is None or scan_age > self.SCAN_TIMEOUT_SEC
        odom_stale = odom_age is None or odom_age > self.ODOM_TIMEOUT_SEC

        return scan_stale or odom_stale

    def compute_min_ttc(self) -> Optional[float]:
        if self.last_scan is None:
            return None

        if self.odom_speed < self.TTC_MIN_SPEED:
            return None

        scan = self.last_scan
        min_ttc = None

        for i, r in enumerate(scan.ranges):
            if r is None:
                continue

            r = float(r)

            if not math.isfinite(r):
                continue

            if r < max(float(scan.range_min), self.TTC_MIN_RANGE):
                continue

            if r > float(scan.range_max):
                continue

            angle = float(scan.angle_min) + i * float(scan.angle_increment)

            if abs(angle) > self.ttc_front_angle_rad:
                continue

            forward_component = math.cos(angle)

            if forward_component <= 0.0:
                continue

            closing_speed = self.odom_speed * forward_component

            if closing_speed <= 1e-6:
                continue

            ttc = r / closing_speed

            if min_ttc is None or ttc < min_ttc:
                min_ttc = ttc

        return min_ttc

    def front_is_clear(self) -> bool:
        if self.last_scan is None:
            return False

        scan = self.last_scan

        for i, r in enumerate(scan.ranges):
            if r is None:
                continue

            r = float(r)

            if not math.isfinite(r):
                continue

            if r < float(scan.range_min) or r > float(scan.range_max):
                continue

            angle = float(scan.angle_min) + i * float(scan.angle_increment)

            if abs(angle) <= self.front_clear_angle_rad:
                if r < self.front_clear_distance:
                    return False

        return True

    # ============================================================
    # BRAKE STATE HELPERS
    # ============================================================

    def activate_brake(self, reason: str):
        now_ns = self.get_clock().now().nanoseconds
        hold_ns = int(self.MINIMUM_BRAKE_TIME_SEC * 1e9)

        self.brake_active = True
        self.brake_reason = reason
        self.brake_until_ns = max(self.brake_until_ns, now_ns + hold_ns)

    def try_release_brake(self, min_ttc: Optional[float], stale: bool):
        if not self.brake_active:
            return

        now_ns = self.get_clock().now().nanoseconds

        if now_ns < self.brake_until_ns:
            return

        if stale and self.STOP_ON_STALE_DATA:
            self.clear_count = 0
            return

        if self.odom_speed > self.RELEASE_SPEED_THRESHOLD:
            self.clear_count = 0
            return

        # Holding forward gas should never clear/bypass the brake.
        if self.forward_command_held():
            self.clear_count = 0
            return

        if not self.front_is_clear():
            self.clear_count = 0
            return

        if min_ttc is None:
            self.clear_count += 1
        elif min_ttc >= self.ttc_release_threshold:
            self.clear_count += 1
        else:
            self.clear_count = 0

        if self.clear_count >= self.CLEAR_COUNT_REQUIRED:
            self.brake_active = False
            self.brake_reason = self.STATUS_READY
            self.bad_count = 0
            self.clear_count = 0
            self.last_reverse_request_ns = 0
            self.publish_motor_brake(0.0)

    # ============================================================
    # PUBLISH HELPERS
    # ============================================================

    def publish_safety_stop(self):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"

        msg.drive.speed = 0.0
        msg.drive.steering_angle = self.mux_cmd_steering
        msg.drive.steering_angle_velocity = 0.0
        msg.drive.acceleration = self.SAFETY_STOP_ACCELERATION
        msg.drive.jerk = 0.0

        self.safety_drive_pub.publish(msg)

    def publish_safety_reverse(self):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"

        msg.drive.speed = self.reverse_command_speed()
        msg.drive.steering_angle = self.reverse_command_steering()
        msg.drive.steering_angle_velocity = 0.0
        msg.drive.acceleration = self.REVERSE_ACCELERATION
        msg.drive.jerk = 0.0

        self.safety_drive_pub.publish(msg)

    def publish_motor_brake(self, value: float):
        if not self.use_motor_brake:
            return

        signed_value = self.motor_brake_sign * float(value)
        self.motor_brake_pub.publish(Float64(data=signed_value))

    def publish_status(self, min_ttc: Optional[float], status_text: str):
        self.active_pub.publish(Bool(data=self.brake_active))
        self.status_pub.publish(String(data=status_text))

        if min_ttc is None:
            self.min_ttc_pub.publish(Float64(data=-1.0))
        else:
            self.min_ttc_pub.publish(Float64(data=float(min_ttc)))

    def log_status(
        self,
        min_ttc: Optional[float],
        status_text: str,
        stale: bool,
        reverse_active: bool,
        motor_brake_active: bool,
    ):
        if not self.debug:
            return

        now_ns = self.get_clock().now().nanoseconds
        interval_ns = int(self.DEBUG_RATE_SEC * 1e9)

        status_changed = status_text != self.last_logged_status
        time_ready = now_ns - self.last_debug_ns >= interval_ns

        if not status_changed and not time_ready:
            return

        ttc_text = "None" if min_ttc is None else f"{min_ttc:.2f}s"

        reverse_age = -1.0
        if self.last_reverse_request_ns > 0:
            reverse_age = float(now_ns - self.last_reverse_request_ns) / 1e9

        self.get_logger().info(
            f"status={status_text} | "
            f"active={self.brake_active} | "
            f"motor_brake={motor_brake_active} | "
            f"odom_speed={self.odom_speed:.2f} | "
            f"ttc={ttc_text} | "
            f"front_clear={self.front_is_clear()} | "
            f"manual_cmd={self.manual_cmd_speed:.2f} | "
            f"auto_cmd={self.auto_cmd_speed:.2f} | "
            f"forward_held={self.forward_command_held()} | "
            f"manual_reverse_raw={self.manual_reverse_requested_raw()} | "
            f"reverse_hold={reverse_active} | "
            f"reverse_age={reverse_age:.2f} | "
            f"reverse_speed={self.reverse_command_speed():.2f} | "
            f"stale={stale} | "
            f"bad={self.bad_count} | "
            f"clear={self.clear_count}"
        )

        self.last_logged_status = status_text
        self.last_debug_ns = now_ns

    # ============================================================
    # MAIN LOOP
    # ============================================================

    def on_timer(self):
        now_ns = self.get_clock().now().nanoseconds

        if not self.enabled:
            self.brake_active = False
            self.brake_reason = self.STATUS_DISABLED
            self.last_reverse_request_ns = 0
            self.publish_motor_brake(0.0)
            self.publish_status(None, self.STATUS_DISABLED)
            return

        stale = self.data_is_stale(now_ns)
        min_ttc = self.compute_min_ttc()

        # Update manual reverse memory every loop before deciding output.
        self.update_reverse_memory(now_ns)

        if stale and self.STOP_ON_STALE_DATA:
            self.activate_brake(self.STATUS_BRAKING_STALE)

        ttc_danger = min_ttc is not None and min_ttc < self.ttc_threshold

        if ttc_danger:
            self.bad_count += 1
            self.clear_count = 0
        else:
            self.bad_count = 0

        if self.bad_count >= self.TRIGGER_COUNT_REQUIRED:
            self.activate_brake(self.STATUS_BRAKING_TTC)

        self.try_release_brake(min_ttc, stale)

        reverse_active = self.brake_active and self.reverse_requested(now_ns)
        motor_brake_active = False

        if self.brake_active:
            if reverse_active:
                # During manual reverse, never apply brake current.
                # Publish manual reverse through safety directly.
                self.publish_motor_brake(0.0)
                self.publish_safety_reverse()
                status_text = self.STATUS_ALLOWING_REVERSE
            else:
                # During brake, block forward motion.
                self.publish_safety_stop()

                if self.use_motor_brake and self.odom_speed > self.MOTOR_BRAKE_MIN_SPEED:
                    self.publish_motor_brake(self.motor_brake_value)
                    motor_brake_active = True
                else:
                    self.publish_motor_brake(0.0)

                status_text = self.brake_reason
        else:
            self.publish_motor_brake(0.0)
            status_text = self.STATUS_READY

        self.publish_status(min_ttc, status_text)
        self.log_status(min_ttc, status_text, stale, reverse_active, motor_brake_active)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleTTCBrake()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.publish_motor_brake(0.0)
        except Exception:
            pass

        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()