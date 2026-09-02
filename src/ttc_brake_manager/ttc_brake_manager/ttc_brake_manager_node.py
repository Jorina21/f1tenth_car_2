#!/usr/bin/env python3
import math
from typing import Optional

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy, LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float64, Bool, String


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


class TTCBrakeManager(Node):
    """
    Safety supervisor for ROS 2 Humble F1TENTH.

    Main jobs:
      1) Watch LiDAR + odometry speed and stop before collision.
      2) Hard brake when deadman is released.
      3) Optional controller soft brake.
      4) Optional brake hold.
      5) Publish zero-speed stop commands to /safety/drive for ackermann_mux.

    Important:
      - TTC uses measured odom speed, not commanded speed.
      - Brake values are treated as amps, not normalized 0.0-1.0.
      - brake_sign can be used to flip brake polarity if your brake topic direction is inverted.
    """

    MODE_NONE = "NONE"
    MODE_HARD_TTC = "HARD_TTC"
    MODE_HARD_DEADMAN = "HARD_DEADMAN"
    MODE_SOFT = "SOFT"
    MODE_HOLD = "HOLD"
    MODE_STALE = "STALE_SENSOR"

    def __init__(self):
        super().__init__("ttc_brake_manager")

        # -------------------------
        # Topics
        # -------------------------
        self.declare_parameter("joy_topic", "/joy")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("cmd_topic", "/ackermann_cmd")

        self.declare_parameter("brake_topic", "/commands/motor/brake")
        self.declare_parameter("drive_enable_topic", "/drive_enable")
        self.declare_parameter("safety_drive_topic", "/safety/drive")
        self.declare_parameter("debug_mode_topic", "/ttc_brake_manager/mode")

        # -------------------------
        # Controller mapping
        # -------------------------
        self.declare_parameter("deadman_button_index", 4)
        self.declare_parameter("deadman_pressed_value", 1)

        self.declare_parameter("soft_brake_axis_index", 3)
        self.declare_parameter("soft_brake_axis_mode", "auto")
        self.declare_parameter("soft_brake_deadzone", 0.08)

        # -------------------------
        # Brake values
        # These are amps for /commands/motor/brake.
        # Do NOT clamp these to 0.0-1.0.
        # -------------------------
        self.declare_parameter("hard_brake", 5.0)
        self.declare_parameter("soft_brake_max", 3.0)
        self.declare_parameter("brake_sign", 1.0)

        self.declare_parameter("enable_brake_hold", False)
        self.declare_parameter("stop_speed_threshold", 0.10)
        self.declare_parameter("brake_hold", 1.0)

        # -------------------------
        # Mux seizure behavior
        # -------------------------
        self.declare_parameter("stop_on_soft_brake", True)
        self.declare_parameter("stop_on_brake_hold", False)

        # -------------------------
        # TTC
        # -------------------------
        self.declare_parameter("ttc_enabled", False)
        self.declare_parameter("ttc_threshold", 0.55)
        self.declare_parameter("ttc_release_threshold", 0.80)
        self.declare_parameter("ttc_min_speed", 0.40)
        self.declare_parameter("front_angle_deg", 30.0)
        self.declare_parameter("ttc_min_range", 0.20)
        self.declare_parameter("use_abs_odom_speed", True)

        # -------------------------
        # Close-distance emergency stop
        # -------------------------
        self.declare_parameter("enable_distance_stop", False)
        self.declare_parameter("distance_stop_range", 0.35)
        self.declare_parameter("distance_stop_front_angle_deg", 15.0)

        # -------------------------
        # Safety latch timing
        # -------------------------
        self.declare_parameter("emergency_hold_time", 0.35)
        self.declare_parameter("deadman_release_hold_time", 0.20)
        self.declare_parameter("soft_brake_release_hold_time", 0.10)
        self.declare_parameter("hold_release_hold_time", 0.10)
        self.declare_parameter("stale_sensor_hold_time", 0.40)

        # -------------------------
        # Stale sensor protection
        # -------------------------
        self.declare_parameter("require_fresh_scan", True)
        self.declare_parameter("require_fresh_odom", True)
        self.declare_parameter("scan_timeout_sec", 0.30)
        self.declare_parameter("odom_timeout_sec", 0.30)
        self.declare_parameter("joy_timeout_sec", 0.75)

        # -------------------------
        # Loop / publish
        # -------------------------
        self.declare_parameter("publish_rate", 50.0)
        self.declare_parameter("brake_publish_epsilon", 0.001)

        # -------------------------
        # Debug
        # -------------------------
        self.declare_parameter("debug_brake_mode", True)
        self.declare_parameter("debug_ttc", True)
        self.declare_parameter("debug_ttc_rate_sec", 0.5)

        # -------------------------
        # Load params
        # -------------------------
        self.joy_topic = str(self.get_parameter("joy_topic").value)
        self.scan_topic = str(self.get_parameter("scan_topic").value)
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.cmd_topic = str(self.get_parameter("cmd_topic").value)

        self.brake_topic = str(self.get_parameter("brake_topic").value)
        self.drive_enable_topic = str(self.get_parameter("drive_enable_topic").value)
        self.safety_drive_topic = str(self.get_parameter("safety_drive_topic").value)
        self.debug_mode_topic = str(self.get_parameter("debug_mode_topic").value)

        self.deadman_button_index = int(self.get_parameter("deadman_button_index").value)
        self.deadman_pressed_value = int(self.get_parameter("deadman_pressed_value").value)

        self.soft_brake_axis_index = int(self.get_parameter("soft_brake_axis_index").value)
        self.soft_brake_axis_mode = str(self.get_parameter("soft_brake_axis_mode").value)
        self.soft_brake_deadzone = float(self.get_parameter("soft_brake_deadzone").value)

        self.hard_brake = float(self.get_parameter("hard_brake").value)
        self.soft_brake_max = float(self.get_parameter("soft_brake_max").value)
        self.brake_sign = float(self.get_parameter("brake_sign").value)

        self.enable_brake_hold = bool(self.get_parameter("enable_brake_hold").value)
        self.stop_speed_threshold = float(self.get_parameter("stop_speed_threshold").value)
        self.brake_hold = float(self.get_parameter("brake_hold").value)

        self.stop_on_soft_brake = bool(self.get_parameter("stop_on_soft_brake").value)
        self.stop_on_brake_hold = bool(self.get_parameter("stop_on_brake_hold").value)

        self.ttc_enabled = bool(self.get_parameter("ttc_enabled").value)
        self.ttc_threshold = float(self.get_parameter("ttc_threshold").value)
        self.ttc_release_threshold = float(self.get_parameter("ttc_release_threshold").value)
        self.ttc_min_speed = float(self.get_parameter("ttc_min_speed").value)
        self.front_angle_rad = math.radians(float(self.get_parameter("front_angle_deg").value))
        self.ttc_min_range = float(self.get_parameter("ttc_min_range").value)
        self.use_abs_odom_speed = bool(self.get_parameter("use_abs_odom_speed").value)

        self.enable_distance_stop = bool(self.get_parameter("enable_distance_stop").value)
        self.distance_stop_range = float(self.get_parameter("distance_stop_range").value)
        self.distance_stop_front_angle_rad = math.radians(
            float(self.get_parameter("distance_stop_front_angle_deg").value)
        )

        self.emergency_hold_time = float(self.get_parameter("emergency_hold_time").value)
        self.deadman_release_hold_time = float(self.get_parameter("deadman_release_hold_time").value)
        self.soft_brake_release_hold_time = float(self.get_parameter("soft_brake_release_hold_time").value)
        self.hold_release_hold_time = float(self.get_parameter("hold_release_hold_time").value)
        self.stale_sensor_hold_time = float(self.get_parameter("stale_sensor_hold_time").value)

        self.require_fresh_scan = bool(self.get_parameter("require_fresh_scan").value)
        self.require_fresh_odom = bool(self.get_parameter("require_fresh_odom").value)
        self.scan_timeout_sec = float(self.get_parameter("scan_timeout_sec").value)
        self.odom_timeout_sec = float(self.get_parameter("odom_timeout_sec").value)
        self.joy_timeout_sec = float(self.get_parameter("joy_timeout_sec").value)

        self.publish_rate = float(self.get_parameter("publish_rate").value)
        self.brake_publish_epsilon = float(self.get_parameter("brake_publish_epsilon").value)

        self.debug_brake_mode = bool(self.get_parameter("debug_brake_mode").value)
        self.debug_ttc = bool(self.get_parameter("debug_ttc").value)
        self.debug_ttc_rate_sec = float(self.get_parameter("debug_ttc_rate_sec").value)

        # -------------------------
        # Parameter safety
        # -------------------------
        self.hard_brake = max(0.0, self.hard_brake)
        self.soft_brake_max = max(0.0, self.soft_brake_max)
        self.brake_hold = max(0.0, self.brake_hold)

        # Only allow brake sign to be +1 or -1.
        if self.brake_sign >= 0.0:
            self.brake_sign = 1.0
        else:
            self.brake_sign = -1.0

        self.ttc_threshold = max(0.01, self.ttc_threshold)
        self.ttc_release_threshold = max(self.ttc_release_threshold, self.ttc_threshold)
        self.ttc_min_speed = max(0.0, self.ttc_min_speed)
        self.ttc_min_range = max(0.0, self.ttc_min_range)

        self.distance_stop_range = max(0.0, self.distance_stop_range)
        self.publish_rate = max(1.0, self.publish_rate)

        # -------------------------
        # Runtime state
        # -------------------------
        self.deadman_held = False
        self.soft_brake_norm = 0.0

        self.cmd_speed = 0.0
        self.cmd_steering = 0.0
        self.odom_speed = 0.0

        self.last_scan: Optional[LaserScan] = None
        self.last_scan_ns = 0
        self.last_odom_ns = 0
        self.last_joy_ns = 0
        self.last_cmd_ns = 0

        self.safety_active = False
        self.safety_mode = self.MODE_NONE
        self.safety_release_after_ns = 0

        self.braking_active = False
        self.last_logged_mode = None
        self._last_ttc_log_ns = 0

        # -------------------------
        # Publishers
        # -------------------------
        self.brake_pub = self.create_publisher(Float64, self.brake_topic, 10)
        self.enable_pub = self.create_publisher(Bool, self.drive_enable_topic, 10)
        self.safety_drive_pub = self.create_publisher(
            AckermannDriveStamped,
            self.safety_drive_topic,
            10,
        )
        self.mode_pub = self.create_publisher(String, self.debug_mode_topic, 10)

        # -------------------------
        # Subscribers
        # -------------------------
        self.create_subscription(Joy, self.joy_topic, self.cb_joy, 10)
        self.create_subscription(LaserScan, self.scan_topic, self.cb_scan, 10)
        self.create_subscription(Odometry, self.odom_topic, self.cb_odom, 10)
        self.create_subscription(AckermannDriveStamped, self.cmd_topic, self.cb_cmd, 10)

        # -------------------------
        # Timer
        # -------------------------
        period = 1.0 / self.publish_rate
        self.timer = self.create_timer(period, self.on_timer)

        self.get_logger().info(
            "ttc_brake_manager started.\n"
            f"  joy_topic:              {self.joy_topic}\n"
            f"  scan_topic:             {self.scan_topic}\n"
            f"  odom_topic:             {self.odom_topic}\n"
            f"  cmd_topic:              {self.cmd_topic}\n"
            f"  brake_topic:            {self.brake_topic}\n"
            f"  safety_drive:           {self.safety_drive_topic}\n"
            f"  deadman_button_index:   {self.deadman_button_index}\n"
            f"  deadman_pressed_value:  {self.deadman_pressed_value}\n"
            f"  hard_brake:             {self.hard_brake:.2f}\n"
            f"  soft_brake_max:         {self.soft_brake_max:.2f}\n"
            f"  brake_sign:             {self.brake_sign:.1f}\n"
            f"  ttc_enabled:            {self.ttc_enabled}\n"
            f"  ttc_threshold:          {self.ttc_threshold:.2f}s\n"
            f"  ttc_release:            {self.ttc_release_threshold:.2f}s\n"
            f"  front_angle:            {math.degrees(self.front_angle_rad):.1f} deg\n"
            "IMPORTANT: ackermann_mux should be the only publisher to /ackermann_cmd."
        )

    # -------------------------
    # Callbacks
    # -------------------------
    def cb_joy(self, msg: Joy):
        self.last_joy_ns = self.get_clock().now().nanoseconds

        if self.deadman_button_index < len(msg.buttons):
            button_value = int(msg.buttons[self.deadman_button_index])
            self.deadman_held = button_value == self.deadman_pressed_value
        else:
            self.deadman_held = False
            self.get_logger().warn(
                "deadman_button_index out of range",
                throttle_duration_sec=1.0,
            )

        if self.soft_brake_axis_index < len(msg.axes):
            raw = float(msg.axes[self.soft_brake_axis_index])
            self.soft_brake_norm = self.normalize_trigger(raw)
        else:
            self.soft_brake_norm = 0.0
            self.get_logger().warn(
                "soft_brake_axis_index out of range",
                throttle_duration_sec=1.0,
            )

    def cb_scan(self, msg: LaserScan):
        self.last_scan = msg
        self.last_scan_ns = self.get_clock().now().nanoseconds

    def cb_odom(self, msg: Odometry):
        self.last_odom_ns = self.get_clock().now().nanoseconds

        vx = float(msg.twist.twist.linear.x)
        vy = float(msg.twist.twist.linear.y)
        speed_mag = math.hypot(vx, vy)

        if self.use_abs_odom_speed:
            self.odom_speed = speed_mag
        else:
            self.odom_speed = vx

    def cb_cmd(self, msg: AckermannDriveStamped):
        self.last_cmd_ns = self.get_clock().now().nanoseconds
        self.cmd_speed = float(msg.drive.speed)
        self.cmd_steering = float(msg.drive.steering_angle)

    # -------------------------
    # Input helpers
    # -------------------------
    def normalize_trigger(self, raw: float) -> float:
        mode = self.soft_brake_axis_mode

        if mode == "1_to_minus1":
            # released: +1, pressed: -1
            x = (1.0 - raw) * 0.5
        elif mode == "0_to_1":
            # released: 0, pressed: 1
            x = raw
        elif mode == "minus1_to_1":
            # released: -1, pressed: +1
            x = (raw + 1.0) * 0.5
        else:
            # Auto mode for common controller trigger formats.
            if raw > 0.5:
                x = (1.0 - raw) * 0.5
            elif raw < -0.5:
                x = (raw + 1.0) * 0.5
            else:
                x = raw

        x = clamp(x, 0.0, 1.0)

        if x < self.soft_brake_deadzone:
            x = 0.0

        return x

    def age_sec(self, stamp_ns: int, now_ns: int) -> Optional[float]:
        if stamp_ns <= 0:
            return None
        return float(now_ns - stamp_ns) / 1e9

    def data_is_stale(self, now_ns: int) -> bool:
        scan_age = self.age_sec(self.last_scan_ns, now_ns)
        odom_age = self.age_sec(self.last_odom_ns, now_ns)
        joy_age = self.age_sec(self.last_joy_ns, now_ns)

        if self.require_fresh_scan:
            if scan_age is None or scan_age > self.scan_timeout_sec:
                return True

        if self.require_fresh_odom:
            if odom_age is None or odom_age > self.odom_timeout_sec:
                return True

        # If joystick disappears, fail safe by treating deadman as released.
        if joy_age is None or joy_age > self.joy_timeout_sec:
            self.deadman_held = False

        return False

    # -------------------------
    # TTC logic
    # -------------------------
    def compute_min_ttc(self) -> Optional[float]:
        if not self.ttc_enabled:
            return None

        if self.last_scan is None:
            return None

        v = max(0.0, float(self.odom_speed))

        if v < self.ttc_min_speed:
            return None

        scan = self.last_scan
        min_ttc = None

        for i, r in enumerate(scan.ranges):
            if r is None:
                continue

            r = float(r)

            if not math.isfinite(r):
                continue

            if r < max(scan.range_min, self.ttc_min_range):
                continue

            if r > scan.range_max:
                continue

            angle = float(scan.angle_min) + i * float(scan.angle_increment)

            if abs(angle) > self.front_angle_rad:
                continue

            projection = math.cos(angle)

            if projection <= 0.0:
                continue

            closing_speed = v * projection

            if closing_speed <= 1e-6:
                continue

            ttc = r / closing_speed

            if min_ttc is None or ttc < min_ttc:
                min_ttc = ttc

        return min_ttc

    def front_distance_too_close(self) -> bool:
        if not self.enable_distance_stop:
            return False

        if self.last_scan is None:
            return False

        scan = self.last_scan

        for i, r in enumerate(scan.ranges):
            if r is None:
                continue

            r = float(r)

            if not math.isfinite(r):
                continue

            if r < scan.range_min or r > scan.range_max:
                continue

            angle = float(scan.angle_min) + i * float(scan.angle_increment)

            if abs(angle) <= self.distance_stop_front_angle_rad:
                if r <= self.distance_stop_range:
                    return True

        return False

    # -------------------------
    # Safety latch helpers
    # -------------------------
    def set_safety_latch(self, mode: str, hold_seconds: float):
        now_ns = self.get_clock().now().nanoseconds
        release_ns = now_ns + int(max(hold_seconds, 0.0) * 1e9)

        if not self.safety_active or mode != self.safety_mode:
            self.safety_active = True
            self.safety_mode = mode
            self.safety_release_after_ns = release_ns
            return

        if release_ns > self.safety_release_after_ns:
            self.safety_release_after_ns = release_ns

    def clear_safety_if_released(
        self,
        min_ttc: Optional[float],
        want_distance_stop: bool,
        want_deadman_stop: bool,
        want_soft_stop: bool,
        want_hold_stop: bool,
        want_stale_stop: bool,
    ):
        if not self.safety_active:
            return

        now_ns = self.get_clock().now().nanoseconds

        if now_ns < self.safety_release_after_ns:
            return

        if self.safety_mode == self.MODE_HARD_TTC:
            ttc_clear = min_ttc is None or min_ttc >= self.ttc_release_threshold

            if ttc_clear and not want_distance_stop:
                self.safety_active = False
                self.safety_mode = self.MODE_NONE

        elif self.safety_mode == self.MODE_STALE:
            if not want_stale_stop:
                self.safety_active = False
                self.safety_mode = self.MODE_NONE

        elif self.safety_mode == self.MODE_HARD_DEADMAN:
            if not want_deadman_stop:
                self.safety_active = False
                self.safety_mode = self.MODE_NONE

        elif self.safety_mode == self.MODE_SOFT:
            if not want_soft_stop:
                self.safety_active = False
                self.safety_mode = self.MODE_NONE

        elif self.safety_mode == self.MODE_HOLD:
            if not want_hold_stop:
                self.safety_active = False
                self.safety_mode = self.MODE_NONE

    # -------------------------
    # Publish helpers
    # -------------------------
    def publish_safety_stop(self):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"

        msg.drive.speed = 0.0
        msg.drive.steering_angle = float(self.cmd_steering)
        msg.drive.steering_angle_velocity = 0.0
        msg.drive.acceleration = 0.0
        msg.drive.jerk = 0.0

        self.safety_drive_pub.publish(msg)

    def publish_mode(self, mode: str):
        msg = String()
        msg.data = mode
        self.mode_pub.publish(msg)

    def publish_brake(self, brake_value: float):
        brake_cmd = self.brake_sign * float(brake_value)
        self.brake_pub.publish(Float64(data=brake_cmd))

    def log_mode_if_changed(self, mode: str, brake: float, min_ttc: Optional[float]):
        if not self.debug_brake_mode:
            return

        if mode == self.last_logged_mode:
            return

        ttc_str = "None" if min_ttc is None else f"{min_ttc:.2f}s"
        signed_brake = self.brake_sign * float(brake)

        self.get_logger().info(
            f"Brake Mode: {mode} | brake={brake:.2f} | signed_brake={signed_brake:.2f} | "
            f"deadman={self.deadman_held} | "
            f"cmd_speed={self.cmd_speed:.2f} | "
            f"odom_speed={self.odom_speed:.2f} | "
            f"cmd_steer={self.cmd_steering:.3f} | "
            f"min_ttc={ttc_str}"
        )

        self.last_logged_mode = mode

    # -------------------------
    # Main timer
    # -------------------------
    def on_timer(self):
        now_ns = self.get_clock().now().nanoseconds

        stale = self.data_is_stale(now_ns)
        min_ttc = self.compute_min_ttc()
        distance_stop = self.front_distance_too_close()

        if self.debug_ttc and min_ttc is not None:
            interval_ns = int(max(self.debug_ttc_rate_sec, 0.05) * 1e9)

            if now_ns - self._last_ttc_log_ns >= interval_ns:
                self.get_logger().info(
                    f"Min TTC: {min_ttc:.2f}s | odom_speed={self.odom_speed:.2f}"
                )
                self._last_ttc_log_ns = now_ns

        soft_brake = self.soft_brake_norm * self.soft_brake_max

        want_stale_stop = stale
        want_deadman_stop = not self.deadman_held
        want_ttc_stop = min_ttc is not None and min_ttc < self.ttc_threshold
        want_distance_stop = distance_stop

        want_soft = soft_brake > self.brake_publish_epsilon
        want_soft_stop = want_soft and self.stop_on_soft_brake

        want_hold = False

        if self.enable_brake_hold:
            no_major_stop = not (
                want_ttc_stop
                or want_distance_stop
                or want_deadman_stop
                or want_soft
                or want_stale_stop
            )

            if no_major_stop:
                if abs(self.odom_speed) <= self.stop_speed_threshold:
                    if abs(self.cmd_speed) <= self.stop_speed_threshold:
                        want_hold = True

        want_hold_stop = want_hold and self.stop_on_brake_hold

        # Priority:
        # 1) TTC or distance emergency
        # 2) stale sensor
        # 3) deadman released
        # 4) soft brake
        # 5) hold
        if want_ttc_stop or want_distance_stop:
            self.set_safety_latch(self.MODE_HARD_TTC, self.emergency_hold_time)

        elif want_stale_stop:
            self.set_safety_latch(self.MODE_STALE, self.stale_sensor_hold_time)

        elif want_deadman_stop:
            self.set_safety_latch(
                self.MODE_HARD_DEADMAN,
                self.deadman_release_hold_time,
            )

        elif want_soft_stop:
            self.set_safety_latch(
                self.MODE_SOFT,
                self.soft_brake_release_hold_time,
            )

        elif want_hold_stop:
            self.set_safety_latch(
                self.MODE_HOLD,
                self.hold_release_hold_time,
            )

        self.clear_safety_if_released(
            min_ttc=min_ttc,
            want_distance_stop=want_distance_stop,
            want_deadman_stop=want_deadman_stop,
            want_soft_stop=want_soft_stop,
            want_hold_stop=want_hold_stop,
            want_stale_stop=want_stale_stop,
        )

        drive_enable = self.deadman_held and not (
            self.safety_active
            and self.safety_mode in (self.MODE_HARD_TTC, self.MODE_STALE)
        )

        self.enable_pub.publish(Bool(data=drive_enable))

        if self.safety_active:
            if self.safety_mode in (
                self.MODE_HARD_TTC,
                self.MODE_HARD_DEADMAN,
                self.MODE_STALE,
            ):
                brake = self.hard_brake
                mode = self.safety_mode

            elif self.safety_mode == self.MODE_SOFT:
                brake = max(soft_brake, self.brake_publish_epsilon)
                mode = self.MODE_SOFT

            elif self.safety_mode == self.MODE_HOLD:
                brake = self.brake_hold
                mode = self.MODE_HOLD

            else:
                brake = 0.0
                mode = self.MODE_NONE

        else:
            if want_ttc_stop or want_distance_stop:
                brake = self.hard_brake
                mode = self.MODE_HARD_TTC

            elif want_stale_stop:
                brake = self.hard_brake
                mode = self.MODE_STALE

            elif want_deadman_stop:
                brake = self.hard_brake
                mode = self.MODE_HARD_DEADMAN

            elif want_soft:
                brake = soft_brake
                mode = self.MODE_SOFT

            elif want_hold:
                brake = self.brake_hold
                mode = self.MODE_HOLD

            else:
                brake = 0.0
                mode = self.MODE_NONE

        self.log_mode_if_changed(mode, brake, min_ttc)
        self.publish_mode(mode)

        if self.safety_active:
            self.publish_safety_stop()

        if abs(brake) > self.brake_publish_epsilon:
            self.braking_active = True
            self.publish_brake(brake)
        else:
            if self.braking_active:
                self.brake_pub.publish(Float64(data=0.0))
                self.braking_active = False


def main(args=None):
    rclpy.init(args=args)
    node = TTCBrakeManager()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        try:
            node.brake_pub.publish(Float64(data=0.0))
        except Exception:
            pass

        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()