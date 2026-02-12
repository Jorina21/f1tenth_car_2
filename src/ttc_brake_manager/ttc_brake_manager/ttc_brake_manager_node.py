#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float64, Bool


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


class TTCBrakeManager(Node):
    """
    Brake authority + safety stop injector for ackermann_mux.

    Key behaviors:
      - Publishes /commands/motor/brake ONLY when braking is active.
        When braking ends: publishes a single 0.0 and then goes silent.
      - When braking is active, publishes a STOP AckermannDriveStamped into the mux
        at a high-priority input topic (default: /safety/drive).
        This prevents speed commands from overpowering brake current.

    Priority (highest to lowest):
      1) TTC emergency -> hard brake + mux stop
      2) Deadman released -> hard brake + mux stop
      3) L2 pressed -> soft brake + mux stop (configurable)
      4) Optional brake-hold when cmd_speed ~ 0 (configurable)
      5) Otherwise -> no brake, no mux stop
    """

    def __init__(self):
        super().__init__("ttc_brake_manager")

        # Topics
        self.declare_parameter("joy_topic", "/joy")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("cmd_topic", "/ackermann_cmd")      # read commanded speed for TTC estimate
        self.declare_parameter("brake_topic", "/commands/motor/brake")
        self.declare_parameter("drive_enable_topic", "/drive_enable")

        # New: Safety stop injection into mux input
        self.declare_parameter("safety_drive_topic", "/safety/drive")
        self.declare_parameter("stop_on_soft_brake", True)         # if True, L2 braking also injects STOP into mux
        self.declare_parameter("stop_on_brake_hold", True)         # if True, brake-hold also injects STOP into mux

        # PS5 mappings
        self.declare_parameter("l1_button_index", 4)
        self.declare_parameter("l2_axis_index", 2)
        self.declare_parameter("l2_axis_mode", "auto")             # auto | 0_to_1 | 1_to_minus1 | minus1_to_1
        self.declare_parameter("soft_brake_deadzone", 0.08)

        # Brake tuning
        self.declare_parameter("hard_brake", 0.85)
        self.declare_parameter("soft_brake_max", 0.45)

        # Optional: controlled stop when cmd_speed ~ 0 (prevents coasting)
        self.declare_parameter("enable_brake_hold", True)
        self.declare_parameter("stop_speed_threshold", 0.10)       # m/s
        self.declare_parameter("brake_hold", 0.25)                 # small brake to prevent roll/coast

        # TTC tuning
        self.declare_parameter("ttc_enabled", True)
        self.declare_parameter("ttc_threshold", 0.55)
        self.declare_parameter("ttc_release_threshold", 0.80)
        self.declare_parameter("ttc_min_speed", 0.40)
        self.declare_parameter("front_angle_deg", 30.0)
        self.declare_parameter("ttc_min_range", 0.20)
        self.declare_parameter("emergency_hold_time", 0.35)

        # Publishing / behavior
        self.declare_parameter("publish_rate", 50.0)
        self.declare_parameter("brake_publish_epsilon", 1e-3)      # below this, treat as "no brake"

        # Load params
        self.joy_topic = str(self.get_parameter("joy_topic").value)
        self.scan_topic = str(self.get_parameter("scan_topic").value)
        self.cmd_topic = str(self.get_parameter("cmd_topic").value)
        self.brake_topic = str(self.get_parameter("brake_topic").value)
        self.drive_enable_topic = str(self.get_parameter("drive_enable_topic").value)

        self.safety_drive_topic = str(self.get_parameter("safety_drive_topic").value)
        self.stop_on_soft_brake = bool(self.get_parameter("stop_on_soft_brake").value)
        self.stop_on_brake_hold = bool(self.get_parameter("stop_on_brake_hold").value)

        self.l1_button_index = int(self.get_parameter("l1_button_index").value)
        self.l2_axis_index = int(self.get_parameter("l2_axis_index").value)
        self.l2_axis_mode = str(self.get_parameter("l2_axis_mode").value)
        self.soft_brake_deadzone = float(self.get_parameter("soft_brake_deadzone").value)

        self.hard_brake = float(self.get_parameter("hard_brake").value)
        self.soft_brake_max = float(self.get_parameter("soft_brake_max").value)

        self.enable_brake_hold = bool(self.get_parameter("enable_brake_hold").value)
        self.stop_speed_threshold = float(self.get_parameter("stop_speed_threshold").value)
        self.brake_hold = float(self.get_parameter("brake_hold").value)

        self.ttc_enabled = bool(self.get_parameter("ttc_enabled").value)
        self.ttc_threshold = float(self.get_parameter("ttc_threshold").value)
        self.ttc_release_threshold = float(self.get_parameter("ttc_release_threshold").value)
        self.ttc_min_speed = float(self.get_parameter("ttc_min_speed").value)
        self.front_angle_rad = math.radians(float(self.get_parameter("front_angle_deg").value))
        self.ttc_min_range = float(self.get_parameter("ttc_min_range").value)
        self.emergency_hold_time = float(self.get_parameter("emergency_hold_time").value)

        self.publish_rate = float(self.get_parameter("publish_rate").value)
        self.brake_publish_epsilon = float(self.get_parameter("brake_publish_epsilon").value)

        # State
        self.deadman_held = False
        self.l2_norm = 0.0
        self.cmd_speed = 0.0
        self.last_scan: LaserScan | None = None

        self.emergency_active = False
        self.emergency_until_ns = 0

        # Track brake publishing state
        self.braking_active = False  # whether we are actively publishing brake commands

        # Publishers
        self.brake_pub = self.create_publisher(Float64, self.brake_topic, 10)
        self.enable_pub = self.create_publisher(Bool, self.drive_enable_topic, 10)

        # New: publish STOP ackermann command into mux input
        self.safety_drive_pub = self.create_publisher(AckermannDriveStamped, self.safety_drive_topic, 10)

        # Subscribers
        self.create_subscription(Joy, self.joy_topic, self.cb_joy, 10)
        self.create_subscription(LaserScan, self.scan_topic, self.cb_scan, 10)
        self.create_subscription(AckermannDriveStamped, self.cmd_topic, self.cb_cmd, 10)

        # Timer loop
        period = 1.0 / max(self.publish_rate, 1.0)
        self.timer = self.create_timer(period, self.on_timer)

        self.get_logger().info(
            "ttc_brake_manager started with safety stop injection.\n"
            f"  safety_drive_topic: {self.safety_drive_topic}\n"
            f"  brake_topic:        {self.brake_topic}\n"
            f"  cmd_topic:          {self.cmd_topic}\n"
            "Make sure ackermann_mux subscribes to safety_drive_topic with highest priority."
        )

    # ---------- Joy ----------
    def cb_joy(self, msg: Joy):
        if self.l1_button_index < len(msg.buttons):
            self.deadman_held = (msg.buttons[self.l1_button_index] == 1)
        else:
            self.deadman_held = False
            self.get_logger().warn_throttle(2.0, "L1 index out of range")

        if self.l2_axis_index < len(msg.axes):
            raw = float(msg.axes[self.l2_axis_index])
            self.l2_norm = self.normalize_l2(raw)
        else:
            self.l2_norm = 0.0
            self.get_logger().warn_throttle(2.0, "L2 index out of range")

    def normalize_l2(self, raw: float) -> float:
        mode = self.l2_axis_mode

        if mode == "1_to_minus1":
            x = (1.0 - raw) * 0.5
        elif mode == "0_to_1":
            x = raw
        elif mode == "minus1_to_1":
            x = (raw + 1.0) * 0.5
        else:
            # auto-detect common cases
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

    # ---------- Commanded speed ----------
    def cb_cmd(self, msg: AckermannDriveStamped):
        self.cmd_speed = float(msg.drive.speed)

    # ---------- Scan ----------
    def cb_scan(self, msg: LaserScan):
        self.last_scan = msg

    def compute_min_ttc(self) -> float | None:
        if not self.ttc_enabled or self.last_scan is None:
            return None

        v = max(0.0, float(self.cmd_speed))
        if v < self.ttc_min_speed:
            return None

        scan = self.last_scan
        a_min = float(scan.angle_min)
        a_inc = float(scan.angle_increment)

        min_ttc = None

        for i, r in enumerate(scan.ranges):
            if r is None or not math.isfinite(r):
                continue
            if r < max(scan.range_min, self.ttc_min_range) or r > scan.range_max:
                continue

            ang = a_min + i * a_inc
            if abs(ang) > self.front_angle_rad:
                continue

            c = math.cos(ang)
            if c <= 0.0:
                continue

            v_close = v * c
            if v_close <= 1e-6:
                continue

            ttc = r / v_close
            if min_ttc is None or ttc < min_ttc:
                min_ttc = ttc

        return min_ttc

    def publish_safety_stop(self):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed = 0.0
        msg.drive.steering_angle = 0.0
        msg.drive.acceleration = 0.0
        msg.drive.jerk = 0.0
        self.safety_drive_pub.publish(msg)

    # ---------- Main loop ----------
    def on_timer(self):
        now_ns = self.get_clock().now().nanoseconds

        # TTC emergency with hysteresis + hold
        min_ttc = self.compute_min_ttc()

        if self.emergency_active and now_ns >= self.emergency_until_ns:
            if (min_ttc is None) or (min_ttc >= self.ttc_release_threshold):
                self.emergency_active = False

        if (not self.emergency_active) and (min_ttc is not None) and (min_ttc < self.ttc_threshold):
            self.emergency_active = True
            self.emergency_until_ns = now_ns + int(self.emergency_hold_time * 1e9)

        # drive_enable (optional for other nodes)
        drive_enable = self.deadman_held and (not self.emergency_active)
        self.enable_pub.publish(Bool(data=drive_enable))

        # Determine braking intent
        soft_brake = self.l2_norm * self.soft_brake_max
        want_hard = self.emergency_active or (not self.deadman_held)
        want_soft = soft_brake > self.brake_publish_epsilon

        want_hold = False
        if self.enable_brake_hold and (not want_hard) and (not want_soft):
            if abs(self.cmd_speed) <= self.stop_speed_threshold:
                want_hold = True

        # Decide final brake value and whether to inject safety stop
        if want_hard:
            brake = self.hard_brake
            inject_stop = True
        elif want_soft:
            brake = soft_brake
            inject_stop = self.stop_on_soft_brake
        elif want_hold:
            brake = self.brake_hold
            inject_stop = self.stop_on_brake_hold
        else:
            brake = 0.0
            inject_stop = False

        # Inject STOP into mux if braking/stop mode is active
        if inject_stop:
            self.publish_safety_stop()

        # Publish brake ONLY when active; when stopping brake, send one 0.0 and go silent
        if abs(brake) > self.brake_publish_epsilon:
            self.braking_active = True
            self.brake_pub.publish(Float64(data=float(brake)))
        else:
            if self.braking_active:
                # release once
                self.brake_pub.publish(Float64(data=0.0))
                self.braking_active = False
            # else: silent


def main():
    rclpy.init()
    node = TTCBrakeManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
