import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Int32


class ThrottleMix(Node):
    def __init__(self):
        super().__init__('throttle_mix')

        self.drive_mode = "DRIVE"

        # Button edges
        self.prev_shift = 0       # R1
        self.prev_gear_up = 0     # Triangle
        self.prev_gear_down = 0   # X

        # Gearbox info
        self.current_gear = 0   # Gear 1 index (0 = Gear 1)

        self.gear_max_speeds = [1.5, 3.0, 6.0, 10]   # 4 gears
        self.gear_accel_rates = [0.06, 0.10, 0.15, 0.20]

        # For throttle smoothing
        self.prev_speed = 0.0

        # ROS subscriptions
        self.sub_joy = self.create_subscription(
            Joy, '/joy', self.joy_cb, 10)

        # Publisher
        self.pub = self.create_publisher(
            AckermannDriveStamped,
            '/ackermann_cmd',
            10)

        # Gear indicator publisher
        self.pub_gear = self.create_publisher(
            Int32,
            '/current_gear',
            10
        )

        self.get_logger().info("ThrottleMix: Manual 4-Gear Transmission Loaded.")

    def joy_cb(self, msg):

        # ===== STEERING (LEFT STICK) =====
        steer = msg.axes[0] * 0.34

        # Buttons
        deadman = msg.buttons[4] == 1      # L1
        handbrake = msg.buttons[6] == 1    # L2

        # ===== Trigger normalization =====
        raw_gas = msg.axes[4]    # R2
        raw_brake = msg.axes[3]  # L2 #

        gas = (1.0 - raw_gas) / 2.0
        brake = (1.0 - raw_brake) / 2.0

        if gas < 0.05: gas = 0.0
        if brake < 0.05: brake = 0.0

        # ============================
        #    DRIVE / REVERSE TOGGLE
        # ============================
        current_shift = msg.buttons[5]  # R1

        if current_shift == 1 and self.prev_shift == 0:
            self.drive_mode = "REVERSE" if self.drive_mode == "DRIVE" else "DRIVE"
            self.get_logger().info(f">>> MODE: {self.drive_mode}")

        self.prev_shift = current_shift

        # ============================
        #         GEAR UP / DOWN
        # ============================
        gear_up = msg.buttons[3]    # Triangle
        gear_down = msg.buttons[1]  # X

        # ---- Gear Up ----
        if gear_up == 1 and self.prev_gear_up == 0:
            if self.current_gear < 3:
                self.current_gear += 1
                self.get_logger().info(f">>> GEAR UP → {self.current_gear + 1}")

        self.prev_gear_up = gear_up

        # ---- Gear Down ----
        if gear_down == 1 and self.prev_gear_down == 0:
            if self.current_gear > 0:
                self.current_gear -= 1
                self.get_logger().info(f">>> GEAR DOWN → {self.current_gear + 1}")

        self.prev_gear_down = gear_down

        # Publish gear indicator
        gear_msg = Int32()
        gear_msg.data = self.current_gear + 1   # Publish 1–4
        self.pub_gear.publish(gear_msg)

        # Get active gear parameters
        max_forward = self.gear_max_speeds[self.current_gear]
        accel_rate = self.gear_accel_rates[self.current_gear]
        max_reverse = 1.0 # 1.0
        brake_strength = 12.0

        # ============================
        #         HAND BRAKE
        # ============================
        if handbrake:
            out = AckermannDriveStamped()
            out.drive.speed = 0.0
            out.drive.steering_angle = steer
            self.prev_speed = 0.0
            self.pub.publish(out)
            return

        # ============================
        #          DEADMAN
        # ============================
        if not deadman:
            self.prev_speed = 0.0
            return
            out = AckermannDriveStamped()
            out.drive.speed = 0.0
            out.drive.steering_angle = steer
            self.prev_speed = 0.0
            self.pub.publish(out)
            return

        # ============================
        #       NO INPUT → STOP
        # ============================
        if gas == 0.0 and brake == 0.0:
            out = AckermannDriveStamped()
            out.drive.speed = 0.0
            out.drive.steering_angle = steer
            self.prev_speed = 0.0
            self.pub.publish(out)
            return

        # ============================
        #       NORMAL OPERATION
        # ============================
        out = AckermannDriveStamped()
        out.drive.steering_angle = steer

        if self.drive_mode == "DRIVE":
            speed = gas * max_forward - brake * brake_strength
            if speed < 0.0:
                speed = 0.0
        else:
            speed = -(gas * max_reverse) + brake * max_reverse
            if speed > 0.0:
                speed = 0.0

        # ============================
        #    ACCELERATION SMOOTHING
        # ============================
        speed = self.prev_speed + accel_rate * (speed - self.prev_speed)
        self.prev_speed = speed

        out.drive.speed = speed
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ThrottleMix()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
