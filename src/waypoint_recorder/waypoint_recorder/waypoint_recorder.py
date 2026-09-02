import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
import csv
import time

class WaypointRecorder(Node):
    def __init__(self):
        super().__init__('waypoint_recorder')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(0.1, self.record_point)

        self.last_x = None
        self.last_y = None

        self.file = open('waypoints.csv', 'w')
        self.writer = csv.writer(self.file)
        self.writer.writerow(['x', 'y'])

        self.get_logger().info("Waypoint recorder started")

    def record_point(self):
        try:
            t = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time()
            )

            x = t.transform.translation.x
            y = t.transform.translation.y

            # Downsample: only save if moved enough
            if self.last_x is not None:
                dist = ((x - self.last_x)**2 + (y - self.last_y)**2)**0.5
                if dist < 0.1:
                    return

            self.writer.writerow([x, y])
            self.last_x = x
            self.last_y = y

            self.get_logger().info(f"Saved: {x:.2f}, {y:.2f}")

        except Exception as e:
            pass


def main():
    rclpy.init()
    node = WaypointRecorder()
    rclpy.spin(node)

    node.file.close()
    rclpy.shutdown()


if __name__ == '__main__':
    main()