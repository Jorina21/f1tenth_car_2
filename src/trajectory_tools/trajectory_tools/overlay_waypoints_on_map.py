#!/usr/bin/env python3

import csv
from pathlib import Path

import matplotlib

# Save plots without opening a GUI window
matplotlib.use("Agg")

import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import yaml

import rclpy
from rclpy.node import Node


class OverlayWaypointsOnMap(Node):
    def __init__(self):
        super().__init__("overlay_waypoints_on_map")

        self.declare_parameter("waypoints", "")
        self.declare_parameter("map_yaml", "")
        self.declare_parameter("show_indices", False)
        self.declare_parameter(
            "output_dir",
            "/home/arc/f1tenth_ws/waypoints/plots",
        )

        self.waypoints_path = Path(
            self.get_parameter("waypoints").get_parameter_value().string_value
        )

        self.map_yaml_path = Path(
            self.get_parameter("map_yaml").get_parameter_value().string_value
        )

        self.show_indices = (
            self.get_parameter("show_indices").get_parameter_value().bool_value
        )

        self.output_dir = Path(
            self.get_parameter("output_dir").get_parameter_value().string_value
        )

        self.run_overlay()

    def load_waypoints(self, csv_path: Path):
        x_vals = []
        y_vals = []

        with open(csv_path, "r", newline="") as f:
            reader = csv.DictReader(f)

            if reader.fieldnames is None:
                raise ValueError("Waypoint CSV has no header row.")

            if "x" not in reader.fieldnames or "y" not in reader.fieldnames:
                raise ValueError("Input CSV must contain 'x' and 'y' columns.")

            for row in reader:
                x_vals.append(float(row["x"]))
                y_vals.append(float(row["y"]))

        if not x_vals:
            raise ValueError("No waypoint data found in CSV.")

        return x_vals, y_vals

    def load_map_metadata(self, yaml_path: Path):
        with open(yaml_path, "r") as f:
            map_info = yaml.safe_load(f)

        required_keys = ["image", "resolution", "origin"]

        for key in required_keys:
            if key not in map_info:
                raise ValueError(f"Map YAML missing required key: {key}")

        image_name = map_info["image"]
        resolution = float(map_info["resolution"])
        origin = map_info["origin"]

        if len(origin) < 2:
            raise ValueError("Map YAML origin must contain at least [x, y].")

        return image_name, resolution, float(origin[0]), float(origin[1])

    def world_to_pixel(self, x, y, origin_x, origin_y, resolution, image_height):
        px = (x - origin_x) / resolution
        py = image_height - ((y - origin_y) / resolution)
        return px, py

    def run_overlay(self):
        if str(self.waypoints_path) == "":
            raise ValueError("ROS parameter 'waypoints' is required.")

        if str(self.map_yaml_path) == "":
            raise ValueError("ROS parameter 'map_yaml' is required.")

        if not self.waypoints_path.exists():
            raise FileNotFoundError(f"Waypoint CSV not found: {self.waypoints_path}")

        if not self.map_yaml_path.exists():
            raise FileNotFoundError(f"Map YAML not found: {self.map_yaml_path}")

        self.get_logger().info(f"Loading waypoints: {self.waypoints_path}")
        self.get_logger().info(f"Loading map yaml: {self.map_yaml_path}")

        x_vals, y_vals = self.load_waypoints(self.waypoints_path)

        image_name, resolution, origin_x, origin_y = self.load_map_metadata(
            self.map_yaml_path
        )

        map_image_path = self.map_yaml_path.parent / image_name

        if not map_image_path.exists():
            raise FileNotFoundError(f"Map image not found: {map_image_path}")

        self.get_logger().info(f"Loading map image: {map_image_path}")

        map_img = mpimg.imread(str(map_image_path))
        image_height = map_img.shape[0]

        pixel_x = []
        pixel_y = []

        for x, y in zip(x_vals, y_vals):
            px, py = self.world_to_pixel(
                x=x,
                y=y,
                origin_x=origin_x,
                origin_y=origin_y,
                resolution=resolution,
                image_height=image_height,
            )

            pixel_x.append(px)
            pixel_y.append(py)

        self.output_dir.mkdir(parents=True, exist_ok=True)

        output_file = self.output_dir / f"{self.waypoints_path.stem}_overlay.png"

        plt.figure(figsize=(10, 8))
        plt.imshow(map_img, cmap="gray", origin="upper")
        plt.plot(pixel_x, pixel_y, linewidth=1.0, label="Waypoints")
        plt.scatter(pixel_x, pixel_y, s=8, label="Points")

        if self.show_indices:
            for i, (px, py) in enumerate(zip(pixel_x, pixel_y)):
                plt.text(px, py, str(i), fontsize=6)

        plt.title("Waypoints Overlay on SLAM Map")
        plt.xlabel("Pixel X")
        plt.ylabel("Pixel Y")
        plt.legend()
        plt.grid(False)

        plt.savefig(output_file, dpi=300, bbox_inches="tight")
        plt.close()

        self.get_logger().info(f"Saved overlay plot to: {output_file}")


def main(args=None):
    rclpy.init(args=args)

    node = None

    try:
        node = OverlayWaypointsOnMap()
    except Exception as e:
        print(f"[overlay_waypoints_on_map] ERROR: {e}")
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()