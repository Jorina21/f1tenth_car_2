#!/usr/bin/env python3

import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import yaml


def load_waypoints(csv_path: str):
    x_vals = []
    y_vals = []

    with open(csv_path, "r", newline="") as f:
        reader = csv.DictReader(f)

        if "x" not in reader.fieldnames or "y" not in reader.fieldnames:
            raise ValueError("Input CSV must contain 'x' and 'y' columns.")

        for row in reader:
            x_vals.append(float(row["x"]))
            y_vals.append(float(row["y"]))

    if not x_vals:
        raise ValueError("No waypoint data found in CSV.")

    return x_vals, y_vals


def load_map_metadata(yaml_path: str):
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


def world_to_pixel(x, y, origin_x, origin_y, resolution, image_height):
    px = (x - origin_x) / resolution
    py = image_height - ((y - origin_y) / resolution)
    return px, py


def main():
    parser = argparse.ArgumentParser(
        description="Overlay waypoint CSV on top of a ROS map image."
    )
    parser.add_argument(
        "--waypoints",
        required=True,
        help="Path to waypoint CSV with x,y columns",
    )
    parser.add_argument(
        "--map-yaml",
        required=True,
        help="Path to ROS map YAML file",
    )
    parser.add_argument(
        "--show-indices",
        action="store_true",
        help="Show waypoint indices",
    )

    args = parser.parse_args()

    waypoint_path = Path(args.waypoints)
    yaml_path = Path(args.map_yaml)

    if not waypoint_path.exists():
        raise FileNotFoundError(f"Waypoint CSV not found: {waypoint_path}")

    if not yaml_path.exists():
        raise FileNotFoundError(f"Map YAML not found: {yaml_path}")

    x_vals, y_vals = load_waypoints(str(waypoint_path))
    image_name, resolution, origin_x, origin_y = load_map_metadata(str(yaml_path))

    pgm_path = yaml_path.parent / image_name
    if not pgm_path.exists():
        raise FileNotFoundError(f"Map image not found: {pgm_path}")

    map_img = mpimg.imread(str(pgm_path))
    image_height = map_img.shape[0]

    pixel_x = []
    pixel_y = []

    for x, y in zip(x_vals, y_vals):
        px, py = world_to_pixel(
            x=x,
            y=y,
            origin_x=origin_x,
            origin_y=origin_y,
            resolution=resolution,
            image_height=image_height,
        )
        pixel_x.append(px)
        pixel_y.append(py)

    plt.figure(figsize=(10, 8))
    plt.imshow(map_img, cmap="gray", origin="upper")
    plt.plot(pixel_x, pixel_y, linewidth=1.0, label="Waypoints")
    plt.scatter(pixel_x, pixel_y, s=8, label="Points")

    if args.show_indices:
        for i, (px, py) in enumerate(zip(pixel_x, pixel_y)):
            plt.text(px, py, str(i), fontsize=6)

    plt.title("Waypoints Overlay on SLAM Map")
    plt.xlabel("Pixel X")
    plt.ylabel("Pixel Y")
    plt.legend()
    plt.grid(False)
    plt.show()


if __name__ == "__main__":
    main()