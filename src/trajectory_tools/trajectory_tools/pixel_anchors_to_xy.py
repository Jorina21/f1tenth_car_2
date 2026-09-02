#!/usr/bin/env python3

import os
import argparse
import yaml
import numpy as np
from scipy.interpolate import splprep, splev


def load_map_info(yaml_path):
    with open(yaml_path, "r") as f:
        map_info = yaml.safe_load(f)

    image_path = map_info["image"]

    if not os.path.isabs(image_path):
        image_path = os.path.join(os.path.dirname(yaml_path), image_path)

    resolution = float(map_info["resolution"])
    origin = map_info["origin"]

    try:
        import cv2
        img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)

        if img is None:
            raise RuntimeError(f"Could not load map image: {image_path}")

        map_height = img.shape[0]

    except Exception as e:
        raise RuntimeError(f"Could not read map image height from {image_path}: {e}")

    return resolution, origin, map_height, image_path


def pixel_to_world(pixel_x, pixel_y, map_height, resolution, origin):
    world_x = origin[0] + (pixel_x + 0.5) * resolution
    world_y = origin[1] + (map_height - pixel_y - 0.5) * resolution

    return world_x, world_y


def parse_anchor_string(anchor_string):
    anchors = []

    pairs = anchor_string.split(";")

    for pair in pairs:
        pair = pair.strip()

        if not pair:
            continue

        x_str, y_str = pair.split(",")

        pixel_x = float(x_str.strip())
        pixel_y = float(y_str.strip())

        anchors.append((pixel_x, pixel_y))

    if len(anchors) < 4:
        raise RuntimeError("Need at least 4 anchor points.")

    return anchors


def load_anchor_file(anchor_file):
    anchors = []

    with open(anchor_file, "r") as f:
        for line in f:
            line = line.strip()

            if not line:
                continue

            if line.startswith("#"):
                continue

            if "x" in line.lower() and "y" in line.lower():
                continue

            parts = line.split(",")

            if len(parts) < 2:
                continue

            pixel_x = float(parts[0].strip())
            pixel_y = float(parts[1].strip())

            anchors.append((pixel_x, pixel_y))

    if len(anchors) < 4:
        raise RuntimeError("Need at least 4 anchor points in anchor file.")

    return anchors


def smooth_closed_world_path(world_x, world_y, spacing, smoothing):
    """
    Race-day safe version:
    This does linear interpolation between anchor points instead of spline smoothing.
    It prevents the path from cutting across the inside of the track.
    """

    world_x = np.asarray(world_x)
    world_y = np.asarray(world_y)

    # Make sure path closes by repeating first point at the end if needed
    if np.hypot(world_x[0] - world_x[-1], world_y[0] - world_y[-1]) > 1e-6:
        world_x = np.append(world_x, world_x[0])
        world_y = np.append(world_y, world_y[0])

    out_x = []
    out_y = []

    for i in range(len(world_x) - 1):
        x0 = world_x[i]
        y0 = world_y[i]
        x1 = world_x[i + 1]
        y1 = world_y[i + 1]

        segment_length = np.hypot(x1 - x0, y1 - y0)

        num_points = max(2, int(segment_length / spacing))

        for j in range(num_points):
            t = j / float(num_points)

            x = (1.0 - t) * x0 + t * x1
            y = (1.0 - t) * y0 + t * y1

            out_x.append(x)
            out_y.append(y)

    return np.asarray(out_x), np.asarray(out_y)


def save_xy(output_path, x_values, y_values, header):
    output_dir = os.path.dirname(output_path)

    if output_dir:
        os.makedirs(output_dir, exist_ok=True)

    data = np.column_stack([x_values, y_values])

    if header:
        np.savetxt(
            output_path,
            data,
            delimiter=",",
            fmt="%.6f",
            header="x,y",
            comments=""
        )
    else:
        np.savetxt(
            output_path,
            data,
            delimiter=",",
            fmt="%.6f"
        )


def main():
    parser = argparse.ArgumentParser(
        description="Generate x,y waypoints from manually selected pixel anchor points on a ROS map."
    )

    parser.add_argument(
        "--map-yaml",
        required=True,
        help="Path to ROS map YAML file."
    )

    parser.add_argument(
        "--out",
        required=True,
        help="Output CSV path."
    )

    parser.add_argument(
        "--anchors",
        default=None,
        help='Anchor points as "x1,y1;x2,y2;x3,y3;..." in pixel coordinates.'
    )

    parser.add_argument(
        "--anchor-file",
        default=None,
        help="CSV file with pixel anchor points. Format: pixel_x,pixel_y"
    )

    parser.add_argument(
        "--spacing",
        type=float,
        default=0.08,
        help="Meters between generated waypoints."
    )

    parser.add_argument(
        "--smoothing",
        type=float,
        default=1.0,
        help="Spline smoothing. Higher = smoother but may cut corners more."
    )

    parser.add_argument(
        "--reverse",
        action="store_true",
        help="Reverse output waypoint order."
    )

    parser.add_argument(
        "--header",
        action="store_true",
        help="Write x,y header."
    )

    args = parser.parse_args()

    if args.anchors is None and args.anchor_file is None:
        raise RuntimeError("Provide either --anchors or --anchor-file.")

    resolution, origin, map_height, image_path = load_map_info(args.map_yaml)

    if args.anchor_file is not None:
        anchors = load_anchor_file(args.anchor_file)
    else:
        anchors = parse_anchor_string(args.anchors)

    print()
    print(f"Loaded map image: {image_path}")
    print(f"Resolution: {resolution} m/pixel")
    print(f"Origin: {origin}")
    print(f"Map height: {map_height} pixels")
    print(f"Anchor count: {len(anchors)}")
    print(f"Spacing: {args.spacing}")
    print(f"Smoothing: {args.smoothing}")
    print()

    world_x = []
    world_y = []

    for pixel_x, pixel_y in anchors:
        x, y = pixel_to_world(
            pixel_x,
            pixel_y,
            map_height,
            resolution,
            origin
        )

        world_x.append(x)
        world_y.append(y)

    x_values, y_values = smooth_closed_world_path(
        world_x,
        world_y,
        spacing=args.spacing,
        smoothing=args.smoothing
    )

    if args.reverse:
        x_values = x_values[::-1]
        y_values = y_values[::-1]

    save_xy(args.out, x_values, y_values, args.header)

    print("Done.")
    print(f"Saved x,y CSV: {args.out}")
    print(f"Generated points: {len(x_values)}")
    print("CSV format: x,y")


if __name__ == "__main__":
    main()
