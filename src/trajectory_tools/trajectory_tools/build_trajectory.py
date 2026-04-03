#!/usr/bin/env python3

import argparse
import csv
import math
from pathlib import Path


def load_waypoints(csv_path: str):
    waypoints = []

    with open(csv_path, "r", newline="") as f:
        reader = csv.DictReader(f)

        if "x" not in reader.fieldnames or "y" not in reader.fieldnames:
            raise ValueError("Input CSV must contain 'x' and 'y' columns.")

        for row in reader:
            x = float(row["x"])
            y = float(row["y"])
            waypoints.append((x, y))

    if len(waypoints) < 3:
        raise ValueError("Need at least 3 waypoints to compute yaw and curvature.")

    return waypoints


def compute_yaw(waypoints):
    """
    Compute heading angle at each waypoint using neighboring points.
    """
    n = len(waypoints)
    yaw_list = []

    for i in range(n):
        if i == 0:
            x1, y1 = waypoints[i]
            x2, y2 = waypoints[i + 1]
        elif i == n - 1:
            x1, y1 = waypoints[i - 1]
            x2, y2 = waypoints[i]
        else:
            x1, y1 = waypoints[i - 1]
            x2, y2 = waypoints[i + 1]

        yaw = math.atan2(y2 - y1, x2 - x1)
        yaw_list.append(yaw)

    return yaw_list


def triangle_area(x1, y1, x2, y2, x3, y3):
    return abs(
        0.5 * (
            x1 * (y2 - y3) +
            x2 * (y3 - y1) +
            x3 * (y1 - y2)
        )
    )


def distance(p1, p2):
    return math.hypot(p2[0] - p1[0], p2[1] - p1[1])


def compute_curvature(waypoints):
    """
    Approximate curvature at each waypoint using 3-point geometry.
    Curvature = 1 / radius of circumcircle through 3 points.
    """
    n = len(waypoints)
    curvature_list = [0.0] * n

    for i in range(1, n - 1):
        p1 = waypoints[i - 1]
        p2 = waypoints[i]
        p3 = waypoints[i + 1]

        a = distance(p1, p2)
        b = distance(p2, p3)
        c = distance(p1, p3)

        area = triangle_area(p1[0], p1[1], p2[0], p2[1], p3[0], p3[1])

        # Prevent divide-by-zero for nearly collinear points
        if area < 1e-9 or a < 1e-9 or b < 1e-9 or c < 1e-9:
            curvature = 0.0
        else:
            curvature = (4.0 * area) / (a * b * c)

        curvature_list[i] = curvature

    # Copy neighbor values to endpoints
    curvature_list[0] = curvature_list[1]
    curvature_list[-1] = curvature_list[-2]

    return curvature_list


def compute_velocity_profile(curvature_list, v_max=4.0, v_min=1.0, k_gain=2.0):
    """
    Simple curvature-based speed profile.

    Lower curvature -> higher speed
    Higher curvature -> lower speed

    Formula:
        v = v_max / (1 + k_gain * curvature)

    Then clamp to [v_min, v_max]
    """
    velocity_list = []

    for curvature in curvature_list:
        v = v_max / (1.0 + k_gain * curvature)
        v = max(v_min, min(v, v_max))
        velocity_list.append(v)

    return velocity_list


def save_trajectory(csv_path, waypoints, yaw_list, curvature_list, velocity_list):
    fieldnames = ["x", "y", "yaw", "curvature", "velocity"]

    with open(csv_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()

        for (x, y), yaw, curvature, velocity in zip(
            waypoints, yaw_list, curvature_list, velocity_list
        ):
            writer.writerow({
                "x": x,
                "y": y,
                "yaw": yaw,
                "curvature": curvature,
                "velocity": velocity,
            })


def main():
    parser = argparse.ArgumentParser(description="Build enriched trajectory CSV from raw x,y waypoints.")
    parser.add_argument("--input", required=True, help="Path to raw waypoint CSV with columns x,y")
    parser.add_argument("--output", required=True, help="Path to output trajectory CSV")
    parser.add_argument("--v-max", type=float, default=4.0, help="Maximum velocity")
    parser.add_argument("--v-min", type=float, default=1.0, help="Minimum velocity")
    parser.add_argument("--k-gain", type=float, default=2.0, help="Curvature-to-speed scaling factor")

    args = parser.parse_args()

    input_path = Path(args.input)
    output_path = Path(args.output)

    if not input_path.exists():
        raise FileNotFoundError(f"Input file not found: {input_path}")

    waypoints = load_waypoints(str(input_path))
    yaw_list = compute_yaw(waypoints)
    curvature_list = compute_curvature(waypoints)
    velocity_list = compute_velocity_profile(
        curvature_list,
        v_max=args.v_max,
        v_min=args.v_min,
        k_gain=args.k_gain,
    )

    save_trajectory(
        str(output_path),
        waypoints,
        yaw_list,
        curvature_list,
        velocity_list,
    )

    print(f"Loaded {len(waypoints)} waypoints from: {input_path}")
    print(f"Saved enriched trajectory to: {output_path}")
    print("Columns: x, y, yaw, curvature, velocity")


if __name__ == "__main__":
    main()