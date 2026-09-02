#!/usr/bin/env python3

import argparse
import csv
import math
from pathlib import Path

import matplotlib

# Save plots without opening a GUI window
matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np


RAW_DIR = Path("/home/arc/f1tenth_ws/waypoints/raw")
ENRICHED_DIR = Path("/home/arc/f1tenth_ws/waypoints/enriched_waypoints")
PLOTS_DIR = Path("/home/arc/f1tenth_ws/waypoints/plots")


def resolve_input_path(input_arg: str) -> Path:
    input_path = Path(input_arg)

    if input_path.is_absolute():
        return input_path

    return RAW_DIR / input_path


def make_default_output_path(input_path: Path) -> Path:
    return ENRICHED_DIR / f"{input_path.stem}_enriched.csv"


def make_default_plot_path(input_path: Path) -> Path:
    return PLOTS_DIR / f"{input_path.stem}_speed.png"


def load_xy(csv_path: Path):
    points = []

    with open(csv_path, "r", newline="") as f:
        reader = csv.DictReader(f)

        if reader.fieldnames is None:
            raise ValueError("CSV has no header row.")

        if "x" not in reader.fieldnames or "y" not in reader.fieldnames:
            raise ValueError("CSV must contain x and y columns.")

        for row in reader:
            points.append([float(row["x"]), float(row["y"])])

    if len(points) < 3:
        raise ValueError("Need at least 3 waypoints.")

    return np.array(points, dtype=float)


def moving_average_closed(points: np.ndarray, window_size: int):
    n = len(points)
    half_window = window_size // 2
    smoothed = np.zeros_like(points)

    for i in range(n):
        total = np.zeros(2)

        for offset in range(-half_window, half_window + 1):
            idx = (i + offset) % n
            total += points[idx]

        smoothed[i] = total / window_size

    return smoothed


def moving_average_open(points: np.ndarray, window_size: int):
    n = len(points)
    half_window = window_size // 2
    smoothed = np.zeros_like(points)

    for i in range(n):
        total = np.zeros(2)

        for offset in range(-half_window, half_window + 1):
            idx = i + offset

            if idx < 0:
                idx = 0
            elif idx >= n:
                idx = n - 1

            total += points[idx]

        smoothed[i] = total / window_size

    return smoothed


def smooth_points(points: np.ndarray, window_size: int, passes: int, closed_loop: bool):
    if window_size < 3:
        raise ValueError("smooth_window must be at least 3.")

    if window_size % 2 == 0:
        raise ValueError("smooth_window must be odd. Use 5, 7, 9, 11, etc.")

    if passes < 0:
        raise ValueError("smooth_passes must be 0 or greater.")

    smoothed = points.copy()

    if passes == 0:
        return smoothed

    smooth_function = moving_average_closed if closed_loop else moving_average_open

    for _ in range(passes):
        smoothed = smooth_function(smoothed, window_size)

    return smoothed


def compute_yaw(points: np.ndarray, closed_loop: bool):
    n = len(points)
    yaw = np.zeros(n)

    for i in range(n):
        if closed_loop:
            p1 = points[(i - 1) % n]
            p2 = points[(i + 1) % n]
        else:
            if i == 0:
                p1 = points[i]
                p2 = points[i + 1]
            elif i == n - 1:
                p1 = points[i - 1]
                p2 = points[i]
            else:
                p1 = points[i - 1]
                p2 = points[i + 1]

        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]

        yaw[i] = math.atan2(dy, dx)

    return yaw


def compute_curvature(points: np.ndarray, closed_loop: bool):
    n = len(points)
    curvature = np.zeros(n)

    index_range = range(n) if closed_loop else range(1, n - 1)

    for i in index_range:
        if closed_loop:
            p1 = points[(i - 1) % n]
            p2 = points[i]
            p3 = points[(i + 1) % n]
        else:
            p1 = points[i - 1]
            p2 = points[i]
            p3 = points[i + 1]

        a = np.linalg.norm(p2 - p1)
        b = np.linalg.norm(p3 - p2)
        c = np.linalg.norm(p3 - p1)

        if a < 1e-9 or b < 1e-9 or c < 1e-9:
            curvature[i] = 0.0
            continue

        area = abs(np.cross(p2 - p1, p3 - p1)) / 2.0

        if area < 1e-9:
            curvature[i] = 0.0
            continue

        curvature[i] = (4.0 * area) / (a * b * c)

    if not closed_loop:
        curvature[0] = curvature[1]
        curvature[-1] = curvature[-2]

    return curvature


def smooth_1d(values: np.ndarray, window_size: int):
    if window_size <= 1:
        return values

    if window_size % 2 == 0:
        raise ValueError("speed_smooth_window must be odd. Use 5, 7, 9, etc.")

    kernel = np.ones(window_size) / window_size
    pad = window_size // 2
    padded = np.pad(values, (pad, pad), mode="edge")

    return np.convolve(padded, kernel, mode="valid")[: len(values)]


def generate_speed_profile(
    curvature: np.ndarray,
    min_speed: float,
    max_speed: float,
    max_lateral_accel: float,
    lookahead_points: int,
    speed_smooth_window: int,
):
    speeds = np.zeros(len(curvature))

    for i, kappa in enumerate(curvature):
        if kappa < 1e-5:
            speed = max_speed
        else:
            speed = math.sqrt(max_lateral_accel / kappa)

        speeds[i] = max(min_speed, min(max_speed, speed))

    adjusted = speeds.copy()

    if lookahead_points > 0:
        for i in range(len(speeds)):
            future_end = min(i + lookahead_points, len(speeds))
            future_min = np.min(speeds[i:future_end])
            adjusted[i] = min(speeds[i], future_min)

    adjusted = smooth_1d(adjusted, speed_smooth_window)
    adjusted = np.clip(adjusted, min_speed, max_speed)

    return adjusted


def save_trajectory(
    csv_path: Path,
    points: np.ndarray,
    yaw: np.ndarray,
    curvature: np.ndarray,
    speed: np.ndarray,
):
    csv_path.parent.mkdir(parents=True, exist_ok=True)

    with open(csv_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["x", "y", "yaw", "curvature", "speed"])

        for p, yaw_i, curvature_i, speed_i in zip(points, yaw, curvature, speed):
            writer.writerow(
                [
                    round(float(p[0]), 6),
                    round(float(p[1]), 6),
                    round(float(yaw_i), 6),
                    round(float(curvature_i), 6),
                    round(float(speed_i), 3),
                ]
            )


def save_plot(
    plot_path: Path,
    points: np.ndarray,
    curvature: np.ndarray,
    speed: np.ndarray,
    color_by: str,
):
    plot_path.parent.mkdir(parents=True, exist_ok=True)

    if color_by == "curvature":
        color_data = curvature
        label = "Curvature"
    else:
        color_data = speed
        label = "Speed"

    plt.figure(figsize=(9, 7))
    scatter = plt.scatter(points[:, 0], points[:, 1], c=color_data, s=12)
    plt.plot(points[:, 0], points[:, 1], linewidth=0.8)
    plt.colorbar(scatter, label=label)
    plt.xlabel("x")
    plt.ylabel("y")
    plt.title(f"Enriched Trajectory Colored by {label}")
    plt.axis("equal")
    plt.grid(True)
    plt.savefig(plot_path, dpi=300, bbox_inches="tight")
    plt.close()


def main():
    parser = argparse.ArgumentParser(
        description="Generate enriched F1TENTH waypoints from raw x,y waypoints."
    )

    parser.add_argument(
        "--input",
        required=True,
        help=(
            "Input waypoint CSV filename inside /home/arc/f1tenth_ws/waypoints/raw, "
            "or a full absolute path."
        ),
    )

    parser.add_argument(
        "--output",
        default="",
        help=(
            "Optional output enriched CSV path. If not given, saves to "
            "/home/arc/f1tenth_ws/waypoints/enriched_waypoints/<input_name>_enriched.csv"
        ),
    )

    parser.add_argument(
        "--plot",
        default="",
        help=(
            "Optional output plot path. If not given, saves to "
            "/home/arc/f1tenth_ws/waypoints/plots/<input_name>_speed.png"
        ),
    )

    parser.add_argument("--smooth-window", type=int, default=9)
    parser.add_argument("--smooth-passes", type=int, default=2)
    parser.add_argument("--open-path", action="store_true")
    parser.add_argument("--min-speed", type=float, default=0.4)
    parser.add_argument("--max-speed", type=float, default=1.2)
    parser.add_argument("--max-lateral-accel", type=float, default=1.2)
    parser.add_argument("--lookahead-points", type=int, default=8)
    parser.add_argument("--speed-smooth-window", type=int, default=7)
    parser.add_argument(
        "--color-by",
        choices=["speed", "curvature"],
        default="speed",
    )

    args = parser.parse_args()

    input_path = resolve_input_path(args.input)
    output_path = Path(args.output) if args.output else make_default_output_path(input_path)
    plot_path = Path(args.plot) if args.plot else make_default_plot_path(input_path)

    closed_loop = not args.open_path

    if not input_path.exists():
        raise FileNotFoundError(f"Input waypoint file not found: {input_path}")

    raw_points = load_xy(input_path)

    smoothed_points = smooth_points(
        points=raw_points,
        window_size=args.smooth_window,
        passes=args.smooth_passes,
        closed_loop=closed_loop,
    )

    yaw = compute_yaw(smoothed_points, closed_loop=closed_loop)
    curvature = compute_curvature(smoothed_points, closed_loop=closed_loop)

    speed = generate_speed_profile(
        curvature=curvature,
        min_speed=args.min_speed,
        max_speed=args.max_speed,
        max_lateral_accel=args.max_lateral_accel,
        lookahead_points=args.lookahead_points,
        speed_smooth_window=args.speed_smooth_window,
    )

    save_trajectory(
        csv_path=output_path,
        points=smoothed_points,
        yaw=yaw,
        curvature=curvature,
        speed=speed,
    )

    save_plot(
        plot_path=plot_path,
        points=smoothed_points,
        curvature=curvature,
        speed=speed,
        color_by=args.color_by,
    )

    print("")
    print("Generated enriched waypoints successfully.")
    print(f"Input raw waypoints:      {input_path}")
    print(f"Output enriched CSV:      {output_path}")
    print(f"Output plot:              {plot_path}")
    print("")
    print("Output columns:")
    print("x,y,yaw,curvature,speed")
    print("")
    print("Settings used:")
    print(f"smooth_window:            {args.smooth_window}")
    print(f"smooth_passes:            {args.smooth_passes}")
    print(f"closed_loop:              {closed_loop}")
    print(f"min_speed:                {args.min_speed}")
    print(f"max_speed:                {args.max_speed}")
    print(f"max_lateral_accel:        {args.max_lateral_accel}")
    print(f"lookahead_points:         {args.lookahead_points}")
    print(f"speed_smooth_window:      {args.speed_smooth_window}")
    print("")


if __name__ == "__main__":
    main()