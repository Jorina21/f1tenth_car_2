#!/usr/bin/env python3

import argparse
import csv
from pathlib import Path


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

    if len(x_vals) < 3:
        raise ValueError("Need at least 3 waypoints to smooth.")

    return x_vals, y_vals


def moving_average_closed(data, window_size):
    """
    Smooth a closed-loop path using circular indexing.
    """
    n = len(data)
    half_window = window_size // 2
    smoothed = []

    for i in range(n):
        total = 0.0
        count = 0

        for offset in range(-half_window, half_window + 1):
            idx = (i + offset) % n
            total += data[idx]
            count += 1

        smoothed.append(total / count)

    return smoothed


def moving_average_open(data, window_size):
    """
    Smooth an open path using edge clamping.
    """
    n = len(data)
    half_window = window_size // 2
    smoothed = []

    for i in range(n):
        total = 0.0
        count = 0

        for offset in range(-half_window, half_window + 1):
            idx = i + offset
            if idx < 0:
                idx = 0
            elif idx >= n:
                idx = n - 1

            total += data[idx]
            count += 1

        smoothed.append(total / count)

    return smoothed


def smooth_waypoints(x_vals, y_vals, window_size=9, closed_loop=True, passes=1):
    if window_size < 3:
        raise ValueError("window_size must be at least 3.")
    if window_size % 2 == 0:
        raise ValueError("window_size must be odd.")
    if passes < 1:
        raise ValueError("passes must be at least 1.")

    smooth_fn = moving_average_closed if closed_loop else moving_average_open

    x_smooth = x_vals[:]
    y_smooth = y_vals[:]

    for _ in range(passes):
        x_smooth = smooth_fn(x_smooth, window_size)
        y_smooth = smooth_fn(y_smooth, window_size)

    return x_smooth, y_smooth


def save_waypoints(csv_path: str, x_vals, y_vals):
    output_path = Path(csv_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    with open(output_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=["x", "y"])
        writer.writeheader()

        for x, y in zip(x_vals, y_vals):
            writer.writerow({"x": x, "y": y})


def main():
    parser = argparse.ArgumentParser(
        description="Smooth raw waypoint CSV using moving average filtering."
    )
    parser.add_argument(
        "--input",
        required=True,
        help="Path to input waypoint CSV with x,y columns",
    )
    parser.add_argument(
        "--output",
        required=True,
        help="Path to output smoothed waypoint CSV",
    )
    parser.add_argument(
        "--window-size",
        type=int,
        default=9,
        help="Odd smoothing window size (default: 9)",
    )
    parser.add_argument(
        "--passes",
        type=int,
        default=2,
        help="Number of smoothing passes (default: 2)",
    )
    parser.add_argument(
        "--open-path",
        action="store_true",
        help="Treat path as open instead of closed loop",
    )

    args = parser.parse_args()

    input_path = Path(args.input)
    if not input_path.exists():
        raise FileNotFoundError(f"Input file not found: {input_path}")

    x_vals, y_vals = load_waypoints(str(input_path))

    x_smooth, y_smooth = smooth_waypoints(
        x_vals=x_vals,
        y_vals=y_vals,
        window_size=args.window_size,
        closed_loop=not args.open_path,
        passes=args.passes,
    )

    save_waypoints(args.output, x_smooth, y_smooth)

    print(f"Loaded {len(x_vals)} raw waypoints from: {input_path}")
    print(f"Saved smoothed waypoints to: {args.output}")
    print(f"window_size={args.window_size}, passes={args.passes}, closed_loop={not args.open_path}")


if __name__ == "__main__":
    main()