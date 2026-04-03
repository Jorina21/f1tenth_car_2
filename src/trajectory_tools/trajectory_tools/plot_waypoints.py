#!/usr/bin/env python3

import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt


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

    if len(x_vals) == 0:
        raise ValueError("No waypoint data found in CSV.")

    return x_vals, y_vals


def main():
    parser = argparse.ArgumentParser(
        description="Plot raw waypoint CSV using x and y columns."
    )
    parser.add_argument(
        "--input",
        required=True,
        help="Path to waypoint CSV with columns x,y",
    )
    parser.add_argument(
        "--show-indices",
        action="store_true",
        help="Show waypoint index labels on the plot",
    )

    args = parser.parse_args()

    input_path = Path(args.input)
    if not input_path.exists():
        raise FileNotFoundError(f"Input file not found: {input_path}")

    x_vals, y_vals = load_waypoints(str(input_path))

    plt.figure(figsize=(8, 6))
    plt.scatter(x_vals, y_vals, s=10, label="Waypoints")
    plt.plot(x_vals, y_vals, linewidth=0.8, label="Path")

    if args.show_indices:
        for i, (x, y) in enumerate(zip(x_vals, y_vals)):
            plt.text(x, y, str(i), fontsize=6)

    plt.xlabel("x")
    plt.ylabel("y")
    plt.title("Raw Waypoints")
    plt.axis("equal")
    plt.grid(True)
    plt.legend()
    plt.show()


if __name__ == "__main__":
    main()