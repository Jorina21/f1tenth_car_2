#!/usr/bin/env python3

import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt


def load_trajectory(csv_path: str):
    x_vals = []
    y_vals = []
    curvature_vals = []
    velocity_vals = []

    with open(csv_path, "r", newline="") as f:
        reader = csv.DictReader(f)

        required = {"x", "y", "curvature", "velocity"}
        if not required.issubset(set(reader.fieldnames)):
            raise ValueError("CSV must contain x, y, curvature, velocity columns.")

        for row in reader:
            x_vals.append(float(row["x"]))
            y_vals.append(float(row["y"]))
            curvature_vals.append(float(row["curvature"]))
            velocity_vals.append(float(row["velocity"]))

    return x_vals, y_vals, curvature_vals, velocity_vals


def main():
    parser = argparse.ArgumentParser(description="Plot trajectory colored by velocity or curvature.")
    parser.add_argument("--input", required=True, help="Path to enriched trajectory CSV")
    parser.add_argument(
        "--color-by",
        choices=["velocity", "curvature"],
        default="velocity",
        help="Which field to color the path by",
    )
    args = parser.parse_args()

    input_path = Path(args.input)
    if not input_path.exists():
        raise FileNotFoundError(f"Input file not found: {input_path}")

    x_vals, y_vals, curvature_vals, velocity_vals = load_trajectory(str(input_path))

    if args.color_by == "velocity":
        color_data = velocity_vals
        label = "Velocity"
    else:
        color_data = curvature_vals
        label = "Curvature"

    plt.figure(figsize=(8, 6))
    scatter = plt.scatter(x_vals, y_vals, c=color_data, s=10)
    plt.plot(x_vals, y_vals, linewidth=0.8)
    plt.colorbar(scatter, label=label)
    plt.xlabel("x")
    plt.ylabel("y")
    plt.title(f"Trajectory colored by {label.lower()}")
    plt.axis("equal")
    plt.grid(True)
    plt.show()


if __name__ == "__main__":
    main()