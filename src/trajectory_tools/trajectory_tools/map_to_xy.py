#!/usr/bin/env python3

import os
import argparse
import yaml
import cv2
import numpy as np

from scipy.ndimage import distance_transform_edt, label
from scipy.interpolate import splprep, splev
from skimage.morphology import skeletonize


def load_map(yaml_path):
    with open(yaml_path, "r") as f:
        map_info = yaml.safe_load(f)

    image_path = map_info["image"]

    if not os.path.isabs(image_path):
        image_path = os.path.join(os.path.dirname(yaml_path), image_path)

    img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)

    if img is None:
        raise RuntimeError(f"Could not load map image: {image_path}")

    resolution = float(map_info["resolution"])
    origin = map_info["origin"]
    negate = int(map_info.get("negate", 0))
    free_thresh = float(map_info.get("free_thresh", 0.25))

    if negate:
        img = 255 - img

    return img, resolution, origin, image_path, free_thresh


def pixel_to_world(row, col, map_height, resolution, origin):
    x = origin[0] + (col + 0.5) * resolution
    y = origin[1] + (map_height - row - 0.5) * resolution
    return x, y


def flood_fill_from_seed(free_mask, seed_x, seed_y):
    height, width = free_mask.shape

    seed_col = int(seed_x)
    seed_row = int(seed_y)

    if seed_row < 0 or seed_row >= height or seed_col < 0 or seed_col >= width:
        raise RuntimeError("Seed point is outside the image.")

    if not free_mask[seed_row, seed_col]:
        raise RuntimeError(
            "Seed point is not in free space. Pick a white pixel inside the actual track lane."
        )

    flood_input = free_mask.astype(np.uint8) * 255
    mask = np.zeros((height + 2, width + 2), dtype=np.uint8)
    filled = flood_input.copy()

    cv2.floodFill(
        image=filled,
        mask=mask,
        seedPoint=(seed_col, seed_row),
        newVal=128,
        loDiff=0,
        upDiff=0,
        flags=4
    )

    selected = filled == 128
    return selected


def connected_label_8(mask):
    structure = np.ones((3, 3), dtype=np.uint8)
    labeled, num_features = label(mask, structure=structure)
    return labeled, num_features


def keep_largest_component(mask):
    labeled, num_features = connected_label_8(mask)

    if num_features == 0:
        raise RuntimeError("No connected component found.")

    sizes = np.bincount(labeled.ravel())
    sizes[0] = 0

    largest_label = np.argmax(sizes)

    return labeled == largest_label


def count_neighbors(mask):
    kernel = np.ones((3, 3), dtype=np.uint8)
    kernel[1, 1] = 0
    return cv2.filter2D(mask.astype(np.uint8), -1, kernel)


def get_neighbors(point, point_set):
    row, col = point

    neighbors_8 = [
        (-1, -1), (-1, 0), (-1, 1),
        (0, -1),           (0, 1),
        (1, -1),  (1, 0),  (1, 1),
    ]

    out = []

    for dr, dc in neighbors_8:
        candidate = (row + dr, col + dc)

        if candidate in point_set:
            out.append(candidate)

    return out


def remove_small_skeleton_spurs(skeleton, iterations):
    skeleton = skeleton.copy()

    if iterations <= 0:
        return skeleton

    for _ in range(iterations):
        degree = count_neighbors(skeleton)
        endpoints = skeleton & (degree <= 1)

        if not endpoints.any():
            break

        skeleton[endpoints] = False

    return skeleton


def peel_to_loop_core(skeleton, max_iterations=5000):
    """
    Repeatedly remove endpoints until only the loop core remains.
    This is the key fix for your current map: it removes the bad branches/spurs
    that cause shortcuts through the middle.
    """
    loop_core = skeleton.copy()

    for _ in range(max_iterations):
        degree = count_neighbors(loop_core)
        endpoints = loop_core & (degree <= 1)

        if not endpoints.any():
            break

        loop_core[endpoints] = False

    return loop_core


def choose_start_point(skeleton, seed_x, seed_y):
    points = np.argwhere(skeleton)

    if len(points) == 0:
        raise RuntimeError("No skeleton points found.")

    seed = np.array([seed_y, seed_x])
    distances = np.linalg.norm(points - seed, axis=1)
    index = np.argmin(distances)

    return tuple(points[index])


def trace_closed_loop(skeleton, seed_x, seed_y):
    points = np.argwhere(skeleton)

    if len(points) < 30:
        raise RuntimeError("Loop skeleton is too small.")

    point_set = set(map(tuple, points))
    start = choose_start_point(skeleton, seed_x, seed_y)

    neighbors = get_neighbors(start, point_set)

    if len(neighbors) == 0:
        raise RuntimeError("Start point has no neighbors.")

    path = [start]
    previous = None
    current = start

    for _ in range(len(points) + 5000):
        current_neighbors = get_neighbors(current, point_set)

        if previous is None:
            candidates = current_neighbors
        else:
            candidates = [n for n in current_neighbors if n != previous]

        if not candidates:
            break

        if start in candidates and len(path) > 30:
            break

        if previous is not None and len(candidates) > 1:
            previous_vector = np.array(current) - np.array(previous)

            def straightness_score(candidate):
                new_vector = np.array(candidate) - np.array(current)
                denom = np.linalg.norm(previous_vector) * np.linalg.norm(new_vector)

                if denom < 1e-9:
                    return -999.0

                return np.dot(previous_vector, new_vector) / denom

            candidates.sort(key=straightness_score, reverse=True)

        next_point = candidates[0]

        path.append(next_point)
        previous = current
        current = next_point

    return np.array(path)


def smooth_closed_xy(x_values, y_values, spacing, smoothing):
    x_values = np.asarray(x_values)
    y_values = np.asarray(y_values)

    keep = np.ones(len(x_values), dtype=bool)
    keep[1:] = np.hypot(np.diff(x_values), np.diff(y_values)) > 1e-4

    x_values = x_values[keep]
    y_values = y_values[keep]

    if len(x_values) < 20:
        raise RuntimeError("Not enough points to smooth path.")

    tck, _ = splprep([x_values, y_values], s=smoothing, per=True)

    dx = np.diff(x_values, append=x_values[0])
    dy = np.diff(y_values, append=y_values[0])
    total_length = np.sum(np.hypot(dx, dy))

    num_points = max(50, int(total_length / spacing))
    u = np.linspace(0.0, 1.0, num_points, endpoint=False)

    smooth_x, smooth_y = splev(u, tck)

    return np.asarray(smooth_x), np.asarray(smooth_y)


def save_xy_csv(output_path, x_values, y_values, include_header):
    output_dir = os.path.dirname(output_path)

    if output_dir:
        os.makedirs(output_dir, exist_ok=True)

    data = np.column_stack([x_values, y_values])

    if include_header:
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
        description="Generate x,y centerline waypoints from a ROS map using seed-based lane selection."
    )

    parser.add_argument(
        "--map-yaml",
        required=True,
        help="Path to ROS map YAML file"
    )

    parser.add_argument(
        "--out",
        required=True,
        help="Output CSV path for x,y points"
    )

    parser.add_argument(
        "--seed-x",
        type=int,
        required=True,
        help="Pixel X inside the actual drivable lane"
    )

    parser.add_argument(
        "--seed-y",
        type=int,
        required=True,
        help="Pixel Y inside the actual drivable lane"
    )

    parser.add_argument(
        "--free-threshold",
        type=int,
        default=250,
        help="Pixel value considered free space. Use 250 for white map lanes."
    )

    parser.add_argument(
        "--wall-margin",
        type=float,
        default=0.05,
        help="Meters to stay away from walls before extracting centerline"
    )

    parser.add_argument(
        "--spacing",
        type=float,
        default=0.08,
        help="Meters between generated x,y points"
    )

    parser.add_argument(
        "--smoothing",
        type=float,
        default=0.20,
        help="Spline smoothing amount. Higher = smoother path"
    )

    parser.add_argument(
        "--spur-prune-iterations",
        type=int,
        default=0,
        help="Optional light spur pruning before loop-core extraction"
    )

    parser.add_argument(
        "--reverse",
        action="store_true",
        help="Reverse waypoint order"
    )

    parser.add_argument(
        "--header",
        action="store_true",
        help="Add x,y header to CSV"
    )

    args = parser.parse_args()

    img, resolution, origin, image_path, yaml_free_thresh = load_map(args.map_yaml)

    map_height = img.shape[0]

    print()
    print(f"Loaded map image: {image_path}")
    print(f"Map resolution: {resolution} m/pixel")
    print(f"Map origin: {origin}")
    print(f"Free threshold: {args.free_threshold}")
    print(f"Wall margin: {args.wall_margin} m")
    print(f"Wall margin pixels: {args.wall_margin / resolution:.2f}")
    print(f"Spacing: {args.spacing} m")
    print(f"Smoothing: {args.smoothing}")
    print(f"Seed pixel: x={args.seed_x}, y={args.seed_y}")
    print()

    free_space = img >= args.free_threshold

    selected_lane = flood_fill_from_seed(
        free_space,
        args.seed_x,
        args.seed_y
    )

    selected_pixels = int(np.count_nonzero(selected_lane))
    print(f"Selected lane pixels: {selected_pixels}")

    if selected_pixels < 100:
        raise RuntimeError("Selected lane region too small. Pick a better seed point.")

    distance_to_wall_pixels = distance_transform_edt(selected_lane)
    wall_margin_pixels = args.wall_margin / resolution

    safe_lane = selected_lane & (distance_to_wall_pixels > wall_margin_pixels)

    safe_pixels = int(np.count_nonzero(safe_lane))
    print(f"Safe lane pixels: {safe_pixels}")

    if safe_pixels < 100:
        raise RuntimeError("Safe lane too small. Lower --wall-margin.")

    skeleton = skeletonize(safe_lane)

    skeleton_pixels = int(np.count_nonzero(skeleton))
    print(f"Skeleton pixels before cleanup: {skeleton_pixels}")

    if skeleton_pixels < 30:
        raise RuntimeError("Skeleton too small. Lower --wall-margin.")

    if args.spur_prune_iterations > 0:
        skeleton = remove_small_skeleton_spurs(skeleton, args.spur_prune_iterations)

    skeleton = keep_largest_component(skeleton)

    skeleton_pixels = int(np.count_nonzero(skeleton))
    print(f"Skeleton pixels after largest component: {skeleton_pixels}")

    loop_core = peel_to_loop_core(skeleton)

    loop_core = keep_largest_component(loop_core)

    loop_core_pixels = int(np.count_nonzero(loop_core))
    print(f"Loop-core pixels after peeling: {loop_core_pixels}")

    if loop_core_pixels < 30:
        raise RuntimeError(
            "Loop core is too small. Try a different seed point or slightly lower wall margin."
        )

    path_pixels = trace_closed_loop(
        loop_core,
        args.seed_x,
        args.seed_y
    )

    print(f"Traced loop pixels: {len(path_pixels)}")

    if len(path_pixels) < 30:
        raise RuntimeError("Traced loop is too small.")

    raw_x = []
    raw_y = []

    for row, col in path_pixels:
        x, y = pixel_to_world(row, col, map_height, resolution, origin)
        raw_x.append(x)
        raw_y.append(y)

    x_values, y_values = smooth_closed_xy(
        raw_x,
        raw_y,
        spacing=args.spacing,
        smoothing=args.smoothing
    )

    if args.reverse:
        x_values = x_values[::-1]
        y_values = y_values[::-1]

    save_xy_csv(args.out, x_values, y_values, args.header)

    print()
    print("Done.")
    print(f"Saved x,y waypoints to: {args.out}")
    print(f"Generated points: {len(x_values)}")
    print()
    print("CSV format:")
    print("x,y")


if __name__ == "__main__":
    main()