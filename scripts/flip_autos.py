#!/usr/bin/env python3
"""
Radially flips a single PathPlanner auto and its referenced paths around the
center of the field.

The 2026 REBUILT field has 180-degree rotational symmetry. This script generates
the opposite-alliance equivalent by rotating all coordinates 180 degrees around
the field center.

Flip math:
  new_x = field_length - old_x
  new_y = field_width  - old_y
  new_rotation = rotation + 180 (wrapped to [-180, 180])

Output files are prefixed with "FLIPPED " and have alliance tags swapped
(Red<->Blue, [R]<->[B]) so they're easy to identify as generated.

Usage:
  python scripts/flip_autos.py "Red middle sprint v2"         # flip auto + its paths
  python scripts/flip_autos.py --dry-run "[R] Depot"           # preview without writing

No external dependencies required - uses only the Python standard library.
"""

import argparse
import copy
import json
import os
import sys
from pathlib import Path


# --- Field dimensions (meters) for 2026 REBUILT ---
# Read from navgrid.json if available, otherwise use these defaults.
DEFAULT_FIELD_LENGTH = 16.54
DEFAULT_FIELD_WIDTH = 8.07


def find_deploy_dir():
    """Locate the pathplanner deploy directory relative to the script or cwd."""
    candidates = [
        Path(__file__).resolve().parent.parent / "src" / "main" / "deploy" / "pathplanner",
        Path.cwd() / "src" / "main" / "deploy" / "pathplanner",
    ]
    for candidate in candidates:
        if candidate.is_dir():
            return candidate
    print("ERROR: Could not find src/main/deploy/pathplanner directory.")
    print("Run this script from the project root or the scripts/ folder.")
    sys.exit(1)


def read_field_dimensions(deploy_dir):
    """Read field dimensions from navgrid.json, falling back to defaults."""
    navgrid_path = deploy_dir / "navgrid.json"
    if navgrid_path.exists():
        with open(navgrid_path, "r") as f:
            navgrid = json.load(f)
        field_size = navgrid.get("field_size", {})
        length = field_size.get("x", DEFAULT_FIELD_LENGTH)
        width = field_size.get("y", DEFAULT_FIELD_WIDTH)
        return length, width
    return DEFAULT_FIELD_LENGTH, DEFAULT_FIELD_WIDTH


FLIPPED_PREFIX = "FLIPPED "


# --- Name flipping ---

def swap_alliance_in_name(name):
    """Swap alliance tags in a name. Red->Blue and Blue->Red."""
    name = name.replace("[R]", "<<R>>").replace("[B]", "<<B>>")
    name = name.replace("Red", "<<RED>>").replace("Blue", "<<BLUE>>")
    name = name.replace("red", "<<red>>").replace("blue", "<<blue>>")
    name = name.replace("<<R>>", "[B]").replace("<<B>>", "[R]")
    name = name.replace("<<RED>>", "Blue").replace("<<BLUE>>", "Red")
    name = name.replace("<<red>>", "blue").replace("<<blue>>", "red")
    return name


def make_flipped_name(name):
    """Add 'FLIPPED ' prefix and swap alliance tags."""
    if name.startswith(FLIPPED_PREFIX):
        # Strip prefix, swap back, then re-prefix with the new swap
        inner = name[len(FLIPPED_PREFIX):]
        return FLIPPED_PREFIX + swap_alliance_in_name(inner)
    return FLIPPED_PREFIX + swap_alliance_in_name(name)


# --- Coordinate flipping ---

def wrap_rotation(degrees):
    """Wrap a rotation in degrees to the (-180, 180] range."""
    while degrees > 180:
        degrees -= 360
    while degrees <= -180:
        degrees += 360
    return round(degrees, 4)


def flip_point(x, y, field_length, field_width):
    """Rotate a point 180 degrees around the field center."""
    return round(field_length - x, 6), round(field_width - y, 6)


def flip_rotation(degrees):
    """Rotate heading by 180 degrees."""
    return wrap_rotation(degrees + 180)


# --- Path flipping ---

def flip_path_data(path_data, field_length, field_width):
    """Flip all coordinates and rotations in a PathPlanner .path file."""
    data = copy.deepcopy(path_data)

    for wp in data.get("waypoints", []):
        for key in ("anchor", "prevControl", "nextControl"):
            point = wp.get(key)
            if point is not None:
                point["x"], point["y"] = flip_point(
                    point["x"], point["y"], field_length, field_width
                )

    for rt in data.get("rotationTargets", []):
        if "rotationDegrees" in rt:
            rt["rotationDegrees"] = flip_rotation(rt["rotationDegrees"])

    for ptz in data.get("pointTowardsZones", []):
        pos = ptz.get("fieldPosition")
        if pos is not None:
            pos["x"], pos["y"] = flip_point(
                pos["x"], pos["y"], field_length, field_width
            )

    goal = data.get("goalEndState")
    if goal and "rotation" in goal:
        goal["rotation"] = flip_rotation(goal["rotation"])

    start = data.get("idealStartingState")
    if start and "rotation" in start:
        start["rotation"] = flip_rotation(start["rotation"])

    return data


# --- Auto flipping ---

def flip_auto_command(command, flip_name_fn):
    """Recursively update path name references in an auto command tree."""
    cmd = copy.deepcopy(command)

    if cmd.get("type") == "path":
        path_name = cmd.get("data", {}).get("pathName")
        if path_name:
            cmd["data"]["pathName"] = flip_name_fn(path_name)

    nested = cmd.get("data", {}).get("commands")
    if nested:
        cmd["data"]["commands"] = [
            flip_auto_command(c, flip_name_fn) for c in nested
        ]

    return cmd


def flip_auto_data(auto_data, flip_name_fn):
    """Flip a PathPlanner .auto file by updating all path references."""
    data = copy.deepcopy(auto_data)
    if data.get("command"):
        data["command"] = flip_auto_command(data["command"], flip_name_fn)
    return data


def collect_path_names_from_command(command):
    """Recursively collect all path names referenced in an auto command tree."""
    names = []
    if command.get("type") == "path":
        name = command.get("data", {}).get("pathName")
        if name:
            names.append(name)
    for child in command.get("data", {}).get("commands", []):
        names.extend(collect_path_names_from_command(child))
    return names


# --- File I/O ---

def read_json(file_path):
    with open(file_path, "r") as f:
        return json.load(f)


def write_json(file_path, data, dry_run=False):
    if dry_run:
        print(f"  [DRY RUN] Would write: {file_path}")
        return
    os.makedirs(os.path.dirname(file_path), exist_ok=True)
    with open(file_path, "w", newline="\n") as f:
        json.dump(data, f, indent=2)
        f.write("\n")
    print(f"  Wrote: {file_path}")


# --- Main logic ---

def flip_single_path(paths_dir, path_name, field_length, field_width, dry_run):
    """Flip a single .path file and write as a FLIPPED copy."""
    src = paths_dir / f"{path_name}.path"
    if not src.exists():
        print(f"  WARNING: Path file not found: {src}")
        return False

    data = read_json(src)
    flipped = flip_path_data(data, field_length, field_width)
    dest_name = make_flipped_name(path_name)
    dest = paths_dir / f"{dest_name}.path"
    write_json(dest, flipped, dry_run)
    return True


def flip_single_auto(autos_dir, paths_dir, auto_name, field_length, field_width, dry_run):
    """Flip a single .auto file and its referenced paths."""
    src = autos_dir / f"{auto_name}.auto"
    if not src.exists():
        print(f"  ERROR: Auto file not found: {src}")
        return False

    auto_data = read_json(src)

    # Flip referenced paths
    path_names = collect_path_names_from_command(auto_data.get("command", {}))
    unique_paths = list(dict.fromkeys(path_names))  # deduplicate, preserve order
    if unique_paths:
        print(f"  Flipping {len(unique_paths)} referenced path(s)...")
        for pn in unique_paths:
            flip_single_path(paths_dir, pn, field_length, field_width, dry_run)

    # Flip the auto
    flipped = flip_auto_data(auto_data, make_flipped_name)
    dest_name = make_flipped_name(auto_name)
    dest = autos_dir / f"{dest_name}.auto"
    write_json(dest, flipped, dry_run)
    return True


def main():
    parser = argparse.ArgumentParser(
        description="Radially flip a PathPlanner auto and its paths for alliance conversion (2026 REBUILT)."
    )
    parser.add_argument(
        "auto",
        help="Name of the auto to flip (without .auto extension). Also flips its referenced paths.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Preview what would be written without actually writing files.",
    )
    args = parser.parse_args()

    deploy_dir = find_deploy_dir()
    field_length, field_width = read_field_dimensions(deploy_dir)
    paths_dir = deploy_dir / "paths"
    autos_dir = deploy_dir / "autos"

    print(f"PathPlanner dir: {deploy_dir}")
    print(f"Field dimensions: {field_length}m x {field_width}m")
    print()

    print(f"Flipping auto: {args.auto}")
    flip_single_auto(autos_dir, paths_dir, args.auto, field_length, field_width, args.dry_run)

    print()
    print("Done.")


if __name__ == "__main__":
    main()
