#!/usr/bin/env python3
"""
Mirrors a single PathPlanner auto and its referenced paths across the field
centerline (left <-> right) while staying on the same alliance.

The 2026 REBUILT field is symmetric about the horizontal centerline
(y = field_width / 2). This script generates the left/right mirror of an auto
by reflecting all Y coordinates and negating rotations.

Mirror math:
  new_x = old_x                     (same alliance side)
  new_y = field_width - old_y        (reflected across centerline)
  new_rotation = -rotation           (mirrored heading)

Output files are prefixed with "MIRRORED " and have side tags swapped
(Left<->Right, [L]<->[R]) so they're easy to identify as generated.
Mirrored paths are placed in a subfolder named after the new auto.

Usage:
  python scripts/mirror_autos.py "Blue Left Middle"         # mirror auto + its paths
  python scripts/mirror_autos.py --dry-run "Red Right Middle" # preview without writing

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


MIRRORED_PREFIX = "MIRRORED "


# --- Name mirroring ---

def swap_side_in_name(name):
    """Swap left/right tags in a name. Left->Right and Right->Left."""
    name = name.replace("[L]", "<<L>>").replace("[R]", "<<R>>")
    name = name.replace("Left", "<<LEFT>>").replace("Right", "<<RIGHT>>")
    name = name.replace("left", "<<left>>").replace("right", "<<right>>")
    name = name.replace("<<L>>", "[R]").replace("<<R>>", "[L]")
    name = name.replace("<<LEFT>>", "Right").replace("<<RIGHT>>", "Left")
    name = name.replace("<<left>>", "right").replace("<<right>>", "left")
    return name


def strip_master(name):
    """Remove 'Master' (case insensitive) from a name and clean up extra spaces."""
    import re
    name = re.sub(r'(?i)master\s*', '', name)
    return ' '.join(name.split())


def make_mirrored_name(name):
    """Add 'MIRRORED ' prefix, swap side tags, and strip 'Master'."""
    if name.startswith(MIRRORED_PREFIX):
        # Strip prefix, swap back, then re-prefix with the new swap
        inner = name[len(MIRRORED_PREFIX):]
        return MIRRORED_PREFIX + strip_master(swap_side_in_name(inner))
    return MIRRORED_PREFIX + strip_master(swap_side_in_name(name))


# --- Coordinate mirroring ---

def wrap_rotation(degrees):
    """Wrap a rotation in degrees to the (-180, 180] range."""
    while degrees > 180:
        degrees -= 360
    while degrees <= -180:
        degrees += 360
    return round(degrees, 4)


def mirror_point(x, y, field_width):
    """Mirror a point across the field centerline (y = field_width / 2)."""
    return round(x, 6), round(field_width - y, 6)


def mirror_rotation(degrees):
    """Mirror heading across the horizontal centerline (negate the angle)."""
    return wrap_rotation(-degrees)


# --- Path mirroring ---

def mirror_path_data(path_data, field_width):
    """Mirror all coordinates and rotations in a PathPlanner .path file."""
    data = copy.deepcopy(path_data)

    for wp in data.get("waypoints", []):
        for key in ("anchor", "prevControl", "nextControl"):
            point = wp.get(key)
            if point is not None:
                point["x"], point["y"] = mirror_point(
                    point["x"], point["y"], field_width
                )
        linked = wp.get("linkedName")
        if linked is not None:
            wp["linkedName"] = make_mirrored_name(linked)

    for rt in data.get("rotationTargets", []):
        if "rotationDegrees" in rt:
            rt["rotationDegrees"] = mirror_rotation(rt["rotationDegrees"])

    for ptz in data.get("pointTowardsZones", []):
        pos = ptz.get("fieldPosition")
        if pos is not None:
            pos["x"], pos["y"] = mirror_point(
                pos["x"], pos["y"], field_width
            )

    goal = data.get("goalEndState")
    if goal and "rotation" in goal:
        goal["rotation"] = mirror_rotation(goal["rotation"])

    start = data.get("idealStartingState")
    if start and "rotation" in start:
        start["rotation"] = mirror_rotation(start["rotation"])

    return data


# --- Auto mirroring ---

def mirror_auto_command(command, mirror_name_fn):
    """Recursively update path name references in an auto command tree."""
    cmd = copy.deepcopy(command)

    if cmd.get("type") == "path":
        path_name = cmd.get("data", {}).get("pathName")
        if path_name:
            cmd["data"]["pathName"] = mirror_name_fn(path_name)

    nested = cmd.get("data", {}).get("commands")
    if nested:
        cmd["data"]["commands"] = [
            mirror_auto_command(c, mirror_name_fn) for c in nested
        ]

    return cmd


def mirror_auto_data(auto_data, mirror_name_fn):
    """Mirror a PathPlanner .auto file by updating all path references."""
    data = copy.deepcopy(auto_data)
    if data.get("command"):
        data["command"] = mirror_auto_command(data["command"], mirror_name_fn)
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

def mirror_single_path(paths_dir, path_name, field_width, dest_folder, dry_run):
    """Mirror a single .path file and write as a MIRRORED copy into dest_folder."""
    src = paths_dir / f"{path_name}.path"
    if not src.exists():
        print(f"  WARNING: Path file not found: {src}")
        return False

    data = read_json(src)
    mirrored = mirror_path_data(data, field_width)
    mirrored["folder"] = dest_folder
    dest_name = make_mirrored_name(path_name)
    dest = paths_dir / dest_folder / f"{dest_name}.path"
    write_json(dest, mirrored, dry_run)
    return True


def mirror_single_auto(autos_dir, paths_dir, auto_name, field_width, dry_run):
    """Mirror a single .auto file and its referenced paths."""
    src = autos_dir / f"{auto_name}.auto"
    if not src.exists():
        print(f"  ERROR: Auto file not found: {src}")
        return False

    auto_data = read_json(src)
    dest_auto_name = make_mirrored_name(auto_name)

    # Mirror referenced paths into a folder named after the new auto
    path_names = collect_path_names_from_command(auto_data.get("command", {}))
    unique_paths = list(dict.fromkeys(path_names))  # deduplicate, preserve order
    if unique_paths:
        print(f"  Mirroring {len(unique_paths)} referenced path(s) into folder '{dest_auto_name}'...")
        for pn in unique_paths:
            mirror_single_path(paths_dir, pn, field_width, dest_auto_name, dry_run)

    # Mirror the auto
    mirrored = mirror_auto_data(auto_data, make_mirrored_name)
    dest = autos_dir / f"{dest_auto_name}.auto"
    write_json(dest, mirrored, dry_run)
    return True


def main():
    parser = argparse.ArgumentParser(
        description="Mirror a PathPlanner auto and its paths left<->right across the field centerline (2026 REBUILT)."
    )
    parser.add_argument(
        "auto",
        help="Name of the auto to mirror (without .auto extension). Also mirrors its referenced paths.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Preview what would be written without actually writing files.",
    )
    args = parser.parse_args()

    deploy_dir = find_deploy_dir()
    _, field_width = read_field_dimensions(deploy_dir)
    paths_dir = deploy_dir / "paths"
    autos_dir = deploy_dir / "autos"

    print(f"PathPlanner dir: {deploy_dir}")
    print(f"Field width: {field_width}m (centerline at {field_width / 2}m)")
    print()

    print(f"Mirroring auto: {args.auto}")
    mirror_single_auto(autos_dir, paths_dir, args.auto, field_width, args.dry_run)

    print()
    print("Done.")


if __name__ == "__main__":
    main()
