#!/usr/bin/env python3
"""
Radially flips PathPlanner autos and paths around the center of the field.

The 2026 REBUILT field has 180-degree rotational symmetry. Red alliance paths
are the default. This script generates Blue alliance equivalents by rotating
all coordinates 180 degrees around the field center.

Flip math:
  new_x = field_length - old_x
  new_y = field_width  - old_y
  new_rotation = rotation + 180 (wrapped to [-180, 180])

Output files are prefixed with "FLIPPED " so they're easy to identify as generated.
Review them in PathPlanner, then rename the ones you want to keep.

Usage:
  python scripts/flip_autos.py                        # flip all [R]/Red files
  python scripts/flip_autos.py --auto "[R] Depot"     # flip one auto + its paths
  python scripts/flip_autos.py --path "[R] Depot 1"   # flip one path only
  python scripts/flip_autos.py --reverse              # flip Blue -> Red instead
  python scripts/flip_autos.py --dry-run              # preview without writing

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
    """Swap Red alliance tags to Blue in a name."""
    name = name.replace("[R]", "[B]")
    name = name.replace("Red", "Blue")
    name = name.replace("red", "blue")
    return name


def make_flipped_name(name):
    """Add 'FLIPPED ' prefix and swap Red->Blue. Idempotent."""
    if name.startswith(FLIPPED_PREFIX):
        return name
    return FLIPPED_PREFIX + swap_alliance_in_name(name)


def is_red_name(name):
    """Check if a name indicates a Red alliance path/auto."""
    return "[R]" in name or "Red" in name or "red" in name


def is_blue_name(name):
    """Check if a name indicates a Blue alliance path/auto."""
    return "[B]" in name or "Blue" in name or "blue" in name


def is_flipped_name(name):
    """Check if a name is already a FLIPPED output."""
    return name.startswith(FLIPPED_PREFIX)


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

    # Flip waypoints
    for wp in data.get("waypoints", []):
        for key in ("anchor", "prevControl", "nextControl"):
            point = wp.get(key)
            if point is not None:
                point["x"], point["y"] = flip_point(
                    point["x"], point["y"], field_length, field_width
                )

    # Flip rotation targets
    for rt in data.get("rotationTargets", []):
        if "rotationDegrees" in rt:
            rt["rotationDegrees"] = flip_rotation(rt["rotationDegrees"])

    # Flip point-towards zones
    for ptz in data.get("pointTowardsZones", []):
        pos = ptz.get("fieldPosition")
        if pos is not None:
            pos["x"], pos["y"] = flip_point(
                pos["x"], pos["y"], field_length, field_width
            )

    # Flip goal end state rotation
    goal = data.get("goalEndState")
    if goal and "rotation" in goal:
        goal["rotation"] = flip_rotation(goal["rotation"])

    # Flip ideal starting state rotation
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

    # Recurse into nested command groups (sequential, parallel, deadline, race)
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


def flip_single_auto(autos_dir, paths_dir, auto_name, field_length, field_width,
                     dry_run, flip_paths=True):
    """Flip a single .auto file and optionally its referenced paths."""
    src = autos_dir / f"{auto_name}.auto"
    if not src.exists():
        print(f"  ERROR: Auto file not found: {src}")
        return False

    auto_data = read_json(src)

    # Flip referenced paths first
    if flip_paths:
        path_names = collect_path_names_from_command(auto_data.get("command", {}))
        if path_names:
            print(f"  Flipping {len(path_names)} referenced path(s)...")
            for pn in path_names:
                flip_single_path(paths_dir, pn, field_length, field_width, dry_run)

    # Flip the auto (path references get FLIPPED prefix too)
    flipped = flip_auto_data(auto_data, make_flipped_name)
    dest_name = make_flipped_name(auto_name)
    dest = autos_dir / f"{dest_name}.auto"
    write_json(dest, flipped, dry_run)
    return True


def flip_all(deploy_dir, field_length, field_width, reverse, dry_run):
    """Flip all alliance-tagged autos and paths, outputting FLIPPED copies."""
    paths_dir = deploy_dir / "paths"
    autos_dir = deploy_dir / "autos"

    detect_fn = is_blue_name if reverse else is_red_name
    src_label = "Blue" if reverse else "Red"

    # Flip paths
    flipped_paths = set()
    if paths_dir.exists():
        for path_file in sorted(paths_dir.glob("*.path")):
            name = path_file.stem
            if is_flipped_name(name):
                continue
            if detect_fn(name):
                print(f"Flipping path: {name} ({src_label} -> FLIPPED)")
                flip_single_path(paths_dir, name, field_length, field_width, dry_run)
                flipped_paths.add(name)

    # Flip autos (and any referenced paths not already flipped)
    if autos_dir.exists():
        for auto_file in sorted(autos_dir.glob("*.auto")):
            name = auto_file.stem
            if is_flipped_name(name):
                continue
            if detect_fn(name):
                print(f"Flipping auto: {name} ({src_label} -> FLIPPED)")
                auto_data = read_json(auto_file)
                ref_paths = collect_path_names_from_command(auto_data.get("command", {}))

                # Flip any referenced paths we haven't already handled
                for pn in ref_paths:
                    if pn not in flipped_paths:
                        print(f"  Flipping referenced path: {pn}")
                        flip_single_path(
                            paths_dir, pn, field_length, field_width, dry_run
                        )
                        flipped_paths.add(pn)

                # Flip the auto
                flipped = flip_auto_data(auto_data, make_flipped_name)
                dest_name = make_flipped_name(name)
                dest = autos_dir / f"{dest_name}.auto"
                write_json(dest, flipped, dry_run)


def main():
    parser = argparse.ArgumentParser(
        description="Radially flip PathPlanner autos/paths for alliance conversion (2026 REBUILT)."
    )
    parser.add_argument(
        "--auto",
        help="Flip a single auto by name (without .auto extension). Also flips its referenced paths.",
    )
    parser.add_argument(
        "--path",
        help="Flip a single path by name (without .path extension).",
    )
    parser.add_argument(
        "--reverse",
        action="store_true",
        help="Flip Blue -> Red instead of the default Red -> Blue.",
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

    if args.path:
        print(f"Flipping path: {args.path}")
        flip_single_path(paths_dir, args.path, field_length, field_width, args.dry_run)
    elif args.auto:
        print(f"Flipping auto: {args.auto}")
        flip_single_auto(
            autos_dir, paths_dir, args.auto, field_length, field_width,
            args.dry_run
        )
    else:
        src_label = "Blue" if args.reverse else "Red"
        print(f"Flipping all {src_label}-tagged autos and paths -> FLIPPED copies...")
        print()
        flip_all(deploy_dir, field_length, field_width, args.reverse, args.dry_run)

    print()
    print("Done.")


if __name__ == "__main__":
    main()
