#!/usr/bin/env python3
"""
Generates all four field-corner variants of BLine path JSON files.

Given a MASTER BLine path JSON (authored for one corner, e.g. Red Right or
Blue Right), produces four variants — one for each corner of the field:

  1. Original             (unchanged copy, or skipped if already in place)
  2. MIRRORED             (reflected across Y centerline: Left <-> Right)
  3. FLIPPED              (rotated 180 degrees: Red <-> Blue)
  4. FLIPPED MIRRORED     (both transforms combined)

Transform math:
  Mirror:  new_x = old_x,                new_y = field_width - old_y,  new_rot = -old_rot
  Flip:    new_x = field_length - old_x,  new_y = field_width - old_y,  new_rot = old_rot + pi
  Both:    new_x = field_length - old_x,  new_y = old_y,               new_rot = pi - old_rot

Name transforms swap alliance/side tags to match the existing PathPlanner
conventions (Red<->Blue, Left<->Right).

Usage:
  python scripts/generate_bline_variants.py                          # all paths in autos/paths/
  python scripts/generate_bline_variants.py "Blue Right Aggressive first swipe"  # single path
  python scripts/generate_bline_variants.py --dry-run                # preview without writing

No external dependencies required - uses only the Python standard library.
"""

import argparse
import copy
import json
import math
import os
import sys
from pathlib import Path


# --- Field dimensions (meters) for 2026 REBUILT ---
DEFAULT_FIELD_LENGTH = 16.54
DEFAULT_FIELD_WIDTH = 8.07


def find_bline_paths_dir():
    """Locate the BLine paths directory relative to the script or cwd."""
    candidates = [
        Path(__file__).resolve().parent.parent / "src" / "main" / "deploy" / "autos" / "paths",
        Path.cwd() / "src" / "main" / "deploy" / "autos" / "paths",
    ]
    for candidate in candidates:
        if candidate.is_dir():
            return candidate
    print("ERROR: Could not find src/main/deploy/autos/paths directory.")
    print("Run this script from the project root or the scripts/ folder.")
    sys.exit(1)


def read_field_dimensions():
    """Read field dimensions from navgrid.json, falling back to defaults."""
    candidates = [
        Path(__file__).resolve().parent.parent / "src" / "main" / "deploy" / "pathplanner" / "navgrid.json",
        Path.cwd() / "src" / "main" / "deploy" / "pathplanner" / "navgrid.json",
    ]
    for navgrid_path in candidates:
        if navgrid_path.exists():
            with open(navgrid_path, "r") as f:
                navgrid = json.load(f)
            field_size = navgrid.get("field_size", {})
            length = field_size.get("x", DEFAULT_FIELD_LENGTH)
            width = field_size.get("y", DEFAULT_FIELD_WIDTH)
            return length, width
    return DEFAULT_FIELD_LENGTH, DEFAULT_FIELD_WIDTH


# --- Name transforms ---

def swap_alliance_in_name(name):
    """Swap alliance tags: Red<->Blue."""
    name = name.replace("Red", "<<RED>>").replace("Blue", "<<BLUE>>")
    name = name.replace("red", "<<red>>").replace("blue", "<<blue>>")
    name = name.replace("<<RED>>", "Blue").replace("<<BLUE>>", "Red")
    name = name.replace("<<red>>", "blue").replace("<<blue>>", "red")
    return name


def swap_side_in_name(name):
    """Swap side tags: Left<->Right."""
    name = name.replace("Left", "<<LEFT>>").replace("Right", "<<RIGHT>>")
    name = name.replace("left", "<<left>>").replace("right", "<<right>>")
    name = name.replace("<<LEFT>>", "Right").replace("<<RIGHT>>", "Left")
    name = name.replace("<<left>>", "right").replace("<<right>>", "left")
    return name


def make_mirrored_name(name):
    """MIRRORED prefix + swap Left<->Right."""
    return "MIRRORED " + swap_side_in_name(name)


def make_flipped_name(name):
    """FLIPPED prefix + swap Red<->Blue."""
    return "FLIPPED " + swap_alliance_in_name(name)


def make_flipped_mirrored_name(name):
    """FLIPPED MIRRORED prefix + swap both alliance and side."""
    return "FLIPPED MIRRORED " + swap_alliance_in_name(swap_side_in_name(name))


# --- Coordinate transforms ---

def wrap_rotation_rad(radians):
    """Wrap a rotation in radians to (-pi, pi]."""
    while radians > math.pi:
        radians -= 2 * math.pi
    while radians <= -math.pi:
        radians += 2 * math.pi
    return radians


def mirror_xy(x, y, field_width):
    """Mirror across Y centerline (Left <-> Right)."""
    return x, field_width - y


def mirror_rotation(rot_rad):
    """Mirror rotation (negate)."""
    return wrap_rotation_rad(-rot_rad)


def flip_xy(x, y, field_length, field_width):
    """Rotate 180 degrees around field center (Red <-> Blue)."""
    return field_length - x, field_width - y


def flip_rotation(rot_rad):
    """Rotate heading by 180 degrees."""
    return wrap_rotation_rad(rot_rad + math.pi)


def flip_mirror_xy(x, y, field_length, field_width):
    """Both flip and mirror."""
    return field_length - x, y


def flip_mirror_rotation(rot_rad):
    """Both flip and mirror rotation: pi - rot."""
    return wrap_rotation_rad(math.pi - rot_rad)


# --- Path element transforms ---

def transform_path(path_data, xy_fn, rot_fn):
    """Apply coordinate and rotation transforms to all elements in a BLine path."""
    data = copy.deepcopy(path_data)

    for elem in data.get("path_elements", []):
        elem_type = elem.get("type")

        if elem_type == "waypoint":
            tt = elem.get("translation_target", {})
            if "x_meters" in tt and "y_meters" in tt:
                tt["x_meters"], tt["y_meters"] = xy_fn(tt["x_meters"], tt["y_meters"])

            rt = elem.get("rotation_target", {})
            if "rotation_radians" in rt:
                rt["rotation_radians"] = rot_fn(rt["rotation_radians"])

        elif elem_type == "translation":
            if "x_meters" in elem and "y_meters" in elem:
                elem["x_meters"], elem["y_meters"] = xy_fn(elem["x_meters"], elem["y_meters"])

        elif elem_type == "rotation":
            if "rotation_radians" in elem:
                elem["rotation_radians"] = rot_fn(elem["rotation_radians"])

        # event_trigger elements have no coordinates or rotations — leave unchanged

    return data


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

def generate_variants(paths_dir, path_name, field_length, field_width, dry_run):
    """Generate MIRRORED, FLIPPED, and FLIPPED MIRRORED variants of a single path."""
    src = paths_dir / f"{path_name}.json"
    if not src.exists():
        print(f"  ERROR: Path file not found: {src}")
        return False

    data = read_json(src)

    variants = [
        (
            make_mirrored_name(path_name),
            lambda x, y: mirror_xy(x, y, field_width),
            mirror_rotation,
        ),
        (
            make_flipped_name(path_name),
            lambda x, y: flip_xy(x, y, field_length, field_width),
            flip_rotation,
        ),
        (
            make_flipped_mirrored_name(path_name),
            lambda x, y: flip_mirror_xy(x, y, field_length, field_width),
            flip_mirror_rotation,
        ),
    ]

    for variant_name, xy_fn, rot_fn in variants:
        transformed = transform_path(data, xy_fn, rot_fn)
        dest = paths_dir / f"{variant_name}.json"
        write_json(dest, transformed, dry_run)

    return True


def is_generated_name(name):
    """Check if a path name is a generated variant (not an original)."""
    return name.startswith("MIRRORED ") or name.startswith("FLIPPED ")


def discover_master_paths(paths_dir):
    """Find all non-generated .json path files in the directory."""
    masters = []
    for f in sorted(paths_dir.glob("*.json")):
        name = f.stem
        if not is_generated_name(name):
            masters.append(name)
    return masters


def main():
    parser = argparse.ArgumentParser(
        description="Generate all four field-corner variants of BLine path JSON files."
    )
    parser.add_argument(
        "path",
        nargs="?",
        default=None,
        help="Name of the path to transform (without .json extension). "
             "If omitted, generates variants for all non-generated paths.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Preview what would be written without actually writing files.",
    )
    args = parser.parse_args()

    paths_dir = find_bline_paths_dir()
    field_length, field_width = read_field_dimensions()

    print(f"BLine paths dir: {paths_dir}")
    print(f"Field dimensions: {field_length}m x {field_width}m")
    print()

    if args.path:
        names = [args.path]
    else:
        names = discover_master_paths(paths_dir)
        if not names:
            print("No master path files found.")
            return
        print(f"Found {len(names)} master path(s):")
        for n in names:
            print(f"  - {n}")
        print()

    for name in names:
        print(f"Generating variants for: {name}")
        generate_variants(paths_dir, name, field_length, field_width, args.dry_run)
        print()

    print("Done.")


if __name__ == "__main__":
    main()
