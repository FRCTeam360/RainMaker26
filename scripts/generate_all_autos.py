#!/usr/bin/env python3
"""
Generates all three alliance/side variants of a MASTER Red Right auto.

Given a single MASTER Red Right auto, produces:
  - MIRRORED Red Left        -> RED LEFT AUTOS   (left/right mirror)
  - FLIPPED Blue Right       -> BLUE RIGHT AUTOS (180-degree flip)
  - FLIPPED MIRRORED Blue Left -> BLUE LEFT AUTOS  (flip of the mirror)

Usage:
  python scripts/generate_all_autos.py "MASTER Red Right Middle Hook"
  python scripts/generate_all_autos.py --dry-run "MASTER Red Right Middle Hook"

No external dependencies required - uses only the Python standard library.
"""

import argparse
import sys
from pathlib import Path

# Allow imports from the scripts/ directory
sys.path.insert(0, str(Path(__file__).resolve().parent))

import mirror_autos
import flip_autos


def main():
    parser = argparse.ArgumentParser(
        description="Generate MIRRORED, FLIPPED, and FLIPPED MIRRORED variants of a MASTER Red Right auto."
    )
    parser.add_argument(
        "auto",
        help="Name of the MASTER Red Right auto (without .auto extension).",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Preview what would be written without actually writing files.",
    )
    args = parser.parse_args()

    deploy_dir = mirror_autos.find_deploy_dir()
    field_length, field_width = mirror_autos.read_field_dimensions(deploy_dir)
    paths_dir = deploy_dir / "paths"
    autos_dir = deploy_dir / "autos"

    print(f"PathPlanner dir: {deploy_dir}")
    print(f"Field dimensions: {field_length}m x {field_width}m")
    print()

    # 1. Mirror: Red Right -> Red Left (RED LEFT AUTOS)
    mirrored_name = mirror_autos.make_mirrored_name(args.auto)
    print(f"[1/3] Mirroring -> '{mirrored_name}' (RED LEFT AUTOS)")
    mirror_autos.mirror_single_auto(autos_dir, paths_dir, deploy_dir, args.auto, field_width, args.dry_run)
    print()

    # 2. Flip: Red Right -> Blue Right (BLUE RIGHT AUTOS)
    flipped_name = flip_autos.make_flipped_name(args.auto)
    print(f"[2/3] Flipping  -> '{flipped_name}' (BLUE RIGHT AUTOS)")
    flip_autos.flip_single_auto(autos_dir, paths_dir, deploy_dir, args.auto, field_length, field_width, args.dry_run)
    print()

    # 3. Flip the mirror: Red Left -> Blue Left (BLUE LEFT AUTOS)
    flipped_mirrored_name = flip_autos.make_flipped_name(mirrored_name)
    print(f"[3/3] Flipping mirrored -> '{flipped_mirrored_name}' (BLUE LEFT AUTOS)")
    flip_autos.flip_single_auto(autos_dir, paths_dir, deploy_dir, mirrored_name, field_length, field_width, args.dry_run)
    print()

    print("Done.")


if __name__ == "__main__":
    main()
