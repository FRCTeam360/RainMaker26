#!/usr/bin/env python3
"""
Audits the PathPlanner deploy folder and reports which .path files are not
referenced by any .auto file.

Useful for cleaning up orphaned paths that are no longer used in any auto.

Usage:
  python scripts/audit_paths.py              # list unused paths
  python scripts/audit_paths.py --all        # also show which autos use each path
  python scripts/audit_paths.py --delete     # delete unused paths (with confirmation)

No external dependencies required - uses only the Python standard library.
"""

import argparse
import json
import sys
from pathlib import Path


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


def load_auto_references(autos_dir):
    """
    Returns a dict mapping path_name -> list of auto names that reference it.
    """
    refs = {}
    for auto_file in sorted(autos_dir.glob("*.auto")):
        try:
            with open(auto_file, "r") as f:
                auto_data = json.load(f)
        except (json.JSONDecodeError, OSError) as e:
            print(f"  WARNING: Could not read {auto_file.name}: {e}")
            continue

        path_names = collect_path_names_from_command(auto_data.get("command", {}))
        for pn in path_names:
            refs.setdefault(pn, []).append(auto_file.stem)

    return refs


def main():
    parser = argparse.ArgumentParser(
        description="Audit PathPlanner paths to find which are unused by any auto."
    )
    parser.add_argument(
        "--all",
        action="store_true",
        help="Also show used paths and which autos reference them.",
    )
    parser.add_argument(
        "--delete",
        action="store_true",
        help="Prompt to delete unused path files.",
    )
    args = parser.parse_args()

    deploy_dir = find_deploy_dir()
    paths_dir = deploy_dir / "paths"
    autos_dir = deploy_dir / "autos"

    print(f"PathPlanner dir: {deploy_dir}")
    print()

    # Collect all .path files on disk
    all_path_files = sorted(paths_dir.glob("*.path"))
    all_path_names = {p.stem for p in all_path_files}

    # Collect all path names referenced by autos
    refs = load_auto_references(autos_dir)
    referenced_names = set(refs.keys())

    # Paths referenced by autos but missing from disk
    missing = referenced_names - all_path_names
    # Paths on disk not referenced by any auto
    unused = all_path_names - referenced_names
    # Paths on disk that are referenced
    used = all_path_names & referenced_names

    # --- Report ---
    print(f"Autos:       {len(list(autos_dir.glob('*.auto')))}")
    print(f"Paths total: {len(all_path_names)}")
    print(f"  Used:      {len(used)}")
    print(f"  Unused:    {len(unused)}")
    if missing:
        print(f"  Missing:   {len(missing)}  (referenced by autos but file not found!)")
    print()

    if missing:
        print("=== MISSING paths (autos reference these but the file is gone) ===")
        for name in sorted(missing):
            autos_list = ", ".join(refs[name])
            print(f"  {name!r}  <-- used by: {autos_list}")
        print()

    if unused:
        print("=== UNUSED paths (no auto references these) ===")
        for name in sorted(unused):
            print(f"  {name}")
        print()
    else:
        print("No unused paths found.")
        print()

    if args.all and used:
        print("=== USED paths ===")
        for name in sorted(used):
            autos_list = ", ".join(refs[name])
            print(f"  {name}  <-- {autos_list}")
        print()

    if args.delete and unused:
        print(f"Delete {len(unused)} unused path file(s)? [y/N] ", end="", flush=True)
        answer = input().strip().lower()
        if answer == "y":
            for name in sorted(unused):
                path_file = paths_dir / f"{name}.path"
                path_file.unlink()
                print(f"  Deleted: {path_file.name}")
            print(f"\nDeleted {len(unused)} file(s).")
        else:
            print("Aborted. No files deleted.")


if __name__ == "__main__":
    main()
