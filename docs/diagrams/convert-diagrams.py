"""Convert all Mermaid diagrams to SVG and PNG recursively, in parallel."""

import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
PNG_SCALE = 4


def convert(mmd_file: Path, fmt: str) -> tuple[str, bool]:
    """Run mmdc to convert a single .mmd file. Returns (description, success)."""
    out_dir = mmd_file.parent / fmt
    out_dir.mkdir(exist_ok=True)
    out_file = out_dir / f"{mmd_file.stem}.{fmt}"

    desc = f"{mmd_file.relative_to(SCRIPT_DIR)} -> {fmt}/{out_file.name}"
    cmd = ["mmdc", "-i", str(mmd_file), "-o", str(out_file)]
    if fmt == "png":
        cmd += ["--scale", str(PNG_SCALE)]

    result = subprocess.run(cmd, capture_output=True, text=True, shell=True)
    if result.returncode != 0:
        return f"FAIL {desc}\n  {result.stderr.strip()}", False
    return f"  OK {desc}", True


def main():
    mmd_files = sorted(SCRIPT_DIR.rglob("*.mmd"))
    if not mmd_files:
        print("No .mmd files found.")
        return

    tasks = []
    for mmd in mmd_files:
        tasks.append((mmd, "svg"))
        tasks.append((mmd, "png"))

    print(f"Converting {len(mmd_files)} diagrams ({len(tasks)} outputs)...\n")

    failures = 0
    with ThreadPoolExecutor() as pool:
        futures = {pool.submit(convert, mmd, fmt): (mmd, fmt) for mmd, fmt in tasks}
        for future in as_completed(futures):
            msg, ok = future.result()
            print(msg)
            if not ok:
                failures += 1

    print(f"\nDone! {len(tasks) - failures}/{len(tasks)} succeeded.")
    if failures:
        sys.exit(1)


if __name__ == "__main__":
    main()
