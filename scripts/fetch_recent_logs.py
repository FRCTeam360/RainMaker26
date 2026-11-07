"""fetch_recent_logs.py — Download every AdvantageKit (.wpilog) and Phoenix (.hoot) log
modified on the roboRIO in the last N hours (default 48) into logs/.

Searches both the roboRIO log directory and the USB drive. Files already downloaded
(same relative path and size) are skipped, so it is safe to run repeatedly.

Usage:
    python scripts/fetch_recent_logs.py              # last 48 hours
    python scripts/fetch_recent_logs.py --hours 12   # last 12 hours
    python scripts/fetch_recent_logs.py --dry-run    # list what would be downloaded
"""

import argparse
import os
import re
import subprocess
import sys

ROBORIO_HOST = "lvuser@10.3.60.2"
ROBORIO_LOG_DIR = "/home/lvuser/logs"
DEFAULT_USB_LOG_DIR = "/U"
REPO_ROOT = os.path.normpath(os.path.join(os.path.dirname(__file__), ".."))
LOCAL_LOG_DIR = os.path.join(REPO_ROOT, "logs")
CONSTANTS_JAVA = os.path.join(REPO_ROOT, "src", "main", "java", "frc", "robot", "Constants.java")

DEFAULT_LOOKBACK_HOURS = 48
MINUTES_PER_HOUR = 60
SSH_CONNECT_TIMEOUT_SECONDS = 3
# Phoenix writes hoot files into dated subfolders, so search a few levels deep
MAX_SEARCH_DEPTH = 3
LOG_EXTENSIONS = ("wpilog", "hoot")
BYTES_PER_MEGABYTE = 1024 * 1024

SSH_OPTIONS = ["-o", f"ConnectTimeout={SSH_CONNECT_TIMEOUT_SECONDS}", "-o", "BatchMode=yes"]

# ── Helpers ──────────────────────────────────────────────────────────────────


def die(msg):
    print(f"\033[31mError: {msg}\033[0m", file=sys.stderr)
    sys.exit(1)


def info(msg):
    print(f"\033[36m{msg}\033[0m")


def detect_usb_log_dir():
    """Read USB log path from Constants.java, fallback to /U."""
    try:
        with open(CONSTANTS_JAVA, encoding="utf-8") as constants_file:
            constants_text = constants_file.read()
    except OSError:
        return DEFAULT_USB_LOG_DIR

    match = re.search(r'USB_ROOT_DIRECTORY\s*=\s*"([^"]+)"', constants_text)
    if match:
        return match.group(1)
    return DEFAULT_USB_LOG_DIR


def find_recent_logs(remote_dir, lookback_minutes):
    """Return [(remote_path, relative_path, size_bytes)] for recent logs under remote_dir.

    Returns None if the directory could not be searched (e.g. no USB drive).
    """
    name_filter = " -o ".join(f"-name '*.{ext}'" for ext in LOG_EXTENSIONS)
    # -H follows remote_dir itself if it is a symlink (/U -> /media/sda1)
    command = (
        f'[ -d "{remote_dir}" ] || exit 2; '
        f'find -H "{remote_dir}" -maxdepth {MAX_SEARCH_DEPTH} -type f '
        f"\\( {name_filter} \\) -mmin -{lookback_minutes} "
        "-exec stat -c '%s|%n' {} + 2>/dev/null; true"
    )
    result = subprocess.run(
        ["ssh", *SSH_OPTIONS, ROBORIO_HOST, command], capture_output=True, text=True
    )
    if result.returncode != 0:
        return None

    prefix = remote_dir.rstrip("/") + "/"
    logs = []
    for line in result.stdout.splitlines():
        size_text, _, remote_path = line.partition("|")
        if not size_text.isdigit() or not remote_path.startswith(prefix):
            continue
        relative_path = remote_path[len(prefix) :]
        # Phoenix writes to <USB>/logs/<date>/, store it as logs/<date>/ rather than logs/logs/
        relative_path = relative_path.removeprefix("logs/")
        logs.append((remote_path, relative_path, int(size_text)))
    return logs


def is_already_downloaded(relative_path, size_bytes):
    local_path = os.path.join(LOCAL_LOG_DIR, *relative_path.split("/"))
    return os.path.isfile(local_path) and os.path.getsize(local_path) == size_bytes


def download_logs(logs):
    """SCP logs into LOCAL_LOG_DIR, preserving subfolders. Returns the number that failed."""
    # Group by destination folder so each folder takes a single scp connection
    by_local_dir = {}
    for remote_path, relative_path, _ in logs:
        local_dir = os.path.join(LOCAL_LOG_DIR, *relative_path.split("/")[:-1])
        by_local_dir.setdefault(local_dir, []).append(remote_path)

    failed_count = 0
    for local_dir, remote_paths in sorted(by_local_dir.items()):
        os.makedirs(local_dir, exist_ok=True)
        sources = [f"{ROBORIO_HOST}:{path}" for path in remote_paths]
        result = subprocess.run(["scp", *SSH_OPTIONS, *sources, local_dir])
        if result.returncode != 0:
            print(f"\033[31m✗ Failed to download into {local_dir}\033[0m", file=sys.stderr)
            failed_count += len(remote_paths)
    return failed_count


# ── Main ─────────────────────────────────────────────────────────────────────


def main():
    parser = argparse.ArgumentParser(
        description="Download recent .wpilog and .hoot logs from the roboRIO and USB drive."
    )
    parser.add_argument(
        "--hours",
        type=float,
        default=DEFAULT_LOOKBACK_HOURS,
        help=f"how far back to look, in hours (default {DEFAULT_LOOKBACK_HOURS})",
    )
    parser.add_argument(
        "--dry-run", action="store_true", help="list logs that would be downloaded and exit"
    )
    args = parser.parse_args()
    lookback_minutes = max(1, round(args.hours * MINUTES_PER_HOUR))

    info(f"Connecting to roboRIO at {ROBORIO_HOST}...")
    check = subprocess.run(["ssh", *SSH_OPTIONS, ROBORIO_HOST, "true"], capture_output=True)
    if check.returncode != 0:
        die(f"Cannot reach {ROBORIO_HOST}. Are you on the robot network?")

    # Keyed by relative path so a log present in both places is only fetched once
    recent_logs = {}
    for source_name, remote_dir in (("roboRIO", ROBORIO_LOG_DIR), ("USB", detect_usb_log_dir())):
        info(f"Searching {source_name} ({remote_dir}) for logs from the last {args.hours:g}h...")
        logs = find_recent_logs(remote_dir, lookback_minutes)
        if logs is None:
            print(f"  {source_name} unavailable (path missing or not readable)")
            continue
        print(f"  Found {len(logs)} log(s)")
        for log in logs:
            recent_logs.setdefault(log[1], log)

    if not recent_logs:
        info("No logs found in that window. (Is the roboRIO clock set? It syncs from the DS.)")
        return

    to_download = []
    for log in sorted(recent_logs.values(), key=lambda log: log[1]):
        if is_already_downloaded(log[1], log[2]):
            continue
        to_download.append(log)

    skipped_count = len(recent_logs) - len(to_download)
    total_megabytes = sum(log[2] for log in to_download) / BYTES_PER_MEGABYTE
    info(
        f"\n{len(to_download)} new log(s) to download ({total_megabytes:.1f} MB), "
        f"{skipped_count} already downloaded"
    )
    for _, relative_path, size_bytes in to_download:
        print(f"  {relative_path}  ({size_bytes / BYTES_PER_MEGABYTE:.1f} MB)")

    if args.dry_run or not to_download:
        return

    failed_count = download_logs(to_download)
    if failed_count:
        die(f"{failed_count} log(s) failed to download.")
    print(f"\033[32m✓ Downloaded {len(to_download)} log(s) to logs/\033[0m")


if __name__ == "__main__":
    main()
