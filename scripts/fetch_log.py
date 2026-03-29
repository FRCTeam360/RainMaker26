"""fetch_log.py — TUI to list AdvantageKit logs on the roboRIO and SCP one into logs/."""

import subprocess
import sys
import os
import msvcrt
import re

ROBORIO_HOST = "lvuser@10.3.60.2"
ROBORIO_LOG_DIR = "/home/lvuser/logs"
DEFAULT_USB_LOG_DIR = "/U"
REPO_ROOT = os.path.normpath(os.path.join(os.path.dirname(__file__), ".."))
LOCAL_LOG_DIR = os.path.join(REPO_ROOT, "logs")
CONSTANTS_JAVA = os.path.join(REPO_ROOT, "src", "main", "java", "frc", "robot", "Constants.java")

MAX_VISIBLE = 20

# ── Helpers ──────────────────────────────────────────────────────────────────


def die(msg):
    print(f"\033[31mError: {msg}\033[0m", file=sys.stderr)
    sys.exit(1)


def info(msg):
    print(f"\033[36m{msg}\033[0m")


def run_ssh(command):
    """Run a command on the roboRIO via ssh and return stdout lines."""
    result = subprocess.run(
        ["ssh", "-o", "ConnectTimeout=3", "-o", "BatchMode=yes", ROBORIO_HOST, command],
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        return None
    return result.stdout.strip().splitlines()


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


def print_log_section(title, path, entries, max_rows=8):
    """Display a small section listing logs for one source."""
    print(f"\n\033[1m{title}\033[0m")
    print(f"Path: {path}")

    if entries is None:
        print("  (Unavailable: drive missing or path not readable)")
        return

    if not entries:
        print("  (No logs found)")
        return

    for name in entries[:max_rows]:
        print(f"  {name}")
    if len(entries) > max_rows:
        print(f"  ... +{len(entries) - max_rows} more")


def read_key():
    """Read a keypress on Windows. Returns 'up', 'down', 'enter', 'q', or None."""
    ch = msvcrt.getwch()
    if ch == "\r":
        return "enter"
    if ch in ("q", "Q"):
        return "q"
    if ch == "\x1b":
        return "q"
    if ch in ("\x00", "\xe0"):
        # Special key — read the second byte
        ch2 = msvcrt.getwch()
        if ch2 == "H":
            return "up"
        if ch2 == "P":
            return "down"
    return None


def choose_menu_index(options, title, instructions):
    """Display a scrollable menu and return selected index, or None on Q."""
    visible_count = min(len(options), MAX_VISIBLE)
    half_visible = visible_count // 2
    cursor = 0

    def draw_menu(first_draw=False):
        if not first_draw:
            print(f"\033[{visible_count}A", end="")

        start = cursor - half_visible
        start = max(start, 0)
        if start + visible_count > len(options):
            start = max(len(options) - visible_count, 0)

        for i in range(start, min(start + visible_count, len(options))):
            if i == cursor:
                print(f"\033[7m > {options[i]} \033[0m\033[K")
            else:
                print(f"   {options[i]}\033[K")

    print(f"\n\033[1m{title}\033[0m")
    print(f"{instructions}\n")
    draw_menu(first_draw=True)

    while True:
        key = read_key()
        if key == "enter":
            return cursor
        if key == "q":
            return None
        if key == "up" and cursor > 0:
            cursor -= 1
        elif key == "down" and cursor < len(options) - 1:
            cursor += 1
        else:
            continue
        draw_menu()


# ── Connectivity check ──────────────────────────────────────────────────────

info(f"Connecting to roboRIO at {ROBORIO_HOST}...")
check = subprocess.run(
    ["ssh", "-o", "ConnectTimeout=3", "-o", "BatchMode=yes", ROBORIO_HOST, "true"],
    capture_output=True,
)
if check.returncode != 0:
    die(f"Cannot reach {ROBORIO_HOST}. Are you on the robot network?")

# ── Fetch remote log lists ──────────────────────────────────────────────────

usb_log_dir = detect_usb_log_dir()

info(f"Fetching roboRIO log list from {ROBORIO_LOG_DIR}...")
rio_logs = run_ssh(f'ls -1t "{ROBORIO_LOG_DIR}"')

info(f"Fetching USB log list from {usb_log_dir}...")
usb_logs = run_ssh(f'ls -1t "{usb_log_dir}"')

print_log_section("roboRIO Logs", ROBORIO_LOG_DIR, rio_logs)
print_log_section("USB (U Drive) Logs", usb_log_dir, usb_logs)

if not rio_logs and not usb_logs:
    die(f"No logs found in {ROBORIO_LOG_DIR} or {usb_log_dir}.")

# ── Source + log menus ──────────────────────────────────────────────────────

sources = [
    {"name": "roboRIO", "path": ROBORIO_LOG_DIR, "logs": rio_logs},
    {"name": "USB (U drive)", "path": usb_log_dir, "logs": usb_logs},
]


def describe_source(source):
    logs = source["logs"]
    if logs is None:
        return "unavailable"
    if not logs:
        return "no logs"
    return f"{len(logs)} logs"


selected_source_name = None
selected_remote_dir = None
selected_log = None

while selected_log is None:
    source_options = [f"{source['name']} [{describe_source(source)}]" for source in sources]
    source_index = choose_menu_index(
        source_options,
        "Choose log source",
        "↑/↓ to navigate, Enter to open, Q to quit",
    )

    if source_index is None:
        print("\nCancelled.")
        sys.exit(0)

    source = sources[source_index]
    source_logs = source["logs"]

    if source_logs is None:
        info(f"\n{source['name']} is unavailable (path not readable: {source['path']}).")
        continue
    if not source_logs:
        info(f"\n{source['name']} has no logs in {source['path']}.")
        continue

    log_index = choose_menu_index(
        source_logs,
        f"Select a {source['name']} log to download ({len(source_logs)} found)",
        "↑/↓ to navigate, Enter to select, Q to go back",
    )

    if log_index is None:
        continue

    selected_source_name = source["name"]
    selected_remote_dir = source["path"]
    selected_log = source_logs[log_index]

# ── Download ────────────────────────────────────────────────────────────────

os.makedirs(LOCAL_LOG_DIR, exist_ok=True)

info(f"\nDownloading {selected_log} from {selected_source_name}...")
result = subprocess.run(
    ["scp", "-r", f"{ROBORIO_HOST}:{selected_remote_dir}/{selected_log}", LOCAL_LOG_DIR],
)

if result.returncode != 0:
    die("SCP failed.")

print(f"\033[32m✓ Saved to logs/{selected_log}\033[0m")
