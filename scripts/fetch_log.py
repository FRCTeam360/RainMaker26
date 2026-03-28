"""fetch_log.py — TUI to list AdvantageKit logs on the roboRIO and SCP one into logs/."""

import subprocess
import sys
import os
import msvcrt

ROBORIO_HOST = "lvuser@10.3.60.2"
REMOTE_LOG_DIR = "/home/lvuser/logs"
REPO_ROOT = os.path.normpath(os.path.join(os.path.dirname(__file__), ".."))
LOCAL_LOG_DIR = os.path.join(REPO_ROOT, "logs")

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


# ── Connectivity check ──────────────────────────────────────────────────────

info(f"Connecting to roboRIO at {ROBORIO_HOST}...")
check = subprocess.run(
    ["ssh", "-o", "ConnectTimeout=3", "-o", "BatchMode=yes", ROBORIO_HOST, "true"],
    capture_output=True,
)
if check.returncode != 0:
    die(f"Cannot reach {ROBORIO_HOST}. Are you on the robot network?")

# ── Fetch remote log list ───────────────────────────────────────────────────

info(f"Fetching log list from {REMOTE_LOG_DIR}...")
logs = run_ssh(f"ls -1t {REMOTE_LOG_DIR}")

if not logs:
    die(f"No logs found in {REMOTE_LOG_DIR}.")

# ── TUI selector ────────────────────────────────────────────────────────────

visible_count = min(len(logs), MAX_VISIBLE)
half_visible = visible_count // 2
cursor = 0


def draw_menu(first_draw=False):
    """Draw the scrollable menu, overwriting previous draw."""
    if not first_draw:
        # Move cursor up to redraw
        print(f"\033[{visible_count}A", end="")

    start = cursor - half_visible
    start = max(start, 0)
    if start + visible_count > len(logs):
        start = max(len(logs) - visible_count, 0)

    for i in range(start, min(start + visible_count, len(logs))):
        if i == cursor:
            print(f"\033[7m > {logs[i]} \033[0m\033[K")
        else:
            print(f"   {logs[i]}\033[K")


print(f"\n\033[1mSelect a log to download ({len(logs)} found)\033[0m")
print("↑/↓ to navigate, Enter to select, Q to quit\n")

draw_menu(first_draw=True)

while True:
    key = read_key()
    if key == "enter":
        break
    if key == "q":
        print("\nCancelled.")
        sys.exit(0)
    if key == "up" and cursor > 0:
        cursor -= 1
    elif key == "down" and cursor < len(logs) - 1:
        cursor += 1
    else:
        continue
    draw_menu()

selected_log = logs[cursor]

# ── Download ────────────────────────────────────────────────────────────────

os.makedirs(LOCAL_LOG_DIR, exist_ok=True)

info(f"\nDownloading {selected_log}...")
result = subprocess.run(
    ["scp", "-r", f"{ROBORIO_HOST}:{REMOTE_LOG_DIR}/{selected_log}", LOCAL_LOG_DIR],
)

if result.returncode != 0:
    die("SCP failed.")

print(f"\033[32m✓ Saved to logs/{selected_log}\033[0m")
