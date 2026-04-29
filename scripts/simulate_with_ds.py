"""simulate_with_ds.py — Run ./gradlew simulateJava with the halsim_ds_socket
HAL extension enabled so the NI FRC Driver Station can connect to the simulated
robot on 127.0.0.1.

Flow:
  1. (Re)build build/sim/release_java.json via `./gradlew simulateExternalJavaRelease`
     so we know where halsim_ds_socket lives on this machine.
  2. Set HALSIM_EXTENSIONS to that .dll/.so/.dylib and prepend the JNI library
     dir to PATH / LD_LIBRARY_PATH / DYLD_LIBRARY_PATH so the loader can resolve
     the extension's native deps (wpiHal, ntcore, ...).
  3. Exec `./gradlew simulateJava`. The forked simulation JVM inherits the env,
     loads the DS socket extension at HAL init, and starts listening on the
     standard FRC DS ports (1110 UDP / 1115 / 1130 / 1140).

Once you see "********** Robot program startup complete **********", open the
FRC Driver Station. With your team number set, the DS auto-detects localhost
simulations. If it doesn't, open Setup → check "Use a Robot IP/URL Override"
and enter 127.0.0.1.

Usage:
  python scripts/simulate_with_ds.py [--rebuild-manifest]

  --rebuild-manifest   Re-run simulateExternalJavaRelease even if the JSON
                       already exists. Use after vendor dep changes.

Prerequisites:
  - build.gradle must enable desktop builds (`def includeDesktopSupport = true`).
  - FRC Game Tools (the NI Driver Station) installed on the host.
"""

import json
import os
import platform
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
MANIFEST = REPO_ROOT / "build" / "sim" / "release_java.json"
GRADLEW = REPO_ROOT / ("gradlew.bat" if os.name == "nt" else "gradlew")


def info(msg):
    print(f"\033[36m{msg}\033[0m")


def warn(msg):
    print(f"\033[33m{msg}\033[0m")


def die(msg):
    print(f"\033[31mError: {msg}\033[0m", file=sys.stderr)
    sys.exit(1)


def run_gradle(args, env=None):
    info(f"$ ./gradlew {' '.join(args)}")
    rc = subprocess.run([str(GRADLEW)] + args, cwd=REPO_ROOT, env=env).returncode
    if rc != 0:
        die(f"gradle exited with code {rc}")


def ensure_manifest(rebuild):
    if rebuild or not MANIFEST.exists():
        info("Producing simulation manifest...")
        run_gradle(["simulateExternalJavaRelease"])
    if not MANIFEST.exists():
        die(f"Manifest not produced at {MANIFEST} — desktop builds may be disabled.")


def find_ds_socket():
    raw = json.loads(MANIFEST.read_text())
    if not raw:
        die("Empty simulation manifest — confirm `includeDesktopSupport = true` in build.gradle.")
    target = raw[0]
    extensions = target.get("extensions") or []
    library_dir = target.get("libraryDir") or ""
    for ext in extensions:
        haystack = (ext.get("libName", "") + " " + ext.get("name", "")).lower()
        if "ds_socket" in haystack:
            return ext["libName"], library_dir
    available = ", ".join(e.get("name", "?") for e in extensions) or "(none)"
    die(f"halsim_ds_socket extension not found. Available: {available}")


def build_env(ds_lib, library_dir):
    env = os.environ.copy()
    env["HALSIM_EXTENSIONS"] = ds_lib

    if library_dir:
        if os.name == "nt":
            env["PATH"] = library_dir + os.pathsep + env.get("PATH", "")
        elif platform.system() == "Darwin":
            env["DYLD_LIBRARY_PATH"] = library_dir + os.pathsep + env.get("DYLD_LIBRARY_PATH", "")
        else:
            env["LD_LIBRARY_PATH"] = library_dir + os.pathsep + env.get("LD_LIBRARY_PATH", "")
    return env


def main():
    rebuild = "--rebuild-manifest" in sys.argv
    ensure_manifest(rebuild=rebuild)

    ds_lib, library_dir = find_ds_socket()
    info(f"DS Socket extension: {ds_lib}")
    info(f"Native library dir:  {library_dir or '(none)'}")
    print()
    info("Starting simulation. When robot code is up, open the FRC Driver Station —")
    info("it will detect the simulated robot at 127.0.0.1. (Override Robot IP in")
    info("DS Setup if it doesn't auto-connect.)")
    print()

    run_gradle(["simulateJava"], env=build_env(ds_lib, library_dir))


if __name__ == "__main__":
    main()
