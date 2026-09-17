"""simctl.py — headless WPILib simulation control for autonomous test loops.

  build     [--tests]                 gradle jar (+ unit tests) using the WPILib JDK
  start     [--restart]               launch sim headless (no GUI), wait until NT is live
  stop                                kill the running sim
  status                              is it running? where are its logs?
  ds        --mode auto|teleop|test|disabled [--alliance red1] [--game-data R]
  autos                               list auto chooser options
  select-auto NAME                    pick an auto in the chooser
  logscan   [--run-dir DIR]           summarize errors/exceptions in the sim console log
  scenario  FILE.json [--no-build] [--keep-running]
            build → start → record WPILOG → run phases while evaluating checks → scan
            console → stop → write report.json. Exit 0 = PASS, 1 = FAIL, 2 = infra error.

The sim runs on this machine only. Nothing here ever connects to a real robot.
"""

from __future__ import annotations

import argparse
import datetime as dt
import faulthandler
import json
import os
import re
import signal
import socket
import subprocess
import sys
import threading
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[3]
sys.path.insert(0, str(REPO / ".claude/skills/live-telemetry/scripts"))
import ntlive  # noqa: E402

JAVA_HOME = Path(os.environ.get("WPILIB_JAVA_HOME", Path.home() / "wpilib/2026/jdk"))
JNI_DIR = REPO / "build/jni/release"
JAR = REPO / "build/libs/RainMaker26.jar"
RUNS = REPO / "build/sim-runs"
CURRENT = RUNS / "current.json"
WS_URL = "ws://127.0.0.1:3300/wpilibws"
CHOOSER = "/SmartDashboard/Auto Chooser"
# Same rule as the "Simulation Test" CI job (.github/workflows/sim-test.yml).
ERROR_RE = re.compile(r"Error at|Exception")

MODES = {
    "disabled": {">enabled": False},
    "auto": {">enabled": True, ">autonomous": True, ">test": False},
    "teleop": {">enabled": True, ">autonomous": False, ">test": False},
    "test": {">enabled": True, ">autonomous": False, ">test": True},
}


def log(msg):
    print(f"[simctl {dt.datetime.now():%H:%M:%S}] {msg}", flush=True)


def gradle_env():
    env = dict(os.environ)
    env["JAVA_HOME"] = str(JAVA_HOME)
    return env


# ── Build ────────────────────────────────────────────────────────────────────


def build(tests: bool) -> bool:
    tasks = ["jar", "simulateExternalJavaRelease"] + (["test"] if tests else [])
    log(f"gradle {' '.join(tasks)}")
    out = RUNS / "last-build.log"
    RUNS.mkdir(parents=True, exist_ok=True)
    with open(out, "w") as f:
        rc = subprocess.run(["./gradlew", *tasks, "--console=plain"], cwd=REPO, env=gradle_env(),
                            stdout=f, stderr=subprocess.STDOUT).returncode
    if rc != 0:
        tail = out.read_text().splitlines()[-40:]
        print("\n".join(tail))
        log(f"BUILD FAILED (full log: {out})")
        return False
    if not (JNI_DIR / "libhalsim_ws_server.dylib").exists():
        log("libhalsim_ws_server missing — build.gradle needs `wpi.sim.addWebsocketsServer()`")
        return False
    log("build ok")
    return True


# ── Process control ──────────────────────────────────────────────────────────


def port_open(port: int) -> bool:
    with socket.socket() as s:
        s.settimeout(0.3)
        return s.connect_ex(("127.0.0.1", port)) == 0


def read_current():
    if not CURRENT.exists():
        return None
    info = json.loads(CURRENT.read_text())
    try:
        os.kill(info["pid"], 0)
    except OSError:
        return None
    return info


def start(restart: bool = False, run_dir: Path | None = None) -> dict:
    info = read_current()
    if info and not restart:
        log(f"sim already running (pid {info['pid']}) — use --restart")
        return info
    if info:
        stop()
    if port_open(5810) or port_open(3300):
        raise RuntimeError("ports 5810/3300 are busy — another sim (GUI?) is running; close it first")
    run_dir = run_dir or RUNS / dt.datetime.now().strftime("%Y%m%d-%H%M%S")
    run_dir.mkdir(parents=True, exist_ok=True)
    env = dict(os.environ)
    env["HALSIM_EXTENSIONS"] = str(JNI_DIR / "libhalsim_ws_server.dylib")
    env["DYLD_LIBRARY_PATH"] = str(JNI_DIR)
    console = open(run_dir / "robot.log", "w")
    proc = subprocess.Popen(
        [str(JAVA_HOME / "bin/java"), f"-Djava.library.path={JNI_DIR}", "-jar", str(JAR)],
        cwd=REPO, env=env, stdout=console, stderr=subprocess.STDOUT, start_new_session=True,
    )
    info = {"pid": proc.pid, "run_dir": str(run_dir), "started": time.time()}
    CURRENT.write_text(json.dumps(info))
    log(f"sim pid {proc.pid}, console → {run_dir / 'robot.log'}")

    nt = ntlive.LiveNT(name="simctl-start")
    ready = False
    try:
        deadline = time.monotonic() + 90
        while time.monotonic() < deadline:
            if proc.poll() is not None:
                raise RuntimeError(f"sim exited during startup (code {proc.returncode}); see {run_dir / 'robot.log'}")
            if port_open(3300) and nt.inst.isConnected() and nt.inst.getTopic(
                    "/AdvantageKit/DriverStation/Enabled").exists():
                log(f"sim ready after {time.time() - info['started']:.1f}s")
                ready = True
                return info
            time.sleep(0.5)
        raise RuntimeError("sim did not become ready within 90s")
    finally:
        nt.close()
        if not ready:
            stop()


def stop():
    info = read_current()
    if not info:
        log("no sim running")
        return
    try:
        os.killpg(info["pid"], signal.SIGTERM)
        for _ in range(50):
            os.kill(info["pid"], 0)
            time.sleep(0.1)
        os.killpg(info["pid"], signal.SIGKILL)
    except OSError:
        pass
    CURRENT.unlink(missing_ok=True)
    log(f"stopped sim pid {info['pid']}")


# ── Driver Station (HALSim websocket) ───────────────────────────────────────


class DriverStation:
    """Keeps a websocket open and re-sends the desired DS state once a second."""

    def __init__(self):
        from websockets.sync.client import connect

        self.ws = connect(WS_URL, max_size=None, close_timeout=1).__enter__()
        self.state = {">ds": True, ">enabled": False, ">station": "blue1", ">game_data": ""}
        self.lock = threading.Lock()
        self.alive = True
        threading.Thread(target=self._drain, daemon=True).start()
        threading.Thread(target=self._heartbeat, daemon=True).start()

    def _drain(self):  # the server streams device updates; discard them
        try:
            while self.alive:
                self.ws.recv()
        except Exception:  # noqa: BLE001 — connection closed
            pass

    def _send(self, msg):
        with self.lock:
            self.ws.send(json.dumps(msg))

    def _heartbeat(self):
        while self.alive:
            time.sleep(1.0)
            try:
                self.push()
            except Exception:  # noqa: BLE001
                return

    def push(self):
        self._send({"type": "DriverStation", "device": "", "data": {**self.state, ">new_data": True}})

    def set(self, mode=None, alliance=None, game_data=None, match_time=None):
        if mode:
            self.state.update(MODES[mode])
        if alliance:
            self.state[">station"] = alliance
        if game_data is not None:
            self.state[">game_data"] = game_data
        if match_time is not None:
            self.state[">match_time"] = match_time
        self.push()

    def joystick(self, port: int, axes=(), buttons=(), povs=()):
        self._send({"type": "Joystick", "device": str(port),
                    "data": {">axes": list(axes), ">buttons": list(buttons), ">povs": list(povs)}})

    def close(self):
        self.alive = False
        try:
            self.set(mode="disabled")
            self.ws.close()
        except Exception:  # noqa: BLE001
            pass


# ── Auto chooser ─────────────────────────────────────────────────────────────


def list_autos(nt):
    key = f"{CHOOSER}/options"
    nt.subscribe(keys=[key, f"{CHOOSER}/active"])
    nt.wait_topic(key, timeout=10)
    time.sleep(0.5)
    nt.pump()
    return nt.value(key) or [], nt.value(f"{CHOOSER}/active")


def select_auto(nt, name: str):
    options, _ = list_autos(nt)
    if name not in options:
        raise ValueError(f"auto '{name}' not in chooser. Options: {options}")
    nt.publish_string(f"{CHOOSER}/selected", name)
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline:
        nt.pump()
        if nt.value(f"{CHOOSER}/active") == name:
            log(f"auto selected: {name}")
            return
        time.sleep(0.1)
    raise RuntimeError(f"chooser did not report '{name}' as active")


# ── Console log scan ─────────────────────────────────────────────────────────


def logscan(run_dir: Path, max_lines: int = 30) -> dict:
    path = run_dir / "robot.log"
    if not path.exists():
        return {"errors": 0, "lines": []}
    hits = [ln.rstrip() for ln in path.read_text(errors="replace").splitlines()
            if ERROR_RE.search(ln)]
    uniq = list(dict.fromkeys(hits))
    return {"errors": len(hits), "unique": len(uniq), "lines": uniq[:max_lines]}


# ── Scenario ─────────────────────────────────────────────────────────────────


def run_scenario(path: Path, no_build: bool, keep_running: bool) -> int:
    spec = json.loads(path.read_text())
    run_dir = RUNS / f"{dt.datetime.now():%Y%m%d-%H%M%S}-{path.stem}"
    report = {"scenario": str(path), "run_dir": str(run_dir), "git": git_state()}

    if not no_build and not build(tests=spec.get("run_unit_tests", False)):
        return 2
    try:
        start(restart=True, run_dir=run_dir)
    except RuntimeError as e:
        log(f"INFRA ERROR: {e}")
        return 2

    nt = ntlive.LiveNT(name="simctl-scenario")
    nt.wait_connected()
    ds = rec = None
    try:
        ds = DriverStation()
        rec = nt.start_recording(str(run_dir / "telemetry.wpilog"))
        ds.set(mode="disabled", alliance=spec.get("alliance", "blue1"), game_data=spec.get("game_data", ""))
        if spec.get("auto"):
            select_auto(nt, spec["auto"])
        time.sleep(spec.get("settle_seconds", 2))

        phases = spec["phases"]
        total = sum(p["seconds"] for p in phases)
        done = threading.Event()
        results_box = {}
        checker = threading.Thread(target=lambda: results_box.update(
            r=ntlive.run_checks(nt, spec, total + 0.5, should_stop=done.is_set)), daemon=True)
        checker.start()
        for p in phases:
            log(f"phase: {p['mode']} for {p['seconds']}s")
            ds.set(mode=p["mode"], match_time=p.get("match_time"))
            for js in p.get("joysticks", []):
                ds.joystick(js.get("port", 0), js.get("axes", []), js.get("buttons", []), js.get("povs", []))
            time.sleep(p["seconds"])
        done.set()
        checker.join(timeout=10)
        ds.set(mode="disabled")
        time.sleep(0.5)
        results = results_box.get("r", [])
    finally:
        if rec:
            nt.stop_recording(rec)
        if ds:
            ds.close()
        nt.close()
        if not keep_running:
            stop()

    scan = logscan(run_dir)
    max_errors = spec.get("max_console_errors", 0)
    results.append({"name": "console errors", "kind": "final", "passed": scan["errors"] <= max_errors,
                    "detail": f"{scan['errors']} error lines ({scan['unique']} unique), limit {max_errors}"})
    report.update(results=results, console=scan, passed=all(r["passed"] for r in results),
                  wpilog=str(run_dir / "telemetry.wpilog"))
    (run_dir / "report.json").write_text(json.dumps(report, indent=2, default=str))

    print(f"\n=== scenario {path.stem} ===")
    ok = ntlive.print_results(results)
    if scan["lines"]:
        print("console errors (first unique lines):")
        for ln in scan["lines"][:10]:
            print("   ", ln[:200])
    print(f"report: {run_dir / 'report.json'}\nwpilog: {run_dir / 'telemetry.wpilog'}")
    return 0 if ok else 1


def git_state():
    def git(*a):
        return subprocess.run(["git", *a], cwd=REPO, capture_output=True, text=True).stdout.strip()
    return {"sha": git("rev-parse", "--short", "HEAD"), "dirty_files": git("status", "--short").splitlines()}


# ── CLI ──────────────────────────────────────────────────────────────────────


def main():
    faulthandler.register(signal.SIGUSR1)  # `kill -USR1 <pid>` dumps stacks if a run hangs
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("build"); p.add_argument("--tests", action="store_true")
    p = sub.add_parser("start"); p.add_argument("--restart", action="store_true")
    sub.add_parser("stop")
    sub.add_parser("status")
    p = sub.add_parser("ds"); p.add_argument("--mode", choices=MODES, required=True)
    p.add_argument("--alliance"); p.add_argument("--game-data")
    p.add_argument("--hold", type=float, default=0, help="keep the DS connection open this many seconds")
    sub.add_parser("autos")
    p = sub.add_parser("select-auto"); p.add_argument("name")
    p = sub.add_parser("logscan"); p.add_argument("--run-dir")
    p = sub.add_parser("scenario"); p.add_argument("file")
    p.add_argument("--no-build", action="store_true"); p.add_argument("--keep-running", action="store_true")
    a = ap.parse_args()

    if a.cmd == "build":
        return 0 if build(a.tests) else 1
    if a.cmd == "start":
        print(json.dumps(start(a.restart)))
    elif a.cmd == "stop":
        stop()
    elif a.cmd == "status":
        info = read_current()
        print(json.dumps(info or {"running": False}, indent=2))
    elif a.cmd == "ds":
        ds = DriverStation()
        ds.set(mode=a.mode, alliance=a.alliance, game_data=a.game_data)
        time.sleep(max(a.hold, 0.5))
        if a.hold:  # after holding, leave the robot disabled so it doesn't keep driving
            ds.close()
        else:
            ds.alive = False
            ds.ws.close()
    elif a.cmd in ("autos", "select-auto"):
        nt = ntlive.LiveNT(name="simctl")
        nt.wait_connected()
        if a.cmd == "autos":
            options, active = list_autos(nt)
            print(json.dumps({"active": active, "options": options}, indent=2))
        else:
            select_auto(nt, a.name)
        nt.close()
    elif a.cmd == "logscan":
        run_dir = Path(a.run_dir) if a.run_dir else Path((read_current() or {}).get("run_dir") or
                                                         max(RUNS.glob("2*"), key=os.path.getmtime))
        print(json.dumps(logscan(run_dir), indent=2))
    elif a.cmd == "scenario":
        return run_scenario(Path(a.file), a.no_build, a.keep_running)
    return 0


if __name__ == "__main__":
    sys.exit(main())
