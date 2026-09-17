---
name: sim-loop
description: Autonomously iterate on robot code in headless WPILib simulation — build, launch sim without the GUI, drive the Driver Station (enable auto/teleop, alliance, game data, joysticks, auto chooser), run scenario files with live telemetry checks, read the verdict, fix, repeat. Use when asked to make a change and prove it works in sim, tune behavior against a measurable goal, or reproduce a bug in simulation.
argument-hint: [goal or scenario file] [max iterations]
---

# Simulation Loop — SIM

Closed loop: **edit → build → simulate a scenario → check live telemetry → read evidence → edit again**. The loop never needs the sim GUI or a human at the Driver Station. It uses the `live-telemetry` skill for all reads and checks.

## When to use this skill

Use this when a change can be judged from sim telemetry: autos, superstructure state transitions, drive behavior, loop timing, or startup errors. Don't use it for anything that depends on real hardware (CAN, vision tuning, current limits). Use logs from the robot for those.

## Input

`$ARGUMENTS`: the goal (e.g. "shooter should reach FIRING within 5 s in Double Trench auto") or a scenario path. An optional iteration cap defaults to **5**.

## Prerequisites

- WPILib 2026 JDK at `~/wpilib/2026/jdk`. Override the path with `WPILIB_JAVA_HOME`.
- `build.gradle` must contain `wpi.sim.addWebsocketsServer().defaultEnabled = false`. This lets a script drive the Driver Station, and the normal GUI sim is unaffected.
- No other sim running. Ports 5810 and 3300 must be free.

## Tools

```bash
S=.claude/skills/sim-loop/scripts/simctl
$S build [--tests]                  # gradle jar (+ unit tests); log at build/sim-runs/last-build.log
$S scenario <file.json> [--no-build] [--keep-running] [--auto NAME]   # the whole loop step; exit 0 PASS / 1 FAIL / 2 infra
$S start [--restart] | stop | status
$S ds --mode auto|teleop|test|disabled [--alliance red1] [--game-data R] [--hold SECONDS]
$S autos                            # chooser options
$S select-auto "<name>"
$S logscan                          # console errors (same rule as CI's Simulation Test)
```

`scenario` does the following in order:
1. Builds the code.
2. Starts the sim headless.
3. Records all NT data to `telemetry.wpilog`.
4. Sets alliance, game data and auto.
5. Runs the phases while evaluating checks at 50 Hz.
6. Scans the console for errors.
7. Stops the sim.
8. Writes `build/sim-runs/<timestamp>-<name>/report.json`.

Run it in the foreground with a timeout of about 2× the scenario length plus 60 s. For long batches, use `run_in_background`.

## Scenario format

Examples are in `scenarios/`:
- `auto-smoke.json`: the auto drives, shoots, and nothing errors.
- `auto-duration.json`: does the auto finish inside 15 s? Auto stays enabled for 25 s so an overrun shows its real finish time. It reads `Robot/AutoCommandRunning`.
- `teleop-drive.json`: joystick input moves the robot.

Pass `--auto "<name>"` to run any scenario with a different auto (names come from `simctl autos`).

```json
{
  "alliance": "blue1", "game_data": "", "auto": "[PathPlanner] FLIPPED Blue Right Double Trench",
  "settle_seconds": 2, "run_unit_tests": false, "max_console_errors": 0,
  "phases": [
    {"mode": "auto", "seconds": 15},
    {"mode": "teleop", "seconds": 3, "joysticks": [{"port": 0, "axes": [0, -0.8, 0, 0, 0, 0], "buttons": [false, false]}]}
  ],
  "checks": [ ...live-telemetry check spec... ]
}
```

- Joystick port 0 is the driver controller and port 1 is the operator.
- Xbox axes are ordered `[LX, LY, LT, RT, RX, RY]`, and pushing a stick forward gives a negative Y.
- Button indices start at 0 in the array (index 0 = button 1 = A).
- Put new scenarios for a task in `scenarios/`. Name the file after the behavior, not the ticket.

## The loop

1. **Define done before touching code.**
   - Write or choose a scenario whose checks encode the goal. Include guard checks: loop time, alerts, and console errors.
   - Discover keys with `ntlive topics` on a running sim (`simctl start`, then `simctl stop`).
   - If the goal can't be expressed as a check, add a `Logger.recordOutput(...)` for it. That is a legitimate part of the change.
2. **Baseline.** Run the scenario on the unmodified code and record which checks fail. A check that already passes proves nothing about your change. Tighten it or add one.
3. **Iterate**, at most N times. Each iteration:
   - Make one focused change.
   - Run `simctl scenario <file>`.
     - Exit 2 means an infrastructure or build problem. Fix that first: read `last-build.log` or `robot.log`.
     - Exit 1 means some checks failed. Read which ones, and the `detail` in `report.json`.
   - Diagnose from evidence:
     - `ntlive watch` on a sim started with `--keep-running`
     - the run's `telemetry.wpilog`, read via wpilog-mcp or `DataLogReader`
     - `robot.log`
   - Append one line to `build/sim-runs/journal.md`: iteration, hypothesis, change, result.
4. **Finish.**
   - Run `./gradlew test` and `./gradlew spotlessApply`.
   - Rerun the scenario once more on the final code.
   - Run `simctl stop` and confirm `simctl status` shows nothing running.

## Stop and ask the user when

- The iteration cap is hit or the same check fails twice for the same reason.
- The fix would change an autonomous routine or path, CAN IDs, motor configs, or vendor deps (CLAUDE.md "Ask First").
- A check threshold would have to be loosened to pass.
- The behavior depends on physics the sim doesn't model.

## Rules

- Sim only. Never deploy, and never point `simctl` or anything that publishes at the robot.
- Don't commit unless asked. Leave `build/sim-runs/` artifacts in place for review.
- Sim success is necessary but not sufficient. Say what the sim doesn't cover.

## Output

- Final verdict table: the check results from the last run.
- Iterations used, and the journal lines.
- Files changed.
- `report.json` and `telemetry.wpilog` paths.
- What still needs real-robot verification.
