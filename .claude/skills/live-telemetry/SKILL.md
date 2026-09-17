---
name: live-telemetry
description: Connect live to robot or simulator NetworkTables (AdvantageKit NT4) to read, stream, record, and assert on telemetry — pose, superstructure states, loop timing, alerts. Use to verify that a code change behaves correctly while the sim (or robot) is running, to inspect a value right now, or to record a WPILOG of a live session. For already-saved .wpilog files use wpilog-mcp instead.
argument-hint: [topics|get|watch|record|check] [keys or spec]
---

# Live Telemetry — LVT

Reads AdvantageKit telemetry straight off NetworkTables (port 5810). Nothing has to be written to a file first. Struct values (Pose2d, ChassisSpeeds, Translation3d, ...) are decoded to JSON.

## When to use this skill

- To answer "what is the robot doing right now": use `get` and `watch`.
- To prove a change works: use `check`, which evaluates pass/fail expressions against live data.
- To keep evidence of a session: `record` writes a `.wpilog` for AdvantageScope or wpilog-mcp.
- For logs that are already on disk (`logs/*.wpilog`), use the wpilog-mcp tools instead.

## Input

`$ARGUMENTS`: a subcommand plus keys or a check spec. With no arguments, run `topics` against the sim.

## Prerequisites

- An NT server must be running. For sim, start it with the `sim-loop` skill (`simctl start`).
- The first run of the tool creates `build/simtools-venv` with pyntcore and websockets. This happens automatically.

## Commands

All commands run from the repo root:

```bash
T=.claude/skills/live-telemetry/scripts/ntlive
$T topics --filter Superstructure                                        # discover keys + types
$T get RealOutputs/Swerve/CurrentPose "RealOutputs/Swerve/CurrentPose::translation.x"
$T watch RealOutputs/Superstructure/CurrentSuperState RealOutputs/LoggedRobot/FullCycleMS --duration 10 --hz 10
$T record --out build/sim-runs/manual.wpilog --duration 30
$T check --spec my-checks.json --duration 15 [--json]                     # exit 0 = PASS, 1 = FAIL
```

**Key syntax:**
- A key without a leading `/` gets `/AdvantageKit/` prepended. Examples: `RealOutputs/...` for `Logger.recordOutput`, `DriverStation/Enabled`, and `<Subsystem>/...` for `@AutoLog` inputs.
- Other tables need full paths, e.g. `/SmartDashboard/Auto Chooser/active`.
- `::a.b` digs into a decoded struct: `::translation.x`, `::rotation.value`, `::vx`, and `::0.x` for array index 0.

**Real robot:** add `--host 10.3.60.2`. Only use `topics`, `get`, `watch` and `record` there. Never run anything that publishes (see the sim-loop skill) against the robot. `ntlive` refuses to publish to any non-localhost host.

## Check spec

```json
{"checks": [
  {"name": "reaches FIRING", "kind": "eventually", "within": 10,
   "expr": "v('RealOutputs/Superstructure/Shooter/CurrentState') == 'FIRING'"},
  {"name": "moved > 1 m", "kind": "eventually",
   "expr": "dist(v('RealOutputs/Swerve/CurrentPose'), start('RealOutputs/Swerve/CurrentPose')) > 1.0"},
  {"name": "loop < 100 ms", "kind": "always", "after": 3, "expr": "v('RealOutputs/LoggedRobot/FullCycleMS') < 100"},
  {"name": "no error alerts", "kind": "never", "expr": "len(v('RealOutputs/Alerts/errors')) > 0"},
  {"name": "stopped at end", "kind": "final", "expr": "abs(v('RealOutputs/ShotCalculator/HubShotCalc/robotSpeeds::vx')) < 0.1"}
]}
```

**Check kinds:**
- `always`: the expression must be true on every sample.
- `never`: it must be false on every sample.
- `eventually`: it must become true at least once.
- `final`: only the last sample counts.

**Window options:** `after` and `within` (seconds) limit when a check is evaluated.

**Expression helpers:**
- `v(key)`: the latest value.
- `start(key)`: the first value seen.
- `age(key)`: seconds since the key last updated.
- `t`: seconds since the check started.
- `dist(poseA, poseB)`
- `abs`, `min`, `max`, `len`, `any`, `all`, `hypot`, `math`

**Missing data fails the check.** A check that never gets data reports `never evaluated (no data)`. Treat that as a wrong key, not a pass. Fix the key with `topics --filter`.

## Writing good checks

- Verify the key exists with `topics --filter` before you use it in a check. Keys like `Flywheel/LaunchCount` may exist but never change in sim.
- Assert on the behavior the change was meant to produce, and add at least one guard check (loop time, alerts, no console exceptions) for regressions.
- Look at real values with `watch` before choosing a threshold. Never loosen a threshold just to make a failing check pass without saying so.

## Output

Report each check as PASS/FAIL with its detail line, list the keys you watched, and give the `.wpilog` path if you recorded one. Present evidence, not conclusions: "shooter reached FIRING at t=6.2s", not "shooting works".
