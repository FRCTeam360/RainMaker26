# Claude Code Review Bug Report — 2026-03-02

A comprehensive audit of all merged PRs in RainMaker26 to identify real bugs caught by Claude code review (`claude[bot]`) that were subsequently addressed.

## Critical Runtime Bugs

### PR #404 — Flywheel kicker bang bang and intake tuning

- **Missing `break` statements in HopperRoller switch/case (x2):** The `UNJAMMING` case fell through to `OFF`, making the unjamming state dead code — the -0.8 duty cycle was immediately overwritten by `stop()`. Fixed via explicit "added break statement" commits.
- **Missing feedforward on REVLib Slot 1:** Hold velocity control at 4500 RPM would run with zero feedforward, causing large steady-state error.

### PR #339 — HoodIONoOp

- **Infinite recursion causing StackOverflowError:** Every `Hood` instance created another `Hood(new HoodIONoOp())` as a field, which recursively created another, guaranteeing a crash at construction. Fixed in follow-up commit.
- **Missing `class` keyword:** `public static HoodIONoOp implements HoodIO` would not compile.

### PR #321 — Intake pivot and hopper roller setup

- **Missing `break` in IntakePivot switch:** DEPLOYED fell through to STOWED then OFF, so `setPosition(0.0)` was always the final call regardless of state — DEPLOYED did nothing. Commit message explicitly says "reviewed claude code".
- **HopperRoller command bypassed state machine:** Direct IO calls were overwritten every cycle by `applyState()`.

### PR #301 — Flywheel bang-bang controller (46 commits of iteration)

- **Multiple compile errors:** Missing closing braces swallowed methods, wrong motor array size (4 instead of 2).
- **`isAtSetpointVelocity()` called twice per cycle:** Mutated debouncers as a side effect, corrupting state.
- **Stator current limit configured but disabled:** `StatorCurrentLimitEnable = false` allowed unlimited current with bang-bang control.
- **StatusSignal values read without refresh:** Stale CTRE data returned silently.

## Crash Risks

### PR #287 — Facing hub while shooting

- **`DriverStation.getAlliance().get()` crash:** Would throw `NoSuchElementException` before a match starts or in simulation. Fixed via "alliance check for facing hub while driving" commit. A suggestion was applied directly with "Apply suggestion from @claude[bot]" as the commit message.

### PR #293 — IntakePivotMotionMagic

- **`zeroButton` and `brakeButton` never initialized:** Guaranteed `NullPointerException` on any access.

## State Machine Logic Errors

### PR #386 — Intaking upgrade (Agitation)

- **`default` case set wrong internal state:** OFF requests set `currentState = AT_SETPOINT` instead of `OFF`, making the OFF state unreachable — the arm would always actively drive to position 0 instead of stopping.

### PR #392 — Testing pull requests

- **Unjamming state never set intake state:** Intake continued running forward while indexer/hopper/kicker all reversed, fighting the unjam.
- **Default passive state deployed intake pivot:** Robot's idle posture changed from stowed to deployed. Reverted before scrimmage.

## Configuration and Safety Bugs

### PR #367 — Reduce loop time with signal caching

- **NT4Publisher disabled for all real robot configs:** Zero live AdvantageKit telemetry on physical hardware — no driver dashboard, no live tuning. Fixed via "Uncommented NT4Publisher" commit directly after review.
- **Invalid cast `(boolean)` on `Object` reference:** Compile error in `FlywheelKickerIOWB.java`. Needed `(Boolean)` wrapper type.

### PR #356 — Practice bot setup 2

- **`sensorActivated` hardcoded to `true`:** Indexer/kicker always behaved as if a game piece was detected regardless of actual sensor state.

## Logic and Data Bugs

### PR #324 — Shot Calculator Cleanup

- **`MIN_DISTANCE_METERS` and `MAX_DISTANCE_METERS` never assigned, defaulted to 0.0:** Distance checks short-circuited for all positive distances.

### PR #352 — Fixing auto mirroring

- **`() -> false` permanently disabled alliance path flipping:** Blue alliance autos would follow wrong (Red-side) paths.

### PR #348 — FMS to Superstructure

- **Null `hubActive` Boolean silently fell through:** Method could return `null`, and `null == true` evaluates to `false` silently instead of failing explicitly.

### PR #300 — Isolate drivebase from superstructure

- **`isAlignedToTarget()` returns stale results in autonomous:** The method only returns meaningful results while `FieldCentricFacingAngle` is actively applied — not during PathPlanner auto sequences.

### PR #295 — Shoot with ShotCalculator

- **Logging wrong state variable:** A Logger line logged `currentSuperState` but should have logged `currentShooterState`. Fixed in "final code review changes" commit.

## Summary

| Category                                             | Count |
| ---------------------------------------------------- | ----- |
| Missing `break` / fall-through bugs                  | 4     |
| Compile errors (missing braces, keywords, casts)     | 6     |
| Null / crash risks                                   | 3     |
| State machine logic errors                           | 4     |
| Safety / config issues (disabled limits, telemetry)  | 3     |
| Dead code / unassigned fields                        | 3     |
| **Total real bugs caught**                           | **~35+** |

## Most Impactful Catches

1. **Missing `break` statements** (PRs #404, #301, #321) — caused completely wrong runtime behavior on the robot
2. **Infinite recursion** (PR #339) — guaranteed crash at construction
3. **Disabled NT4Publisher** (PR #367) — no telemetry on real hardware
4. **Stator current limit disabled** (PR #301) — safety risk with bang-bang control
5. **Alliance crash** (PR #287) — robot code crash before match starts

## Evidence of Fixes

Multiple PRs have commits explicitly referencing Claude review:

- PR #321: Commit message says "reviewed claude code"
- PR #287: Commit says "Apply suggestion from @claude[bot]" with co-author attribution
- PR #404: Explicit "added break statement" commits after review
- PR #339: Complete restructuring to fix infinite recursion
- PR #367: "Uncommented NT4Publisher" commit directly after review
- PR #295: "final code review changes" commit after review
- PR #392: Changes reverted before scrimmage based on review findings
