---
name: robot-config-parity-check
description: Verify every robot config in RobotContainer.java (SIM, WOODBOT, PRACTICEBOT, COMPBOT) initializes every subsystem field, using Noop IO where hardware is absent. Use when reviewing a PR that touches RobotContainer, adding/removing a subsystem, or when asked to check robot config parity.
argument-hint: No arguments required
---

CLAUDE.md requires: "Every robot configuration in `RobotContainer` must initialize all subsystems. Use Noop IO implementations ... for hardware not present." This eliminates null checks elsewhere. This skill statically verifies that rule holds after a change, rather than trusting it was followed.

## Steps

### 1. Find the subsystem field list

Read `src/main/java/frc/robot/RobotContainer.java`. Find the subsystem field declarations near the top of the class (e.g. `drivetrain`, `intakePivot`, `vision`, `flywheel`, `hood`, `indexer`, `intakeRoller`, `flywheelKicker`, `hopperRoller`, `hopperSensor`) — these are the fields every config's `switch` block must assign. Don't hardcode this list from memory; re-derive it each run since subsystems get added/removed.

### 2. Find the per-config switch block

Locate the `switch (Constants.getRobotType())` block in the `RobotContainer()` constructor. Each `case` (`SIM`, `WOODBOT`, `PRACTICEBOT`, `COMPBOT`/`default`) is one robot config.

### 3. Check assignment coverage per case

For each `case` block, check that every subsystem field from step 1 is assigned exactly once before the `break`. A field assigned via a Noop IO constructor (e.g. `new IntakePivot(new IntakePivotIONoop())` or `new HopperRoller(new HopperRollerIONoop())`) counts as satisfying the rule — that's the intended pattern for absent hardware, not a violation.

Flag:
- **Missing assignment**: a subsystem field never assigned in a case block (would leave it `null`, causing an NPE elsewhere since the rest of the codebase assumes non-null).
- **Fallthrough risk**: a `case` without a `break`/`return` that could unintentionally skip into the next case's assignments.

### 4. Cross-check against actual IO interfaces

For each subsystem, confirm a Noop implementation exists (search for `<Subsystem>IONoop` under `src/main/java/frc/robot/subsystems/`) wherever a config skips real hardware, OR confirm the code uses the interface-default pattern (`new <Subsystem>IO() {}`) per `SubsystemArchitecture.md`. Either is acceptable; a config that leaves the field unassigned is not.

## Output

Report a table: one row per config (`SIM`, `WOODBOT`, `PRACTICEBOT`, `COMPBOT`), one column per subsystem field, marking ✅ (real hardware), ✅ Noop (Noop/interface-default), or ❌ MISSING. If everything passes, say so briefly — don't produce a large table for a clean result, just confirm parity holds.

If any ❌ is found, cite the exact `RobotContainer.java` line range for that `case` block and state which Noop implementation to add (or interface-default pattern to use) to fix it. Do not edit the file automatically — report findings for the user to apply, since this touches every robot config and should be reviewed before changing.
