# Copilot Instructions

## Project Overview

This is the robot code for RainMaker26 — FRC 360's 26th robot for the 2026 season, REBUILT. Java 17 project using WPILib, AdvantageKit, CTRE Phoenix 6, REVLib, and PathPlanner with Gradle (GradleRIO).

## Tech Stack & References

- **WPILib 2026**: [Javadoc](https://github.wpilib.org/allwpilib/docs/release/java/index.html) · [Docs](https://docs.wpilib.org/en/stable/)
- **CTRE Phoenix 6 26.1.0**: [Javadoc](https://api.ctr-electronics.com/phoenix6/stable/java/index.html) · [Docs](https://v6.docs.ctr-electronics.com/en/stable/)
- **REVLib 2026.0.1**: [Javadoc](https://codedocs.revrobotics.com/java/com/revrobotics/package-summary.html) · [Docs](https://docs.revrobotics.com/revlib)
- **PathplannerLib 2026.1.2**: [Javadoc](https://pathplanner.dev/api/java/) · [Docs](https://pathplanner.dev/home.html)
- **AdvantageKit 26.0.0**: [Javadoc](https://docs.advantagekit.org/javadoc/) · [Docs](https://docs.advantagekit.org/)

## Project Structure

```
src/main/java/frc/robot/
├── Robot.java              # Main robot class
├── RobotContainer.java     # Subsystem and command bindings
├── Constants.java          # Robot-wide constants (CAN IDs, ports, tuning values)
├── subsystems/             # Hardware abstraction (drivetrain, intake, etc.)
└── commands/               # Command-based actions
```

## Build & Test Commands

- Build: `./gradlew build`
- Test: `./gradlew test`
- Deploy to robot: `./gradlew deploy`
- Simulate: `./gradlew simulateJava`
- Format check: `./gradlew spotlessCheck`
- Format apply: `./gradlew spotlessApply`
- Static analysis: `./gradlew spotbugsMain`

## WPILib Command Composition Semantics

Always use these exact semantics — do not guess:

- `alongWith()` → `ParallelCommandGroup` — ends when **ALL** commands finish
- `raceWith()` → `ParallelRaceGroup` — ends when **ANY** command finishes
- `deadlineFor()` → `ParallelDeadlineGroup` — ends when the **deadline** (calling) command finishes, interrupts all others
- `andThen()` → `SequentialCommandGroup` — runs in order, ends when last finishes

### Command lifecycle types

- `InstantCommand` / `runOnce()` — runs once, finishes immediately
- `RunCommand` / `run()` — runs every cycle, **never finishes on its own**
- `runEnd()` — runs every cycle with an end action, **never finishes on its own**
- `startEnd()` — runs a start action on init; runs an end action when interrupted, **never finishes on its own**

### Common pitfalls

- PathPlanner NamedCommands must terminate — an infinite command (e.g., `run()`) will stall the entire auto sequence.
- `FieldCentricFacingAngle.HeadingController.atSetpoint()` returns stale/invalid results when the facing-angle request is not actively being applied; ensure it's being reset when called.
- CTRE `StatusSignal` values must be refreshed before reading; stale signals return old data silently.

## Code Patterns

- Use Command-based programming: superstructure owns non-drivetrain subsystems which own hardware, commands define actions.
- Superstructure state transition commands are fire and forget.
- Subsystems are controlled by the superstructure through setting wanted states and can be unhooked and run by commands through a separate state for testing.
- **Infrastructure constants** (CAN IDs, sensor ports, physical hardware config) always go in `Constants.java`.
- **Tuning constants** (PID gains, setpoints, tolerances, speeds) should be named `private static final` variables in the file where they're used.
- Avoid unnamed literals — give values descriptive names **with units** (e.g., `MAX_VELOCITY_MPS`, `STALL_CURRENT_AMPS`, `TIMEOUT_SECONDS`).
- Use AdvantageKit's `@AutoLog` on IO input classes for hardware telemetry.
- Use `Logger.recordOutput()` for all other values, grouping by subsystem, class, or state type.
- Subsystems extend `SubsystemBase` and have IO layers with specific hardware implementations following FRC 6328's architecture.
- Every robot configuration in `RobotContainer` must initialize all subsystems. Use Noop IO implementations (e.g., `IntakePivotIONoop`) for hardware not present on a given robot.

## Naming Conventions

### States (Enum Values) — `ALL_CAPS` with underscores

- Present participles for actions: `INTAKING`, `SHOOTING`, `SPINNING_UP`
- Adjectives/nouns for conditions: `IDLE`, `READY_TO_FIRE`, `AT_SETPOINT`
- Compound states: `SPINUP_SHOOTING`, `AUTO_ALIGN`, `X_OUT_SHOOTING`
- Always spell out words fully (no "2" for "TO", "4" for "FOR")

### Classes

- **Subsystems** — PascalCase nouns, no "Subsystem" suffix: `Intake`, `Flywheel`, `Hood`
- **IO interfaces** — `<Subsystem>IO`: `IntakeIO`, `FlywheelIO`
- **IO implementations** — `<Subsystem>IO<Type>`: `IntakeIOTalonFX`, `IntakeIOSim`, `IntakeIONoop`
- **Command classes** — PascalCase with `Command` suffix: `FlywheelTuneCommand`, `AutoAlignCommand`
- **Utility classes** — Purpose + type suffix: `ShotCalculator`, `AllianceFlipUtil`

### Methods

- Robot action methods — camelCase, verb-first: `shootWithSpinup()`, `intakeUntilNote()`
- Configuration methods — set-prefix: `setHoodPosition()`, `setFlywheelSpeed()`

## Code Quality Rules

- All loops must have clear termination conditions (no unbounded while loops).
- No recursion (stack overflow risk in real-time systems).
- No dynamic allocation in periodic/command execution (only in initialization).
- Constants must be `final` and immutable where possible.
- Null checks present for all hardware objects before use.
- Commands must have clear `isFinished()` conditions or be bound/sequenced to stop when instructed.
- Sensor values must be validated/clamped before use.
- Units must be documented in variable names or comments (meters, radians, seconds).
- One subsystem per hardware device (clear resource ownership).
- Add Javadoc with `@param` and `@return` tags to public methods.

## PR Review Guidelines

When reviewing a PR, focus on breaking changes only — not nitpicks:

- Only review lines that are part of the PR diff, not pre-existing issues.
- Flag behavioral changes (command lifecycle changes, removed cleanup/stop actions, changed motor inversions).
- Flag cross-config impact on shared code (SuperStructure, RobotContainer, command factories).
- Flag hardcoded sensor overrides that gate state transitions or safety logic.
- Flag CAN ID conflicts, disabled functionality, and API signature changes.
- Do not flag style, formatting, naming preferences, missing Javadoc, or code organization choices unless they cause a breaking change.
- CI merge blockers: Build, Test, Format Check, and Simulation Test only.
