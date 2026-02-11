# AGENTS.md

## Project Overview

This is the robot code for RainMaker26 FRC 360's 26th robot for the 2026 season, REBUILT. It is a Java-based project using WPILib for FIRST Robotics Competition.

## Tech Stack

- Language: Java 17
- Build tool: Gradle (with GradleRIO plugin)
- Frameworks/Libraries: WPILib, AdvantageKit, CTRE Phoenix, PathPlanner

## Vendor Dependencies & Javadocs

| Library        | Version  | Javadoc                                                                               | User Guide                                             |
| -------------- | -------- | ------------------------------------------------------------------------------------- | ------------------------------------------------------ |
| WPILib         | 2026     | [Javadoc](https://github.wpilib.org/allwpilib/docs/release/java/index.html)           | [Docs](https://docs.wpilib.org/en/stable/)             |
| CTRE Phoenix 6 | 26.1.0   | [Javadoc](https://api.ctr-electronics.com/phoenix6/stable/java/index.html)            | [Docs](https://v6.docs.ctr-electronics.com/en/stable/) |
| REVLib         | 2026.0.1 | [Javadoc](https://codedocs.revrobotics.com/java/com/revrobotics/package-summary.html) | [Docs](https://docs.revrobotics.com/revlib)            |
| PathplannerLib | 2026.1.2 | [Javadoc](https://pathplanner.dev/api/java/)                                          | [Docs](https://pathplanner.dev/home.html)              |
| AdvantageKit   | 26.0.0   | [Javadoc](https://docs.advantagekit.org/javadoc/)                                     | [Docs](https://docs.advantagekit.org/)                 |

## Project Structure

```
src/main/java/frc/robot/
├── Robot.java              # Main robot class
├── RobotContainer.java     # Subsystem and command bindings
├── Constants.java          # Robot-wide constants (CAN IDs, ports, tuning values)
├── subsystems/             # Hardware abstraction (drivetrain, intake, etc.)
└── commands/               # Command-based actions
```

## Setup Commands

# Prerequisites: Install WPILib (https://docs.wpilib.org/en/stable/docs/getting-started/getting-started-frc-control-system/wpilib-setup.html)

# Build the project

./gradlew build

# Deploy to robot (when connected to robot network)

./gradlew deploy

# Simulate robot code locally

./gradlew simulateJava

## Testing

Run tests with: ./gradlew test
Always run tests before committing.

## Formatting

Check formatting with: ./gradlew spotlessCheck
Apply formatting with: ./gradlew spotlessApply

## Static Analysis

Run SpotBugs to find potential bugs: ./gradlew spotbugsMain
View the report: open build/reports/spotbugs/spotbugs.html

## Code Patterns

- Use Command-based programming: subsystems own hardware, commands define actions
- **Infrastructure constants** (CAN IDs, sensor ports, physical hardware config) always go in Constants.java
- **Tuning constants** (PID gains, setpoints, tolerances, speeds) should be named `private static final` variables in the file where they're used
- Avoid unnamed literals - give values descriptive names **with units** (e.g., `MAX_VELOCITY_MPS`, `STALL_CURRENT_AMPS`, `TIMEOUT_SECONDS`)
- Use AdvantageKit's @AutoLogOutput for telemetry on important values
- Subsystems extend SubsystemBase; commands extend Command or use robot action methods
- Subsystems have IO layers with specific hardware implementations (other than the superstructure) following FRC 6328's architecture

## Naming Conventions

### States (Enum Values)

Use `ALL_CAPS` with underscores for separation:

- **Present participles** for action states: `INTAKING`, `SHOOTING`, `SPINNING_UP`, `EJECTING`, `AIMING`
- **Adjectives/nouns** for condition states: `IDLE`, `READY_TO_FIRE`, `AT_SETPOINT`, `DISABLED`
- **Compound states**: Be descriptive: `SPINUP_SHOOTING`, `AUTO_ALIGN`, `X_OUT_SHOOTING`
- Always spell out words fully (no "2" for "TO", "4" for "FOR")
- Check spelling before committing (`PREPARING` not `PREPAREING`, `ALIGN` not `ALIGHN`)

### Commands

**Command classes** - PascalCase with descriptive `Command` suffix:

- `FlywheelTuneCommand`, `DriveToPositionCommand`, `AutoAlignCommand`, `ShootAtTargetCommand`

**Robot Action methods** - camelCase, verb-first:

- `shootWithSpinup()`, `intakeUntilNote()`, `alignToTarget()`, `driveToPosition()`
- Set-prefix for configuration: `setHoodPosition()`, `setFlywheelSpeed()`
- Avoid "Cmd" abbreviation - use full "Command" suffix or drop it entirely

### Classes

**Subsystems** - PascalCase nouns (no "Subsystem" suffix):

- `Intake`, `Flywheel`, `Hood`, `Indexer`, `IntakePivot`, `SuperStructure`

**IO interfaces** - `<Subsystem>IO` pattern:

- `IntakeIO`, `FlywheelIO`, `HoodIO`, `IndexerIO`

**IO implementations** - `<Subsystem>IO<Type>` pattern:

- `IntakeIOSim`, `IntakeIOTalonFX`, `FlywheelIOSparkMax`, `HoodIOSim`

**Utility classes** - Descriptive purpose + type suffix:

- `ShotCalculator`, `FieldVisualizer`, `CommandLogger`, `AllianceFlipUtil`

## Code Review Checklist

When reviewing PRs, check for the following (inspired by NASA's "Power of 10" for safety-critical code). **Only flag items that are violated — do not mention items that pass. Keep reviews focused on the 2-3 most important issues rather than enumerating every check.**

### Static Analysis & Safety

- [ ] All loops have clear termination conditions (no unbounded while loops)
- [ ] No recursion (stack overflow risk in real-time systems)
- [ ] Functions are <60 lines (split complex logic into helper methods)
- [ ] No unused private methods or fields (run IDE inspections)
- [ ] All return values are checked or explicitly ignored with comment
- [ ] No dynamic allocation in periodic/command execution (only in initialization)
- [ ] Constants are final and immutable where possible
- [ ] Null checks present for all hardware objects before use

### Code Clarity

- [ ] Variable scope minimized (declare variables close to usage)
- [ ] Infrastructure constants (CAN IDs, ports) in Constants.java; tuning values (setpoints, gains) as named constants in their usage file
- [ ] No unnamed numeric literals - all values have descriptive names with units (e.g., `MAX_VELOCITY_MPS`, `STALL_CURRENT_AMPS`, `SPEAKER_RPM`)
- [ ] Method names clearly describe what they do (verb-noun pattern)
- [ ] Complex boolean expressions extracted to named variables
- [ ] Public API has Javadoc with @param and @return tags. Suggest the ([JavaDoc guide](https://www.baeldung.com/javadoc)) when relevant.
- [ ] Enum states follow naming conventions (present participles for actions, adjectives for conditions, no typos or "2" for "TO")
- [ ] Command classes use PascalCase with `Command` suffix; robot action methods are verb-first camelCase
- [ ] Class names follow conventions (subsystems without "Subsystem" suffix, IO pattern for interfaces/implementations)

### Robotics-Specific

- [ ] Commands have clear isFinished() conditions or are bound/sequenced in a way that they stop when instructed (no infinite commands without interruption)
- [ ] Sensor values validated/clamped before use
- [ ] Units documented in variable names or comments (meters, radians, etc.)
- [ ] Resource ownership clear (one subsystem per hardware device)

## Do

- Write clear, commented code
- Add Javadoc comments to public methods
- Keep files small and focused
- Log important values with AdvantageKit for debugging
- Use units consistently (meters, radians, seconds)

## Don't

- Hard-code CAN IDs or port numbers (use Constants.java)
- Use unnamed numeric literals - always use named constants with descriptive names
- Skip null checks on hardware initialization in RobotContainer.java
- Delete existing tests
- Modify vendor dependencies without team discussion

## Ask First

- Before adding new dependencies
- Before restructuring folders
- Before changing CAN IDs or motor configurations
- Before modifying autonomous routines at all
