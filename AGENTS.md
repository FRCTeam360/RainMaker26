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

## Code Patterns

- Use Command-based programming: subsystems own hardware, commands define actions
- Constants go in Constants.java, organized by subsystem inner classes
- CAN IDs and port numbers are never magic numbers - always reference Constants
- Use AdvantageKit's @AutoLogOutput for telemetry on important values
- Subsystems extend SubsystemBase; commands extend Command or use factory methods
- Subsystems have IO layers with specific hardware implementations (other than the superstructure) following FRC 6328's architecture

## Do

- Write clear, commented code
- Add Javadoc comments to public methods
- Keep files small and focused
- Log important values with AdvantageKit for debugging
- Use units consistently (meters, radians, seconds)

## Don't

- Hard-code CAN IDs or port numbers (use Constants.java)
- Skip null checks on hardware initialization in RobotContainer.java
- Delete existing tests
- Modify vendor dependencies without team discussion

## Ask First

- Before adding new dependencies
- Before restructuring folders
- Before changing CAN IDs or motor configurations
- Before modifying autonomous routines at all
