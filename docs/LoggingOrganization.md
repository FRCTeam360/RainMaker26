# AdvantageKit Logging Organization

All `Logger.recordOutput()` keys follow this folder hierarchy. Use it as a reference when adding new logs.

---

## `Robot/`

| Key | Source | Description |
|-----|--------|-------------|
| `Robot/LoopTiming/PreSchedulerSeconds` | `Robot.java` | Time before scheduler runs |
| `Robot/LoopTiming/SchedulerSeconds` | `Robot.java` | Time scheduler takes |
| `Robot/LoopTiming/PostSchedulerSeconds` | `Robot.java` | Time after scheduler |
| `Robot/LoopTiming/LoopTimeSeconds` | `RobotContainer.java` | Total loop time |
| `Robot/LoopTiming/Overrun` | `RobotContainer.java` | Whether loop overran |
| `Robot/LoopTiming/OverrunCount` | `RobotContainer.java` | Cumulative overrun count |
| `Robot/AutoInitTiming/OnEnableSeconds` | `Robot.java` | |
| `Robot/AutoInitTiming/GetAutoCommandSeconds` | `Robot.java` | |
| `Robot/AutoInitTiming/ScheduleSeconds` | `Robot.java` | |
| `Robot/AutoInitTiming/TotalSeconds` | `Robot.java` | |
| `Robot/AutoWinner` | `Robot.java` | Winning auto name |

---

## `Swerve/`

| Key | Source | Description |
|-----|--------|-------------|
| `Swerve/CurrentPose` | `CommandSwerveDrivetrain.java` | |
| `Swerve/Rotation2d` | `CommandSwerveDrivetrain.java` | Raw heading |
| `Swerve/CurrentState` | `CommandSwerveDrivetrain.java` | Module states |
| `Swerve/TargetState` | `CommandSwerveDrivetrain.java` | Module targets |
| `Swerve/UsingVision` | `CommandSwerveDrivetrain.java` | Whether vision measurements are being accepted |
| `Swerve/IsDefenseMode` | `CommandSwerveDrivetrain.java` | |
| `Swerve/DynamicHeadingToleranceDeg` | `CommandSwerveDrivetrain.java` | |
| `Swerve/ActivePath` | `RobotContainer.java` | Current PathPlanner path |
| `Swerve/TargetPathPose` | `RobotContainer.java` | |
| `Swerve/XOutAligning/HasDriverInput` | `XOutWhileAligningCommand.java` | |
| `Swerve/XOutAligning/State` | `XOutWhileAligningCommand.java` | `FACING_ANGLE` or `X_OUT` |

---

## `Vision/`

| Key | Source | Description |
|-----|--------|-------------|
| `Vision/<camera>/` | `Vision.java` | AutoLogged IO inputs per camera (via `Logger.processInputs`) |
| `Vision/Snapshot` | `Vision.java` | True when a snapshot is active |
| `Vision/Stats/TotalDetections` | `Vision.java` | Cumulative pose updates attempted |
| `Vision/Stats/RejectedMeasurements` | `Vision.java` | Cumulative rejected measurements |
| `Vision/Stats/RejectionRate` | `Vision.java` | Fraction rejected |

---

## `Superstructure/`

Top-level superstructure state lives directly here.

| Key | Source | Description |
|-----|--------|-------------|
| `Superstructure/WantedSuperState` | `SuperStructure.java` | |
| `Superstructure/CurrentSuperState` | `SuperStructure.java` | |
| `Superstructure/PreviousSuperState` | `SuperStructure.java` | |
| `Superstructure/ControlState` | `SuperStructure.java` | |
| `Superstructure/HubActive` | `SuperStructure.java` | |

### `Superstructure/StateMachines/`

State machine wanted/current/previous states and their internal logic logs.

#### `Superstructure/StateMachines/Intake/`

| Key | Source |
|-----|--------|
| `Superstructure/StateMachines/Intake/WantedState` | `IntakeStateMachine.java` |
| `Superstructure/StateMachines/Intake/CurrentState` | `IntakeStateMachine.java` |
| `Superstructure/StateMachines/Intake/PreviousState` | `IntakeStateMachine.java` |
| `Superstructure/StateMachines/Intake/BallsOverHopperState` | `IntakeStateMachine.java` |

#### `Superstructure/StateMachines/Shooter/`

| Key | Source |
|-----|--------|
| `Superstructure/StateMachines/Shooter/WantedState` | `ShooterStateMachine.java` |
| `Superstructure/StateMachines/Shooter/CurrentState` | `ShooterStateMachine.java` |
| `Superstructure/StateMachines/Shooter/PreviousState` | `ShooterStateMachine.java` |
| `Superstructure/StateMachines/Shooter/Shooting/FlywheelState` | `ShooterStateMachine.java` |
| `Superstructure/StateMachines/Shooter/Shooting/FlywheelReady` | `ShooterStateMachine.java` |
| `Superstructure/StateMachines/Shooter/Shooting/HoodReady` | `ShooterStateMachine.java` |
| `Superstructure/StateMachines/Shooter/Shooting/DrivetrainAligned` | `ShooterStateMachine.java` |
| `Superstructure/StateMachines/Shooter/Shooting/TargetReady` | `ShooterStateMachine.java` |
| `Superstructure/StateMachines/Shooter/Shooting/InBangBang` | `ShooterStateMachine.java` |
| `Superstructure/StateMachines/Shooter/Shooting/SubsystemsReady` | `ShooterStateMachine.java` |
| `Superstructure/StateMachines/Shooter/Shooting/ShouldFire` | `ShooterStateMachine.java` |
| `Superstructure/StateMachines/Shooter/Shooting/DisturbanceActive` | `ShooterStateMachine.java` |
| `Superstructure/StateMachines/Shooter/Shooting/DisturbanceTimedOut` | `ShooterStateMachine.java` |
| `Superstructure/StateMachines/Shooter/Shooting/DisturbanceWindowSec` | `ShooterStateMachine.java` |

#### `Superstructure/StateMachines/Shooter/ShotCalculator/<name>/`

One folder per `ShotCalculator` instance (keyed by `name` constructor argument).

| Key | Description |
|-----|-------------|
| `.../cached` | Whether a cached result was returned |
| `.../virtualTarget` | Velocity-compensated aiming point (Pose2d) |
| `.../targetPosition` | Actual target position (Pose2d) |
| `.../hubPosition` | Hub center point |
| `.../distanceToTarget` | Lookahead distance in meters |
| `.../targetFlywheelSpeed` | Commanded flywheel speed |
| `.../targetHoodAngle` | Commanded hood angle |
| `.../targetHeading` | Commanded drivebase heading |
| `.../lookaheadPose` | Shooter lookahead position (Pose2d) |
| `.../robotCenterLookahead` | Robot center lookahead (Pose2d) |
| `.../timeOfFlightSecs` | Computed time of flight |
| `.../isValid` | Whether shot distance is within range |
| `.../robotSpeeds` | Robot chassis speeds at calculation time |

#### `Superstructure/StateMachines/TargetSelection/`

| Key | Source |
|-----|--------|
| `Superstructure/StateMachines/TargetSelection/WantedState` | `TargetSelectionStateMachine.java` |
| `Superstructure/StateMachines/TargetSelection/CurrentState` | `TargetSelectionStateMachine.java` |
| `Superstructure/StateMachines/TargetSelection/PreviousState` | `TargetSelectionStateMachine.java` |

### `Superstructure/Subsystems/`

Hardware subsystem state. Each subsystem logs its own wanted/current/previous states plus hardware-specific outputs.

| Key | Source |
|-----|--------|
| `Superstructure/Subsystems/Climber/WantedState` | `Climber.java` |
| `Superstructure/Subsystems/Climber/CurrentState` | `Climber.java` |
| `Superstructure/Subsystems/Climber/PreviousState` | `Climber.java` |
| `Superstructure/Subsystems/Indexer/WantedState` | `Indexer.java` |
| `Superstructure/Subsystems/Indexer/CurrentState` | `Indexer.java` |
| `Superstructure/Subsystems/Indexer/PreviousState` | `Indexer.java` |
| `Superstructure/Subsystems/Indexer/ControlState` | `Indexer.java` |
| `Superstructure/Subsystems/HopperRoller/WantedState` | `HopperRoller.java` |
| `Superstructure/Subsystems/HopperRoller/CurrentState` | `HopperRoller.java` |
| `Superstructure/Subsystems/HopperRoller/PreviousState` | `HopperRoller.java` |
| `Superstructure/Subsystems/HopperRoller/ControlState` | `HopperRoller.java` |
| `Superstructure/Subsystems/HopperSensor/SensorActivatedDebounced` | `HopperSensor.java` |
| `Superstructure/Subsystems/HopperSensor/PreviousDebouncedSensorActivated` | `HopperSensor.java` |
| `Superstructure/Subsystems/IntakePivot/WantedState` | `IntakePivot.java` |
| `Superstructure/Subsystems/IntakePivot/CurrentState` | `IntakePivot.java` |
| `Superstructure/Subsystems/IntakePivot/PreviousState` | `IntakePivot.java` |
| `Superstructure/Subsystems/IntakePivot/ControlState` | `IntakePivot.java` |
| `Superstructure/Subsystems/IntakePivot/TargetPositionDegrees` | `IntakePivot.java` |
| `Superstructure/Subsystems/IntakePivot/Mechanism2d` | `IntakePivotVisualizer.java` |
| `Superstructure/Subsystems/IntakeRoller/WantedState` | `IntakeRoller.java` |
| `Superstructure/Subsystems/IntakeRoller/CurrentState` | `IntakeRoller.java` |
| `Superstructure/Subsystems/IntakeRoller/PreviousState` | `IntakeRoller.java` |
| `Superstructure/Subsystems/IntakeRoller/ControlState` | `IntakeRoller.java` |
| `Superstructure/Subsystems/Flywheel/WantedState` | `Flywheel.java` |
| `Superstructure/Subsystems/Flywheel/CurrentState` | `Flywheel.java` |
| `Superstructure/Subsystems/Flywheel/PreviousState` | `Flywheel.java` |
| `Superstructure/Subsystems/Flywheel/ControlState` | `Flywheel.java` |
| `Superstructure/Subsystems/Flywheel/LaunchCount` | `Flywheel.java` |
| `Superstructure/Subsystems/Hood/WantedState` | `Hood.java` |
| `Superstructure/Subsystems/Hood/CurrentState` | `Hood.java` |
| `Superstructure/Subsystems/Hood/PreviousState` | `Hood.java` |
| `Superstructure/Subsystems/Hood/ControlState` | `Hood.java` |
| `Superstructure/Subsystems/Hood/IsHoodUp` | `Hood.java` |
| `Superstructure/Subsystems/Hood/Mechanism2d` | `HoodVisualizer.java` |
| `Superstructure/Subsystems/FlywheelKicker/WantedState` | `FlywheelKicker.java` |
| `Superstructure/Subsystems/FlywheelKicker/CurrentState` | `FlywheelKicker.java` |
| `Superstructure/Subsystems/FlywheelKicker/PreviousState` | `FlywheelKicker.java` |
| `Superstructure/Subsystems/FlywheelKicker/ControlState` | `FlywheelKicker.java` |
| `Superstructure/Subsystems/FlywheelKicker/KickCount` | `FlywheelKicker.java` |

> **Note:** AutoLogged IO inputs for each subsystem are logged via `Logger.processInputs("Superstructure/Subsystems/<Name>", inputs)` — those fields appear under the same folder automatically.

---

## `Utils/`

| Key | Source | Description |
|-----|--------|-------------|
| `Utils/HubShift/Phase` | `HubShiftTracker.java` | Current hub shift phase |
| `Utils/HubShift/TimeLeftInPhase` | `HubShiftTracker.java` | |
| `Utils/HubShift/AutoWinner` | `HubShiftTracker.java` | |
| `Utils/PositionUtils/IsInDuckZone` | `PositionUtils.java` | |
| `Utils/PositionUtils/IsInAllianceZone` | `PositionUtils.java` | |
| `Utils/PositionUtils/CloserPassTarget` | `PositionUtils.java` | |

---

## `Commands/`

| Key | Source | Description |
|-----|--------|-------------|
| `Commands/<CommandName>/Running` | `CommandLogger.java` | True while command is scheduled |

---

## Conventions

- **No spaces in keys.** Use `CamelCase` for multi-word segments (e.g., `UsingVision`, not `Using Vision`).
- **Capitalize first letter** of each path segment (e.g., `Snapshot`, not `snapshot`).
- **Units in the key name** where the value has physical units (e.g., `TargetPositionDegrees`, `TimeOfFlightSecs`, `DistanceToTargetMeters`).
- **State machines go under `Superstructure/StateMachines/`**, hardware subsystems go under `Superstructure/Subsystems/`.
- **ShotCalculator instances** are nested under the state machine that owns them: `Superstructure/StateMachines/Shooter/ShotCalculator/<name>/`.
- **Drivetrain-coupled commands** (like `XOutWhileAligningCommand`) log under `Swerve/` since their output is drivetrain behavior, not a generic command.
