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
| `Swerve/Using Vision` | `CommandSwerveDrivetrain.java` | Whether vision measurements are being accepted. ⚠ Key contains a space — violates the no-spaces convention below; source has not been modified under this skill. |
| `Swerve/Is Defense Mode` | `CommandSwerveDrivetrain.java` | ⚠ Key contains spaces — same convention violation as above. |
| `Swerve/DynamicHeadingToleranceDeg` | `CommandSwerveDrivetrain.java` | |
| `Swerve/AutoRotationOverride/TargetDeg` | `CommandSwerveDrivetrain.java` | Target heading for PathPlanner rotation override |
| `Swerve/AutoRotationOverride/ErrorDeg` | `CommandSwerveDrivetrain.java` | |
| `Swerve/AutoRotationOverride/OutputRadPerS` | `CommandSwerveDrivetrain.java` | |
| `Swerve/AutoRotationOverride/Active` | `CommandSwerveDrivetrain.java` | Whether the rotation override is currently controlling PathPlanner's heading |
| `Swerve/Pigeon/YawDeg` | `CommandSwerveDrivetrain.java` | |
| `Swerve/Pigeon/PitchDeg` | `CommandSwerveDrivetrain.java` | |
| `Swerve/Pigeon/RollDeg` | `CommandSwerveDrivetrain.java` | |
| `Swerve/Pigeon/AngularVelocityZDegPerSec` | `CommandSwerveDrivetrain.java` | |
| `Swerve/HeadingSetpointDeg` | `CommandSwerveDrivetrain.java` | Facing-angle heading controller setpoint |
| `Swerve/XOutAligning/HasDriverInput` | `XOutWhileAligningCommand.java` | |
| `Swerve/XOutAligning/State` | `XOutWhileAligningCommand.java` | `FACING_ANGLE` or `X_OUT` |

> **Possibly stale:** `Swerve/ActivePath` and `Swerve/TargetPathPose` (previously attributed to `RobotContainer.java`) were not found in source. `CommandSwerveDrivetrain.java` now logs `Autos/PathPlanner/activePath` and `Autos/PathPlanner/targetPose` (see `Autos/` below), which look like the renamed successors — confirm before treating the old keys as fully removed.

---

## `Autos/`

PathPlanner and BLine path-following telemetry, logged from `CommandSwerveDrivetrain.java`.

| Key | Source | Description |
|-----|--------|-------------|
| `Autos/PathPlanner/currentPose` | `CommandSwerveDrivetrain.java` | |
| `Autos/PathPlanner/targetPose` | `CommandSwerveDrivetrain.java` | |
| `Autos/PathPlanner/activePath` | `CommandSwerveDrivetrain.java` | |
| `Autos/BLine/<key>` | `CommandSwerveDrivetrain.java` | One entry per BLine `FollowPath` logging callback (pose/translation-list/boolean); `<key>` is the callback's raw key with a `FollowPath/` prefix stripped, via `blineKey()` |

---

## `PathPlannerAutos/`

| Key | Source | Description |
|-----|--------|-------------|
| `PathPlannerAutos/MissingStartPose` | `AutoChooser.java` | Logged with the auto name as the value when a selected PathPlanner auto has no start pose |
| `PathPlannerAutos/FailedCast` | `AutoChooser.java` | Logged with the auto name as the value when an auto command fails to cast to the expected type |

> Note the `PathPlannerAutos/` (this section) vs `Autos/PathPlanner/` (above) naming — two different top-level segments for PathPlanner-related telemetry from two different files. Not reconciled here since that's a source-code naming decision, not a doc fix.

---

## `BLineAutos/`

| Key | Source | Description |
|-----|--------|-------------|
| `BLineAutos/MissingPaths/<autoName>` | `BLineAutos.java` | One entry per BLine auto that failed to load, keyed by auto name; value is the exception message |

---

## `Auto/`

| Key | Source | Description |
|-----|--------|-------------|
| `Auto/ShootAtHub/LaunchCount` | `RobotContainer.java` | |
| `Auto/ShootAtHub/NoLaunchForTimeout` | `RobotContainer.java` | |
| `Auto/ShootAtHub/HopperEmpty` | `RobotContainer.java` | |

---

## `Vision/`

| Key | Source | Description |
|-----|--------|-------------|
| `Vision/<camera>/` | `Vision.java` | AutoLogged IO inputs per camera (via `Logger.processInputs`) |
| `Vision/snapshot` | `Vision.java` | True when a snapshot is active. ⚠ Lowercase — violates the capitalize-first-letter convention below. |
| `Vision/Total Detections` | `Vision.java` | Cumulative pose updates attempted. ⚠ Contains a space; also no longer nested under a `Stats/` segment. |
| `Vision/Rejected Measurements` | `Vision.java` | Cumulative rejected measurements. ⚠ Contains a space; no `Stats/` segment. |
| `Vision/Rejection Rate` | `Vision.java` | Fraction rejected. ⚠ Contains a space; no `Stats/` segment. |

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

### `Superstructure/` state machines

State machine wanted/current/previous states and their internal logic logs. ⚠ These previously lived under a `Superstructure/StateMachines/<Name>/` prefix; source now logs them directly under `Superstructure/<Name>/` with no `StateMachines/` segment. Paths below reflect current source — confirm the rename is intentional.

#### `Superstructure/Intake/`

| Key | Source |
|-----|--------|
| `Superstructure/Intake/WantedState` | `IntakeStateMachine.java` |
| `Superstructure/Intake/CurrentState` | `IntakeStateMachine.java` |
| `Superstructure/Intake/PreviousState` | `IntakeStateMachine.java` |

> **Possibly stale:** `BallsOverHopperState` was not found anywhere in source (searched the full `src/main/java` tree) — no renamed equivalent found either. Confirm whether this was intentionally removed before deleting the row.

#### `Superstructure/Shooter/`

| Key | Source |
|-----|--------|
| `Superstructure/Shooter/WantedState` | `ShooterStateMachine.java` |
| `Superstructure/Shooter/CurrentState` | `ShooterStateMachine.java` |
| `Superstructure/Shooter/PreviousState` | `ShooterStateMachine.java` |
| `Superstructure/Shooter/Shooting/FlywheelReady` | `ShooterStateMachine.java` |
| `Superstructure/Shooter/Shooting/HoodReady` | `ShooterStateMachine.java` |
| `Superstructure/Shooter/Shooting/DrivetrainAligned` | `ShooterStateMachine.java` |
| `Superstructure/Shooter/Shooting/TargetReady` | `ShooterStateMachine.java` |
| `Superstructure/Shooter/Shooting/InBangBang` | `ShooterStateMachine.java` |
| `Superstructure/Shooter/Shooting/SubsystemsReady` | `ShooterStateMachine.java` |
| `Superstructure/Shooter/Shooting/ShouldFire` | `ShooterStateMachine.java` |
| `Superstructure/Shooter/Shooting/DisturbanceActive` | `ShooterStateMachine.java` |
| `Superstructure/Shooter/Shooting/DisturbanceTimedOut` | `ShooterStateMachine.java` |
| `Superstructure/Shooter/Shooting/DisturbanceWindowSec` | `ShooterStateMachine.java` |

#### `Superstructure/TargetSelection/`

| Key | Source |
|-----|--------|
| `Superstructure/TargetSelection/WantedState` | `TargetSelectionStateMachine.java` |
| `Superstructure/TargetSelection/CurrentState` | `TargetSelectionStateMachine.java` |
| `Superstructure/TargetSelection/PreviousState` | `TargetSelectionStateMachine.java` |

### `Superstructure/Subsystems/`

Hardware subsystem state. Each subsystem logs its own wanted/current/previous states plus hardware-specific outputs.

| Key | Source |
|-----|--------|
| `Superstructure/Subsystems/Indexer/WantedState` | `Indexer.java` |
| `Superstructure/Subsystems/Indexer/CurrentState` | `Indexer.java` |
| `Superstructure/Subsystems/Indexer/PreviousState` | `Indexer.java` |
| `Superstructure/Subsystems/Indexer/ControlState` | `Indexer.java` |
| `Superstructure/Subsystems/HopperRoller/WantedState` | `HopperRoller.java` |
| `Superstructure/Subsystems/HopperRoller/CurrentState` | `HopperRoller.java` |
| `Superstructure/Subsystems/HopperRoller/PreviousState` | `HopperRoller.java` |
| `Superstructure/Subsystems/HopperRoller/ControlState` | `HopperRoller.java` |
| `Superstructure/Subsystems/HopperSensor/WantedState` | `HopperSensor.java` |
| `Superstructure/Subsystems/HopperSensor/CurrentState` | `HopperSensor.java` |
| `Superstructure/Subsystems/HopperSensor/PreviousState` | `HopperSensor.java` |
| `Superstructure/Subsystems/HopperSensor/Connected` | `HopperSensor.java` |
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
| `Superstructure/Subsystems/IntakeRoller/UnjamAttempts` | `IntakeRoller.java` |
| `Superstructure/Subsystems/IntakeRoller/IsJammed` | `IntakeRoller.java` |
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

> **Note:** AutoLogged IO inputs for each subsystem are logged via `Logger.processInputs("Subsystems/<Name>", inputs)` — they appear under `RealInputs/Subsystems/` in AdvantageScope, separate from the `RealOutputs/Superstructure/Subsystems/` state outputs by design.
>
> **Exception:** `Indexer` does not call `Logger.processInputs` itself — it inherits `StateMachineSubsystem`'s base `periodic()`, which calls `Logger.processInputs(logKey, inputs)` where `logKey = getClass().getSimpleName()`. For `Indexer` this resolves to plain `"Indexer"`, not `"Subsystems/Indexer"`, so its AutoLogged inputs appear under `RealInputs/Indexer` rather than `RealInputs/Subsystems/Indexer`. `Indexer` is currently the only subclass of `StateMachineSubsystem`.
>
> **Possibly stale:** `Superstructure/Subsystems/HopperSensor/SensorActivatedDebounced` and `.../PreviousDebouncedSensorActivated` were not found in source — `HopperSensor.java` now logs the standard `WantedState`/`CurrentState`/`PreviousState`/`Connected` set shown above instead. Confirm before treating the old two as removed rather than renamed.

---

## `ShotCalculator/`

⚠ Previously documented as nested under `Superstructure/StateMachines/Shooter/ShotCalculator/<name>/`. Source builds `basePath = "ShotCalculator/" + name` — this is a top-level namespace, not nested under `Superstructure/`. One folder per `ShotCalculator` instance; both current instances are constructed with static name literals (`RobotContainer.java`): `HubShotCalc` and `PassCalc`.

| Key | Description |
|-----|-------------|
| `ShotCalculator/<name>/cached` | Whether a cached result was returned |
| `ShotCalculator/<name>/virtualTarget` | Velocity-compensated aiming point (Pose2d) |
| `ShotCalculator/<name>/targetPosition` | Actual target position (Pose2d) |
| `ShotCalculator/<name>/hubPosition` | Hub center point |
| `ShotCalculator/<name>/distanceToTarget` | Lookahead distance in meters |
| `ShotCalculator/<name>/targetFlywheelSpeed` | Commanded flywheel speed |
| `ShotCalculator/<name>/targetHoodAngle` | Commanded hood angle |
| `ShotCalculator/<name>/targetHeading` | Commanded drivebase heading |
| `ShotCalculator/<name>/lookaheadPose` | Shooter lookahead position (Pose2d) |
| `ShotCalculator/<name>/robotCenterLookahead` | Robot center lookahead (Pose2d) |
| `ShotCalculator/<name>/timeOfFlightSecs` | Computed time of flight |
| `ShotCalculator/<name>/isValid` | Whether shot distance is within range |
| `ShotCalculator/<name>/robotSpeeds` | Robot chassis speeds at calculation time |
| `ShotCalculator/<name>/currentHeadingDeg` | Current robot heading at calculation time |
| `ShotCalculator/<name>/headingErrorDeg` | Signed heading error to target, wrapped to ±180° |

---

## `Utils/`

| Key | Source | Description |
|-----|--------|-------------|
| `Utils/HubShift/Phase` | `HubShiftTracker.java` | Current hub shift phase |
| `Utils/HubShift/TimeLeftInPhase` | `HubShiftTracker.java` | |
| `Utils/HubShift/AutoWinner` | `HubShiftTracker.java` | |
| `Utils/PositionUtils/IsInDuckZone` | `PositionUtils.java` | |
| `Utils/PositionUtils/IsInAllianceZone` | `PositionUtils.java` | |
| `Utils/PositionUtils/IsInOppAllianceZone` | `PositionUtils.java` | |
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
- **State machines go directly under `Superstructure/<Name>/`**, hardware subsystems go under `Superstructure/Subsystems/`.
- **ShotCalculator instances** are top-level, not nested under `Superstructure/`: `ShotCalculator/<name>/`.
- **Drivetrain-coupled commands** (like `XOutWhileAligningCommand`) log under `Swerve/` since their output is drivetrain behavior, not a generic command.
