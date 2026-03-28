# Lakitu Recovery System — Implementation Guide

This document describes the code changes needed to integrate the Lakitu recovery system into the robot. For the design rationale and methodology, see [lakitu-recovery-system.md](lakitu-recovery-system.md).

## Files Changed

| File | Change Type | Purpose |
|---|---|---|
| `commands/LakituFollowPathCommand.java` | **New** | Core recovery state machine |
| `subsystems/CommandSwerveDrivetrain.java` | **Modified** | AutoBuilder wiring, target pose plumbing, sim stall injection |
| `RobotContainer.java` | **Modified** | Target pose callback, path registration |

---

## 1. LakituFollowPathCommand.java (New File)

**Location:** `src/main/java/frc/robot/commands/LakituFollowPathCommand.java`

This is the core of the system — a `Command` that wraps a `FollowPathCommand` with deviation monitoring and recovery logic.

### Path Registry

A static `Set<String>` holds the names of paths that should get Lakitu behavior. Paths not in the set are followed normally.

```java
private static final Set<String> registeredPaths = new HashSet<>();

public static void registerPath(String pathName) {
  registeredPaths.add(pathName);
}

public static boolean isRegistered(String pathName) {
  return registeredPaths.contains(pathName);
}
```

Registration happens at startup (in `RobotContainer`). The path name must match the `.path` filename exactly (e.g., `"Lakitu Path 1"`).

### Constants

```java
private static final double RECOVERY_THRESHOLD_METERS = 1.5;
private static final int MAX_RECOVERY_ATTEMPTS = 2;
private static final double RECOVERY_MAX_VELOCITY_MPS = 3.0;
private static final double RECOVERY_MAX_ACCELERATION_MPSSQ = 6.0;
private static final double RECOVERY_MAX_ANGULAR_VELOCITY_RAD_PER_SEC = Math.toRadians(360);
private static final double RECOVERY_MAX_ANGULAR_ACCELERATION_RAD_PER_SEC_SQ = Math.toRadians(540);
private static final double END_TOLERANCE_METERS = 0.5;
```

### Constructor

```java
public LakituFollowPathCommand(
    PathPlannerPath path,
    Function<PathPlannerPath, Command> commandBuilder,
    Supplier<Pose2d> poseSupplier,
    Supplier<Pose2d> targetPoseSupplier,
    Subsystem... requirements)
```

| Parameter | Source | Purpose |
|---|---|---|
| `path` | From AutoBuilder's `configureCustom` lambda | The original path to follow |
| `commandBuilder` | `CommandSwerveDrivetrain::buildFollowPathCommand` | Factory that creates a `FollowPathCommand` for any path (original or recovery) |
| `poseSupplier` | `CommandSwerveDrivetrain::getPosition` | Robot's current estimated pose |
| `targetPoseSupplier` | `CommandSwerveDrivetrain::getLatestPathTargetPose` | Path follower's current target pose, updated each cycle via `PathPlannerLogging` callback |
| `requirements` | The drivetrain subsystem | Prevents other commands from using the drivetrain during recovery |

### State Machine

Three states: `FOLLOWING`, `RECOVERING`, `FAILED`.

**`initialize()`** — Resets state to `FOLLOWING`, zeroes recovery attempts, calls `startFollowing()` to create and initialize the inner `FollowPathCommand`.

**`execute()`** — Dispatches to the current state's handler:

#### FOLLOWING state (`executeFollowing()`)

Each cycle:

1. Calls `activeCommand.execute()` — this runs the inner `FollowPathCommand`, which also fires the `PathPlannerLogging` target pose callback (updating `targetPoseSupplier`).

2. **End-of-path check**: If `activeCommand.isFinished()` returns true (the path's timer expired):
   - Compute distance from robot to the path's **end pose** (last element of `originalPath.getPathPoses()`)
   - If within `END_TOLERANCE_METERS` (0.5m): path completed successfully, set `pathCompleted = true`
   - If outside tolerance: the robot didn't make it to the end. Increment recovery attempts and trigger recovery back to the path **start**, which will restart the full path. This handles the case where a collision late in the path causes the robot to be far from the end when the timer expires.

3. **Mid-path deviation check**: Compute distance from robot's current pose to the path follower's current target pose.
   - If distance > `RECOVERY_THRESHOLD_METERS` (1.5m): interrupt the path, trigger recovery
   - Both checks increment `recoveryAttempts` and transition to `FAILED` if `MAX_RECOVERY_ATTEMPTS` is exceeded

#### RECOVERING state (`executeRecovering()`)

Each cycle:

1. Calls `activeCommand.execute()` on the recovery path's `FollowPathCommand`
2. When `activeCommand.isFinished()`: recovery path complete, call `startFollowing()` to restart the original path, transition back to `FOLLOWING`

#### FAILED state

Does nothing. `isFinished()` returns true, letting the auto sequence advance.

### Recovery Path Construction (`startRecovery()`)

```java
private void startRecovery() {
  Pose2d currentPose = poseSupplier.get();
  Pose2d checkpointPose = originalPath.getStartingHolonomicPose()
      .orElseGet(() -> originalPath.getPathPoses().get(0));

  Translation2d delta = checkpointPose.getTranslation().minus(currentPose.getTranslation());
  Rotation2d travelHeading = delta.getAngle();

  List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
      new Pose2d(currentPose.getTranslation(), travelHeading),
      new Pose2d(checkpointPose.getTranslation(), travelHeading));

  PathPlannerPath recoveryPath = new PathPlannerPath(
      waypoints,
      new PathConstraints(
          RECOVERY_MAX_VELOCITY_MPS, RECOVERY_MAX_ACCELERATION_MPSSQ,
          RECOVERY_MAX_ANGULAR_VELOCITY_RAD_PER_SEC,
          RECOVERY_MAX_ANGULAR_ACCELERATION_RAD_PER_SEC_SQ),
      null,  // no idealStartingState for on-the-fly paths
      new GoalEndState(0.0, checkpointPose.getRotation()));

  activeCommand = commandBuilder.apply(recoveryPath);
  activeCommand.initialize();
}
```

Key details:
- The `Pose2d` rotation in `waypointsFromPoses` represents **travel direction**, not robot heading. Both waypoints use the angle from current position to checkpoint.
- `GoalEndState` rotation is the robot's **chassis rotation** at the end — set to match the original path's starting rotation so the path restart is seamless.
- `idealStartingState` is `null` because on-the-fly paths start from the robot's current velocity.

### Manual Command Delegation

The inner `FollowPathCommand` is **not scheduled** with the `CommandScheduler`. Instead, `LakituFollowPathCommand` calls its lifecycle methods directly:

```
LakituFollowPathCommand (scheduled by auto)
  └── activeCommand (FollowPathCommand, manually driven)
        ├── .initialize()  — called in startFollowing() / startRecovery()
        ├── .execute()     — called each cycle from our execute()
        ├── .isFinished()  — polled each cycle to detect completion
        └── .end()         — called when switching commands or on our end()
```

This avoids subsystem requirement conflicts — only `LakituFollowPathCommand` is registered with the scheduler as requiring the drivetrain.

### AdvantageKit Logging

| Key | Type | When |
|---|---|---|
| `Lakitu/Active` | boolean | Set true on start, false on end |
| `Lakitu/State` | String | On every state transition |
| `Lakitu/Path` | String | On path start/restart |
| `Lakitu/DeviationMeters` | double | Every cycle during FOLLOWING |
| `Lakitu/EndDeviationMeters` | double | When path timer expires but robot is outside tolerance |
| `Lakitu/RecoveryAttempts` | int | On each recovery trigger |
| `Lakitu/RecoveryTarget` | Pose2d | When recovery path is generated |

---

## 2. CommandSwerveDrivetrain.java (Modified)

Three changes to this file:

### 2a. AutoBuilder switched to `configureCustom`

**Before:** `AutoBuilder.configure(...)` — PathPlanner internally creates `FollowPathCommand` instances. No way to intercept.

**After:** `AutoBuilder.configureCustom(...)` — We provide a `Function<PathPlannerPath, Command>` that decides what command to create for each path.

```java
AutoBuilder.configureCustom(
    (PathPlannerPath path) -> {
      if (LakituFollowPathCommand.isRegistered(path.name)) {
        return new LakituFollowPathCommand(
            path,
            this::buildFollowPathCommand,
            this::getPosition,
            this::getLatestPathTargetPose,
            this);
      }
      return buildFollowPathCommand(path);
    },
    this::getPosition,
    this::resetPose,
    () -> false,
    true);
```

**Why `configureCustom` instead of `configure`:** The normal `configure` call doesn't expose a hook to wrap individual path commands. `configureCustom` is PathPlanner's API for exactly this — it takes a command builder function and uses it whenever `AutoBuilder.followPath()` is called (including from `.auto` files in the GUI).

**Caveats of `configureCustom`:**
- Event markers along paths are NOT triggered automatically (not an issue — our autos use event markers only between paths via NamedCommands)
- Paths are NOT auto-flipped for red alliance (not an issue — we already had `() -> false`)

### 2b. `buildFollowPathCommand` extracted as a method

Previously the `FollowPathCommand` construction was inline in `AutoBuilder.configure()`. Now it's a named method so both the `configureCustom` lambda and `LakituFollowPathCommand` can call it:

```java
private Command buildFollowPathCommand(PathPlannerPath path) {
  RobotConfig config = ...;  // same WOODBOT vs GUI logic as before
  return new FollowPathCommand(
      path,
      this::getPosition,
      this::getVelocity,
      (speeds, feedforwards) -> { /* same drive output consumer */ },
      new PPHolonomicDriveController(...),
      config,
      () -> false,
      this);
}
```

This is called:
- By `configureCustom` for unregistered paths (normal behavior)
- By `LakituFollowPathCommand` for both the original path and recovery paths

### 2c. Target pose plumbing

New field and accessors to pass the path follower's target pose from the `PathPlannerLogging` callback to `LakituFollowPathCommand`:

```java
private Pose2d latestPathTargetPose = new Pose2d();

public Pose2d getLatestPathTargetPose() {
  return latestPathTargetPose;
}

public void setLatestPathTargetPose(Pose2d pose) {
  this.latestPathTargetPose = pose;
}
```

**Why this indirection?** `PathPlannerLogging.setLogTargetPoseCallback` is a global singleton — there's one callback for the entire system. The callback fires inside `FollowPathCommand.execute()`, which `LakituFollowPathCommand` calls manually. The updated pose is immediately available in the same cycle for the deviation check.

### 2d. Simulate stall for testing (sim only)

A `LoggedNetworkBoolean` at `/Tuning/Swerve/SimulateStall` that, when set to `true`, brakes the drivetrain for 1 second while the path follower keeps running. This creates an artificial deviation for testing Lakitu in simulation.

```java
private final LoggedNetworkBoolean simulateStall =
    new LoggedNetworkBoolean("/Tuning/Swerve/SimulateStall", false);
private static final double STALL_DURATION_SECONDS = 1.0;
private double stallStartTime = -1;
```

Inside the drive output consumer in `buildFollowPathCommand`:
```java
(speeds, feedforwards) -> {
  setCommandedSpeeds(speeds);
  // When stall is triggered, brake for 1 second then auto-reset
  if (simulateStall.get() && stallStartTime < 0) {
    stallStartTime = Timer.getFPGATimestamp();
  }
  if (stallStartTime >= 0) {
    if (Timer.getFPGATimestamp() - stallStartTime < STALL_DURATION_SECONDS) {
      setControl(xOutReq);  // X-out brake
      return;
    }
    stallStartTime = -1;
    simulateStall.set(false);
  }
  setControl(m_pathApplyRobotSpeeds.withSpeeds(...));
}
```

The path follower still runs (target pose advances), but the robot doesn't move. After 1 second the stall auto-clears. This reliably triggers Lakitu's threshold check.

---

## 3. RobotContainer.java (Modified)

### 3a. Target pose callback updated

**Before:**
```java
PathPlannerLogging.setLogTargetPoseCallback(
    pose -> Logger.recordOutput("Swerve/TargetPathPose", pose));
```

**After:**
```java
PathPlannerLogging.setLogTargetPoseCallback(
    pose -> {
      Logger.recordOutput("Swerve/TargetPathPose", pose);
      drivetrain.setLatestPathTargetPose(pose);
    });
```

This is the bridge between PathPlanner's internal target pose computation and Lakitu's deviation check. The logging behavior is unchanged.

### 3b. Path registration

Register paths that should get Lakitu recovery. Add these calls during robot initialization (e.g., in the constructor or `configureAutos()`):

```java
LakituFollowPathCommand.registerPath("Lakitu Path 1");
LakituFollowPathCommand.registerPath("Lakitu Path 2");
LakituFollowPathCommand.registerPath("Lakitu Path 3");
LakituFollowPathCommand.registerPath("Lakitu Path 4");
```

To add Lakitu to a competition auto path, just add its name here. To remove it, delete the line. No changes to `.path` or `.auto` files needed.

---

## How It All Connects

```
Robot Init
  │
  ├─ RobotContainer registers Lakitu paths
  ├─ CommandSwerveDrivetrain.configureAutoBuilder()
  │    └─ AutoBuilder.configureCustom(commandBuilder, ...)
  │         └─ commandBuilder checks LakituFollowPathCommand.isRegistered()
  │
  └─ RobotContainer sets up PathPlannerLogging callback
       └─ callback stores target pose on drivetrain

Auto Start
  │
  ├─ AutoBuilder builds auto from .auto file
  │    └─ For each "path" block, calls commandBuilder(path)
  │         ├─ Registered path → LakituFollowPathCommand
  │         └─ Unregistered path → FollowPathCommand (normal)
  │
  └─ CommandScheduler runs the auto command sequence

During Path Execution (registered path)
  │
  ├─ LakituFollowPathCommand.execute()
  │    ├─ Calls inner FollowPathCommand.execute()
  │    │    └─ Fires PathPlannerLogging callback → updates targetPose
  │    │
  │    ├─ Reads targetPose, compares to current pose
  │    │    ├─ Within threshold → continue
  │    │    └─ Exceeds threshold → interrupt, build recovery path, transition to RECOVERING
  │    │
  │    └─ On path timer expiry:
  │         ├─ Within END_TOLERANCE of end pose → done
  │         └─ Outside tolerance → trigger recovery (restart full path)
  │
  └─ RECOVERING: follow recovery path → on completion → restart original path
```

---

## Testing in Simulation

1. Select the "Lakitu Path" auto in the auto chooser
2. Run autonomous in sim
3. Mid-path, set `/Tuning/Swerve/SimulateStall` to `true` in NetworkTables
4. Robot brakes for 1 second while target pose advances
5. After stall clears, deviation exceeds threshold → Lakitu triggers recovery
6. Watch `Lakitu/State`, `Lakitu/DeviationMeters`, and `Lakitu/RecoveryTarget` in AdvantageScope

Inject the stall at different points in the path to test:
- **Early in path**: mid-path deviation check triggers
- **Late in path**: end-of-path tolerance check triggers
- **Multiple times**: verify retry limit and FAILED state
