# Team 254 PathPlanner Fork — Technical Report (FRC 2025)

## Table of Contents

- [How PathPlanner Is Included](#how-pathplanner-is-included)
- [Fork Structure](#fork-structure)
- [Modifications vs Upstream PathPlanner 2025](#modifications-vs-upstream-pathplanner-2025)
  - [Multi-Navgrid Obstacle System](#1-multi-navgrid-obstacle-system)
  - [Controller Feedback Overrides](#2-controller-feedback-overrides)
  - [Custom Math Integration](#3-custom-math-integration)
  - [LTV Controller](#4-ltv-controller)
  - [Unmodified Components](#unmodified-components)
- [Autonomous System Architecture](#autonomous-system-architecture)
  - [Dashboard Configuration](#dashboard-configuration)
  - [Auto Execution Loop](#auto-execution-loop)
  - [Dynamic Constraint Scaling](#dynamic-constraint-scaling)
  - [Ice Cream (Algae on Barge) Scoring](#ice-cream-algae-on-barge-scoring)
  - [Feeder Strategies](#feeder-strategies)
- [Real-Time Performance Architecture](#real-time-performance-architecture)
  - [Three-Thread Architecture](#three-thread-architecture)
  - [Non-Blocking Path Requests](#non-blocking-path-requests)
  - [100Hz Controller Notifier](#100hz-controller-notifier)
  - [Path Caching](#path-caching)
  - [Latency Compensation](#latency-compensation)
  - [Path Stitching](#path-stitching)
  - [Grid Resolution](#grid-resolution)
  - [Full Data Flow](#full-data-flow)
- [Performance Characteristics](#performance-characteristics)
- [Key File Reference](#key-file-reference)

---

## How PathPlanner Is Included

PathPlanner is **forked directly into the source tree** at `com.team254.lib.pathplanner` (49 Java files). It is **not** imported as a vendor dependency — it compiles as part of the main project alongside Team 254's own code. The upstream MIT license (Copyright 2022 Michael Jansen) is preserved in `PathPlanner-License.md`.

There is no vendordeps entry, no Gradle library import, and no `.auto` files. The fork is a self-contained copy of the PathPlanner 2025 library with targeted modifications.

---

## Fork Structure

49 Java files organized into 9 packages:

```
src/main/java/com/team254/lib/pathplanner/
├── auto/
│   ├── AutoBuilder.java
│   ├── AutoBuilderException.java
│   ├── CommandUtil.java
│   └── NamedCommands.java
├── commands/
│   ├── FollowPathCommand.java
│   ├── PathfindingCommand.java
│   ├── PathPlannerAuto.java
│   └── AdditionalCommandTrigger.java
├── config/
│   ├── ModuleConfig.java
│   ├── PIDConstants.java
│   └── RobotConfig.java
├── controllers/
│   ├── PathFollowingController.java
│   ├── PPHolonomicDriveController.java
│   └── PPLTVController.java
├── events/
│   ├── Event.java, EventScheduler.java
│   ├── ScheduleCommandEvent.java, CancelCommandEvent.java
│   ├── PointTowardsZoneEvent.java, PointTowardsZoneTrigger.java
│   └── TriggerEvent.java, OneShotTriggerEvent.java
├── path/
│   ├── PathPlannerPath.java, PathConstraints.java
│   ├── PathPoint.java, Waypoint.java
│   ├── RotationTarget.java, EventMarker.java
│   ├── GoalEndState.java, IdealStartingState.java
│   ├── ConstraintsZone.java, PointTowardsZone.java
│   └── IPathCallback.java
├── pathfinding/
│   ├── Pathfinding.java
│   ├── LocalADStar.java
│   └── Pathfinder.java
├── trajectory/
│   ├── PathPlannerTrajectory.java
│   ├── PathPlannerTrajectoryState.java
│   └── SwerveModuleTrajectoryState.java
└── util/
    ├── DriveFeedforwards.java, FileVersionException.java
    ├── FlippingUtil.java, GeometryUtil.java
    ├── JSONUtil.java, PathPlannerLogging.java
    ├── PPLibTelemetry.java
    └── swerve/SwerveSetpointGenerator.java
```

Additionally, three navgrid JSON files are deployed to the robot:

```
src/main/deploy/pathplanner/
├── navgrid.json          (teleop obstacles)
├── auto_navgrid.json     (autonomous obstacles)
└── backoff_navgrid.json  (minimal obstacles for backing away from reef)
```

---

## Modifications vs Upstream PathPlanner 2025

The bulk of the fork is unmodified PathPlanner 2025. The custom changes are surgical and targeted.

### 1. Multi-Navgrid Obstacle System

**Files:** `LocalADStar.java`, `Pathfinder.java`, `Pathfinding.java`

Upstream PathPlanner loads a single `navgrid.json`. Team 254 loads **three** obstacle grids and switches between them at runtime:

| Grid | File | Purpose |
|------|------|---------|
| Teleop | `navgrid.json` | Full-size obstacles for teleop driving |
| Auto | `auto_navgrid.json` | More permissive margins for autonomous |
| Backoff | `backoff_navgrid.json` | Minimal obstacles for backing away from reef after scoring |

Three new methods were added to the `Pathfinder` interface and exposed through the `Pathfinding` static wrapper:

- `setAutoObstacles()` — switch to autonomous grid
- `setTeleopObstacles()` — switch to teleop grid
- `setBackoffObstacles()` — switch to backoff grid

In `LocalADStar`, all three grids are loaded at construction. Obstacle switching updates `requestObstacles` under a write lock but does not block the planning thread.

### 2. Controller Feedback Overrides

**File:** `PPHolonomicDriveController.java`

Added methods to let external systems (like vision alignment) inject custom feedback during path following:

- `overrideXFeedback()` / `clearXFeedbackOverride()`
- `overrideYFeedback()` / `clearYFeedbackOverride()`
- `overrideXYFeedback()` / `clearXYFeedbackOverride()`
- `overrideRotationFeedback()` / `clearRotationFeedbackOverride()`
- `setControlPoint(Transform2d)` — offset the control point from robot center

This allows hybrid control strategies where PathPlanner handles the gross motion and vision handles fine alignment, without the two systems fighting each other.

### 3. Custom Math Integration

**File:** `PathfindingCommand.java`

Uses `com.team254.lib.util.MathHelpers.reverseInterpolate()` for path stitching. When a new path arrives mid-execution, this function calculates the parametric position along a line segment to find the continuity point, enabling smooth transitions between old and new paths.

### 4. LTV Controller

**File:** `PPLTVController.java`

Wraps WPILib's `LTVUnicycleController` behind PathPlanner's `PathFollowingController` interface. Provides a Linear Time-Varying controller option for differential drivetrains.

### Unmodified Components

The following are essentially stock PathPlanner 2025:

- `FollowPathCommand.java` — standard path following with event scheduling
- `AutoBuilder.java` — standard auto routine building and configuration
- `RobotConfig.java` — standard robot physical configuration
- `PathPlannerPath.java` — standard path representation and file loading
- `PathPlannerTrajectory.java` — standard trajectory generation with feedforward calculations
- Event system — standard event scheduling and triggers
- `SwerveSetpointGenerator.java` — swerve kinematic constraint handling (credits Team 254 in comments as the original authors of the algorithm)

---

## Autonomous System Architecture

Team 254 does **not** use PathPlanner's `.auto` file system. They build autonomous routines **procedurally at runtime** using real-time A* pathfinding.

### Dashboard Configuration

**File:** `AutoModeSelector.java`

Drivers configure auto mode via SmartDashboard/NetworkTables:

| Parameter | Example | Description |
|-----------|---------|-------------|
| ScoreOrder | `"IKLLKL"` | Letters A–L mapping to reef scoring positions |
| LevelOrder | `"44433*"` | Digits 2–4 for reef level, `*` for algae |
| Starting Position | `LEFT_BARGE` | One of LEFT, MIDDLE, or RIGHT |
| Auto Mode | `CUSTOM` | DO_NOTHING or CUSTOM |
| Ice Cream Count | `0–3` | Algae pieces to score on barge |
| Feeder Strategy | `FUNNEL` | FUNNEL or GROUND pickup |

Starting positions map to fixed field poses:

- `LEFT_BARGE` — `(7.0, 7.0)` at 225 degrees
- `MIDDLE_BARGE` — `(7.2, 4.0)` at 180 degrees
- `RIGHT_BARGE` — `(7.0, 1.0)` at 135 degrees

Scoring positions cover 12 reef positions at L4, L3, and L2, plus 6 algae positions.

### Auto Execution Loop

**File:** `PathfindingAuto.java`

The `getNextAction()` method iterates through the score/level sequences:

```
For each step in scoreSequence:
  1. If ALGAE position ('*'):
     - Execute algae intake sequence
     - Disable vision during intake, re-enable after
     - Set exclusive AprilTag to avoid confusion

  2. Else (reef scoring):
     - Switch to tight bumper mode for positions A/B
     - Pathfind to reef position with join path
     - Stage superstructure (elevator/arm) concurrently during transit
     - Race: 1.5s timeout for coral drop vs pathfind completion
     - Run ground intake during path if applicable
     - Switch to wide bumper mode after scoring

  3. After scoring, decide next action:
     - If last position and ice cream count met:
       → Pathfind to closest barge location for algae scoring
     - Else:
       → Pathfind to feeder (FUNNEL or GROUND strategy)

  4. Increment step counter and repeat
```

Navgrid switching happens at transitions:
- `setAutoObstacles()` when approaching reef
- `setBackoffObstacles()` when backing away after scoring

### Dynamic Constraint Scaling

**File:** `PathfindingAutoAlignCommand.java`

Constraints are modified in real-time based on robot state:

| Scenario | Speed Scale | Accel Scale | Notes |
|----------|-------------|-------------|-------|
| Standard pathfinding | 0.8x | 0.4x Y-accel | Normal approach |
| Tall robot (elevator > 1.0m) | — | 0.6x overall, 0.35x Y | Stability when top-heavy |
| Feeder approach | 0.825x speed | 1.15x accel | Aggressive pickup |
| Ice cream approach | 0.7x | 0.7x | Precision placement |
| Join path (normal) | 0.6x | 0.5x | Final approach to target |

Join paths use different interpolation fractions:
- 0.6 for normal scoring positions
- 0.2 for L4 (to handle top-heavy stability at full elevator extension)

An `IPathCallback` dynamically updates constraint zones during path execution based on superstructure staging status, elevator height, and distance to the final stitching point.

### Ice Cream (Algae on Barge) Scoring

"Ice cream" refers to algae pieces placed on the barge.

1. At initialization, remaining ice cream poses are set based on starting position:
   - Left barge: Left → Middle → Right order
   - Right barge: Right → Middle → Left order
   - Poses are flipped for red alliance

2. Smart selection finds the **closest** remaining ice cream position to the robot

3. A pose is generated oriented toward the target using `atan2` from robot to barge position

4. Uses `AutoBuilder.pathfindToPose()` with 0.7x speed/accel constraints

5. Intake logic waits for the coral indexer to clear, runs ground intake during transit

### Feeder Strategies

- **FUNNEL:** `getPathfindToFeederCommand()` — simpler funnel-based intake
- **GROUND:** `getPathfindToFeederGroundCommand()` — includes retry logic for failed ground pickups

---

## Real-Time Performance Architecture

### Three-Thread Architecture

The system is fully decoupled across three threads, each running independently at its own rate:

| Thread | Rate | OS Priority | Responsibility |
|--------|------|-------------|----------------|
| Main robot loop | 50Hz (20ms) | Normal (5) | Poll for new paths, generate trajectories, schedule events |
| A* pathfinding | Background | Low idle, boosted during planning | Compute A* solutions asynchronously |
| Path-following controller | **100Hz (10ms)** | **Real-time (41)** | Sample trajectory, calculate wheel commands, send to motors |

The controller notifier runs at OS priority 41 — above everything else — so it **never gets starved** regardless of what the main loop or pathfinding thread are doing.

### Non-Blocking Path Requests

**Submitting a request (main thread, <1ms):**

```java
requestLock.writeLock().lock();
requestStart = startPos;
requestGoal = goalPos;
requestMinor = true;
requestReset = true;
newPathAvailable = false;
requestLock.writeLock().unlock();
// Returns immediately — no waiting for computation
```

**Planning thread releases lock before expensive work:**

```java
requestLock.readLock().lock();
// Copy all request state into local variables
GridPosition start = requestStart;
GridPosition goal = requestGoal;
// ...
requestLock.readLock().unlock();  // Release BEFORE A*

// Now do expensive A* computation (50-200ms) completely lock-free
doWork(start, goal, ...);
```

**Polling for results (main thread, <1ms):**

```java
if (Pathfinding.isNewPathAvailable()) {
    currentPath = Pathfinding.getCurrentPath();  // Brief read-lock
}
```

The main loop never waits. The planning thread never holds a lock during computation.

### 100Hz Controller Notifier

**File:** `DriveSubsystem.java` — `Controller` inner class

The controller uses **`volatile` fields instead of locks** for zero-contention thread communication:

```java
private volatile PathPlannerTrajectory trajectory;  // Atomic reference swap
private volatile Timer timer;

// Main thread writes:
public void accept(PathPlannerTrajectory t) {
    trajectory = t;       // volatile write
    timer.reset();
    timer.start();
}

// 100Hz notifier reads:
public void run() {
    PathPlannerTrajectory traj = trajectory;  // volatile read
    if (traj == null) return;

    PathPlannerTrajectoryState targetState = traj.sample(timer.get());
    ChassisSpeeds speeds = controller.calculateRobotRelativeSpeeds(
        robotState.getLatestFieldToRobot().getValue(), targetState);
    setControl(pathplannerAutoRequest.withSpeeds(speeds)
        .withWheelForceFeedforwardsX(targetState.feedforwards.robotRelativeForcesXNewtons())
        .withWheelForceFeedforwardsY(targetState.feedforwards.robotRelativeForcesYNewtons()));
}
```

Trajectory sampling uses **O(log n) binary search** with linear interpolation between states — effectively free at 100Hz.

### Path Caching

`LocalADStar` maintains a **120-path LRU cache** with two lookup strategies:

1. **Exact match** — same start and goal grid cells → instant return
2. **Fuzzy match** — searches within a **0.8m radius** (~16–25 nearby grid positions) for a usable cached path

Cache rules:
- Only written when robot velocity < 0.05 m/s (clean initial conditions)
- Cache hits return in **1–2ms** vs **50–200ms** for full A*
- Cached paths are cloned to prevent mutation; if the robot has significant velocity at reuse time, bezier curves are recomputed from the cached waypoints

This is especially valuable during repeated scoring cycles where the robot revisits similar field positions.

### Latency Compensation

When submitting a pathfinding request, the robot pose is predicted **30ms into the future**:

```java
final double kLoopTime = 0.03;
Pose2d predictedPose = currentPose.exp(new Twist2d(
    kLoopTime * currentSpeeds.vxMetersPerSecond,
    kLoopTime * currentSpeeds.vyMetersPerSecond,
    kLoopTime * currentSpeeds.omegaRadiansPerSecond));
```

This accounts for the round-trip latency: main loop detection → pathfinder computation → trajectory generation → controller execution.

### Path Stitching

When a new path arrives while the robot is already following one, `PathfindingCommand` ensures smooth continuity:

1. Find the **closest point** on the new path to the robot's current position
2. Use `MathHelpers.reverseInterpolate()` to calculate the exact parametric position
3. **Trim** the new path to start from that point
4. Generate a new trajectory using the robot's **current velocity** as the initial condition

This prevents acceleration discontinuities (jerk spikes) at path transitions. The robot smoothly blends from the old trajectory into the new one without stopping.

### Grid Resolution

The A* grid uses **0.2m cells** over the 16.54m x 8.05m field, yielding approximately **82 x 40 = 3,280 nodes**. This balances:

- **Planning speed** — small enough search space for the A* to solve in 50–200ms
- **Obstacle fidelity** — 0.2m resolution is fine enough to represent reef posts, walls, and game pieces

The planning thread sleeps 10ms when idle (no busy-waiting) and boosts its own priority during active computation.

### Full Data Flow

```
Main Loop (50Hz)                  Pathfinding Thread              100Hz Notifier
────────────────                  ──────────────────              ──────────────
setProblem()
  write lock (<1ms)  ──────────►  wake up
  set flags, release              read lock, copy request state
  return immediately              release lock

                                  check cache
                                    hit? → skip A*, return path
                                    miss? → run A* (50-200ms, lock-free)

                                  write lock
                                    set currentPath
                                    newPathAvailable = true
                                  release lock

poll isNewPathAvailable()  ◄────
get path (read lock, <1ms)
trim path to current position
generate trajectory (10-50ms)
controller.accept(traj)  ─────────────────────────────────────►  volatile write

                                                                 sample(time) → O(log n)
                                                                 calculate chassis speeds
                                                                 apply feedforwards
                                                                 send to motors
                                                                 (repeats every 10ms)
```

---

## Performance Characteristics

### Timing Budget

| Operation | Thread | Time |
|-----------|--------|------|
| Path request submission | Main | < 1ms |
| Cache lookup (hit) | Pathfinding | 1–2ms |
| A* computation (typical) | Pathfinding | 50–200ms |
| Trajectory generation | Main | 10–50ms |
| Trajectory sampling | Notifier | < 1ms |
| Full notifier cycle | Notifier | < 5ms |

### Memory Usage

| Component | Estimate |
|-----------|----------|
| Path cache (120 paths) | ~500 KB |
| Grid structures (g, rhs, open sets) | ~105 KB |
| Trajectory states (300–1000 per trajectory) | ~80–250 KB |

### Threading Primitives

| Primitive | Location | Purpose |
|-----------|----------|---------|
| `ReentrantReadWriteLock pathLock` | LocalADStar | Protects `currentPath` and `currentPathFull` |
| `ReentrantReadWriteLock requestLock` | LocalADStar | Protects all request state fields |
| `volatile PathPlannerTrajectory` | DriveSubsystem.Controller | Lock-free trajectory handoff to 100Hz thread |
| `volatile Timer` | DriveSubsystem.Controller | Lock-free timer access |
| `boolean newPathAvailable` | LocalADStar | Signals path completion (read under lock) |

---

## Key File Reference

### PathPlanner Fork

| File | Purpose |
|------|---------|
| `lib/pathplanner/pathfinding/LocalADStar.java` | A* pathfinding with multi-grid, caching, background thread |
| `lib/pathplanner/pathfinding/Pathfinder.java` | Interface with custom obstacle-switching methods |
| `lib/pathplanner/pathfinding/Pathfinding.java` | Static wrapper exposing async pathfinding API |
| `lib/pathplanner/commands/PathfindingCommand.java` | Async path consumption, stitching, trajectory generation |
| `lib/pathplanner/commands/FollowPathCommand.java` | Trajectory following with event scheduling |
| `lib/pathplanner/controllers/PPHolonomicDriveController.java` | Swerve controller with feedback overrides |
| `lib/pathplanner/controllers/PPLTVController.java` | LTV unicycle controller adapter |
| `lib/pathplanner/trajectory/PathPlannerTrajectory.java` | Trajectory generation (forward/reverse accel passes) |
| `lib/pathplanner/config/RobotConfig.java` | Robot physical model for trajectory generation |
| `lib/pathplanner/util/FlippingUtil.java` | Red/blue alliance field mirroring |

### Team 254 Auto System

| File | Purpose |
|------|---------|
| `frc2025/auto/PathfindingAuto.java` | Procedural auto builder from score/level strings |
| `frc2025/auto/AutoModeSelector.java` | Dashboard configuration and enum definitions |
| `frc2025/auto/PathfindingWarmupCommand.java` | Pre-warms A* during disabled period |
| `frc2025/commands/PathfindingAutoAlignCommand.java` | Dynamic constraint scaling and vision integration |
| `frc2025/subsystems/drive/DriveSubsystem.java` | PathPlanner configuration and 100Hz controller thread |

### Deploy Files

| File | Purpose |
|------|---------|
| `deploy/pathplanner/navgrid.json` | Teleop obstacle grid |
| `deploy/pathplanner/auto_navgrid.json` | Autonomous obstacle grid (more permissive) |
| `deploy/pathplanner/backoff_navgrid.json` | Backoff obstacle grid (minimal obstacles) |

All paths are relative to `src/main/java/com/team254/` for Java files and `src/main/` for deploy files.
