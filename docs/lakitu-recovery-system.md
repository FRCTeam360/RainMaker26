# Lakitu Auto Recovery System

## Problem Statement

During autonomous, our robot follows pre-planned trajectories using PathPlanner. These trajectories are time-parameterized — at every moment, the path follower has a **target pose** that advances along the path at a pre-computed rate regardless of where the robot actually is.

When a collision occurs mid-path, the robot gets knocked off its trajectory. The path follower's target pose continues advancing along the planned path, while the robot's actual position falls further and further behind. The PID controller attempts to correct, but once the deviation exceeds what proportional control can recover, the situation spirals:

```
Time ──────────────────────────────────────►

Planned:   ●───●───●───●───●───●───●───●  (target pose advances on schedule)
                          ╲
Actual:    ●───●───●───●   ●              (robot knocked off after collision)
                            ╲
                             ●─ ─ ─ ?     (PID can't close the gap)

Result: target runs off the end of the path, auto timing is ruined
```

The robot either finishes the path far from where it should be, or the remaining auto segments start from the wrong position, cascading into a failed autonomous routine.

## Goals

1. **Work within the existing PathPlanner API** — Use PathPlanner's public API for path construction and following. No forks, no monkey-patching, no internal API access. If PathPlanner can do it, we use PathPlanner to do it.
2. **Drop-in wrapper for existing autos** — The recovery system should wrap the standard follow-path command. Existing auto files and path files should not need modification.
3. **On-the-fly path to starting position** — When recovery triggers, an on-the-fly path is constructed from wherever the robot currently is directly to the start of the current path segment, using PathPlanner's on-the-fly path API. No AD* pathfinding, no pre-planned recovery paths — just a straight shot back to the checkpoint. This means path designers must place segment start/end positions in open, unobstructed areas of the field so the recovery path can reach them from anywhere.
4. **GUI workflow unchanged** — Base paths are authored, edited, and maintained entirely through the PathPlanner GUI as normal. The recovery system operates at the command layer — it has no effect on `.path` or `.auto` files. Path designers never need to think about Lakitu when editing paths.
5. **Observable and debuggable** — Every recovery event, state transition, and distance measurement is logged to AdvantageKit so the team can review what happened in post-match analysis.

## The Mario Kart Lakitu Analogy

In Mario Kart, when a kart falls off the track or goes out of bounds, **Lakitu** (the cloud-riding Koopa with a fishing rod) picks the kart up and places it back at the last valid checkpoint. The race continues from that checkpoint — the driver loses some time, but they're back on course and can finish the race.

We apply the same concept to autonomous path following:

| Mario Kart | FRC Auto Recovery |
|---|---|
| Kart goes out of bounds | Robot deviates beyond a distance threshold from the target pose |
| Lakitu picks up the kart | System interrupts the current path follower |
| Kart placed at last checkpoint | Robot retraces the original path back to the **start of the current path segment** |
| Race continues from checkpoint | Path segment restarts from the beginning |

The **checkpoint** is the starting pose of whichever path segment the robot was following when the deviation was detected. Just like in Mario Kart, the driver (auto routine) loses time on recovery, but the alternative — careening further off course — is worse.

## High-Level Methodology

The system wraps the standard path-following command with a monitoring layer that runs every control loop cycle (~20ms):

### Step 1: Monitor Deviation

Every cycle, compare the robot's **current estimated pose** to the path follower's **current target pose**. Compute the translational distance between them.

### Step 2: Detect Threshold Breach

If the distance exceeds a configurable threshold (default: 1.5 meters), the system determines the robot has been knocked irrecoverably off-path. Small deviations from turns or minor bumps stay well below this threshold and are handled normally by the PID controller.

### Step 3: Interrupt and Recover

When the threshold is breached:
1. **Stop** the current path follower immediately
2. **Build a recovery path on-the-fly** from the robot's current position directly to the start of the current path segment using PathPlanner's on-the-fly path API. This is a simple point-to-point path — no obstacle avoidance, which is why path start/end positions must be in open, unobstructed areas of the field.
3. **Follow the recovery path** back to the checkpoint

### Step 4: Restart the Path

Once the robot arrives at the checkpoint (path start), restart the original path segment from the beginning. The path follower generates a fresh trajectory from the robot's current state, so the timing resets cleanly.

### Step 5: Fail-Safe

If recovery itself fails (robot gets knocked off again during the recovery path), the system allows a limited number of retry attempts. After exhausting retries, it **abandons the current path segment** and lets the auto sequence advance to the next command. Losing one segment is better than getting stuck in an infinite recovery loop.

## State Machine

```
                    ┌─────────────────────────────────────────┐
                    │                                         │
                    ▼                                         │
              ┌───────────┐                                   │
     ────────►│ FOLLOWING  │                                   │
   path start │           │                                   │
              └─────┬─────┘                                   │
                    │                                         │
                    │ distance > threshold                    │
                    │                                         │
                    ▼                                         │
              ┌───────────┐    recovery complete         ┌────┴─────┐
              │RECOVERING │ ────────────────────────────►│ RESTART  │
              │           │    (robot at checkpoint)     │ (re-enter│
              └─────┬─────┘                              │ FOLLOWING)│
                    │                                    └──────────┘
                    │ attempts > max
                    │
                    ▼
              ┌───────────┐
              │  FAILED   │ ──────► auto sequence continues
              │ (abandon) │         to next command
              └───────────┘


 Path completes normally
 ┌───────────┐
 │ FOLLOWING  │──── path finished ────► auto sequence continues
 └───────────┘                          to next command
```

**States:**
- **FOLLOWING** — Normal operation. The wrapped path follower is active. Distance is monitored every cycle.
- **RECOVERING** — The original path has been interrupted. The robot is following an on-the-fly path directly back to the checkpoint (start of the current path segment). Distance monitoring is paused during recovery.
- **FAILED** — Maximum recovery attempts exhausted. The command reports itself as finished so the auto sequence can proceed to the next step.

## Data Flow

```
┌──────────────────┐      ┌──────────────────┐
│  Pose Estimator  │      │   Path Follower   │
│  (odometry +     │      │  (target pose     │
│   vision fusion) │      │   advances along  │
│                  │      │   planned path)   │
└────────┬─────────┘      └────────┬──────────┘
         │                         │
         │  current pose           │  target pose
         │                         │
         ▼                         ▼
    ┌─────────────────────────────────────┐
    │         Distance Calculator         │
    │                                     │
    │  distance = current.distanceTo(     │
    │               target)               │
    └──────────────────┬──────────────────┘
                       │
                       ▼
    ┌─────────────────────────────────────┐
    │        Threshold Comparator         │
    │                                     │
    │  if distance > THRESHOLD:           │
    │    → trigger recovery               │
    │  else:                              │
    │    → continue following             │
    └──────────────────┬──────────────────┘
                       │
                       ▼ (on breach)
    ┌─────────────────────────────────────┐
    │     On-The-Fly Path Constructor     │
    │                                     │
    │  Build direct path:                 │
    │  current pose → path start pose     │
    │  using PathPlanner's on-the-fly API │
    └─────────────────────────────────────┘
```

The key insight is that PathPlanner already computes and exposes the target pose every cycle for logging/visualization. The Lakitu system taps into this existing data stream — it adds monitoring logic on top, not a parallel path-tracking computation.

## Configurable Parameters

| Parameter | Default | Rationale |
|---|---|---|
| **Recovery distance threshold** | 1.5 m | Normal tracking error is 0.1–0.3m. Sharp turns may transiently reach ~0.5m. A 1.5m deviation indicates a collision or major disturbance that PID alone cannot recover from. Too low triggers false positives on aggressive paths; too high wastes time before intervening. |
| **Max recovery attempts** | 2 per path segment | Allows for one retry if the first recovery is disrupted (e.g., a second collision). More than 2 attempts likely means the field situation is unrecoverable and time is better spent on the next auto segment. Counter resets at the start of each new path segment. |
| **Recovery path max velocity** | 3.0 m/s | Slower than normal path speed (4.0 m/s) to prioritize accuracy during recovery. The robot needs to arrive precisely at the checkpoint, not race there. |
| **Recovery path max acceleration** | 6.0 m/s² | Reduced from the normal 8.0 m/s² for smoother, more predictable recovery motion. |

All parameters are named constants in code, tuned through testing.

## Tradeoffs and Limitations

### Time Cost
Recovery is not free. Retracing back to the checkpoint and restarting the path segment takes time — potentially 2–4 seconds depending on how far off-path the robot was knocked. In a 15-second autonomous window, this may mean the last path segment or scoring action gets cut. **However**, an unrecovered collision typically ruins the *entire* remaining auto, so spending time on recovery to save 2–3 remaining segments is almost always the better trade.

### Recovery Assumes Clear Line to Checkpoint
The on-the-fly recovery path is a direct route from the robot's current position to the path start. If a field element is between the robot and the checkpoint, the recovery path will drive through it. This is mitigated by the design constraint that path start/end positions must be placed in open, unobstructed areas of the field.

### Vision Pose Estimation During Recovery
If the robot is knocked into a position with poor AprilTag visibility, the pose estimate itself may drift. The recovery system trusts the pose estimator — if the estimate is wrong, recovery will pathfind to the wrong location. This is an inherent limitation: recovery quality is only as good as the pose estimate.

### Not All Collisions Are Recoverable
If a collision pins the robot against a field element or another robot, no amount of pathfinding will help. The max-attempts fail-safe handles this case by eventually giving up and moving on.

### Auto Sequence Timing
The auto sequence is a chain of sequential commands. When the Lakitu wrapper is active on a path segment, the entire sequence pauses until that segment either completes (with or without recovery) or is abandoned. Downstream segments are not aware of the delay — they simply start later. Teams should account for potential recovery time when designing tight auto routines.

### Threshold Tuning Is Field-Dependent
The optimal threshold may vary based on:
- **Robot mass and drivetrain power** — heavier robots resist collisions better but recover slower
- **Path aggressiveness** — fast, sharp paths have larger natural tracking errors
- **Expected defense** — if collisions are frequent, a lower threshold catches problems earlier but risks false triggers

The 1.5m default is a starting point. Teams should tune this in practice matches.
