# Integrating PathPlanner autos with superstructure-owned swerve drive

**The cleanest solution is to redirect PathPlanner's ChassisSpeeds output through your superstructure's control flow using the consumer lambda in `AutoBuilder.configure()`.** PathPlanner never calls your drivebase directly — it invokes a callback you define, and that callback can set state on your superstructure instead of commanding hardware. This "drive request" pattern lets the superstructure remain the single authority over drivebase outputs while PathPlanner does the trajectory math. Teams 254, 401, 1678, and others have proven variants of this architecture at the highest competitive levels across the 2024–2025 seasons, and the pattern scales naturally with CTRE Phoenix 6's `SwerveRequest` abstraction.

## The fundamental conflict and where to intercept it

PathPlanner's `FollowPathCommand` runs inside WPILib's command scheduler and claims the drive subsystem as a requirement. If your superstructure also claims the drive subsystem (or calls `setControl()` on the drivetrain every loop), the two will fight — the scheduler will cancel one when the other starts. This subsystem-ownership collision is the root of the problem, and Chief Delphi threads are littered with teams hitting `"Multiple commands in a parallel composition cannot require the same subsystems"` at runtime.

The escape hatch is **PathPlanner's output consumer**. When you call `AutoBuilder.configure()`, the fourth parameter is a `BiConsumer<ChassisSpeeds, DriveFeedforwards>` (or `Consumer<ChassisSpeeds>` in the simpler overload). PathPlanner calls this lambda every loop iteration during path following — it never touches your hardware directly. This means you can write:

```java
AutoBuilder.configure(
    () -> drivetrain.getState().Pose,
    drivetrain::resetPose,
    () -> drivetrain.getState().Speeds,
    (speeds, feedforwards) -> superstructure.setAutoSpeeds(speeds, feedforwards),
    new PPHolonomicDriveController(
        new PIDConstants(5.0, 0, 0),
        new PIDConstants(5.0, 0, 0)
    ),
    RobotConfig.fromGUISettings(),
    () -> DriverStation.getAlliance().orElse(null) == Alliance.Red,
    drivetrain   // subsystem requirement — see note below
);
```

Instead of the consumer calling `drivetrain.setControl()` directly, it calls `superstructure.setAutoSpeeds()`, which stores the desired speeds. The superstructure's `periodic()` method then decides what to do with them — apply them, modify them, or override them entirely. **This single redirection is the architectural key** that lets PathPlanner coexist with a superstructure that owns drivebase outputs.

One subtlety: the `driveRequirements` parameter (the last vararg) still tells the command scheduler which subsystem the `FollowPathCommand` "requires." If your superstructure is the WPILib `Subsystem` that owns drive, pass the superstructure here instead of the drivetrain. Alternatively, if you want the scheduler to handle mutual exclusion normally, pass the drivetrain but make your superstructure access it through coordinated commands rather than a default command.

## Three proven architectural patterns from top teams

The FRC community has converged on three main approaches, each with different tradeoffs depending on how tightly your superstructure couples to the drivebase.

### Pattern 1: Superstructure manages mechanisms, drive stays independent

This is the **dominant pattern** among elite teams including 254, 2910, 6328, and 401. The drivebase is a standalone subsystem that configures `AutoBuilder` in its own constructor. The superstructure coordinates only non-drive mechanisms (elevator, arm, intake, claw). During autonomous, PathPlanner commands own the drivebase subsystem while `NamedCommands` trigger superstructure state transitions in parallel.

Team 254's 2025 "Undertow" code exemplifies this: their `SuperstructureStateMachine` coordinates intake, indexer, claw, wrist, elevator, and climber through complex multi-step actions, but the drive subsystem runs its own pathfinding auto system independently. Coordination happens through command composition — `SuperstructureCommandFactory` produces commands that interleave with drive commands at the `RobotContainer` level. Team 401's 2025 "Hydrus" code takes this further with a `StrategyManager` that orchestrates both the drivetrain state machine and the `ScoringSubsystem` (their superstructure) via queued actions loaded from JSON files.

The advantage is simplicity: PathPlanner works out of the box because nothing fights over subsystem ownership. The limitation is that the superstructure can't directly gate or modify drive outputs — if you need the superstructure to slow the robot when the elevator is extended, you need an indirect mechanism like a speed-scaling supplier.

### Pattern 2: Drive request object routed through the superstructure

This pattern directly addresses the user's architecture. The superstructure holds a "current drive request" field, and all drive sources — teleop joysticks, PathPlanner, vision alignment — write to it. The superstructure's `periodic()` method reads this field and applies it to the drivetrain, optionally modifying or overriding it based on mechanism state. CTRE Phoenix 6's `SwerveRequest` abstraction makes this particularly clean:

```java
public class Superstructure extends SubsystemBase {
    private final CommandSwerveDrivetrain drivetrain;
    private SwerveRequest currentDriveRequest = new SwerveRequest.Idle();
    private final SwerveRequest.ApplyRobotSpeeds autoRequest =
        new SwerveRequest.ApplyRobotSpeeds();

    public enum DriveMode { TELEOP, AUTO, VISION_ALIGN, LOCKED }
    private DriveMode driveMode = DriveMode.TELEOP;

    /** Called by PathPlanner's output consumer */
    public void setPathPlannerOutput(ChassisSpeeds speeds,
                                      DriveFeedforwards ff) {
        if (driveMode == DriveMode.AUTO) {
            currentDriveRequest = autoRequest.withSpeeds(speeds)
                .withWheelForceFeedforwardsX(ff.robotRelativeForcesXNewtons())
                .withWheelForceFeedforwardsY(ff.robotRelativeForcesYNewtons());
        }
    }

    /** Called by teleop bindings */
    public void setTeleopRequest(SwerveRequest request) {
        if (driveMode == DriveMode.TELEOP) {
            currentDriveRequest = request;
        }
    }

    @Override
    public void periodic() {
        // Superstructure can modify the request based on mechanism state
        if (elevator.isExtended() && currentDriveRequest instanceof
                SwerveRequest.ApplyRobotSpeeds auto) {
            // Scale down speeds when elevator is up
            auto.withSpeeds(scaleDown(auto.Speeds));
        }
        drivetrain.setControl(currentDriveRequest);
        // ... rest of mechanism coordination
    }
}
```

Team 3476 (Code Orange) used a `DriveState` enum with this approach in their 2022 code — their Drive subsystem's `periodic()` switched on `driveState` between `TELEOP`, `STOP`, `DONE`, and auto trajectory-following states, routing all outputs through a single `setSwerveModuleStates()` call. Team 226 (Hammerheads) formalized this with a `DriveController` interface requiring only a `getSpeeds()` method, making it trivial to swap between different drive sources. They noted that "unfortunately, the PathPlanner library does not neatly fit into this pattern, so we templated separate code for it."

### Pattern 3: PathPlanner wrapped as a state in the drivetrain state machine

Team 401's 2025 code demonstrates wrapping PathPlanner's on-the-fly pathfinding as a state within their drivetrain's own finite state machine. Their drivetrain has explicit states like `LinearDrive` (odometry-based approach), `SingleTagLineup` (AprilTag precision alignment), and a PathPlanner OTF state. The superstructure's `StrategyManager` triggers transitions between these states, and **subsystems expose `BooleanSupplier`s** (like "am I lined up?") that other subsystems use to gate their own transitions. During auto, PathPlanner paths feed the drive's path-following state, which exposes lineup status, which gates the scoring subsystem's actions.

This pattern is powerful because the superstructure never directly handles ChassisSpeeds — it commands the drivetrain at a higher abstraction level ("go to this pose" or "follow this path"), and the drivetrain internally decides how to achieve it. The superstructure only needs to know the drivetrain's current state, not its implementation details.

## CTRE Phoenix 6's SwerveRequest as the natural bridge

CTRE's `SwerveRequest` pattern is almost purpose-built for this problem. Every drive source produces a `SwerveRequest` object: teleop produces `SwerveRequest.FieldCentric`, PathPlanner produces `SwerveRequest.ApplyRobotSpeeds`, vision alignment might produce `SwerveRequest.FieldCentricFacingAngle`. The drivetrain has exactly one entry point — `setControl(SwerveRequest)` — which hands the request to the **odometry thread** running at 250Hz on CANivore.

The CTRE-generated `CommandSwerveDrivetrain` configures the PathPlanner bridge in `configureAutoBuilder()`:

```java
private final SwerveRequest.ApplyRobotSpeeds autoRequest =
    new SwerveRequest.ApplyRobotSpeeds();

private void configureAutoBuilder() {
    var config = RobotConfig.fromGUISettings();
    AutoBuilder.configure(
        () -> getState().Pose,
        this::resetPose,
        () -> getState().Speeds,
        (speeds, feedforwards) -> setControl(
            autoRequest.withSpeeds(speeds)
                .withWheelForceFeedforwardsX(
                    feedforwards.robotRelativeForcesXNewtons())
                .withWheelForceFeedforwardsY(
                    feedforwards.robotRelativeForcesYNewtons())
        ),
        new PPHolonomicDriveController(
            new PIDConstants(10, 0, 0),
            new PIDConstants(10, 0, 0)
        ),
        config,
        () -> DriverStation.getAlliance().orElse(null) == Alliance.Red,
        this
    );
}
```

The key line is the consumer lambda. In the standard CTRE example, it calls `setControl()` directly on the drivetrain. **To route through a superstructure, replace that lambda** with a call that stores the request or speeds on the superstructure. The `SwerveRequest` objects are mutable and reused — `withSpeeds()` mutates in place and returns `this` — so storing a reference and applying it later in `periodic()` works cleanly.

For teams wanting even more control, CTRE supports **custom `SwerveRequest` implementations**. You can create a `SuperstructureAwareDrive` request that checks superstructure state before applying module outputs:

```java
public class GatedDriveRequest implements SwerveRequest {
    private final SwerveRequest inner;
    private final BooleanSupplier safeToApply;

    @Override
    public StatusCode apply(SwerveControlParameters params,
                            SwerveModule... modules) {
        return safeToApply.getAsBoolean()
            ? inner.apply(params, modules)
            : new SwerveDriveBrake().apply(params, modules);
    }
}
```

This runs on the **250Hz odometry thread**, giving you sub-4ms reaction time for safety interlocks, but the implementation must be fast since it blocks odometry updates.

## PathPlanner 2025 features that help with superstructure integration

PathPlanner 2025 introduced several features directly relevant to superstructure architectures. **`PPHolonomicDriveController` feedback overrides** let you inject custom PID feedback while a path is running — useful when the superstructure needs vision-based heading correction during auto:

```java
// Override rotation feedback for vision auto-aim during path following
PPHolonomicDriveController.overrideRotationFeedback(
    () -> visionAlignPID.calculate(currentHeading, targetHeading)
);
// Feedforward from the trajectory is preserved; only feedback changes
```

The **`configureCustom()` method** provides maximum flexibility for teams whose architecture truly cannot work with standard AutoBuilder. It takes a `Function<PathPlannerPath, Command>` — you provide a function that converts a path into whatever custom command your superstructure needs. The tradeoff is losing automatic event marker triggering and path flipping.

PathPlanner also now bundles a port of **Team 254's `SwerveSetpointGenerator`**, which teams can call inside their `driveRobotRelative()` method to add traction control and smooth module rotation. This fits naturally inside a superstructure's `periodic()` where it processes drive requests before applying them to hardware.

## Open source repositories worth studying

- **Team 254 (FRC-2025-Public)**: `SuperstructureStateMachine` + `PathfindingAuto` with custom A* pathfinding and PathPlanner trajectory following. Gold standard for superstructure design, though drive is kept separate from the superstructure.
- **Team 401 (2025-Robot-Code)**: Best example of PathPlanner wrapped as a drivetrain state machine state, with `StrategyManager` coordinating drive states and `ScoringSubsystem`. Uses JSON-defined autos and semi-autonomous subsystems with supplier-based gating.
- **Team 1678 (C2024-Public, C2025-Public)**: Non-command-based, 254-style architecture with a request queue for superstructure actions and an `Action` interface for autonomous sequencing. Shows `WaitForSuperstructureAction` pattern for synchronizing drive and mechanisms.
- **Team 6328 (RobotCode2025Public, RobotCode2026Public)**: AdvantageKit-based with hardware IO abstraction. AutoBuilder configured in the Drive subsystem with clean separation from mechanism subsystems.
- **Team 3476 (FRC-2022)**: Explicit `DriveState` enum switching between teleop and auto modes in a single `periodic()` method — the simplest version of the drive request pattern.
- **Team 3847 Spectrum (2025-Spectrum)**: Trigger-based state architecture that eliminates explicit state machines. WPILib Triggers represent robot states, and PathPlanner event markers integrate naturally through the same trigger system.
- **CTRE Phoenix6-Examples (SwerveWithPathPlanner)**: Canonical starting point for CTRE + PathPlanner integration, with the `configureAutoBuilder()` method that demonstrates the consumer lambda bridge.

## Conclusion: choosing the right pattern for your architecture

The right approach depends on how much authority your superstructure needs over drivebase outputs. If the superstructure needs to **gate, scale, or modify** drive commands based on mechanism state (e.g., reduce speed when elevator is raised, brake when an unsafe transition is detected), use **Pattern 2** — route PathPlanner's ChassisSpeeds through the superstructure via the consumer lambda, store them as a drive request, and let `periodic()` apply them after any safety processing. If the superstructure only needs to **coordinate timing** with the drivebase (e.g., start intaking when the robot reaches a waypoint), **Pattern 1** with `NamedCommands` is simpler and battle-tested by the majority of top teams. If your drivetrain has its own complex state machine with multiple operating modes, **Pattern 3** — wrapping PathPlanner as one state among many — gives the cleanest abstraction.

The single most important implementation detail: **PathPlanner's consumer lambda is your injection point.** Whatever architectural pattern you choose, the consumer lambda in `AutoBuilder.configure()` is where PathPlanner's trajectory output enters your control flow. Make that lambda write to your superstructure's state rather than directly to hardware, and the superstructure retains full authority over when and how those speeds reach the motors.
