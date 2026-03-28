package frc.robot.commands;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.function.BiFunction;
import java.util.function.Function;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Wraps a path-following command with collision recovery behavior inspired by Mario Kart's Lakitu.
 *
 * <p>Monitors the distance between the robot's current pose and the path follower's target pose
 * every cycle. If the deviation exceeds a threshold (indicating a collision or major disturbance),
 * the system interrupts the current path, pathfinds back to the start of the current path segment,
 * and restarts the original path.
 *
 * <p>Paths must be registered via {@link #registerPath(String)} to receive Lakitu recovery
 * behavior. Unregistered paths use standard path following.
 *
 * @see <a href="docs/lakitu-recovery-system.md">Lakitu Recovery System Design Doc</a>
 */
public class LakituFollowPathCommand extends Command {

  // --- Deviation thresholds ---
  private static final double RECOVERY_THRESHOLD_METERS = 1.5;
  private static final double END_TOLERANCE_METERS = 0.5;
  private static final double MIN_RECOVERY_DISTANCE_METERS = 0.3;

  // --- Recovery limits ---
  private static final int MAX_RECOVERY_ATTEMPTS = 2;

  // --- Recovery path constraints ---
  private static final PathConstraints RECOVERY_CONSTRAINTS =
      new PathConstraints(
          3.0, // maxVelocityMPS
          6.0, // maxAccelerationMPSSq
          Math.toRadians(360), // maxAngularVelocityRadPerSec
          Math.toRadians(540)); // maxAngularAccelerationRadPerSecSq

  // --- Path registry ---
  private static final Set<String> registeredPaths = new HashSet<>();

  /**
   * Register a path name for Lakitu recovery behavior. Paths not registered will use standard path
   * following when passed through the command builder.
   *
   * @param pathName the name of the path (matching the .path filename)
   */
  public static void registerPath(String pathName) {
    registeredPaths.add(pathName);
  }

  /**
   * Check if a path is registered for Lakitu recovery.
   *
   * @param pathName the path name to check
   * @return true if the path should use Lakitu recovery
   */
  public static boolean isRegistered(String pathName) {
    return registeredPaths.contains(pathName);
  }

  /**
   * Returns the recovery path constraints used for pathfinding back to checkpoints.
   *
   * @return the {@link PathConstraints} for recovery paths
   */
  public static PathConstraints getRecoveryConstraints() {
    return RECOVERY_CONSTRAINTS;
  }

  private enum State {
    FOLLOWING,
    RECOVERING,
    FAILED
  }

  private final PathPlannerPath originalPath;
  private final Function<PathPlannerPath, Command> followCommandBuilder;
  private final BiFunction<Pose2d, PathConstraints, Command> pathfindCommandBuilder;
  private final Supplier<Pose2d> poseSupplier;
  private final Supplier<Pose2d> targetPoseSupplier;

  private Command activeCommand;
  private State state;
  private int recoveryAttempts;
  private boolean pathCompleted;

  /**
   * Creates a new LakituFollowPathCommand.
   *
   * @param path the original path to follow
   * @param followCommandBuilder function that creates a FollowPathCommand for a given path
   * @param pathfindCommandBuilder function that creates a PathfindingCommand to a target pose with
   *     given constraints
   * @param poseSupplier supplier of the robot's current estimated pose
   * @param targetPoseSupplier supplier of the path follower's current target pose (from
   *     PathPlannerLogging callback)
   * @param requirements subsystems required by this command (the drivetrain)
   */
  public LakituFollowPathCommand(
      PathPlannerPath path,
      Function<PathPlannerPath, Command> followCommandBuilder,
      BiFunction<Pose2d, PathConstraints, Command> pathfindCommandBuilder,
      Supplier<Pose2d> poseSupplier,
      Supplier<Pose2d> targetPoseSupplier,
      Subsystem... requirements) {
    this.originalPath = path;
    this.followCommandBuilder = followCommandBuilder;
    this.pathfindCommandBuilder = pathfindCommandBuilder;
    this.poseSupplier = poseSupplier;
    this.targetPoseSupplier = targetPoseSupplier;
    addRequirements(requirements);
  }

  @Override
  public void initialize() {
    state = State.FOLLOWING;
    recoveryAttempts = 0;
    pathCompleted = false;
    startFollowing();
    logState();
  }

  @Override
  public void execute() {
    switch (state) {
      case FOLLOWING:
        executeFollowing();
        break;
      case RECOVERING:
        executeRecovering();
        break;
      case FAILED:
        break;
    }
  }

  @Override
  public void end(boolean interrupted) {
    if (activeCommand != null) {
      activeCommand.end(true);
      activeCommand = null;
    }
    Logger.recordOutput("Lakitu/Active", false);
  }

  @Override
  public boolean isFinished() {
    return pathCompleted || state == State.FAILED;
  }

  // ---------------------------------------------------------------------------
  // State handlers
  // ---------------------------------------------------------------------------

  private void executeFollowing() {
    activeCommand.execute();

    if (activeCommand.isFinished()) {
      activeCommand.end(false);
      handlePathTimerExpired();
      return;
    }

    checkMidPathDeviation();
  }

  private void executeRecovering() {
    activeCommand.execute();

    if (activeCommand.isFinished()) {
      activeCommand.end(false);
      startFollowing();
      transitionTo(State.FOLLOWING);
    }
  }

  // ---------------------------------------------------------------------------
  // Deviation checks
  // ---------------------------------------------------------------------------

  /**
   * Called when the inner FollowPathCommand's timer expires. Checks whether the robot actually
   * reached the end of the path. If not, triggers recovery.
   */
  private void handlePathTimerExpired() {
    List<Pose2d> pathPoses = originalPath.getPathPoses();
    Pose2d endPose = pathPoses.get(pathPoses.size() - 1);
    double endDistance = poseSupplier.get().getTranslation().getDistance(endPose.getTranslation());

    if (endDistance <= END_TOLERANCE_METERS) {
      activeCommand = null;
      pathCompleted = true;
      return;
    }

    Logger.recordOutput("Lakitu/EndDeviationMeters", endDistance);
    triggerRecoveryOrFail();
  }

  /** Called every cycle during FOLLOWING to check if the robot has drifted too far from target. */
  private void checkMidPathDeviation() {
    double deviationMeters =
        poseSupplier.get().getTranslation().getDistance(targetPoseSupplier.get().getTranslation());
    Logger.recordOutput("Lakitu/DeviationMeters", deviationMeters);

    if (deviationMeters > RECOVERY_THRESHOLD_METERS) {
      activeCommand.end(true);
      triggerRecoveryOrFail();
    }
  }

  // ---------------------------------------------------------------------------
  // Recovery logic
  // ---------------------------------------------------------------------------

  /**
   * Attempts recovery if attempts remain, otherwise transitions to FAILED. Shared by both the
   * mid-path deviation check and the end-of-path tolerance check.
   */
  private void triggerRecoveryOrFail() {
    recoveryAttempts++;
    Logger.recordOutput("Lakitu/RecoveryAttempts", recoveryAttempts);

    if (recoveryAttempts > MAX_RECOVERY_ATTEMPTS) {
      activeCommand = null;
      transitionTo(State.FAILED);
      return;
    }

    startRecovery();
    transitionTo(State.RECOVERING);
  }

  private void startFollowing() {
    activeCommand = followCommandBuilder.apply(originalPath);
    activeCommand.initialize();
    Logger.recordOutput("Lakitu/Active", true);
    Logger.recordOutput("Lakitu/Path", originalPath.name);
  }

  private void startRecovery() {
    Pose2d currentPose = poseSupplier.get();
    Pose2d checkpointPose = getCheckpointPose();

    double distanceToCheckpoint =
        currentPose.getTranslation().getDistance(checkpointPose.getTranslation());

    // If already at the checkpoint, skip pathfinding and restart the path directly
    if (distanceToCheckpoint < MIN_RECOVERY_DISTANCE_METERS) {
      startFollowing();
      transitionTo(State.FOLLOWING);
      return;
    }

    activeCommand = pathfindCommandBuilder.apply(checkpointPose, RECOVERY_CONSTRAINTS);
    activeCommand.initialize();

    Logger.recordOutput("Lakitu/RecoveryTarget", checkpointPose);
  }

  // ---------------------------------------------------------------------------
  // Helpers
  // ---------------------------------------------------------------------------

  /** Returns the starting pose of the original path (the "checkpoint" to recover to). */
  private Pose2d getCheckpointPose() {
    return originalPath
        .getStartingHolonomicPose()
        .orElseGet(() -> originalPath.getPathPoses().get(0));
  }

  private void transitionTo(State newState) {
    state = newState;
    logState();
  }

  private void logState() {
    Logger.recordOutput("Lakitu/State", state.name());
    Logger.recordOutput("Lakitu/RecoveryAttempts", recoveryAttempts);
  }
}
