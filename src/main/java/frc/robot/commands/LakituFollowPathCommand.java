package frc.robot.commands;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.utils.PathProvider;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.BiFunction;
import java.util.function.Function;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Wraps a path-following command with collision recovery behavior inspired by Mario Kart's Lakitu.
 *
 * <p>Monitors the distance between the robot's current pose and the path follower's target pose
 * every cycle. If the deviation exceeds a threshold (indicating a collision or major disturbance),
 * the system interrupts the current path, pathfinds to a designated recovery path, and follows that
 * recovery path to get the robot back on track.
 *
 * <p>Recovery paths are pre-planned in PathPlanner and registered as pairs with their original
 * paths via {@link #registerPath(String, String)}. This ensures recovery trajectories are smooth
 * and optimized, rather than restarting the original path from the beginning.
 *
 * <p>Paths registered without a recovery path via {@link #registerPath(String)} will follow
 * normally without recovery behavior.
 *
 * @see <a href="docs/lakitu-recovery-system.md">Lakitu Recovery System Design Doc</a>
 */
public class LakituFollowPathCommand extends Command {

  // --- Deviation thresholds ---
  private static final double RECOVERY_THRESHOLD_METERS = 1.5;
  private static final double END_TOLERANCE_METERS = 0.5;

  // --- Recovery limits ---
  private static final int MAX_RECOVERY_ATTEMPTS = 2;

  // --- Recovery path constraints ---
  private static final PathConstraints RECOVERY_CONSTRAINTS =
      new PathConstraints(
          3.0, // maxVelocityMPS
          6.0, // maxAccelerationMPSSq
          Math.toRadians(360), // maxAngularVelocityRadPerSec
          Math.toRadians(540)); // maxAngularAccelerationRadPerSecSq

  // --- Path registry: original path name -> recovery path name ---
  private static final Map<String, String> registeredPaths = new HashMap<>();

  /**
   * Register a path for Lakitu recovery with a designated recovery path. When the robot is
   * disrupted during the original path, it will pathfind to the recovery path and follow it.
   *
   * @param pathName the name of the original path (matching the .path filename)
   * @param recoveryPathName the name of the recovery path to follow after disruption
   */
  public static void registerPath(String pathName, String recoveryPathName) {
    registeredPaths.put(pathName, recoveryPathName);
  }

  /**
   * Register a path for Lakitu wrapping without a recovery path. The path will follow normally
   * without recovery behavior on disruption.
   *
   * @param pathName the name of the path (matching the .path filename)
   */
  public static void registerPath(String pathName) {
    registeredPaths.put(pathName, null);
  }

  /**
   * Check if a path is registered for Lakitu recovery.
   *
   * @param pathName the path name to check
   * @return true if the path should use Lakitu recovery
   */
  public static boolean isRegistered(String pathName) {
    return registeredPaths.containsKey(pathName);
  }

  /**
   * Returns the recovery path constraints used for pathfinding to recovery paths.
   *
   * @return the {@link PathConstraints} for recovery pathfinding
   */
  public static PathConstraints getRecoveryConstraints() {
    return RECOVERY_CONSTRAINTS;
  }

  private enum State {
    FOLLOWING,
    PATHFINDING_TO_RECOVERY,
    FOLLOWING_RECOVERY,
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
  private PathPlannerPath recoveryPath;

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

    // Pre-load recovery path if one is registered
    String recoveryPathName = registeredPaths.get(path.name);
    if (recoveryPathName != null) {
      this.recoveryPath = PathProvider.fromPathFile(recoveryPathName);
    }
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
      case PATHFINDING_TO_RECOVERY:
        executePathfindingToRecovery();
        break;
      case FOLLOWING_RECOVERY:
        executeFollowingRecovery();
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

      // Only check end-of-path deviation if a recovery path exists
      if (recoveryPath != null) {
        handlePathTimerExpired();
      } else {
        activeCommand = null;
        pathCompleted = true;
      }
      return;
    }

    // Only monitor deviation if a recovery path exists
    if (recoveryPath != null) {
      checkMidPathDeviation();
    }
  }

  private void executePathfindingToRecovery() {
    activeCommand.execute();

    if (activeCommand.isFinished()) {
      activeCommand.end(false);
      startFollowingRecovery();
      transitionTo(State.FOLLOWING_RECOVERY);
    }
  }

  private void executeFollowingRecovery() {
    activeCommand.execute();

    if (activeCommand.isFinished()) {
      activeCommand.end(false);
      activeCommand = null;
      pathCompleted = true;
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

    startPathfindingToRecovery();
    transitionTo(State.PATHFINDING_TO_RECOVERY);
  }

  private void startFollowing() {
    activeCommand = followCommandBuilder.apply(originalPath);
    activeCommand.initialize();
    Logger.recordOutput("Lakitu/Active", true);
    Logger.recordOutput("Lakitu/Path", originalPath.name);
  }

  /** Starts pathfinding toward the recovery path's starting pose. */
  private void startPathfindingToRecovery() {
    Pose2d recoveryTarget =
        recoveryPath.getStartingHolonomicPose().orElseGet(() -> recoveryPath.getPathPoses().get(0));

    activeCommand = pathfindCommandBuilder.apply(recoveryTarget, RECOVERY_CONSTRAINTS);
    activeCommand.initialize();

    Logger.recordOutput("Lakitu/RecoveryTarget", recoveryTarget);
  }

  /** Starts following the recovery path after pathfinding has reached its start. */
  private void startFollowingRecovery() {
    activeCommand = followCommandBuilder.apply(recoveryPath);
    activeCommand.initialize();
    Logger.recordOutput("Lakitu/FollowingRecoveryPath", recoveryPath.name);
  }

  // ---------------------------------------------------------------------------
  // Helpers
  // ---------------------------------------------------------------------------

  private void transitionTo(State newState) {
    state = newState;
    logState();
  }

  private void logState() {
    Logger.recordOutput("Lakitu/State", state.name());
    Logger.recordOutput("Lakitu/RecoveryAttempts", recoveryAttempts);
  }
}
