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

  // Enums

  /** Wanted states set externally to request Lakitu behavior. */
  public enum LakituWantedStates {
    FOLLOW_PATH,
    RECOVER
  }

  /** Internal states representing the resolved Lakitu behavior. */
  public enum LakituInternalStates {
    FOLLOWING,
    PATHFINDING_TO_RECOVERY,
    FOLLOWING_RECOVERY,
    COMPLETED,
    FAILED
  }

  // Dependencies
  private final PathPlannerPath originalPath;
  private final Function<PathPlannerPath, Command> followCommandBuilder;
  private final BiFunction<Pose2d, PathConstraints, Command> pathfindCommandBuilder;
  private final Supplier<Pose2d> poseSupplier;
  private final Supplier<Pose2d> targetPoseSupplier;
  private final PathPlannerPath recoveryPath;

  // State variables
  private LakituWantedStates wantedState = LakituWantedStates.FOLLOW_PATH;
  private LakituInternalStates currentState = LakituInternalStates.FOLLOWING;
  private LakituInternalStates previousState = LakituInternalStates.FOLLOWING;
  private Command activeCommand;
  private int recoveryAttempts;

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
    } else {
      this.recoveryPath = null;
    }
  }

  // State machine methods

  /** Returns the current internal state. */
  public LakituInternalStates getState() {
    return currentState;
  }

  @Override
  public void initialize() {
    wantedState = LakituWantedStates.FOLLOW_PATH;
    currentState = LakituInternalStates.FOLLOWING;
    previousState = LakituInternalStates.FOLLOWING;
    recoveryAttempts = 0;
    startActiveCommand(followCommandBuilder.apply(originalPath));
  }

  @Override
  public void execute() {
    applyState();
    updateState();

    Logger.recordOutput("Lakitu/WantedState", wantedState);
    Logger.recordOutput("Lakitu/CurrentState", currentState);
    Logger.recordOutput("Lakitu/PreviousState", previousState);
    Logger.recordOutput("Lakitu/RecoveryAttempts", recoveryAttempts);
    Logger.recordOutput("Lakitu/Path", originalPath.name);
  }

  @Override
  public void end(boolean interrupted) {
    if (activeCommand != null) {
      activeCommand.end(true);
      activeCommand = null;
    }
  }

  @Override
  public boolean isFinished() {
    return currentState == LakituInternalStates.COMPLETED
        || currentState == LakituInternalStates.FAILED;
  }

  /**
   * Resolves the wanted state into an internal state based on active command completion and
   * deviation checks.
   */
  private void updateState() {
    previousState = currentState;

    switch (wantedState) {
      case FOLLOW_PATH:
        if (activeCommand.isFinished()) {
          activeCommand.end(false);
          if (recoveryPath != null && !isAtEndOfPath()) {
            transitionToRecovery();
          } else {
            currentState = LakituInternalStates.COMPLETED;
          }
        } else if (recoveryPath != null && getDeviation() > RECOVERY_THRESHOLD_METERS) {
          activeCommand.end(true);
          transitionToRecovery();
        }
        break;
      case RECOVER:
        if (activeCommand.isFinished()) {
          activeCommand.end(false);

          if (currentState == LakituInternalStates.PATHFINDING_TO_RECOVERY) {
            currentState = LakituInternalStates.FOLLOWING_RECOVERY;
            startActiveCommand(followCommandBuilder.apply(recoveryPath));
          } else if (currentState == LakituInternalStates.FOLLOWING_RECOVERY) {
            currentState = LakituInternalStates.COMPLETED;
          }
        }
        break;
    }
  }

  /** Executes the active command for the current internal state. */
  private void applyState() {
    switch (currentState) {
      case FOLLOWING:
      case PATHFINDING_TO_RECOVERY:
      case FOLLOWING_RECOVERY:
        activeCommand.execute();
        break;
      case COMPLETED:
      case FAILED:
      default:
        break;
    }
  }

  // ---------------------------------------------------------------------------
  // Helpers
  // ---------------------------------------------------------------------------

  /** Transitions to recovery, or fails if max attempts are exceeded. */
  private void transitionToRecovery() {
    recoveryAttempts++;

    if (recoveryAttempts > MAX_RECOVERY_ATTEMPTS) {
      currentState = LakituInternalStates.FAILED;
      activeCommand = null;
      return;
    }

    wantedState = LakituWantedStates.RECOVER;
    currentState = LakituInternalStates.PATHFINDING_TO_RECOVERY;

    Pose2d recoveryTarget =
        recoveryPath.getStartingHolonomicPose().orElseGet(() -> recoveryPath.getPathPoses().get(0));
    startActiveCommand(pathfindCommandBuilder.apply(recoveryTarget, RECOVERY_CONSTRAINTS));
  }

  /** Returns the distance between the robot's current pose and the path follower's target. */
  private double getDeviation() {
    return poseSupplier
        .get()
        .getTranslation()
        .getDistance(targetPoseSupplier.get().getTranslation());
  }

  /** Returns whether the robot is within tolerance of the original path's end pose. */
  private boolean isAtEndOfPath() {
    List<Pose2d> pathPoses = originalPath.getPathPoses();
    Pose2d endPose = pathPoses.get(pathPoses.size() - 1);
    return poseSupplier.get().getTranslation().getDistance(endPose.getTranslation())
        <= END_TOLERANCE_METERS;
  }

  /** Starts a new active command, initializing it immediately. */
  private void startActiveCommand(Command command) {
    activeCommand = command;
    activeCommand.initialize();
  }
}
