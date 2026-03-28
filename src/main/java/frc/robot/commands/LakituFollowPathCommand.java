package frc.robot.commands;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.function.Function;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Wraps a path-following command with collision recovery behavior inspired by Mario Kart's Lakitu.
 *
 * <p>Monitors the distance between the robot's current pose and the path follower's target pose
 * every cycle. If the deviation exceeds a threshold (indicating a collision or major disturbance),
 * the system interrupts the current path, generates an on-the-fly recovery path back to the start
 * of the current path segment, and restarts the original path.
 *
 * <p>Paths must be registered via {@link #registerPath(String)} to receive Lakitu recovery
 * behavior. Unregistered paths use standard path following.
 *
 * @see <a href="docs/lakitu-recovery-system.md">Lakitu Recovery System Design Doc</a>
 */
public class LakituFollowPathCommand extends Command {

  private static final double RECOVERY_THRESHOLD_METERS = 1.5;
  private static final int MAX_RECOVERY_ATTEMPTS = 2;
  private static final double RECOVERY_MAX_VELOCITY_MPS = 3.0;
  private static final double RECOVERY_MAX_ACCELERATION_MPSSQ = 6.0;
  private static final double RECOVERY_MAX_ANGULAR_VELOCITY_RAD_PER_SEC = Math.toRadians(360);
  private static final double RECOVERY_MAX_ANGULAR_ACCELERATION_RAD_PER_SEC_SQ =
      Math.toRadians(540);

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

  private enum State {
    FOLLOWING,
    RECOVERING,
    FAILED
  }

  private final PathPlannerPath originalPath;
  private final Function<PathPlannerPath, Command> commandBuilder;
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
   * @param commandBuilder function that creates a FollowPathCommand for a given path
   * @param poseSupplier supplier of the robot's current estimated pose
   * @param targetPoseSupplier supplier of the path follower's current target pose (from
   *     PathPlannerLogging callback)
   * @param requirements subsystems required by this command (the drivetrain)
   */
  public LakituFollowPathCommand(
      PathPlannerPath path,
      Function<PathPlannerPath, Command> commandBuilder,
      Supplier<Pose2d> poseSupplier,
      Supplier<Pose2d> targetPoseSupplier,
      Subsystem... requirements) {
    this.originalPath = path;
    this.commandBuilder = commandBuilder;
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

  private void executeFollowing() {
    activeCommand.execute();

    if (activeCommand.isFinished()) {
      activeCommand.end(false);
      activeCommand = null;
      pathCompleted = true;
      return;
    }

    double deviationMeters =
        poseSupplier
            .get()
            .getTranslation()
            .getDistance(targetPoseSupplier.get().getTranslation());
    Logger.recordOutput("Lakitu/DeviationMeters", deviationMeters);

    if (deviationMeters > RECOVERY_THRESHOLD_METERS) {
      recoveryAttempts++;
      Logger.recordOutput("Lakitu/RecoveryAttempts", recoveryAttempts);

      if (recoveryAttempts > MAX_RECOVERY_ATTEMPTS) {
        activeCommand.end(true);
        activeCommand = null;
        state = State.FAILED;
        logState();
        return;
      }

      activeCommand.end(true);
      startRecovery();
      state = State.RECOVERING;
      logState();
    }
  }

  private void executeRecovering() {
    activeCommand.execute();

    if (activeCommand.isFinished()) {
      activeCommand.end(false);
      startFollowing();
      state = State.FOLLOWING;
      logState();
    }
  }

  private void startFollowing() {
    activeCommand = commandBuilder.apply(originalPath);
    activeCommand.initialize();
    Logger.recordOutput("Lakitu/Active", true);
    Logger.recordOutput("Lakitu/Path", originalPath.name);
  }

  private void startRecovery() {
    Pose2d currentPose = poseSupplier.get();
    Pose2d checkpointPose =
        originalPath.getStartingHolonomicPose().orElseGet(() -> originalPath.getPathPoses().get(0));

    Translation2d delta = checkpointPose.getTranslation().minus(currentPose.getTranslation());
    Rotation2d travelHeading = delta.getAngle();

    List<Waypoint> waypoints =
        PathPlannerPath.waypointsFromPoses(
            new Pose2d(currentPose.getTranslation(), travelHeading),
            new Pose2d(checkpointPose.getTranslation(), travelHeading));

    PathPlannerPath recoveryPath =
        new PathPlannerPath(
            waypoints,
            new PathConstraints(
                RECOVERY_MAX_VELOCITY_MPS,
                RECOVERY_MAX_ACCELERATION_MPSSQ,
                RECOVERY_MAX_ANGULAR_VELOCITY_RAD_PER_SEC,
                RECOVERY_MAX_ANGULAR_ACCELERATION_RAD_PER_SEC_SQ),
            null,
            new GoalEndState(0.0, checkpointPose.getRotation()));
    recoveryPath.name = "Lakitu Recovery";

    activeCommand = commandBuilder.apply(recoveryPath);
    activeCommand.initialize();

    Logger.recordOutput("Lakitu/RecoveryTarget", checkpointPose);
  }

  private void logState() {
    Logger.recordOutput("Lakitu/State", state.name());
    Logger.recordOutput("Lakitu/RecoveryAttempts", recoveryAttempts);
  }
}
