package frc.robot.commands;

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.events.EventScheduler;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.HighFrequencyPathController;
import java.util.Collections;
import java.util.Optional;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * A path-following command that delegates trajectory execution to a {@link
 * HighFrequencyPathController} running at 100Hz on a dedicated thread.
 *
 * <p>This command handles trajectory generation and event scheduling on the main 50Hz loop, while
 * the actual PID control and motor output happen at 100Hz on the controller thread. This is the
 * same architecture Team 254 uses in their 2025 PathPlanner fork.
 *
 * <p><b>Main thread (50Hz):</b> Generates trajectory, manages events, checks completion
 *
 * <p><b>Controller thread (100Hz):</b> Samples trajectory, runs PID, sends motor commands
 */
public class FollowPathHFCommand extends Command {
  private final Timer timer = new Timer();
  private final PathPlannerPath originalPath;
  private final Supplier<Pose2d> poseSupplier;
  private final Supplier<ChassisSpeeds> speedsSupplier;
  private final HighFrequencyPathController controller;
  private final RobotConfig robotConfig;
  private final BooleanSupplier shouldFlipPath;
  private final EventScheduler eventScheduler;

  private PathPlannerPath path;
  private PathPlannerTrajectory trajectory;

  /**
   * Constructs a high-frequency path following command.
   *
   * @param path the path to follow
   * @param poseSupplier supplier for current field-relative pose
   * @param speedsSupplier supplier for current robot-relative chassis speeds
   * @param controller the 100Hz controller that will execute the trajectory
   * @param robotConfig robot physical configuration for trajectory generation
   * @param shouldFlipPath whether to flip the path for red alliance
   * @param requirements subsystem requirements (typically the drive subsystem)
   */
  public FollowPathHFCommand(
      PathPlannerPath path,
      Supplier<Pose2d> poseSupplier,
      Supplier<ChassisSpeeds> speedsSupplier,
      HighFrequencyPathController controller,
      RobotConfig robotConfig,
      BooleanSupplier shouldFlipPath,
      Subsystem... requirements) {
    this.originalPath = path;
    this.poseSupplier = poseSupplier;
    this.speedsSupplier = speedsSupplier;
    this.controller = controller;
    this.robotConfig = robotConfig;
    this.shouldFlipPath = shouldFlipPath;
    this.eventScheduler = new EventScheduler();

    Set<Subsystem> driveRequirements = Set.of(requirements);
    addRequirements(requirements);

    var eventReqs = EventScheduler.getSchedulerRequirements(this.originalPath);
    if (!Collections.disjoint(driveRequirements, eventReqs)) {
      throw new IllegalArgumentException(
          "Events triggered during path following cannot require the drive subsystem");
    }
    addRequirements(eventReqs);

    this.path = this.originalPath;
    // Pre-generate the ideal trajectory so it's ready when we initialize
    Optional<PathPlannerTrajectory> idealTrajectory =
        this.path.getIdealTrajectory(this.robotConfig);
    idealTrajectory.ifPresent(traj -> this.trajectory = traj);
  }

  @Override
  public void initialize() {
    if (shouldFlipPath.getAsBoolean() && !originalPath.preventFlipping) {
      path = originalPath.flipPath();
    } else {
      path = originalPath;
    }

    Pose2d currentPose = poseSupplier.get();
    ChassisSpeeds currentSpeeds = speedsSupplier.get();

    double linearVel = Math.hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);

    if (path.getIdealStartingState() != null) {
      boolean idealVelocity =
          Math.abs(linearVel - path.getIdealStartingState().velocityMPS()) <= 0.25;
      boolean idealRotation =
          !robotConfig.isHolonomic
              || Math.abs(
                      currentPose
                          .getRotation()
                          .minus(path.getIdealStartingState().rotation())
                          .getDegrees())
                  <= 30.0;
      if (idealVelocity && idealRotation) {
        trajectory = path.getIdealTrajectory(robotConfig).orElseThrow();
      } else {
        trajectory = path.generateTrajectory(currentSpeeds, currentPose.getRotation(), robotConfig);
      }
    } else {
      trajectory = path.generateTrajectory(currentSpeeds, currentPose.getRotation(), robotConfig);
    }

    // Hand the trajectory to the 100Hz controller — it starts following immediately
    controller.accept(trajectory);

    PathPlannerLogging.logActivePath(path);
    eventScheduler.initialize(trajectory);

    timer.reset();
    timer.start();

    Logger.recordOutput(
        "HighFreqController/TrajectoryTotalTimeSec", trajectory.getTotalTimeSeconds());
  }

  @Override
  public void execute() {
    double currentTime = timer.get();

    Pose2d currentPose = poseSupplier.get();
    PathPlannerLogging.logCurrentPose(currentPose);

    if (trajectory != null) {
      var targetState = trajectory.sample(currentTime);
      PathPlannerLogging.logTargetPose(targetState.pose);
    }

    eventScheduler.execute(currentTime);
  }

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(trajectory.getTotalTimeSeconds());
  }

  @Override
  public void end(boolean interrupted) {
    timer.stop();

    if (!interrupted && path.getGoalEndState().velocityMPS() < 0.1) {
      controller.stop();
    } else {
      // Interrupted or path ends with nonzero velocity — just clear the trajectory
      // so the next command can take over smoothly
      controller.accept(null);
    }

    PathPlannerLogging.logActivePath(null);
    eventScheduler.end();
  }
}
