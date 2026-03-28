package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Timer;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * A 100Hz path-following controller that runs on a high-priority OS thread via a {@link Notifier}.
 *
 * <p>Inspired by Team 254's 2025 threading architecture. The standard PathPlanner controller runs
 * at 50Hz on the main robot loop. This controller runs at 100Hz (10ms period) on a dedicated thread
 * with OS priority 41, providing:
 *
 * <ul>
 *   <li>2x higher PID update rate for tighter trajectory tracking
 *   <li>Decoupled from main loop jitter caused by vision processing, logging, etc.
 *   <li>Lower latency between pose measurement and motor output
 * </ul>
 *
 * <p><b>Thread safety:</b> Uses {@code volatile} fields for zero-contention handoff between the
 * main thread (which sets trajectories) and the controller thread (which samples and drives). No
 * locks are needed because Java guarantees atomic reads/writes of object references.
 */
public class HighFrequencyPathController implements Consumer<PathPlannerTrajectory> {
  private static final double CONTROLLER_PERIOD_SECONDS = 0.01; // 100Hz
  private static final int CONTROLLER_THREAD_PRIORITY = 41;
  private static final double SPEED_DEADBAND_MPS = 0.05;
  private static final double OMEGA_DEADBAND_RAD_PER_SEC = 0.05;

  private final PPHolonomicDriveController controller;
  private final Supplier<Pose2d> poseSupplier;
  private final Consumer<SwerveRequest> driveConsumer;
  private final Notifier notifier;

  // Volatile fields for lock-free thread communication (254 pattern)
  private volatile PathPlannerTrajectory trajectory = null;
  private volatile Timer timer = null;

  private boolean hasSetPriority = false;

  private final SwerveRequest.ApplyRobotSpeeds stopRequest =
      new SwerveRequest.ApplyRobotSpeeds().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final SwerveRequest.ApplyRobotSpeeds driveRequest =
      new SwerveRequest.ApplyRobotSpeeds()
          .withDriveRequestType(DriveRequestType.Velocity)
          .withDesaturateWheelSpeeds(true);

  /**
   * Creates a new high-frequency path controller.
   *
   * @param controller the PathPlanner PID controller to use for trajectory tracking
   * @param poseSupplier supplier for the current field-relative robot pose (must be thread-safe)
   * @param driveConsumer consumer that applies a SwerveRequest to the drivetrain
   */
  public HighFrequencyPathController(
      PPHolonomicDriveController controller,
      Supplier<Pose2d> poseSupplier,
      Consumer<SwerveRequest> driveConsumer) {
    this.controller = controller;
    this.poseSupplier = poseSupplier;
    this.driveConsumer = driveConsumer;
    this.timer = new Timer();
    this.notifier = new Notifier(this::controllerLoop);
    this.notifier.setName("HighFreqPathController");
    this.notifier.startPeriodic(CONTROLLER_PERIOD_SECONDS);
  }

  /**
   * Accepts a new trajectory to follow. Called from the main robot thread when a path command
   * starts or when a new path segment becomes available.
   *
   * <p>Passing {@code null} stops the controller output (used when a path is interrupted
   * mid-motion). Passing a stay-in-place trajectory commands zero output and clears the active
   * trajectory.
   *
   * @param newTrajectory the trajectory to follow, or null to stop output
   */
  @Override
  public void accept(PathPlannerTrajectory newTrajectory) {
    if (newTrajectory == null) {
      trajectory = null;
      return;
    }

    trajectory = newTrajectory;
    timer = new Timer();
    timer.reset();
    timer.start();
    controller.reset(poseSupplier.get(), new ChassisSpeeds());
  }

  /**
   * The 100Hz control loop running on the Notifier thread. Samples the current trajectory, computes
   * PID output, and sends wheel commands directly to the drivetrain.
   */
  private void controllerLoop() {
    if (!hasSetPriority) {
      hasSetPriority = Threads.setCurrentThreadPriority(true, CONTROLLER_THREAD_PRIORITY);
    }

    // Snapshot volatile fields for consistent reads within this cycle
    PathPlannerTrajectory traj = trajectory;
    Timer t = timer;

    if (traj == null || t == null) {
      return;
    }

    // Check if trajectory is complete
    double now = t.get();
    if (now > traj.getTotalTimeSeconds()) {
      return;
    }

    PathPlannerTrajectoryState targetState = traj.sample(now);
    Pose2d currentPose = poseSupplier.get();

    ChassisSpeeds speeds = controller.calculateRobotRelativeSpeeds(currentPose, targetState);
    speeds = applyDeadband(speeds);

    if (DriverStation.isEnabled()) {
      driveConsumer.accept(
          driveRequest
              .withSpeeds(speeds)
              .withWheelForceFeedforwardsX(targetState.feedforwards.robotRelativeForcesXNewtons())
              .withWheelForceFeedforwardsY(targetState.feedforwards.robotRelativeForcesYNewtons()));
    }

    // Log on the controller thread for accurate timing
    Logger.recordOutput("HighFreqController/TargetPose", targetState.pose);
    Logger.recordOutput("HighFreqController/TargetSpeeds", targetState.fieldSpeeds);
    Logger.recordOutput("HighFreqController/TrajectoryTimeSec", now);
  }

  /**
   * Sends a stop command through the high-frequency controller. Used when a path ends at zero
   * velocity.
   */
  public void stop() {
    trajectory = null;
    driveConsumer.accept(stopRequest);
  }

  /**
   * Returns the remaining time on the current trajectory, or 0 if no trajectory is active.
   *
   * @return remaining time in seconds
   */
  public double getRemainingTimeSeconds() {
    PathPlannerTrajectory traj = trajectory;
    Timer t = timer;
    if (traj == null || t == null) {
      return 0.0;
    }
    return Math.max(0.0, traj.getTotalTimeSeconds() - t.get());
  }

  /**
   * Returns whether the controller is actively following a trajectory.
   *
   * @return true if a trajectory is loaded and time has not elapsed
   */
  public boolean isActive() {
    PathPlannerTrajectory traj = trajectory;
    Timer t = timer;
    return traj != null && t != null && t.get() <= traj.getTotalTimeSeconds();
  }

  private static ChassisSpeeds applyDeadband(ChassisSpeeds input) {
    if (Math.hypot(input.vxMetersPerSecond, input.vyMetersPerSecond) < SPEED_DEADBAND_MPS) {
      input.vxMetersPerSecond = 0.0;
      input.vyMetersPerSecond = 0.0;
    }
    if (Math.abs(input.omegaRadiansPerSecond) < OMEGA_DEADBAND_RAD_PER_SEC) {
      input.omegaRadiansPerSecond = 0.0;
    }
    return input;
  }
}
