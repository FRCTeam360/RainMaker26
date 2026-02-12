package frc.robot.subsystems.QuestNav;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import gg.questnav.questnav.QuestNav;
import gg.questnav.questnav.PoseFrame;
import org.littletonrobotics.junction.Logger;

/**
 * Subsystem for integrating QuestNav visual-inertial odometry with the robot's pose estimation.
 *
 * <p>QuestNav uses a Meta Quest 3S headset to provide high-frequency (120Hz) pose updates via
 * NetworkTables. This subsystem reads pose frames from the Quest and feeds them into the swerve
 * drive's Kalman filter-based pose estimator.
 */
public class QuestNavSubsystem extends SubsystemBase {
  private final QuestNav questNav = new QuestNav();
  private final CommandSwerveDrivetrain drivetrain;

  // Transform from robot center to Quest headset position (meters)
  // X = forward, Y = left, Z = up from robot center
  // TODO: Measure and update these values for your specific mount position
  private final Transform3d robotToQuest;

  // Standard deviations for QuestNav measurements [x, y, theta]
  // Lower values = more trust in QuestNav data
  private static final Matrix<N3, N1> QUESTNAV_STD_DEVS = VecBuilder.fill(0.02, 0.02, 0.035);

  // Track connection status
  private boolean wasTracking = false;

  /**
   * Creates a QuestNavSubsystem with the specified robot-to-Quest transform.
   *
   * @param drivetrain The swerve drivetrain to feed pose measurements to
   * @param robotToQuest Transform from robot center to Quest headset mount position
   */
  public QuestNavSubsystem(CommandSwerveDrivetrain drivetrain, Transform3d robotToQuest) {
    this.drivetrain = drivetrain;
    this.robotToQuest = robotToQuest;
  }

  /**
   * Creates a QuestNavSubsystem with default transform values. You should measure and update these
   * for your specific mount.
   *
   * @param drivetrain The swerve drivetrain to feed pose measurements to
   */
  public QuestNavSubsystem(CommandSwerveDrivetrain drivetrain) {
    this(
        drivetrain,
        new Transform3d(
            new Translation3d(0.0, 0.0, 0.5), // 0.5m above robot center - ADJUST THIS
            new Rotation3d(0, 0, 0) // No rotation - ADJUST IF HEADSET IS ROTATED
            ));
  }

  @Override
  public void periodic() {
    // REQUIRED: Must call this every loop for QuestNav to function
    questNav.commandPeriodic();

    // Get all unread pose frames and feed them to the pose estimator
    PoseFrame[] frames = questNav.getAllUnreadPoseFrames();

    boolean isCurrentlyTracking = false;

    for (PoseFrame frame : frames) {
      if (frame.isTracking()) {
        isCurrentlyTracking = true;

        // Get Quest pose and transform to robot-centric coordinates
        Pose3d questPose = frame.questPose3d();
        Pose3d robotPose = questPose.transformBy(robotToQuest.inverse());

        // Feed to swerve drive pose estimator
        drivetrain.addVisionMeasurement(
            robotPose.toPose2d(), frame.dataTimestamp(), QUESTNAV_STD_DEVS);
      }
    }

    // Log tracking status changes
    if (isCurrentlyTracking != wasTracking) {
      if (isCurrentlyTracking) {
        System.out.println("[QuestNav] Tracking acquired");
      } else {
        System.out.println("[QuestNav] Tracking lost");
      }
      wasTracking = isCurrentlyTracking;
    }

    // Log data to AdvantageKit
    Logger.recordOutput("QuestNav/IsTracking", isCurrentlyTracking);
    Logger.recordOutput("QuestNav/FrameCount", frames.length);
    if (frames.length > 0 && frames[frames.length - 1].isTracking()) {
      Pose3d latestQuestPose = frames[frames.length - 1].questPose3d();
      Pose3d latestRobotPose = latestQuestPose.transformBy(robotToQuest.inverse());
      Logger.recordOutput("QuestNav/RobotPose", latestRobotPose);
      Logger.recordOutput("QuestNav/QuestPose", latestQuestPose);
    }
  }

  /**
   * Resets the QuestNav pose to a known robot position. Call this at the start of autonomous when
   * the robot is at a known field position.
   *
   * @param robotPose The known robot pose on the field
   */
  public void resetPose(Pose3d robotPose) {
    Pose3d questPose = robotPose.transformBy(robotToQuest);
    questNav.setPose(questPose);
    System.out.println("[QuestNav] Pose reset to: " + robotPose.toPose2d());
  }

  /**
   * Resets the QuestNav pose using a 2D pose (assumes z=0).
   *
   * @param robotPose2d The known robot pose on the field
   */
  public void resetPose(Pose2d robotPose2d) {
    resetPose(new Pose3d(robotPose2d));
  }

  /**
   * Returns whether the Quest headset is currently tracking.
   *
   * @return true if tracking, false otherwise
   */
  public boolean isTracking() {
    return wasTracking;
  }

  /**
   * Returns the transform from robot center to Quest headset.
   *
   * @return The robot-to-Quest transform
   */
  public Transform3d getRobotToQuestTransform() {
    return robotToQuest;
  }
}
