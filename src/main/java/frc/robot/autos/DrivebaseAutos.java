package frc.robot.autos;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.lib.BLine.Path.PathConstraints;
import frc.robot.lib.BLine.Path.Waypoint;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utils.FieldConstants;
import java.util.List;

/**
 * Drive-only autos for the 2025 drivebase test robot (QuestNav testing).
 *
 * <p>These autos are simple drive-only paths (no shooter/intake commands) intended to exercise the
 * drivetrain + QuestNav fusion without any of the comp-bot shooting logic. Each auto starts at the
 * field center with rotation zero; the physical robot should be placed at that pose before running
 * the auto.
 *
 * <p>Starting poses are derived from the path's first waypoint (same convention as {@link
 * BLineAutos}), which is what the disabled-time AutoChooser pose reset seeds from.
 */
public class DrivebaseAutos {
  /** Prefix for all auto names in this class. */
  public static final String AUTO_PREFIX = "[Drivebase] ";

  // ─────────────────────────────────────────────────────────────────────────────
  // Auto definition constants (conservative; tune on the physical robot)
  // ─────────────────────────────────────────────────────────────────────────────

  // Starting pose for every auto (field center, rotation zero).
  private static final double START_X_METERS = FieldConstants.fieldLength / 2.0;
  private static final double START_Y_METERS = FieldConstants.fieldWidth / 2.0;
  private static final Rotation2d START_ROTATION = Rotation2d.kZero;

  private static final double DRIVE_DISTANCE_METERS = 1.5;
  private static final double TURN_DEGREES = 90.0;
  private static final double MAX_VELOCITY_METERS_PER_SEC = 3.0;
  private static final double MAX_VELOCITY_DEGREES_PER_SEC = 360.0;

  private final FollowPath.Builder pathBuilder;

  /**
   * @param drivetrain the swerve drivetrain subsystem
   */
  public DrivebaseAutos(CommandSwerveDrivetrain drivetrain) {
    this.pathBuilder = drivetrain.createBLinePathBuilder();
  }

  /**
   * Builds a simple auto that drives from the start pose to a target pose.
   *
   * @param autoName display name (without prefix)
   * @param targetTranslation target translation
   * @param targetRotation target rotation
   * @return a named auto whose command follows a straight path from the start pose to the target
   */
  private NamedAutoWithPose buildDriveAuto(
      String autoName, Translation2d targetTranslation, Rotation2d targetRotation) {
    PathConstraints constraints =
        new PathConstraints()
            .setMaxVelocityMetersPerSec(MAX_VELOCITY_METERS_PER_SEC)
            .setMaxVelocityDegPerSec(MAX_VELOCITY_DEGREES_PER_SEC);

    Path path =
        new Path(
            constraints,
            new Waypoint(new Translation2d(START_X_METERS, START_Y_METERS), START_ROTATION),
            new Waypoint(targetTranslation, targetRotation));

    return new NamedAutoWithPose(
        AUTO_PREFIX + autoName, pathBuilder.build(path), path.getStartPose());
  }

  /**
   * Returns the drivebase auto routines as named autos.
   *
   * @return all drivebase autos in the chooser
   */
  public List<NamedAutoWithPose> getNamedAutos() {
    return List.of(
        buildDriveAuto(
            "Drive Forward 1.5m",
            new Translation2d(START_X_METERS + DRIVE_DISTANCE_METERS, START_Y_METERS),
            START_ROTATION),
        buildDriveAuto(
            "Turn 90 deg in Place",
            new Translation2d(START_X_METERS, START_Y_METERS),
            Rotation2d.fromDegrees(TURN_DEGREES)),
        buildDriveAuto(
            "Drive Forward and Turn 90 deg",
            new Translation2d(START_X_METERS + DRIVE_DISTANCE_METERS, START_Y_METERS),
            Rotation2d.fromDegrees(TURN_DEGREES)));
  }
}
