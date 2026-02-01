// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import org.littletonrobotics.junction.Logger;

/**
 * Utility class for visualizing field elements and debug geometry in AdvantageScope. Call {@link
 * #update(Pose2d)} periodically (e.g., in Robot.periodic()) to log visualizations.
 */
public class FieldVisualizer {

  private static final String LOG_PREFIX = "FieldViz/";

  /**
   * Updates all field visualizations. Call this periodically with the current robot pose.
   *
   * @param robotPose The current robot pose on the field
   */
  public static void update(Pose2d robotPose) {
    logHubPoints();
    logLineToHub(robotPose);
  }

  /** Logs the hub center and corner points for visualization. */
  public static void logHubPoints() {
    // Hub center (3D - shows height)
    Logger.recordOutput(
        LOG_PREFIX + "HubCenter3d",
        new Pose3d(FieldConstants.Hub.topCenterPoint, new Rotation3d()));

    // Hub center (2D - for field view)
    Logger.recordOutput(
        LOG_PREFIX + "HubCenter2d",
        new Pose2d(FieldConstants.Hub.topCenterPoint.toTranslation2d(), new Rotation2d()));

    // Hub corners (shows the hub footprint)
    Logger.recordOutput(
        LOG_PREFIX + "HubCorners",
        new Pose2d[] {
          new Pose2d(FieldConstants.Hub.nearLeftCorner, new Rotation2d()),
          new Pose2d(FieldConstants.Hub.nearRightCorner, new Rotation2d()),
          new Pose2d(FieldConstants.Hub.farRightCorner, new Rotation2d()),
          new Pose2d(FieldConstants.Hub.farLeftCorner, new Rotation2d())
        });

    // Inner hub center (scoring target height)
    Logger.recordOutput(
        LOG_PREFIX + "HubInnerCenter",
        new Pose3d(FieldConstants.Hub.innerCenterPoint, new Rotation3d()));

    // Opposing hub center
    Logger.recordOutput(
        LOG_PREFIX + "OppHubCenter",
        new Pose3d(FieldConstants.Hub.oppTopCenterPoint, new Rotation3d()));
  }

  /**
   * Logs a trajectory/line from the robot's current position to the hub center. In AdvantageScope,
   * display this as a "Trajectory" type to see the line.
   *
   * @param robotPose The current robot pose
   */
  public static void logLineToHub(Pose2d robotPose) {
    Translation2d hubCenter = FieldConstants.Hub.topCenterPoint.toTranslation2d();
    Translation2d robotPosition = robotPose.getTranslation();

    // Calculate angle from robot to hub
    Rotation2d angleToHub = hubCenter.minus(robotPosition).getAngle();

    // Log as array of poses - AdvantageScope renders this as a trajectory/path
    Pose2d[] lineToHub =
        new Pose2d[] {
          new Pose2d(robotPosition, angleToHub), // Start at robot, facing hub
          new Pose2d(hubCenter, angleToHub) // End at hub center
        };

    Logger.recordOutput(LOG_PREFIX + "LineToHub", lineToHub);

    // Also log the distance for reference
    double distanceToHub = robotPosition.getDistance(hubCenter);
    Logger.recordOutput(LOG_PREFIX + "DistanceToHubMeters", distanceToHub);

    // Log angle to hub (useful for aiming)
    Logger.recordOutput(LOG_PREFIX + "AngleToHubDegrees", angleToHub.getDegrees());
  }

  /**
   * Logs a line between any two points on the field.
   *
   * @param name The name for this line in the logs
   * @param start The starting point
   * @param end The ending point
   */
  public static void logLine(String name, Translation2d start, Translation2d end) {
    Rotation2d angle = end.minus(start).getAngle();
    Pose2d[] line = new Pose2d[] {new Pose2d(start, angle), new Pose2d(end, angle)};
    Logger.recordOutput(LOG_PREFIX + name, line);
  }

  /**
   * Logs a single point on the field.
   *
   * @param name The name for this point in the logs
   * @param point The point to log
   */
  public static void logPoint(String name, Translation2d point) {
    Logger.recordOutput(LOG_PREFIX + name, new Pose2d(point, new Rotation2d()));
  }

  /**
   * Logs a single 3D point on the field.
   *
   * @param name The name for this point in the logs
   * @param point The 3D point to log
   */
  public static void logPoint3d(String name, edu.wpi.first.math.geometry.Translation3d point) {
    Logger.recordOutput(LOG_PREFIX + name, new Pose3d(point, new Rotation3d()));
  }
}
