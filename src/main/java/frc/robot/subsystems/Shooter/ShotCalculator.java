// Shoot-on-the-move logic adapted from FRC 6328 Mechanical Advantage
// https://github.com/Mechanical-Advantage/RobotCode2026Public

package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.utils.FieldConstants;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Calculates shooting parameters (hood angle, flywheel speed, and drivebase heading) based on the
 * robot's distance to the hub. Uses interpolation maps to convert distance into mechanism
 * setpoints. Supports shoot-on-the-move by compensating for robot velocity using a time-of-flight
 * lookahead.
 */
public class ShotCalculator {
  private final Supplier<Pose2d> robotPoseSupplier;
  private final Supplier<Translation2d> targetSupplier;
  private final Supplier<ChassisSpeeds> velocitySupplier;
  private final InterpolatingDoubleTreeMap shotHoodAngleMap;
  private final InterpolatingDoubleTreeMap shotFlywheelSpeedMap;
  private final InterpolatingDoubleTreeMap timeOfFlightMap;
  private final Transform2d robotToShooter;

  private double minDistanceMeters = 0.0;
  private double maxDistanceMeters = Double.MAX_VALUE;

  /**
   * Holds the calculated shooting parameters for a given robot position.
   *
   * @param targetHeading the angle the drivebase should face toward the hub
   * @param hoodAngle the hood angle setpoint in degrees
   * @param flywheelSpeed the flywheel speed setpoint in RPM
   * @param isValid whether the target is within the effective shooting range
   */
  public record ShootingParams(
      Rotation2d targetHeading,
      double hoodAngle,
      double flywheelSpeed,
      double timeOfFlight,
      boolean isValid) {}

  public record RobotShootingInfo(
      InterpolatingDoubleTreeMap shotHoodAngleMap,
      InterpolatingDoubleTreeMap shotFlywheelSpeedMap,
      InterpolatingDoubleTreeMap timeOfFlightMap,
      Transform2d robotToShooter,
      double minDistanceMeters, // should be 0.0 for hub
      double maxDistanceMeters /* should be 5.0 for hub */) {}

  /**
   * Creates a new ShotCalculator.
   *
   * @param robotPoseSupplier the supplier used to obtain the robot's current pose
   * @param targetSupplier the supplier used to obtain the target position
   * @param velocitySupplier the supplier used to obtain the robot's current chassis speeds for
   *     shoot-on-the-move compensation
   * @param robotShootingInfo the shooting configuration (maps, offsets, range limits)
   */
  public ShotCalculator(
      Supplier<Pose2d> robotPoseSupplier,
      Supplier<Translation2d> targetSupplier,
      Supplier<ChassisSpeeds> velocitySupplier,
      RobotShootingInfo robotShootingInfo) {
    this.robotPoseSupplier = robotPoseSupplier;
    this.targetSupplier = targetSupplier;
    this.velocitySupplier = velocitySupplier;
    this.shotHoodAngleMap = robotShootingInfo.shotHoodAngleMap;
    this.shotFlywheelSpeedMap = robotShootingInfo.shotFlywheelSpeedMap;
    this.timeOfFlightMap = robotShootingInfo.timeOfFlightMap;
    this.robotToShooter = robotShootingInfo.robotToShooter;
    this.minDistanceMeters = robotShootingInfo.minDistanceMeters;
    this.maxDistanceMeters = robotShootingInfo.maxDistanceMeters;
  }

  private ShootingParams cachedShootingParams = null;

  /**
   * Calculates and caches the shooting parameters for the current robot position. Compensates for
   * robot velocity by projecting a lookahead position using the time-of-flight map, so the robot
   * can shoot while moving. If parameters have already been calculated and not cleared, returns the
   * cached result.
   *
   * @return the {@link ShootingParams} containing drivebase heading, hood angle, flywheel speed,
   *     and validity
   */
  public ShootingParams calculateShot() {
    if (cachedShootingParams != null) {
      Logger.recordOutput("ShotCalculator/cached", true);
      return cachedShootingParams;
    }
    Logger.recordOutput("ShotCalculator/cached", false);

    Pose2d robotPosition = robotPoseSupplier.get();
    Pose2d shooterPosition = robotPosition.plus(robotToShooter);
    Translation2d target = targetSupplier.get();

    // Compute field-relative shooter velocity for shoot-on-the-move compensation.
    // Start with the robot's translational velocity rotated into the field frame,
    // then add the rotational contribution at the shooter offset from the robot center.
    ChassisSpeeds robotSpeeds = velocitySupplier.get();
    Rotation2d robotHeading = robotPosition.getRotation();
    Translation2d robotFieldVelocity =
        new Translation2d(robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond)
            .rotateBy(robotHeading);

    double robotAngleRad = robotHeading.getRadians();
    double shooterOffsetX = robotToShooter.getX();
    double shooterOffsetY = robotToShooter.getY();
    double omegaRadPerSec = robotSpeeds.omegaRadiansPerSecond;
    double shooterVelXMps =
        robotFieldVelocity.getX()
            + omegaRadPerSec
                * (shooterOffsetY * Math.cos(robotAngleRad)
                    - shooterOffsetX * Math.sin(robotAngleRad));
    double shooterVelYMps =
        robotFieldVelocity.getY()
            + omegaRadPerSec
                * (shooterOffsetX * Math.cos(robotAngleRad)
                    - shooterOffsetY * Math.sin(robotAngleRad));

    // Calculate the initial distance from the shooter to the target
    double shooterToTargetDistanceMeters = target.getDistance(shooterPosition.getTranslation());

    // Use time of flight to project where the shooter will be when the ball arrives.
    // The robot imparts its velocity onto the ball, so we offset the aiming point
    // by (velocity * timeOfFlight) to lead the target.
    double timeOfFlightSecs = timeOfFlightMap.get(shooterToTargetDistanceMeters);
    Translation2d lookaheadPosition =
        shooterPosition
            .getTranslation()
            .plus(
                new Translation2d(
                    shooterVelXMps * timeOfFlightSecs, shooterVelYMps * timeOfFlightSecs));
    double lookaheadDistanceMeters = target.getDistance(lookaheadPosition);

    double effectiveDistanceMeters =
        Math.max(minDistanceMeters, Math.min(maxDistanceMeters, lookaheadDistanceMeters));

    // Calculate heading from lookahead position to target, then rotate 180°
    // because the shooter is at the back of the robot
    Rotation2d targetHeading =
        target.minus(lookaheadPosition).getAngle().rotateBy(Rotation2d.k180deg);

    double hoodAngle = shotHoodAngleMap.get(effectiveDistanceMeters);
    double flywheelSpeed = shotFlywheelSpeedMap.get(effectiveDistanceMeters);
    double timeOfFlight = timeOfFlightMap.get(effectiveDistanceMeters);
    boolean isValid =
        lookaheadDistanceMeters >= minDistanceMeters
            && lookaheadDistanceMeters <= maxDistanceMeters;

    // The virtual target is the real target offset by the velocity compensation — it represents
    // where the robot is effectively aiming from its current position while moving.
    Translation2d velocityOffset =
        new Translation2d(shooterVelXMps * timeOfFlightSecs, shooterVelYMps * timeOfFlightSecs);
    Translation2d virtualTarget = target.minus(velocityOffset);
    Logger.recordOutput(
        "ShotCalculator/virtualTarget", new Pose2d(virtualTarget, Rotation2d.kZero));
    Logger.recordOutput("ShotCalculator/targetPosition", new Pose2d(target, Rotation2d.kZero));
    Logger.recordOutput("ShotCalculator/hubPosition", FieldConstants.Hub.topCenterPoint);
    Logger.recordOutput("ShotCalculator/distanceToTarget", lookaheadDistanceMeters);
    Logger.recordOutput("ShotCalculator/targetFlywheelSpeed", flywheelSpeed);
    Logger.recordOutput("ShotCalculator/targetHoodAngle", hoodAngle);
    Logger.recordOutput("ShotCalculator/targetHeading", targetHeading);
    Logger.recordOutput(
        "ShotCalculator/lookaheadPose", new Pose2d(lookaheadPosition, targetHeading));
    Logger.recordOutput("ShotCalculator/timeOfFlightSecs", timeOfFlightSecs);
    Logger.recordOutput("ShotCalculator/isValid", isValid);

    cachedShootingParams =
        new ShootingParams(targetHeading, hoodAngle, flywheelSpeed, timeOfFlight, isValid);
    return cachedShootingParams;
  }

  /**
   * Clears the cached shooting parameters, forcing a recalculation on the next call to {@link
   * #calculateShot()}.
   */
  public void clearShootingParams() {
    cachedShootingParams = null;
  }
}
