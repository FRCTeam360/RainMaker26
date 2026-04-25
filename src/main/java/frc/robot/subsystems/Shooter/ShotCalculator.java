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
  private final double maxRobotSpeedMps;

  private final String logCached;
  private final String logVirtualTarget;
  private final String logTargetPosition;
  private final String logHubPosition;
  private final String logDistanceToTarget;
  private final String logTargetFlywheelSpeed;
  private final String logTargetHoodAngle;
  private final String logTargetHeading;
  private final String logLookaheadPose;
  private final String logRobotCenterLookahead;
  private final String logTimeOfFlightSecs;
  private final String logIsValid;
  private final String logRobotSpeeds;
  private final String logCurrentHeadingDeg;
  private final String logHeadingErrorDeg;

  private double minDistanceMeters = 0.0;
  private double maxDistanceMeters = Double.MAX_VALUE;
  private double vPercentageToFlywheelOutput;

  /**
   * Holds the calculated shooting parameters for a given robot position.
   *
   * @param targetHeading the robot heading that aims the shooter at the target, accounting for the
   *     shooter's facing angle relative to the robot
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
      double maxDistanceMeters, /* should be 5.0 for hub */
      double maxRobotSpeedMps,
      double vPercentageToFlywheelOutput) {}

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
      String name,
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
    this.maxRobotSpeedMps = robotShootingInfo.maxRobotSpeedMps;
    this.vPercentageToFlywheelOutput = robotShootingInfo.vPercentageToFlywheelOutput;

    String basePath = "ShotCalculator/" + name;
    this.logCached = basePath + "/cached";
    this.logVirtualTarget = basePath + "/virtualTarget";
    this.logTargetPosition = basePath + "/targetPosition";
    this.logHubPosition = basePath + "/hubPosition";
    this.logDistanceToTarget = basePath + "/distanceToTarget";
    this.logTargetFlywheelSpeed = basePath + "/targetFlywheelSpeed";
    this.logTargetHoodAngle = basePath + "/targetHoodAngle";
    this.logTargetHeading = basePath + "/targetHeading";
    this.logLookaheadPose = basePath + "/lookaheadPose";
    this.logRobotCenterLookahead = basePath + "/robotCenterLookahead";
    this.logTimeOfFlightSecs = basePath + "/timeOfFlightSecs";
    this.logIsValid = basePath + "/isValid";
    this.logRobotSpeeds = basePath + "/robotSpeeds";
    this.logCurrentHeadingDeg = basePath + "/currentHeadingDeg";
    this.logHeadingErrorDeg = basePath + "/headingErrorDeg";
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
      Logger.recordOutput(logCached, true);
      return cachedShootingParams;
    }
    Logger.recordOutput(logCached, false);

    Pose2d robotPosition = robotPoseSupplier.get();
    Pose2d shooterPosition = robotPosition.plus(robotToShooter);
    Translation2d target = targetSupplier.get();

    // Compute field-relative shooter velocity for shoot-on-the-move compensation.
    // Start with the robot's translational velocity rotated into the field frame,
    // then add the rotational contribution at the shooter offset from the robot center
    ChassisSpeeds robotSpeeds = velocitySupplier.get();
    double currentSpeed =
        Math.sqrt(
            Math.pow(robotSpeeds.vxMetersPerSecond, 2)
                + Math.pow(robotSpeeds.vyMetersPerSecond, 2));
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
                // TODO verify shooterOffsetY math
                // may have an error in the math here, need to verify with our own calculations.
                // Since our shooter isn't offset in the Y direction, it doesn't matter.
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
    // Iteratively refine: the lookahead distance differs from the initial distance, so the correct
    // TOF is the one consistent with the lookahead it produces (fixed-point convergence).
    double timeOfFlightSecs = timeOfFlightMap.get(shooterToTargetDistanceMeters);
    double[] lookaheadPosition = {shooterPosition.getX(), shooterPosition.getY()};
    for (int i = 0; i < 10; i++) {
      lookaheadPosition[0] = shooterPosition.getX() + shooterVelXMps * timeOfFlightSecs;
      lookaheadPosition[1] = shooterPosition.getY() + shooterVelYMps * timeOfFlightSecs;
      double newTof =
          timeOfFlightMap.get(
              Math.sqrt(
                  Math.pow(lookaheadPosition[0] - target.getX(), 2)
                      + Math.pow(lookaheadPosition[1] - target.getY(), 2)));
      timeOfFlightSecs = newTof;
    }
    double lookaheadDistanceMeters =
        Math.sqrt(
            Math.pow(lookaheadPosition[0] - target.getX(), 2)
                + Math.pow(lookaheadPosition[1] - target.getY(), 2));

    double effectiveDistanceMeters =
        Math.max(minDistanceMeters, Math.min(maxDistanceMeters, lookaheadDistanceMeters));

    // Calculate the robot heading needed to aim the shooter at the target.
    // First find the field-frame angle from the velocity-compensated robot center to the target,
    // then subtract the shooter's facing angle relative to the robot so the drivetrain
    // orients the shooter toward the target.
    double[] robotCenterLookahead = {
      robotPosition.getX() + robotFieldVelocity.getX() * timeOfFlightSecs,
      robotPosition.getY() + robotFieldVelocity.getY() * timeOfFlightSecs
    };
    double angleToTarget =
        Math.atan2(
            (target.getY() - robotCenterLookahead[1])
                / Math.hypot(
                    target.getX() - robotCenterLookahead[0],
                    target.getY() - robotCenterLookahead[1]),
            (target.getX() - robotCenterLookahead[0])
                / Math.hypot(
                    target.getX() - robotCenterLookahead[0],
                    target.getY() - robotCenterLookahead[1]));
    Rotation2d targetHeading = new Rotation2d(angleToTarget).minus(robotToShooter.getRotation());

    double hoodAngle = shotHoodAngleMap.get(effectiveDistanceMeters);
    double flywheelSpeed =
        shotFlywheelSpeedMap.get(effectiveDistanceMeters)
            + ((currentSpeed / maxRobotSpeedMps) * vPercentageToFlywheelOutput);
    double timeOfFlight = timeOfFlightMap.get(effectiveDistanceMeters);
    boolean isValid =
        lookaheadDistanceMeters >= minDistanceMeters
            && lookaheadDistanceMeters <= maxDistanceMeters;

    // The virtual target is the real target offset by the velocity compensation — it represents
    // where the robot is effectively aiming from its current position while moving.
    Translation2d velocityOffset =
        new Translation2d(shooterVelXMps * timeOfFlightSecs, shooterVelYMps * timeOfFlightSecs);
    Translation2d virtualTarget = target.minus(velocityOffset);
    Logger.recordOutput(logVirtualTarget, new Pose2d(virtualTarget, Rotation2d.kZero));
    Logger.recordOutput(logTargetPosition, new Pose2d(target, Rotation2d.kZero));
    Logger.recordOutput(logHubPosition, FieldConstants.Hub.topCenterPoint);
    Logger.recordOutput(logDistanceToTarget, lookaheadDistanceMeters);
    Logger.recordOutput(logTargetFlywheelSpeed, flywheelSpeed);
    Logger.recordOutput(logTargetHoodAngle, hoodAngle);
    Logger.recordOutput(logTargetHeading, targetHeading);
    double currentHeadingDeg = robotPosition.getRotation().getDegrees();
    Logger.recordOutput(logCurrentHeadingDeg, currentHeadingDeg);
    Logger.recordOutput(
        logHeadingErrorDeg,
        Math.IEEEremainder(targetHeading.getDegrees() - currentHeadingDeg, 360.0));
    Logger.recordOutput(
        logLookaheadPose, new Pose2d(lookaheadPosition[0], lookaheadPosition[1], targetHeading));
    Logger.recordOutput(
        logRobotCenterLookahead,
        new Pose2d(robotCenterLookahead[0], robotCenterLookahead[1], targetHeading));
    Logger.recordOutput(logTimeOfFlightSecs, timeOfFlightSecs);
    Logger.recordOutput(logIsValid, isValid);
    Logger.recordOutput(logRobotSpeeds, robotSpeeds);

    cachedShootingParams =
        new ShootingParams(targetHeading, hoodAngle, flywheelSpeed, timeOfFlight, isValid);
    return cachedShootingParams;
  }

  /**
   * Returns the time-of-flight value at the minimum distance key in the map — i.e., the shortest
   * possible flight time for any shot.
   *
   * @return minimum time of flight in seconds
   */
  public double getMinTimeOfFlightSecs() {
    return timeOfFlightMap.get(minDistanceMeters);
  }

  /**
   * Clears the cached shooting parameters, forcing a recalculation on the next call to {@link
   * #calculateShot()}.
   */
  public void clearShootingParams() {
    cachedShootingParams = null;
  }
}
