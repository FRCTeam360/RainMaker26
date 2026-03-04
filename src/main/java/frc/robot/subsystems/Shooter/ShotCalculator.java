// Shoot-on-the-move logic adapted from FRC 6328 Mechanical Advantage
// https://github.com/Mechanical-Advantage/RobotCode2026Public

package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.filter.LinearFilter;
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
  private final String name;

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
  private final String logHeadingVelocityRadPerSec;

  private double minDistanceMeters = 0.0;
  private double maxDistanceMeters = Double.MAX_VALUE;

  // Heading velocity feedforward state — computed inside calculateShot() so the feedforward
  // is produced in the same place as the heading target (no 1-frame lag, no external derivative).
  // Uses a moving-average filter to smooth the raw frame-to-frame heading delta (matches 6328).
  private static final int HEADING_RATE_FILTER_TAPS = 5; // ~0.1s window at 50 Hz
  private static final double LOOP_PERIOD_SECS = 0.02;
  private final LinearFilter headingRateFilter =
      LinearFilter.movingAverage(HEADING_RATE_FILTER_TAPS);
  private Rotation2d lastTargetHeading = null;

  /**
   * Holds the calculated shooting parameters for a given robot position.
   *
   * @param targetHeading the robot heading that aims the shooter at the target, accounting for the
   *     shooter's facing angle relative to the robot
   * @param headingVelocityRadPerSec the rate of change of the target heading in rad/s, filtered for
   *     use as heading controller feedforward
   * @param hoodAngle the hood angle setpoint in degrees
   * @param flywheelSpeed the flywheel speed setpoint in RPM
   * @param isValid whether the target is within the effective shooting range
   */
  public record ShootingParams(
      Rotation2d targetHeading,
      double headingVelocityRadPerSec,
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
      String name,
      Supplier<Pose2d> robotPoseSupplier,
      Supplier<Translation2d> targetSupplier,
      Supplier<ChassisSpeeds> velocitySupplier,
      RobotShootingInfo robotShootingInfo) {
    this.name = name;
    this.robotPoseSupplier = robotPoseSupplier;
    this.targetSupplier = targetSupplier;
    this.velocitySupplier = velocitySupplier;
    this.shotHoodAngleMap = robotShootingInfo.shotHoodAngleMap;
    this.shotFlywheelSpeedMap = robotShootingInfo.shotFlywheelSpeedMap;
    this.timeOfFlightMap = robotShootingInfo.timeOfFlightMap;
    this.robotToShooter = robotShootingInfo.robotToShooter;
    this.minDistanceMeters = robotShootingInfo.minDistanceMeters;
    this.maxDistanceMeters = robotShootingInfo.maxDistanceMeters;

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
    this.logHeadingVelocityRadPerSec = basePath + "/headingVelocityRadPerSec";
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
    Translation2d lookaheadPosition = shooterPosition.getTranslation();
    for (int i = 0; i < 10; i++) {
      lookaheadPosition =
          shooterPosition
              .getTranslation()
              .plus(
                  new Translation2d(
                      shooterVelXMps * timeOfFlightSecs, shooterVelYMps * timeOfFlightSecs));
      double newTof = timeOfFlightMap.get(target.getDistance(lookaheadPosition));
      timeOfFlightSecs = newTof;
    }
    double lookaheadDistanceMeters = target.getDistance(lookaheadPosition);

    double effectiveDistanceMeters =
        Math.max(minDistanceMeters, Math.min(maxDistanceMeters, lookaheadDistanceMeters));

    // Calculate the robot heading needed to aim the shooter at the target.
    // The drivetrain rotates around the robot center, so we compute the angle from the
    // velocity-compensated robot center (not the shooter position) to the target.
    // Then subtract the shooter's facing angle relative to the robot so the drivetrain
    // orients the shooter toward the target.
    Translation2d robotCenterLookahead =
        robotPosition
            .getTranslation()
            .plus(
                new Translation2d(
                    robotFieldVelocity.getX() * timeOfFlightSecs,
                    robotFieldVelocity.getY() * timeOfFlightSecs));
    Rotation2d angleToTarget = target.minus(robotCenterLookahead).getAngle();
    Rotation2d targetHeading = angleToTarget.minus(robotToShooter.getRotation());

    // Compute heading velocity feedforward (rad/s) using a filtered frame-to-frame derivative.
    // This is produced in the same place as the heading target — no 1-frame lag, no external
    // numerical differentiation. Matches 6328's approach of returning driveVelocity from the
    // calculator. On the first frame (lastTargetHeading == null) we output 0 to avoid a spike.
    double headingVelocityRadPerSec = 0.0;
    if (lastTargetHeading != null) {
      headingVelocityRadPerSec =
          headingRateFilter.calculate(
              targetHeading.minus(lastTargetHeading).getRadians() / LOOP_PERIOD_SECS);
    }
    lastTargetHeading = targetHeading;

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
    Logger.recordOutput(logVirtualTarget, new Pose2d(virtualTarget, Rotation2d.kZero));
    Logger.recordOutput(logTargetPosition, new Pose2d(target, Rotation2d.kZero));
    Logger.recordOutput(logHubPosition, FieldConstants.Hub.topCenterPoint);
    Logger.recordOutput(logDistanceToTarget, lookaheadDistanceMeters);
    Logger.recordOutput(logTargetFlywheelSpeed, flywheelSpeed);
    Logger.recordOutput(logTargetHoodAngle, hoodAngle);
    Logger.recordOutput(logTargetHeading, targetHeading);
    Logger.recordOutput(logLookaheadPose, new Pose2d(lookaheadPosition, targetHeading));
    Logger.recordOutput(logRobotCenterLookahead, new Pose2d(robotCenterLookahead, targetHeading));
    Logger.recordOutput(logTimeOfFlightSecs, timeOfFlightSecs);
    Logger.recordOutput(logIsValid, isValid);
    Logger.recordOutput(logRobotSpeeds, robotSpeeds);
    Logger.recordOutput(logHeadingVelocityRadPerSec, headingVelocityRadPerSec);

    cachedShootingParams =
        new ShootingParams(
            targetHeading,
            headingVelocityRadPerSec,
            hoodAngle,
            flywheelSpeed,
            timeOfFlight,
            isValid);
    return cachedShootingParams;
  }

  /**
   * Clears the cached shooting parameters, forcing a recalculation on the next call to {@link
   * #calculateShot()}.
   */
  public void clearShootingParams() {
    cachedShootingParams = null;
    // Do NOT reset lastTargetHeading or headingRateFilter here — the heading velocity
    // feedforward needs continuity across cache clears to produce a smooth derivative.
    // Only reset them when the command that uses this calculator ends (not every frame).
  }

  /**
   * Resets the heading velocity feedforward state. Call this when the aiming command ends so the
   * filter doesn't carry stale data into the next activation.
   */
  public void resetHeadingState() {
    lastTargetHeading = null;
    headingRateFilter.reset();
  }
}
