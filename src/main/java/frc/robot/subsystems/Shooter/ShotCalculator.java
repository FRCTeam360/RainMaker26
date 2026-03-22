// Shoot-on-the-move logic adapted from FRC 6328 Mechanical Advantage
// https://github.com/Mechanical-Advantage/RobotCode2026Public

package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
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

  // ===== CONSTANTS =====

  // Phase delay compensation (matches 6328): advance the estimated robot pose forward in time by
  // this amount to compensate for sensor/loop latency before computing the shooting solution.
  // 6328 uses 0.03 s (30 ms). Adjust if the robot's effective latency differs.
  private static final double PHASE_DELAY_SECS = 0.03;

  // Heading velocity feedforward state — computed inside calculateShot() so the feedforward
  // is produced in the same place as the heading target (no 1-frame lag, no external derivative).
  // Uses a moving-average filter to smooth the raw frame-to-frame heading delta (matches 6328).
  // Increase taps to smooth more aggressively; decrease for faster response. At 50 Hz, 5 taps ≈
  // 0.1 s window.
  private static final int HEADING_RATE_FILTER_TAPS = 5;
  private static final double LOOP_PERIOD_SECS = 0.02;

  // ===== CONFIGURATION =====

  private final Supplier<Pose2d> robotPoseSupplier;
  private final Supplier<Translation2d> targetSupplier;
  private final Supplier<ChassisSpeeds> velocitySupplier;
  private final InterpolatingDoubleTreeMap shotHoodAngleMap;
  private final InterpolatingDoubleTreeMap shotFlywheelSpeedMap;
  private final InterpolatingDoubleTreeMap timeOfFlightMap;
  private final Transform2d robotToShooter;
  private final String name;
  private final double maxRobotSpeedMps;
  private final double vPercentageToFlywheelOutput;
  private double minDistanceMeters = 0.0;
  private double maxDistanceMeters = Double.MAX_VALUE;

  // ===== LOG PATHS =====

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
  private final String logDrivetrainVelocityRobotRelative;
  private final String logHeadingVelocityRadPerSec;

  // ===== STATE =====

  private ShootingParams cachedShootingParams = null;
  private final LinearFilter headingRateFilter =
      LinearFilter.movingAverage(HEADING_RATE_FILTER_TAPS);
  private Rotation2d lastTargetHeading = null;

  // ===== RECORDS =====

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
    this.logDrivetrainVelocityRobotRelative = basePath + "/drivetrainVelocityRobotRelative";
    this.logHeadingVelocityRadPerSec = basePath + "/headingVelocityRadPerSec";
  }

  // ===== METHODS =====

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

    Pose2d drivetrainPose = robotPoseSupplier.get();
    ChassisSpeeds drivetrainSpeedsRobotRelative = velocitySupplier.get();

    // Convert robot-relative chassis speeds to field-relative once, up front.
    ChassisSpeeds drivetrainSpeedsFieldRelative =
        ChassisSpeeds.fromRobotRelativeSpeeds(
            drivetrainSpeedsRobotRelative, drivetrainPose.getRotation());

    // Advance the drivetrain pose by the phase delay to compensate for sensor/loop latency.
    // This matches 6328's approach: act on where the robot will be in ~30ms rather than
    // where odometry says it is now. Uses Pose2d.exp(Twist2d) for a proper on-manifold advance.
    // Twist2d expects robot-local frame — use robot-relative speeds, not fieldRelative.
    drivetrainPose =
        drivetrainPose.exp(
            new Twist2d(
                drivetrainSpeedsRobotRelative.vxMetersPerSecond * PHASE_DELAY_SECS,
                drivetrainSpeedsRobotRelative.vyMetersPerSecond * PHASE_DELAY_SECS,
                drivetrainSpeedsRobotRelative.omegaRadiansPerSecond * PHASE_DELAY_SECS));
    Pose2d shooterPose = drivetrainPose.plus(robotToShooter);
    Translation2d target = targetSupplier.get();

    // Compute field-relative shooter velocity for shoot-on-the-move compensation.
    // Shooter velocity = drivetrain translational velocity + rotational contribution at the
    // shooter offset (because the shooter is not at the robot center).
    // Rotational contribution uses the standard 2D cross product (ω × r) with the shooter offset
    // rotated into the field frame: vx_extra = -ω·r_y_field, vy_extra = +ω·r_x_field.
    double drivetrainTranslationSpeedMps =
        Math.hypot(
            drivetrainSpeedsFieldRelative.vxMetersPerSecond,
            drivetrainSpeedsFieldRelative.vyMetersPerSecond);
    double drivetrainOmegaRadPerSec = drivetrainSpeedsFieldRelative.omegaRadiansPerSecond;
    Translation2d shooterOffsetFieldRelative =
        new Translation2d(robotToShooter.getX(), robotToShooter.getY())
            .rotateBy(drivetrainPose.getRotation());
    double shooterVelFieldXMps =
        drivetrainSpeedsFieldRelative.vxMetersPerSecond
            + (-drivetrainOmegaRadPerSec * shooterOffsetFieldRelative.getY());
    double shooterVelFieldYMps =
        drivetrainSpeedsFieldRelative.vyMetersPerSecond
            + (drivetrainOmegaRadPerSec * shooterOffsetFieldRelative.getX());

    double shooterToTargetDistanceMeters = target.getDistance(shooterPose.getTranslation());

    // Use time of flight to project where the shooter will be when the ball arrives.
    // The robot imparts its velocity onto the ball, so we offset the aiming point
    // by (velocity * timeOfFlight) to lead the target.
    // Iteratively refine: the lookahead distance differs from the initial distance, so the correct
    // TOF is the one consistent with the lookahead it produces (fixed-point convergence).
    double timeOfFlightSecs = timeOfFlightMap.get(shooterToTargetDistanceMeters);
    Translation2d shooterLookaheadPosition = shooterPose.getTranslation();
    for (int i = 0; i < 10; i++) {
      shooterLookaheadPosition =
          shooterPose
              .getTranslation()
              .plus(
                  new Translation2d(
                      shooterVelFieldXMps * timeOfFlightSecs,
                      shooterVelFieldYMps * timeOfFlightSecs));
      double newTof = timeOfFlightMap.get(target.getDistance(shooterLookaheadPosition));
      timeOfFlightSecs = newTof;
    }
    double lookaheadShotDistanceMeters = target.getDistance(shooterLookaheadPosition);

    double clampedShotDistanceMeters =
        Math.max(minDistanceMeters, Math.min(maxDistanceMeters, lookaheadShotDistanceMeters));

    // Aim from the drivetrain center's lookahead, not the shooter tip's.
    // Subtract the shooter's facing angle so the drivetrain orients the shooter toward the target.
    Translation2d drivetrainLookaheadPosition =
        drivetrainPose
            .getTranslation()
            .plus(
                new Translation2d(
                    drivetrainSpeedsFieldRelative.vxMetersPerSecond * timeOfFlightSecs,
                    drivetrainSpeedsFieldRelative.vyMetersPerSecond * timeOfFlightSecs));
    Rotation2d angleToTarget = target.minus(drivetrainLookaheadPosition).getAngle();
    Rotation2d targetHeading = angleToTarget.minus(robotToShooter.getRotation());

    // Compute heading velocity feedforward (rad/s) using a filtered frame-to-frame derivative.
    // On the first frame output 0 to avoid a spike from the uninitialized lastTargetHeading.
    double headingVelocityRadPerSec = 0.0;
    if (lastTargetHeading != null) {
      headingVelocityRadPerSec =
          headingRateFilter.calculate(
              targetHeading.minus(lastTargetHeading).getRadians() / LOOP_PERIOD_SECS);
    }
    lastTargetHeading = targetHeading;

    double hoodAngle = shotHoodAngleMap.get(clampedShotDistanceMeters);
    double flywheelSpeed =
        shotFlywheelSpeedMap.get(clampedShotDistanceMeters)
            + ((drivetrainTranslationSpeedMps / maxRobotSpeedMps) * vPercentageToFlywheelOutput);
    double timeOfFlight = timeOfFlightMap.get(clampedShotDistanceMeters);
    boolean isValid =
        lookaheadShotDistanceMeters >= minDistanceMeters
            && lookaheadShotDistanceMeters <= maxDistanceMeters;

    Translation2d velocityOffset =
        new Translation2d(
            shooterVelFieldXMps * timeOfFlightSecs, shooterVelFieldYMps * timeOfFlightSecs);

    Translation2d virtualTarget = target.minus(velocityOffset);

    Logger.recordOutput(logVirtualTarget, new Pose2d(virtualTarget, Rotation2d.kZero));
    Logger.recordOutput(logTargetPosition, new Pose2d(target, Rotation2d.kZero));
    Logger.recordOutput(logHubPosition, FieldConstants.Hub.topCenterPoint);
    Logger.recordOutput(logDistanceToTarget, lookaheadShotDistanceMeters);
    Logger.recordOutput(logTargetFlywheelSpeed, flywheelSpeed);
    Logger.recordOutput(logTargetHoodAngle, hoodAngle);
    Logger.recordOutput(logTargetHeading, targetHeading);
    Logger.recordOutput(logLookaheadPose, new Pose2d(shooterLookaheadPosition, targetHeading));
    Logger.recordOutput(
        logRobotCenterLookahead, new Pose2d(drivetrainLookaheadPosition, targetHeading));
    Logger.recordOutput(logTimeOfFlightSecs, timeOfFlightSecs);
    Logger.recordOutput(logIsValid, isValid);
    Logger.recordOutput(logDrivetrainVelocityRobotRelative, drivetrainSpeedsRobotRelative);
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
