// Shoot-on-the-move logic adapted from FRC 6328 Mechanical Advantage
// https://github.com/Mechanical-Advantage/RobotCode2026Public

package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utils.AllianceFlipUtil;
import frc.robot.utils.FieldConstants;
import org.littletonrobotics.junction.Logger;

/**
 * Calculates shooting parameters (hood angle, flywheel speed, and drivebase heading) based on the
 * robot's distance to the hub. Uses interpolation maps to convert distance into mechanism
 * setpoints. Supports shoot-on-the-move by compensating for robot velocity using a time-of-flight
 * lookahead.
 */
public class ShotCalculator {
  private final CommandSwerveDrivetrain drivetrain;

  private static final InterpolatingDoubleTreeMap shotHoodAngleMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap launchFlywheelSpeedMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap timeOfFlightMap =
      new InterpolatingDoubleTreeMap();

  /**
   * Holds the calculated shooting parameters for a given robot position.
   *
   * @param targetHeading the angle the drivebase should face toward the hub
   * @param hoodAngle the hood angle setpoint in degrees
   * @param flywheelSpeed the flywheel speed setpoint in RPM
   * @param isValid whether the target is within the effective shooting range
   */
  public record ShootingParams(
      Rotation2d targetHeading, double hoodAngle, double flywheelSpeed, boolean isValid) {}

  private static final double MIN_DISTANCE_METERS = 0.0;
  private static final double MAX_DISTANCE_METERS = 5.0;

  static {
    // Hood angle map (distance in meters -> hood angle in degrees)
    shotHoodAngleMap.put(5.0, 20.0);
    shotHoodAngleMap.put(4.0, 18.0);
    shotHoodAngleMap.put(3.0, 16.0);
    shotHoodAngleMap.put(2.0, 11.0);
    shotHoodAngleMap.put(1.0, 8.0);
    shotHoodAngleMap.put(0.0, 6.0);

    // Flywheel speed map (distance in meters -> flywheel speed in RPM)
    launchFlywheelSpeedMap.put(5.0, 3750.0);
    launchFlywheelSpeedMap.put(4.0, 3750.0);
    launchFlywheelSpeedMap.put(3.0, 3375.0);
    launchFlywheelSpeedMap.put(2.0, 3000.0);
    launchFlywheelSpeedMap.put(0.0, 2750.0);

    // Time of flight map (distance in meters -> time of flight in seconds)
    // TODO: Fill with measured values from field testing
    timeOfFlightMap.put(0.0, 0.0);
    timeOfFlightMap.put(1.0, 0.0);
    timeOfFlightMap.put(2.0, 0.0);
    timeOfFlightMap.put(3.0, 0.0);
    timeOfFlightMap.put(4.0, 0.0);
    timeOfFlightMap.put(5.0, 0.0);
  }

  /**
   * Creates a new ShotCalculator.
   *
   * @param drivetrain the swerve drivetrain used to obtain the robot's current pose and velocity
   */
  public ShotCalculator(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
  }

  private ShootingParams shootingParams = null;

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
    if (shootingParams != null) {
      Logger.recordOutput("ShotCalculator/cached", true);
      return shootingParams;
    }
    Logger.recordOutput("ShotCalculator/cached", false);

    Pose2d robotPosition = drivetrain.getPosition();
    Pose2d shooterPosition = robotPosition.plus(ShooterConstants.ROBOT_TO_SHOOTER);
    Translation2d hubTranslation =
        AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());

    // Compute field-relative shooter velocity for shoot-on-the-move compensation.
    // Start with the robot's translational velocity rotated into the field frame,
    // then add the rotational contribution at the shooter offset from the robot center.
    ChassisSpeeds robotSpeeds = drivetrain.getVelocity();
    Rotation2d robotHeading = robotPosition.getRotation();
    Translation2d robotFieldVelocity =
        new Translation2d(robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond)
            .rotateBy(robotHeading);

    double robotAngleRad = robotHeading.getRadians();
    double shooterOffsetX = ShooterConstants.ROBOT_TO_SHOOTER.getX();
    double shooterOffsetY = ShooterConstants.ROBOT_TO_SHOOTER.getY();
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
    double shooterToTargetDistanceMeters =
        hubTranslation.getDistance(shooterPosition.getTranslation());

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
    double lookaheadDistanceMeters = hubTranslation.getDistance(lookaheadPosition);

    double effectiveDistanceMeters =
        Math.max(MIN_DISTANCE_METERS, Math.min(MAX_DISTANCE_METERS, lookaheadDistanceMeters));

    // Calculate heading from lookahead position to target, then rotate 180°
    // because the shooter is at the back of the robot
    Rotation2d targetHeading =
        hubTranslation.minus(lookaheadPosition).getAngle().rotateBy(Rotation2d.k180deg);

    double hoodAngle = shotHoodAngleMap.get(effectiveDistanceMeters);
    double flywheelSpeed = launchFlywheelSpeedMap.get(effectiveDistanceMeters);
    boolean isValid =
        lookaheadDistanceMeters >= MIN_DISTANCE_METERS
            && lookaheadDistanceMeters <= MAX_DISTANCE_METERS;

    Logger.recordOutput("ShotCalculator/hubPosition", FieldConstants.Hub.topCenterPoint);
    Logger.recordOutput("ShotCalculator/distanceToTarget", lookaheadDistanceMeters);
    Logger.recordOutput("ShotCalculator/targetFlywheelSpeed", flywheelSpeed);
    Logger.recordOutput("ShotCalculator/targetHoodAngle", hoodAngle);
    Logger.recordOutput("ShotCalculator/targetHeading", targetHeading);
    Logger.recordOutput(
        "ShotCalculator/lookaheadPose", new Pose2d(lookaheadPosition, targetHeading));
    Logger.recordOutput("ShotCalculator/timeOfFlightSecs", timeOfFlightSecs);
    Logger.recordOutput("ShotCalculator/isValid", isValid);

    shootingParams = new ShootingParams(targetHeading, hoodAngle, flywheelSpeed, isValid);
    return shootingParams;
  }

  /**
   * Clears the cached shooting parameters, forcing a recalculation on the next call to {@link
   * #calculateShot()}.
   */
  public void clearShootingParams() {
    shootingParams = null;
  }
}
