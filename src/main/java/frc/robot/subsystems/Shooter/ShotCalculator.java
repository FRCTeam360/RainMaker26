package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.utils.FieldConstants;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Calculates shooting parameters (hood angle, flywheel speed, and drivebase heading) based on the
 * robot's distance to the hub. Uses interpolation maps to convert distance into mechanism
 * setpoints.
 */
public class ShotCalculator {
  private final Supplier<Pose2d> robotPoseSupplier;
  private final Supplier<Translation2d> targetSupplier;
  private final InterpolatingDoubleTreeMap shotHoodAngleMap;
  private final InterpolatingDoubleTreeMap launchFlywheelSpeedMap;
  private final Transform2d robotToShooter;

  private static final InterpolatingDoubleTreeMap timeOfFlightMap =
      new InterpolatingDoubleTreeMap();

  /**
   * Holds the calculated shooting parameters for a given robot position.
   *
   * @param targetHeading the angle the drivebase should face toward the hub
   * @param hoodAngle the hood angle setpoint in degrees
   * @param flywheelSpeed the flywheel speed setpoint in RPM
   */
  public record ShootingParams(Rotation2d targetHeading, double hoodAngle, double flywheelSpeed) {}

  private ShootingParams latestParameters = null;

  private static final double MIN_DISTANCE_METERS = 0.0;
  private static final double MAX_DISTANCE_METERS = 5.0;

  /**
   * Creates a new ShotCalculator.
   *
   * @param robotPoseSupplier the supplier used to obtain the robot's current pose. Robot position
   *     should be drivetrain.getPosition()
   */
  public ShotCalculator(
      Supplier<Pose2d> robotPoseSupplier,
      Supplier<Translation2d>
          targetSupplier, // AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());
      InterpolatingDoubleTreeMap shotHoodAngleMap,
      InterpolatingDoubleTreeMap launchFlywheelSpeedMap,
      Transform2d robotToShooter) {
    this.robotPoseSupplier = robotPoseSupplier;
    this.targetSupplier = targetSupplier;
    this.shotHoodAngleMap = shotHoodAngleMap;
    this.launchFlywheelSpeedMap = launchFlywheelSpeedMap;
    this.robotToShooter = robotToShooter;
  }

  private ShootingParams cachedShootingParams = null;

  /**
   * Calculates and caches the shooting parameters for the current robot position. If parameters
   * have already been calculated and not cleared, returns the cached result.
   *
   * @return the {@link ShootingParams} containing drivebase angle, hood angle, and flywheel speed
   */
  public ShootingParams calculateShot() {
    if (cachedShootingParams != null) {
      Logger.recordOutput("ShotCalculator/cached", true);

      return cachedShootingParams;
    }
    Logger.recordOutput("ShotCalculator/cached", false);
    Pose2d shooterPosition = robotPoseSupplier.get().plus(robotToShooter);

    Translation2d target = targetSupplier.get();

    double distanceToTarget = target.getDistance(shooterPosition.getTranslation());
    distanceToTarget =
        Math.max(MIN_DISTANCE_METERS, Math.min(MAX_DISTANCE_METERS, distanceToTarget));

    // Calculate heading toward hub, then rotate 180° because the shooter
    // is at the back of the robot - robot faces away from hub to shoot at it
    Rotation2d targetHeading =
        target.minus(shooterPosition.getTranslation()).getAngle().rotateBy(Rotation2d.k180deg);
    double hoodAngle = shotHoodAngleMap.get(distanceToTarget);
    double flywheelSpeed = launchFlywheelSpeedMap.get(distanceToTarget);

    Logger.recordOutput("ShotCalculator/hubPosition", FieldConstants.Hub.topCenterPoint);
    Logger.recordOutput("ShotCalculator/distanceToTarget", distanceToTarget);
    Logger.recordOutput("ShotCalculator/targetFlywheelSpeed", flywheelSpeed);
    Logger.recordOutput("ShotCalculator/targetHoodAngle", hoodAngle);
    Logger.recordOutput("ShotCalculator/targetHeading", targetHeading);

    cachedShootingParams = new ShootingParams(targetHeading, hoodAngle, flywheelSpeed);

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
