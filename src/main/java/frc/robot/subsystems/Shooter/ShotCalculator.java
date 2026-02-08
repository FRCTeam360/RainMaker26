package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utils.FieldConstants;
import org.littletonrobotics.junction.Logger;

// how far we are from the hub
// taking that to convert to setpoints for flywhel and hood
public class ShotCalculator {
  private CommandSwerveDrivetrain drivetrain;

  private static final InterpolatingDoubleTreeMap shotHoodAngleMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap launchFlywheelSpeedMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap timeOfFlightMap =
      new InterpolatingDoubleTreeMap();

  public record ShootingParams(Rotation2d targetAngle, double hoodAngle, double flywheelSpeed) {}

  private ShootingParams latestParameters = null;

  private static double minDistance;
  private static double maxDistance;

  static {
    minDistance = 0.0;
    maxDistance = Double.MAX_VALUE;

    shotHoodAngleMap.put(0.0, 0.0);
    launchFlywheelSpeedMap.put(0.0, 0.0);
  }

  public ShotCalculator(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
  }

  // Calculate distance from turret to target
  private double getRobotDistanceFromHub() {
    Pose2d currentPosition = drivetrain.getPosition();
    Translation2d target =
        AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());
    double positionToTargetDistance = target.getDistance(currentPosition.getTranslation());
    return positionToTargetDistance;
  }

  public double getWantedHoodAngle() {
    return shotHoodAngleMap.get(getRobotDistanceFromHub());
  }

  private ShootingParams shootingParams = null;

  /**
   * Calculates and caches the shooting parameters for the current robot position. If parameters
   * have already been calculated and not cleared, returns the cached result.
   *
   * @return the {@link ShootingParams} containing drivebase angle, hood angle, and flywheel speed
   */
  public ShootingParams calculateShot() {
    if (shootingParams != null) {
      Logger.recordOutput("ShotCalculator/cached", true);

      return shootingParams;
    }
    Logger.recordOutput("ShotCalculator/cached", false);
    Pose2d robotPosition = drivetrain.getPosition();
    Pose2d shooterPosition = robotPosition.plus(ShooterConstants.robotToShooter);

    Translation2d hubTranslation =
        AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());
    double distanceToTarget = hubTranslation.getDistance(shooterPosition.getTranslation());

    Rotation2d targetAngle = hubTranslation.minus(shooterPosition.getTranslation()).getAngle();
    double hoodAngle = shotHoodAngleMap.get(distanceToTarget);
    double flywheelSpeed = launchFlywheelSpeedMap.get(distanceToTarget);

    Logger.recordOutput("ShotCalculator/hubPosition", FieldConstants.Hub.topCenterPoint);
    Logger.recordOutput("ShotCalculator/distanceToTarget", distanceToTarget);
    Logger.recordOutput("ShotCalculator/targetAngle", targetAngle);

    shootingParams = new ShootingParams(targetAngle, hoodAngle, flywheelSpeed);

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
