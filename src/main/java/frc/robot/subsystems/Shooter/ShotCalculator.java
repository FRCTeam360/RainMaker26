package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utils.FieldConstants;
import org.littletonrobotics.junction.Logger;

/**
 * Pure calculation logic for shooting parameters. Takes a target point as input and calculates the
 * required hood angle, flywheel speed, and drivebase heading. Does not depend on NetworkTables ---
 * the target point is passed in from the caller.
 *
 * <p>Shared calculation logic is used for both hub shots and custom target shots.
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
   * @param targetAngle the angle the drivebase should face toward the target
   * @param hoodAngle the hood angle setpoint in degrees
   * @param flywheelSpeed the flywheel speed setpoint in RPM
   */
  public record ShootingParams(Rotation2d targetAngle, double hoodAngle, double flywheelSpeed) {}

  private ShootingParams shootingParams = null;

  static {
    shotHoodAngleMap.put(5.0, 18.0);
    shotHoodAngleMap.put(4.0, 15.0);
    shotHoodAngleMap.put(3.0, 12.0);
    shotHoodAngleMap.put(2.0, 8.0);
    shotHoodAngleMap.put(0.0, 5.0);

    launchFlywheelSpeedMap.put(5.0, 3750.0);
    launchFlywheelSpeedMap.put(4.0, 3750.0);
    launchFlywheelSpeedMap.put(3.0, 3250.0);
    launchFlywheelSpeedMap.put(2.0, 3000.0);
    launchFlywheelSpeedMap.put(0.0, 2750.0);
  }

  /**
   * Creates a new ShotCalculator.
   *
   * @param drivetrain the swerve drivetrain used to obtain the robot's current pose
   */
  public ShotCalculator(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
  }

  /**
   * Calculates shooting parameters for the hub center. Convenience method that uses the same
   * underlying calculation as custom targets.
   *
   * @return the {@link ShootingParams} containing drivebase angle, hood angle, and flywheel speed
   */
  public ShootingParams calculateShotToHub() {
    if (DriverStation.getAlliance().get() == Alliance.Blue) {
      return calculateShotToPoint(FieldConstants.Hub.topCenterPoint.toTranslation2d());
    } else {
      return calculateShotToPoint(FieldConstants.Hub.oppTopCenterPoint.toTranslation2d());
    }
  }

  /**
   * Calculates shooting parameters for any arbitrary target point on the field. This is the shared
   * calculation logic used for both hub shots and dashboard target shots.
   *
   * @param target the target position in field coordinates (already alliance-flipped if necessary)
   * @return the {@link ShootingParams} containing drivebase angle, hood angle, and flywheel speed
   */
  public ShootingParams calculateShotToPoint(Translation2d target) {
    Pose2d robotPosition = drivetrain.getPosition();
    Pose2d shooterPosition = robotPosition.plus(ShooterConstants.robotToShooter);

    double distanceToTarget = target.getDistance(shooterPosition.getTranslation());

    Rotation2d targetAngle = target.minus(shooterPosition.getTranslation()).getAngle();
    double hoodAngle = shotHoodAngleMap.get(distanceToTarget);
    double flywheelSpeed = launchFlywheelSpeedMap.get(distanceToTarget);

    // Log computed values
    Logger.recordOutput("ShotCalculator/targetPosition", new Pose2d(target, new Rotation2d()));
    Logger.recordOutput("ShotCalculator/distanceToTarget", distanceToTarget);
    Logger.recordOutput("ShotCalculator/targetAngle", targetAngle);
    Logger.recordOutput("ShotCalculator/hoodAngle", hoodAngle);
    Logger.recordOutput("ShotCalculator/flywheelSpeed", flywheelSpeed);

    shootingParams = new ShootingParams(targetAngle, hoodAngle, flywheelSpeed);

    return shootingParams;
  }

  /**
   * Calculates and caches the shooting parameters for the provided target point. If parameters have
   * already been calculated and not cleared, returns the cached result.
   *
   * @param target the target position in field coordinates
   * @return the {@link ShootingParams} containing drivebase angle, hood angle, and flywheel speed
   */
  public ShootingParams calculateShot(Translation2d target) {
    if (shootingParams != null) {
      Logger.recordOutput("ShotCalculator/cached", true);
      return shootingParams;
    }
    Logger.recordOutput("ShotCalculator/cached", false);
    return calculateShotToPoint(target);
  }

  /**
   * Returns the cached shooting parameters, or null if none have been calculated.
   *
   * @return the cached {@link ShootingParams}, or null
   */
  public ShootingParams getCachedParams() {
    return shootingParams;
  }

  /**
   * Clears the cached shooting parameters, forcing a recalculation on the next call to {@link
   * #calculateShot(Translation2d)} or {@link #calculateShotToPoint(Translation2d)}.
   */
  public void clearShootingParams() {
    shootingParams = null;
  }
}
