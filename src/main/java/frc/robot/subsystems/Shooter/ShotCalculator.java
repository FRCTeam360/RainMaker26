package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utils.AllianceFlipUtil;
import frc.robot.utils.FieldConstants;
import org.littletonrobotics.junction.Logger;

/**
 * Calculates shooting parameters (hood angle, flywheel speed, and drivebase heading) based on the
 * robot's distance to the hub. Uses interpolation maps to convert distance into mechanism
 * setpoints.
 */
public class ShotCalculator {
  private CommandSwerveDrivetrain drivetrain;

  private static final InterpolatingDoubleTreeMap shotHoodAngleMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap launchFlywheelSpeedMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap timeOfFlightMap =
      new InterpolatingDoubleTreeMap();

  /**
   * Holds the calculated shooting parameters for a given robot position.
   *
   * @param targetAngle the angle the drivebase should face toward the hub
   * @param hoodAngle the hood angle setpoint in degrees
   * @param flywheelSpeed the flywheel speed setpoint in RPM
   */
  public record ShootingParams(Rotation2d targetAngle, double hoodAngle, double flywheelSpeed) {}

  private ShootingParams latestParameters = null;

  private static final double MIN_DISTANCE_METERS = 0.0;
  private static final double MAX_DISTANCE_METERS = 5.0;

  static {
    shotHoodAngleMap.put(5.0, 20.0);
    shotHoodAngleMap.put(4.0, 18.0);
    shotHoodAngleMap.put(3.0, 16.0);
    shotHoodAngleMap.put(2.0, 11.0); // THIS IS GOOD
    shotHoodAngleMap.put(1.0, 8.0); // THIS IS GOOD
    shotHoodAngleMap.put(0.0, 6.0);

    launchFlywheelSpeedMap.put(5.0, 3750.0);
    launchFlywheelSpeedMap.put(4.0, 3750.0);
    launchFlywheelSpeedMap.put(3.0, 3375.0);
    launchFlywheelSpeedMap.put(2.0, 3000.0); // THIS IS GOOD
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
    Pose2d shooterPosition = robotPosition.plus(ShooterConstants.ROBOT_TO_SHOOTER);

    Translation2d hubTranslation =
        AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());
    double distanceToTarget = hubTranslation.getDistance(shooterPosition.getTranslation());
    distanceToTarget =
        Math.max(MIN_DISTANCE_METERS, Math.min(MAX_DISTANCE_METERS, distanceToTarget));

    Rotation2d targetHeading;
    if (DriverStation.getAlliance().get() == Alliance.Red) {
      targetHeading = hubTranslation.minus(shooterPosition.getTranslation()).getAngle();
    } else
      targetHeading =
          hubTranslation
              .minus(shooterPosition.getTranslation())
              .getAngle()
              .rotateBy(Rotation2d.k180deg);

    double hoodAngle = shotHoodAngleMap.get(distanceToTarget);
    double flywheelSpeed = launchFlywheelSpeedMap.get(distanceToTarget);

    Logger.recordOutput("ShotCalculator/hubPosition", FieldConstants.Hub.topCenterPoint);
    Logger.recordOutput("ShotCalculator/distanceToTarget", distanceToTarget);
    Logger.recordOutput("ShotCalculator/targetFlywheelSpeed", flywheelSpeed);
    Logger.recordOutput("ShotCalculator/targetHoodAngle", hoodAngle);
    Logger.recordOutput("ShotCalculator/targetHeading", targetHeading);

    shootingParams = new ShootingParams(targetHeading, hoodAngle, flywheelSpeed);

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
