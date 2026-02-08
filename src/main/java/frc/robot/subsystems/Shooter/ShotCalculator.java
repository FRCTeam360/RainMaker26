package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotState;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utils.FieldConstants;

// how far we are from the hub
// taking that to convert to setpoints for flywhel and hood
public class ShotCalculator {
  private CommandSwerveDrivetrain drivetrain;

    private static final InterpolatingTreeMap<Double, Rotation2d> launchHoodAngleMap =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
  private static final InterpolatingDoubleTreeMap launchFlywheelSpeedMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap timeOfFlightMap = new
  InterpolatingDoubleTreeMap();

  public record ShootingParams(
    Rotation2d drivebaseAngle,
    double hoodAngle,
    double flywheelSpeed
  ) {}

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

  private double getShooterDistanceFromHub(){
    Translation2d target =
        AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());
    double shooterToTargetDistance = target.getDistance(getShooterPosition().getTranslation());
    return shooterToTargetDistance;
  }

  private Pose2d getShooterPosition(){
    Pose2d shooterPosition =new Pose2d();
    shooterPosition = drivetrain.getPosition().transformBy(ShooterConstants.robotToShooter);
    return shooterPosition;
  }

  public Rotation2d getWantedHoodAngle() {
    return launchHoodAngleMap.get(getRobotDistanceFromHub());
  }

  private double getWantedFlywheelSpeed() {
    return launchFlywheelSpeedMap.get(getRobotDistanceFromHub());
  }

  private ShootingParams shootingParams = null;

  /**
   * Calculates and caches the shooting parameters for the current robot position.
   * If parameters have already been calculated and not cleared, returns the cached result.
   *
   * @return the {@link ShootingParams} containing drivebase angle, hood angle, and flywheel speed
   */
  public ShootingParams calculateShot() {
    if(shootingParams != null) {
      return shootingParams;
    }
    return shootingParams;
  }

  /**
   * Clears the cached shooting parameters, forcing a recalculation on the next
   * call to {@link #calculateShot()}.
   */
  public void clearShootingParams() {
    shootingParams = null;
  }
}
