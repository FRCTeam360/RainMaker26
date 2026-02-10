package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utils.AllianceFlipUtil;
import frc.robot.utils.FieldConstants;
import org.littletonrobotics.junction.Logger;

/**
 * Calculates shooting parameters (hood angle, flywheel speed, and drivebase heading) based on the
 * robot's distance to a target. Uses interpolation maps to convert distance into mechanism
 * setpoints.
 *
 * <p>Supports a configurable target point set via NetworkTables from a custom web dashboard. When
 * no custom target is active, defaults to the hub center.
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
   * @param targetAngle the angle the drivebase should face toward the target
   * @param hoodAngle the hood angle setpoint in degrees
   * @param flywheelSpeed the flywheel speed setpoint in RPM
   */
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

  // --- NetworkTables subscribers for custom target ---
  private final NetworkTable shootingTable;
  private final DoubleSubscriber targetXSub;
  private final DoubleSubscriber targetYSub;
  private final BooleanSubscriber targetActiveSub;
  private final DoublePublisher targetXPub;
  private final DoublePublisher targetYPub;

  /**
   * Creates a new ShotCalculator.
   *
   * @param drivetrain the swerve drivetrain used to obtain the robot's current pose
   */
  public ShotCalculator(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;

    // Set up NT subscribers for the custom target from the web dashboard
    shootingTable = NetworkTableInstance.getDefault().getTable(ShooterConstants.NT_TABLE);
    targetXSub = shootingTable.getDoubleTopic(ShooterConstants.NT_TARGET_X).subscribe(0.0);
    targetYSub = shootingTable.getDoubleTopic(ShooterConstants.NT_TARGET_Y).subscribe(0.0);
    targetActiveSub =
        shootingTable.getBooleanTopic(ShooterConstants.NT_TARGET_ACTIVE).subscribe(false);

    // Publish current target back so the dashboard can read the effective target
    targetXPub = shootingTable.getDoubleTopic("EffectiveTargetX").publish();
    targetYPub = shootingTable.getDoubleTopic("EffectiveTargetY").publish();
  }

  private ShootingParams shootingParams = null;

  /**
   * Returns whether the custom dashboard target is active.
   *
   * @return true if a custom target has been set via the web dashboard
   */
  public boolean hasCustomTarget() {
    return targetActiveSub.get();
  }

  /**
   * Returns the current target position, accounting for alliance flipping. If a custom target is
   * active (set from the web dashboard), that target is used. Otherwise defaults to the hub center.
   *
   * <p>Custom targets from the dashboard are always in blue-alliance coordinates (origin at blue
   * corner), so they are flipped for red alliance the same way the hub center is.
   *
   * @return the target position as a {@link Translation2d} in field coordinates
   */
  public Translation2d getTargetPosition() {
    if (hasCustomTarget()) {
      Translation2d customTarget = new Translation2d(targetXSub.get(), targetYSub.get());
      return AllianceFlipUtil.apply(customTarget);
    }
    return AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());
  }

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

    Translation2d targetTranslation = getTargetPosition();
    double distanceToTarget = targetTranslation.getDistance(shooterPosition.getTranslation());

    Rotation2d targetAngle = targetTranslation.minus(shooterPosition.getTranslation()).getAngle();
    double hoodAngle = shotHoodAngleMap.get(distanceToTarget);
    double flywheelSpeed = launchFlywheelSpeedMap.get(distanceToTarget);

    Logger.recordOutput("ShotCalculator/customTargetActive", hasCustomTarget());
    Logger.recordOutput(
        "ShotCalculator/targetPosition", new Pose2d(targetTranslation, new Rotation2d()));
    Logger.recordOutput("ShotCalculator/distanceToTarget", distanceToTarget);
    Logger.recordOutput("ShotCalculator/targetAngle", targetAngle);

    // Publish effective target back for the dashboard
    targetXPub.set(targetTranslation.getX());
    targetYPub.set(targetTranslation.getY());

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
