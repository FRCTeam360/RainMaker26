package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.utils.AllianceFlipUtil;
import frc.robot.utils.FieldConstants;
import org.littletonrobotics.junction.Logger;

/**
 * Reads target coordinates from the web dashboard via NetworkTables. Provides the target position
 * to other components while handling alliance coordinate flipping. Logs raw dashboard inputs for
 * debugging and replay.
 */
public class DashboardTargetProvider {

  // --- NetworkTables subscribers for custom target ---
  private final NetworkTable shootingTable;
  private final DoubleSubscriber targetXSub;
  private final DoubleSubscriber targetYSub;
  private final BooleanSubscriber targetActiveSub;
  private final DoublePublisher targetXPub;
  private final DoublePublisher targetYPub;

  // --- Cached values from NetworkTables ---
  private double cachedTargetX = 0.0;
  private double cachedTargetY = 0.0;
  private boolean cachedTargetActive = false;

  /** Creates a new DashboardTargetProvider that reads from the Shooting NT table. */
  public DashboardTargetProvider() {
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

  /**
   * Reads values from NetworkTables and caches them. Call this once per robot cycle (in
   * Robot.periodic() or RobotContainer.periodic()). This ensures all methods use consistent values
   * within a single cycle and minimizes NT calls.
   */
  public void periodic() {
    cachedTargetX = targetXSub.get();
    cachedTargetY = targetYSub.get();
    cachedTargetActive = targetActiveSub.get();

    // Log raw NT dashboard inputs (captured for AdvantageKit replay)
    Logger.recordOutput("DashboardTargetProvider/RawTargetX", cachedTargetX);
    Logger.recordOutput("DashboardTargetProvider/RawTargetY", cachedTargetY);
    Logger.recordOutput("DashboardTargetProvider/RawTargetActive", cachedTargetActive);
  }

  /**
   * Returns whether the custom dashboard target is active.
   *
   * @return true if a custom target has been set via the web dashboard
   */
  public boolean hasCustomTarget() {
    return cachedTargetActive;
  }

  /**
   * Returns the current target position from the dashboard, accounting for alliance flipping. If a
   * custom target is active, that target is used. Otherwise returns null.
   *
   * <p>Custom targets from the dashboard are always in blue-alliance coordinates (origin at blue
   * corner), so they are flipped for red alliance the same way the hub center is.
   *
   * @return the target position as a {@link Translation2d} in field coordinates, or null if no
   *     custom target is active
   */
  public Translation2d getDashboardTarget() {
    if (!hasCustomTarget()) {
      return null;
    }
    Translation2d customTarget = new Translation2d(cachedTargetX, cachedTargetY);
    Translation2d flippedTarget = AllianceFlipUtil.apply(customTarget);

    // Publish effective target back for the dashboard
    targetXPub.set(flippedTarget.getX());
    targetYPub.set(flippedTarget.getY());

    return flippedTarget;
  }

  /**
   * Returns the hub center position, accounting for alliance flipping. Use this as the default
   * target when no dashboard target is active.
   *
   * @return the hub center as a {@link Translation2d} in field coordinates
   */
  public Translation2d getHubTarget() {
    return AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());
  }

  /**
   * Returns the effective target position. If a dashboard target is active, returns that. Otherwise
   * returns the hub center.
   *
   * @return the target position as a {@link Translation2d} in field coordinates
   */
  public Translation2d getEffectiveTarget() {
    Translation2d dashboardTarget = getDashboardTarget();
    if (dashboardTarget != null) {
      return dashboardTarget;
    }
    return getHubTarget();
  }
}
