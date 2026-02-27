package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.utils.FieldConstants.LinesHorizontal;
import frc.robot.utils.FieldConstants.LinesVertical;
import frc.robot.utils.FieldConstants.RightTrench;
import org.littletonrobotics.junction.Logger;

/** Static utility class for position-based field queries. */
public class PositionUtils {

  /** Half-width of the trench structure along the X-axis. Both trenches share the same width. */
  private static final double TRENCH_HALF_WIDTH_METERS = RightTrench.width / 2.0;

  // X-bounds for blue-side trenches (centered on hubCenter)
  private static final double BLUE_TRENCH_MIN_X =
      LinesVertical.hubCenter - TRENCH_HALF_WIDTH_METERS;
  private static final double BLUE_TRENCH_MAX_X =
      LinesVertical.hubCenter + TRENCH_HALF_WIDTH_METERS;

  // X-bounds for red-side trenches (centered on oppHubCenter)
  private static final double RED_TRENCH_MIN_X =
      LinesVertical.oppHubCenter - TRENCH_HALF_WIDTH_METERS;
  private static final double RED_TRENCH_MAX_X =
      LinesVertical.oppHubCenter + TRENCH_HALF_WIDTH_METERS;

  private PositionUtils() {}

  /**
   * Returns whether the shooter is inside a trench duck zone (low-clearance region).
   *
   * <p>The duck zones are the open trench regions on either side of the field. A trench exists at a
   * specific Y-band (left or right of the hub) and a specific X-range (centered on the hub on each
   * alliance side). The shooter must be within both the Y-band and an X-range to be considered
   * under a trench.
   *
   * @param robotPose the current blue-origin robot pose
   * @param robotToShooter the transform from robot center to shooter location
   * @return true if the shooter is within a trench duck zone
   */
  public static boolean isInDuckZone(Pose2d robotPose, Transform2d robotToShooter) {
    Translation2d shooterPosition = robotPose.transformBy(robotToShooter).getTranslation();
    double shooterX = shooterPosition.getX();
    double shooterY = shooterPosition.getY();

    boolean inTrenchYRight =
        shooterY >= LinesHorizontal.rightTrenchOpenEnd
            && shooterY <= LinesHorizontal.rightTrenchOpenStart;
    boolean inTrenchYLeft =
        shooterY >= LinesHorizontal.leftTrenchOpenEnd
            && shooterY <= LinesHorizontal.leftTrenchOpenStart;
    boolean inTrenchYBand = inTrenchYRight || inTrenchYLeft;

    boolean inBlueTrenchX = shooterX >= BLUE_TRENCH_MIN_X && shooterX <= BLUE_TRENCH_MAX_X;
    boolean inRedTrenchX = shooterX >= RED_TRENCH_MIN_X && shooterX <= RED_TRENCH_MAX_X;
    boolean inTrenchXRange = inBlueTrenchX || inRedTrenchX;

    boolean result = inTrenchYBand && inTrenchXRange;
    Logger.recordOutput("PositionUtils/IsInDuckZone", result);
    return result;
  }
}
