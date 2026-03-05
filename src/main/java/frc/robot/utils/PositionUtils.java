package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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

  // If we are on the blue alliance, we use these no fly zones
  private static final Rectangle2d neutralNoFlyZoneWhenOnBlueAlliance =
      new Rectangle2d(
          new Translation2d(LinesVertical.blueHubCenter, LinesHorizontal.rightBumpHubSide),
          new Translation2d(LinesVertical.center, LinesHorizontal.leftBumpHubSide));
  private static final Rectangle2d oppNoFlyZoneWhenOnBlueAlliance =
      new Rectangle2d(
          new Translation2d(
              LinesVertical.redHubCenter, FieldConstants.LinesHorizontal.rightBumpHubSide),
          new Translation2d(FieldConstants.fieldLength, LinesHorizontal.leftBumpHubSide));

  // If we are on the red alliance, we use these no fly zones
  private static final Rectangle2d neutralNoFlyZoneWhenOnRedAlliance =
      new Rectangle2d(
          new Translation2d(LinesVertical.redHubCenter, LinesHorizontal.rightBumpHubSide),
          new Translation2d(LinesVertical.center, LinesHorizontal.leftBumpHubSide));
  private static final Rectangle2d oppNoFlyZoneWhenOnRedAlliance =
      new Rectangle2d(
          new Translation2d(
              LinesVertical.blueHubCenter, FieldConstants.LinesHorizontal.rightBumpHubSide),
          new Translation2d(0.0, LinesHorizontal.leftBumpHubSide));

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
        shooterY >= LinesHorizontal.rightTrenchRailSide
            && shooterY <= LinesHorizontal.rightTrenchHubSide;
    boolean inTrenchYLeft =
        shooterY >= LinesHorizontal.leftTrenchHubSide
            && shooterY <= LinesHorizontal.leftTrenchRailSide;
    boolean inTrenchYBand = inTrenchYRight || inTrenchYLeft;

    boolean inBlueTrenchX = shooterX >= BLUE_TRENCH_MIN_X && shooterX <= BLUE_TRENCH_MAX_X;
    boolean inRedTrenchX = shooterX >= RED_TRENCH_MIN_X && shooterX <= RED_TRENCH_MAX_X;
    boolean inTrenchXRange = inBlueTrenchX || inRedTrenchX;

    boolean result = inTrenchYBand && inTrenchXRange;
    Logger.recordOutput("PositionUtils/IsInDuckZone", result);
    return result;
  }

  /**
   * Returns whether the robot is inside its own alliance zone.
   *
   * <p>The alliance zone boundary is defined by {@link LinesVertical#allianceZone}. For blue
   * alliance, the zone is X &lt;= that boundary. For red alliance, the boundary is flipped and the
   * zone is X &gt;= the flipped boundary. Poses are always in blue-origin coordinates.
   *
   * @param robotPose the current blue-origin robot pose
   * @return true if the robot is within its own alliance zone
   */
  public static boolean isInAllianceZone(Pose2d robotPose) {
    double robotX = robotPose.getX();
    boolean result;
    if (AllianceFlipUtil.shouldFlip()) {
      result =
          robotX
              >= AllianceFlipUtil.applyX(LinesVertical.allianceZone + Units.inchesToMeters(24.0));
    } else {
      result = robotX <= LinesVertical.allianceZone;
    }
    Logger.recordOutput("PositionUtils/IsInAllianceZone", result);
    return result;
  }

  public static boolean isInPassingZone(Pose2d robotPose, Transform2d robotToShooter) {
    Rectangle2d neutralNoFlyZone = neutralNoFlyZoneWhenOnBlueAlliance;
    Rectangle2d oppNoFlyZone = oppNoFlyZoneWhenOnBlueAlliance;

    if (DriverStation.getAlliance().isPresent()) {
      Alliance alliance = DriverStation.getAlliance().get();
      if (alliance == Alliance.Red) {
        neutralNoFlyZone = neutralNoFlyZoneWhenOnRedAlliance;
        oppNoFlyZone = oppNoFlyZoneWhenOnRedAlliance;
      }
    } else {
      return true;
    }
    boolean inDuckZone = PositionUtils.isInDuckZone(robotPose, robotToShooter);
    if (!inDuckZone) {
      if (oppNoFlyZone.contains(robotPose.getTranslation())
          || neutralNoFlyZone.contains(robotPose.getTranslation())) {
        return false;
      } else {
        return true;
      }
    } else {
      return false;
    }
  }

  public static boolean canPass(Pose2d robotPose, Rotation2d robotRotation) {
    Rotation2d shooterRotation = robotRotation.plus(Rotation2d.k180deg);
    double rayLength = FieldConstants.fieldLength;
    Translation2d end = robotPose.getTranslation().plus(new Translation2d(rayLength, shooterRotation));
    Pose2d[] raycast = new Pose2d[] {robotPose, new Pose2d(end, shooterRotation)};
    Logger.recordOutput("Raycast/Line", raycast);

    return true;
  }

  /**
   * Returns whichever of two candidate targets is closer to the robot.
   *
   * @param robotPose the current blue-origin robot pose
   * @param targetA the first candidate target position
   * @param targetB the second candidate target position
   * @return the {@link Translation2d} of the closer target
   */
  public static Translation2d getCloserPassTarget(
      Pose2d robotPose, Translation2d targetA, Translation2d targetB) {
    double distanceToAMeters = robotPose.getTranslation().getDistance(targetA);
    double distanceToBMeters = robotPose.getTranslation().getDistance(targetB);

    Translation2d result = distanceToAMeters <= distanceToBMeters ? targetA : targetB;
    Logger.recordOutput("PositionUtils/CloserPassTarget", new Pose2d(result, Rotation2d.kZero));
    return result;
  }
}
