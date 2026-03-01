package frc;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.utils.AllianceFlipUtil;
import frc.robot.utils.FieldConstants.LinesHorizontal;
import frc.robot.utils.FieldConstants.LinesVertical;
import frc.robot.utils.FieldConstants.RightTrench;
import frc.robot.utils.PositionUtils;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class PositionUtilsTest {

  @BeforeEach
  void setUp() {
    HAL.initialize(500, 0);
  }

  @AfterEach
  void tearDown() {
    // Reset to blue alliance after each test
    DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
    DriverStationSim.notifyNewData();
  }

  private static final Transform2d IDENTITY_TRANSFORM = new Transform2d(0.0, 0.0, new Rotation2d());

  private static final Transform2d SHOOTER_OFFSET = new Transform2d(-0.2, 0.0, new Rotation2d());

  /** X coordinate at the center of the blue-side trench. */
  private static final double BLUE_TRENCH_X = LinesVertical.hubCenter;

  /** X coordinate at the center of the red-side trench. */
  private static final double RED_TRENCH_X = LinesVertical.oppHubCenter;

  /** X coordinate in the middle of the field, between both trench X-ranges. */
  private static final double NEUTRAL_X = LinesVertical.center;

  private static Pose2d poseAt(double x, double y) {
    return new Pose2d(x, y, new Rotation2d());
  }

  // --- Y-band tests (with valid X under blue trench) ---

  @Test
  void robotInRightTrenchBlueSide() {
    double midY = (LinesHorizontal.rightTrenchRailSide + LinesHorizontal.rightTrenchHubSide) / 2.0;
    assertTrue(PositionUtils.isInDuckZone(poseAt(BLUE_TRENCH_X, midY), IDENTITY_TRANSFORM));
  }

  @Test
  void robotInLeftTrenchBlueSide() {
    double midY = (LinesHorizontal.leftTrenchHubSide + LinesHorizontal.leftTrenchRailSide) / 2.0;
    assertTrue(PositionUtils.isInDuckZone(poseAt(BLUE_TRENCH_X, midY), IDENTITY_TRANSFORM));
  }

  @Test
  void robotInRightTrenchRedSide() {
    double midY = (LinesHorizontal.rightTrenchRailSide + LinesHorizontal.rightTrenchHubSide) / 2.0;
    assertTrue(PositionUtils.isInDuckZone(poseAt(RED_TRENCH_X, midY), IDENTITY_TRANSFORM));
  }

  @Test
  void robotInLeftTrenchRedSide() {
    double midY = (LinesHorizontal.leftTrenchHubSide + LinesHorizontal.leftTrenchRailSide) / 2.0;
    assertTrue(PositionUtils.isInDuckZone(poseAt(RED_TRENCH_X, midY), IDENTITY_TRANSFORM));
  }

  @Test
  void robotInCenterOfField() {
    assertFalse(
        PositionUtils.isInDuckZone(
            poseAt(BLUE_TRENCH_X, LinesHorizontal.center), IDENTITY_TRANSFORM));
  }

  @Test
  void robotOnBumpBetweenTrenchAndHub() {
    double bumpMidY = (LinesHorizontal.rightTrenchHubSide + LinesHorizontal.rightBumpRailSide) / 2.0;
    assertFalse(PositionUtils.isInDuckZone(poseAt(BLUE_TRENCH_X, bumpMidY), IDENTITY_TRANSFORM));
  }

  // --- Y boundary tests ---

  @Test
  void robotAtRightTrenchBoundaryStart() {
    assertTrue(
        PositionUtils.isInDuckZone(
            poseAt(BLUE_TRENCH_X, LinesHorizontal.rightTrenchHubSide), IDENTITY_TRANSFORM));
  }

  @Test
  void robotAtRightTrenchBoundaryEnd() {
    assertTrue(
        PositionUtils.isInDuckZone(
            poseAt(BLUE_TRENCH_X, LinesHorizontal.rightTrenchRailSide), IDENTITY_TRANSFORM));
  }

  @Test
  void robotAtLeftTrenchBoundaryStart() {
    assertTrue(
        PositionUtils.isInDuckZone(
            poseAt(BLUE_TRENCH_X, LinesHorizontal.leftTrenchRailSide), IDENTITY_TRANSFORM));
  }

  @Test
  void robotAtLeftTrenchBoundaryEnd() {
    assertTrue(
        PositionUtils.isInDuckZone(
            poseAt(BLUE_TRENCH_X, LinesHorizontal.leftTrenchHubSide), IDENTITY_TRANSFORM));
  }

  // --- X-range tests ---

  @Test
  void correctYButNeutralZoneX() {
    // Right trench Y-band but X is in the middle of the field (no trench overhead)
    double midY = (LinesHorizontal.rightTrenchRailSide + LinesHorizontal.rightTrenchHubSide) / 2.0;
    assertFalse(
        PositionUtils.isInDuckZone(poseAt(NEUTRAL_X, midY), IDENTITY_TRANSFORM),
        "Should not duck when X is in neutral zone even if Y is in trench band");
  }

  @Test
  void correctYButBeyondBlueTrenchX() {
    // Right trench Y-band but X is past the blue trench toward the driver station
    double midY = (LinesHorizontal.rightTrenchRailSide + LinesHorizontal.rightTrenchHubSide) / 2.0;
    double xBehindTrench = LinesVertical.hubCenter - RightTrench.width;
    assertFalse(
        PositionUtils.isInDuckZone(poseAt(xBehindTrench, midY), IDENTITY_TRANSFORM),
        "Should not duck when X is behind the trench structure");
  }

  @Test
  void atBlueTrenchXBoundary() {
    double midY = (LinesHorizontal.rightTrenchRailSide + LinesHorizontal.rightTrenchHubSide) / 2.0;
    double trenchEdgeX = LinesVertical.hubCenter + RightTrench.width / 2.0;
    assertTrue(
        PositionUtils.isInDuckZone(poseAt(trenchEdgeX, midY), IDENTITY_TRANSFORM),
        "Should duck at the exact X boundary of the trench");
  }

  @Test
  void justOutsideBlueTrenchXBoundary() {
    double midY = (LinesHorizontal.rightTrenchRailSide + LinesHorizontal.rightTrenchHubSide) / 2.0;
    double justOutsideX = LinesVertical.hubCenter + RightTrench.width / 2.0 + 0.01;
    assertFalse(
        PositionUtils.isInDuckZone(poseAt(justOutsideX, midY), IDENTITY_TRANSFORM),
        "Should not duck just outside the trench X boundary");
  }

  // --- Shooter offset tests ---

  @Test
  void shooterOffsetPushesIntoTrench() {
    double justAbove = LinesHorizontal.rightTrenchHubSide + 0.3;
    assertFalse(
        PositionUtils.isInDuckZone(poseAt(BLUE_TRENCH_X, justAbove), IDENTITY_TRANSFORM),
        "Robot without offset should be outside trench");

    Transform2d pushDown = new Transform2d(0.0, -0.4, new Rotation2d());
    assertTrue(
        PositionUtils.isInDuckZone(poseAt(BLUE_TRENCH_X, justAbove), pushDown),
        "Shooter with Y-offset should be inside trench");
  }

  @Test
  void shooterOffsetPushesOutOfTrench() {
    double justInside = LinesHorizontal.rightTrenchHubSide - 0.05;
    assertTrue(
        PositionUtils.isInDuckZone(poseAt(BLUE_TRENCH_X, justInside), IDENTITY_TRANSFORM),
        "Robot without offset should be inside trench");

    Transform2d pushUp = new Transform2d(0.0, 0.2, new Rotation2d());
    assertFalse(
        PositionUtils.isInDuckZone(poseAt(BLUE_TRENCH_X, justInside), pushUp),
        "Shooter with Y-offset should be outside trench");
  }

  @Test
  void rotationAffectsOffsetTransform() {
    double centerY = LinesHorizontal.center;
    Pose2d rotatedPose = new Pose2d(BLUE_TRENCH_X, centerY, Rotation2d.fromDegrees(90));

    assertFalse(PositionUtils.isInDuckZone(rotatedPose, SHOOTER_OFFSET));
  }

  @Test
  void zeroRotationWithXOffset() {
    double midY = (LinesHorizontal.rightTrenchRailSide + LinesHorizontal.rightTrenchHubSide) / 2.0;
    Pose2d pose = new Pose2d(BLUE_TRENCH_X, midY, new Rotation2d());

    assertTrue(PositionUtils.isInDuckZone(pose, SHOOTER_OFFSET));
  }

  // --- Alliance zone tests ---

  @Test
  void blueAllianceInsideZone() {
    DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
    DriverStationSim.notifyNewData();

    // Well inside blue alliance zone (near blue driver station)
    double insideX = LinesVertical.allianceZone - 1.0;
    assertTrue(
        PositionUtils.isInAllianceZone(poseAt(insideX, 4.0)),
        "Robot inside blue alliance zone should return true");
  }

  @Test
  void blueAllianceOutsideZone() {
    DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
    DriverStationSim.notifyNewData();

    // Outside blue alliance zone (toward red side)
    double outsideX = LinesVertical.allianceZone + 1.0;
    assertFalse(
        PositionUtils.isInAllianceZone(poseAt(outsideX, 4.0)),
        "Robot outside blue alliance zone should return false");
  }

  @Test
  void blueAllianceAtBoundary() {
    DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
    DriverStationSim.notifyNewData();

    // Exactly at the boundary (inclusive)
    assertTrue(
        PositionUtils.isInAllianceZone(poseAt(LinesVertical.allianceZone, 4.0)),
        "Robot at blue alliance zone boundary should return true");
  }

  @Test
  void redAllianceInsideZone() {
    DriverStationSim.setAllianceStationId(AllianceStationID.Red1);
    DriverStationSim.notifyNewData();

    // Well inside red alliance zone (near red driver station, high X in blue-origin coords)
    double redBoundary = AllianceFlipUtil.applyX(LinesVertical.allianceZone);
    double insideX = redBoundary + 1.0;
    assertTrue(
        PositionUtils.isInAllianceZone(poseAt(insideX, 4.0)),
        "Robot inside red alliance zone should return true");
  }

  @Test
  void redAllianceOutsideZone() {
    DriverStationSim.setAllianceStationId(AllianceStationID.Red1);
    DriverStationSim.notifyNewData();

    // Outside red alliance zone (toward blue side, low X in blue-origin coords)
    double redBoundary = AllianceFlipUtil.applyX(LinesVertical.allianceZone);
    double outsideX = redBoundary - 1.0;
    assertFalse(
        PositionUtils.isInAllianceZone(poseAt(outsideX, 4.0)),
        "Robot outside red alliance zone should return false");
  }

  // --- getCloserPassTarget tests ---

  private static final Translation2d TARGET_A = new Translation2d(2.0, 3.0);
  private static final Translation2d TARGET_B = new Translation2d(8.0, 5.0);

  @Test
  void returnsCloserTargetWhenRobotNearA() {
    Pose2d nearA = poseAt(2.5, 3.0);
    assertEquals(
        TARGET_A,
        PositionUtils.getCloserPassTarget(nearA, TARGET_A, TARGET_B),
        "Should return target A when robot is closer to A");
  }

  @Test
  void returnsCloserTargetWhenRobotNearB() {
    Pose2d nearB = poseAt(7.5, 5.0);
    assertEquals(
        TARGET_B,
        PositionUtils.getCloserPassTarget(nearB, TARGET_A, TARGET_B),
        "Should return target B when robot is closer to B");
  }

  @Test
  void returnsTargetAWhenEquidistant() {
    // Midpoint between A(2,3) and B(8,5) is (5,4)
    Pose2d midpoint = poseAt(5.0, 4.0);
    assertEquals(
        TARGET_A,
        PositionUtils.getCloserPassTarget(midpoint, TARGET_A, TARGET_B),
        "Should return target A when equidistant (tie-break)");
  }

  @Test
  void returnsCloserTargetWhenRobotAtTargetA() {
    Pose2d atA = poseAt(2.0, 3.0);
    assertEquals(
        TARGET_A,
        PositionUtils.getCloserPassTarget(atA, TARGET_A, TARGET_B),
        "Should return target A when robot is exactly at A");
  }

  @Test
  void returnsCloserTargetWhenRobotAtTargetB() {
    Pose2d atB = poseAt(8.0, 5.0);
    assertEquals(
        TARGET_B,
        PositionUtils.getCloserPassTarget(atB, TARGET_A, TARGET_B),
        "Should return target B when robot is exactly at B");
  }

  @Test
  void argumentOrderDoesNotAffectResult() {
    Pose2d nearB = poseAt(7.5, 5.0);
    // Swapped argument order — should still return the closer target (B)
    assertEquals(
        TARGET_B,
        PositionUtils.getCloserPassTarget(nearB, TARGET_B, TARGET_A),
        "Should return closer target regardless of argument order");
  }

  @Test
  void worksWithCoincidentTargets() {
    Translation2d samePoint = new Translation2d(4.0, 4.0);
    Pose2d anywhere = poseAt(0.0, 0.0);
    assertEquals(
        samePoint,
        PositionUtils.getCloserPassTarget(anywhere, samePoint, samePoint),
        "Should return the point when both targets are identical");
  }
}
