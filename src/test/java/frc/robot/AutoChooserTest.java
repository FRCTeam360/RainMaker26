package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.AutoChooser.AutoZone;
import frc.robot.autos.NamedAutoWithPose;
import frc.robot.utils.FieldConstants;
import frc.robot.utils.FieldConstants.LinesHorizontal;
import java.util.Optional;
import org.junit.jupiter.api.Test;

class AutoChooserTest {

  private static final Optional<Alliance> BLUE = Optional.of(Alliance.Blue);
  private static final Optional<Alliance> RED = Optional.of(Alliance.Red);
  private static final Optional<Alliance> NO_ALLIANCE = Optional.empty();

  /** A non-zero blue-side X used so the alliance fallback (0,0) check is not triggered. */
  private static final double BLUE_X = 1.0;

  /** Exact midfield X — startX < midfield is blue, otherwise red. */
  private static final double MIDFIELD_X = FieldConstants.fieldLength / 2.0;

  /** A red-side X that won't trip the (0,0) fallback. */
  private static final double RED_X = FieldConstants.fieldLength - 1.0;

  /** Small epsilon for boundary tests. */
  private static final double EPS = 1e-6;

  private static NamedAutoWithPose autoAt(double x, double y) {
    return new NamedAutoWithPose("test", Commands.none(), new Pose2d(x, y, new Rotation2d()));
  }

  // ─────────────────────────────────────────────────────────────────────────────
  // determineZone — pure Y → AutoZone bucketing in blue-perspective coordinates
  // ─────────────────────────────────────────────────────────────────────────────

  @Test
  void determineZoneRightTrenchInsideTrenchBand() {
    double y = LinesHorizontal.rightTrenchHubSide / 2.0;
    assertEquals(AutoZone.RIGHT_TRENCH, AutoChooser.determineZone(y));
  }

  @Test
  void determineZoneRightTrenchAtRail() {
    assertEquals(
        AutoZone.RIGHT_TRENCH, AutoChooser.determineZone(LinesHorizontal.rightTrenchRailSide));
  }

  @Test
  void determineZoneRightTrenchAbsorbsGapToRightBump() {
    double midGap = (LinesHorizontal.rightTrenchHubSide + LinesHorizontal.rightBumpRailSide) / 2.0;
    assertEquals(
        AutoZone.RIGHT_TRENCH,
        AutoChooser.determineZone(midGap),
        "12-inch gap between right trench and right bump should bucket as right trench");
  }

  @Test
  void determineZoneRightBumpInsideBand() {
    double y = (LinesHorizontal.rightBumpRailSide + LinesHorizontal.rightBumpHubSide) / 2.0;
    assertEquals(AutoZone.RIGHT_BUMP, AutoChooser.determineZone(y));
  }

  @Test
  void determineZoneHubAtCenter() {
    assertEquals(AutoZone.HUB, AutoChooser.determineZone(LinesHorizontal.center));
  }

  @Test
  void determineZoneLeftBumpInsideBand() {
    double y = (LinesHorizontal.leftBumpHubSide + LinesHorizontal.leftBumpRailSide) / 2.0;
    assertEquals(AutoZone.LEFT_BUMP, AutoChooser.determineZone(y));
  }

  @Test
  void determineZoneLeftTrenchAbsorbsGapToLeftBump() {
    double midGap = (LinesHorizontal.leftBumpRailSide + LinesHorizontal.leftTrenchHubSide) / 2.0;
    assertEquals(
        AutoZone.LEFT_TRENCH,
        AutoChooser.determineZone(midGap),
        "12-inch gap between left bump and left trench should bucket as left trench");
  }

  @Test
  void determineZoneLeftTrenchInsideTrenchBand() {
    double y = (LinesHorizontal.leftTrenchHubSide + LinesHorizontal.leftTrenchRailSide) / 2.0;
    assertEquals(AutoZone.LEFT_TRENCH, AutoChooser.determineZone(y));
  }

  @Test
  void determineZoneLeftTrenchAtRail() {
    assertEquals(
        AutoZone.LEFT_TRENCH, AutoChooser.determineZone(LinesHorizontal.leftTrenchRailSide));
  }

  // Boundary tests — ensure each transition lands on the expected side of the boundary.

  @Test
  void determineZoneTrenchToBumpBoundaryRightSide() {
    assertEquals(
        AutoZone.RIGHT_TRENCH, AutoChooser.determineZone(LinesHorizontal.rightBumpRailSide - EPS));
    assertEquals(AutoZone.RIGHT_BUMP, AutoChooser.determineZone(LinesHorizontal.rightBumpRailSide));
  }

  @Test
  void determineZoneBumpToHubBoundaryRightSide() {
    assertEquals(
        AutoZone.RIGHT_BUMP, AutoChooser.determineZone(LinesHorizontal.rightBumpHubSide - EPS));
    assertEquals(AutoZone.HUB, AutoChooser.determineZone(LinesHorizontal.rightBumpHubSide));
  }

  @Test
  void determineZoneHubToBumpBoundaryLeftSide() {
    assertEquals(AutoZone.HUB, AutoChooser.determineZone(LinesHorizontal.leftBumpHubSide));
    assertEquals(
        AutoZone.LEFT_BUMP, AutoChooser.determineZone(LinesHorizontal.leftBumpHubSide + EPS));
  }

  @Test
  void determineZoneBumpToTrenchBoundaryLeftSide() {
    assertEquals(AutoZone.LEFT_BUMP, AutoChooser.determineZone(LinesHorizontal.leftBumpRailSide));
    assertEquals(
        AutoZone.LEFT_TRENCH, AutoChooser.determineZone(LinesHorizontal.leftBumpRailSide + EPS));
  }

  // ─────────────────────────────────────────────────────────────────────────────
  // matchesAlliance — X-based alliance filter (untouched by zone changes)
  // ─────────────────────────────────────────────────────────────────────────────

  @Test
  void matchesAllianceBlueAutoOnBlueAlliance() {
    assertTrue(AutoChooser.matchesAlliance(autoAt(BLUE_X, 4.0), BLUE));
  }

  @Test
  void matchesAllianceRedAutoOnRedAlliance() {
    assertTrue(AutoChooser.matchesAlliance(autoAt(RED_X, 4.0), RED));
  }

  @Test
  void matchesAllianceBlueAutoExcludedOnRedAlliance() {
    assertFalse(AutoChooser.matchesAlliance(autoAt(BLUE_X, 4.0), RED));
  }

  @Test
  void matchesAllianceRedAutoExcludedOnBlueAlliance() {
    assertFalse(AutoChooser.matchesAlliance(autoAt(RED_X, 4.0), BLUE));
  }

  @Test
  void matchesAllianceMidfieldXIsRedSide() {
    // Boundary check: the comparison is startX < midfield → blue, so X == midfield is red.
    assertTrue(AutoChooser.matchesAlliance(autoAt(MIDFIELD_X, 4.0), RED));
    assertFalse(AutoChooser.matchesAlliance(autoAt(MIDFIELD_X, 4.0), BLUE));
  }

  @Test
  void matchesAllianceJustBelowMidfieldIsBlueSide() {
    assertTrue(AutoChooser.matchesAlliance(autoAt(MIDFIELD_X - EPS, 4.0), BLUE));
    assertFalse(AutoChooser.matchesAlliance(autoAt(MIDFIELD_X - EPS, 4.0), RED));
  }

  @Test
  void matchesAllianceFallbackPoseMatchesBothAlliances() {
    assertTrue(AutoChooser.matchesAlliance(autoAt(0.0, 0.0), BLUE));
    assertTrue(AutoChooser.matchesAlliance(autoAt(0.0, 0.0), RED));
  }

  @Test
  void matchesAllianceWithoutAllianceMatchesAny() {
    assertTrue(AutoChooser.matchesAlliance(autoAt(BLUE_X, 4.0), NO_ALLIANCE));
    assertTrue(AutoChooser.matchesAlliance(autoAt(RED_X, 4.0), NO_ALLIANCE));
  }

  // ─────────────────────────────────────────────────────────────────────────────
  // matchesZone — Y-based filter with red-side rotational flip
  // ─────────────────────────────────────────────────────────────────────────────

  @Test
  void matchesZoneAllAlwaysTrue() {
    assertTrue(AutoChooser.matchesZone(autoAt(BLUE_X, LinesHorizontal.center), AutoZone.ALL, BLUE));
    assertTrue(AutoChooser.matchesZone(autoAt(RED_X, 0.5), AutoZone.ALL, RED));
    assertTrue(AutoChooser.matchesZone(autoAt(0.0, 0.0), AutoZone.ALL, NO_ALLIANCE));
  }

  @Test
  void matchesZoneFallbackPoseMatchesEveryZone() {
    for (AutoZone zone : AutoZone.values()) {
      assertTrue(
          AutoChooser.matchesZone(autoAt(0.0, 0.0), zone, BLUE),
          "fallback (0,0) auto should match zone " + zone);
    }
  }

  @Test
  void matchesZoneBlueAutoInLeftTrenchMatchesLeftTrench() {
    double y = (LinesHorizontal.leftTrenchHubSide + LinesHorizontal.leftTrenchRailSide) / 2.0;
    assertTrue(AutoChooser.matchesZone(autoAt(BLUE_X, y), AutoZone.LEFT_TRENCH, BLUE));
    assertFalse(AutoChooser.matchesZone(autoAt(BLUE_X, y), AutoZone.RIGHT_TRENCH, BLUE));
    assertFalse(AutoChooser.matchesZone(autoAt(BLUE_X, y), AutoZone.HUB, BLUE));
  }

  @Test
  void matchesZoneBlueAutoInRightTrenchMatchesRightTrench() {
    double y = LinesHorizontal.rightTrenchHubSide / 2.0;
    assertTrue(AutoChooser.matchesZone(autoAt(BLUE_X, y), AutoZone.RIGHT_TRENCH, BLUE));
    assertFalse(AutoChooser.matchesZone(autoAt(BLUE_X, y), AutoZone.LEFT_TRENCH, BLUE));
  }

  @Test
  void matchesZoneBlueAutoAtCenterMatchesHub() {
    assertTrue(AutoChooser.matchesZone(autoAt(BLUE_X, LinesHorizontal.center), AutoZone.HUB, BLUE));
    assertFalse(
        AutoChooser.matchesZone(autoAt(BLUE_X, LinesHorizontal.center), AutoZone.LEFT_BUMP, BLUE));
  }

  @Test
  void matchesZoneRedAutoOnDriverLeftSideMatchesLeftTrench() {
    // Red driver's "left" = LOW Y in field coords. After Y-flip → high Y → LEFT_TRENCH.
    double leftTrenchYBlue =
        (LinesHorizontal.leftTrenchHubSide + LinesHorizontal.leftTrenchRailSide) / 2.0;
    double driverLeftYRed = FieldConstants.fieldWidth - leftTrenchYBlue;
    assertTrue(
        AutoChooser.matchesZone(autoAt(RED_X, driverLeftYRed), AutoZone.LEFT_TRENCH, RED),
        "Red auto on driver's left should match LEFT_TRENCH after Y flip");
    assertFalse(AutoChooser.matchesZone(autoAt(RED_X, driverLeftYRed), AutoZone.RIGHT_TRENCH, RED));
  }

  @Test
  void matchesZoneRedAutoOnDriverRightSideMatchesRightTrench() {
    // Red driver's "right" = HIGH Y in field coords (where blue's left trench sits).
    // After Y-flip → low Y → RIGHT_TRENCH.
    double driverRightYRed =
        (LinesHorizontal.leftTrenchHubSide + LinesHorizontal.leftTrenchRailSide) / 2.0;
    assertTrue(
        AutoChooser.matchesZone(autoAt(RED_X, driverRightYRed), AutoZone.RIGHT_TRENCH, RED),
        "Red auto at high field Y should match RIGHT_TRENCH from driver's perspective");
    assertFalse(AutoChooser.matchesZone(autoAt(RED_X, driverRightYRed), AutoZone.LEFT_TRENCH, RED));
  }

  @Test
  void matchesZoneRedAutoBumpAlsoMirroredFromDriverPerspective() {
    // High Y (blue's left bump) on red alliance → driver's right bump.
    double leftBumpYBlue =
        (LinesHorizontal.leftBumpHubSide + LinesHorizontal.leftBumpRailSide) / 2.0;
    assertTrue(AutoChooser.matchesZone(autoAt(RED_X, leftBumpYBlue), AutoZone.RIGHT_BUMP, RED));
    assertFalse(AutoChooser.matchesZone(autoAt(RED_X, leftBumpYBlue), AutoZone.LEFT_BUMP, RED));
  }

  @Test
  void matchesZoneNoAllianceUsesUnflippedY() {
    // Without an alliance set, the Y flip is skipped, so high Y always maps to LEFT_TRENCH.
    double y = (LinesHorizontal.leftTrenchHubSide + LinesHorizontal.leftTrenchRailSide) / 2.0;
    assertTrue(AutoChooser.matchesZone(autoAt(RED_X, y), AutoZone.LEFT_TRENCH, NO_ALLIANCE));
  }

  @Test
  void matchesZoneHubMatchesBothAlliancesAtCenter() {
    // Hub spans the field's vertical center, so flipping Y leaves the bucket unchanged.
    double centerY = LinesHorizontal.center;
    assertTrue(AutoChooser.matchesZone(autoAt(BLUE_X, centerY), AutoZone.HUB, BLUE));
    assertTrue(AutoChooser.matchesZone(autoAt(RED_X, centerY), AutoZone.HUB, RED));
  }
}
