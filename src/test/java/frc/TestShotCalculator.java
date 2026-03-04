package frc;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.Shooter.ShotCalculator;
import frc.robot.subsystems.Shooter.ShotCalculator.RobotShootingInfo;
import frc.robot.subsystems.Shooter.ShotCalculator.ShootingParams;
import java.util.function.Supplier;
import org.junit.jupiter.api.Test;

public class TestShotCalculator {
  @Test
  void shotCalculatorTestZero() {
    final InterpolatingDoubleTreeMap interpolatingTreeMapTestZero =
        new InterpolatingDoubleTreeMap();
    interpolatingTreeMapTestZero.put(1.0, 0.0);

    RobotShootingInfo RobotShootingInfo =
        new RobotShootingInfo(
            interpolatingTreeMapTestZero,
            interpolatingTreeMapTestZero,
            interpolatingTreeMapTestZero,
            new Transform2d(),
            0.0,
            5.0);

    ShotCalculator testShotCalculator =
        new ShotCalculator(
            "test",
            // public Pose2d(double x, double y, Rotation2d rotation)
            () -> new Pose2d(1.0, 0.0, new Rotation2d()),
            () -> new Translation2d(),
            () -> new ChassisSpeeds(),
            RobotShootingInfo);
    ShootingParams cachedShootingParams = testShotCalculator.calculateShot();

    assertEquals(0.0, cachedShootingParams.hoodAngle());

    assertEquals(0.0, cachedShootingParams.flywheelSpeed());

    assertTrue(cachedShootingParams.targetHeading().equals(Rotation2d.k180deg));
  }

  private Pose2d testPose;

  @Test
  void shotCalculatorTestCache() {
    final InterpolatingDoubleTreeMap interpolatingTreeMapTest1 = new InterpolatingDoubleTreeMap();
    final InterpolatingDoubleTreeMap interpolatingTreeMapTest2 = new InterpolatingDoubleTreeMap();
    final InterpolatingDoubleTreeMap interpolatingTreeMapTest3 = new InterpolatingDoubleTreeMap();
    interpolatingTreeMapTest1.put(1.0, 0.0);
    interpolatingTreeMapTest2.put(3.5, 2.0);
    interpolatingTreeMapTest3.put(2.3, 4.25);

    RobotShootingInfo RobotShootingInfo =
        new RobotShootingInfo(
            interpolatingTreeMapTest1,
            interpolatingTreeMapTest2,
            interpolatingTreeMapTest3,
            new Transform2d(),
            0.0,
            5.0);
    testPose = new Pose2d(1.0, 0.0, new Rotation2d());
    Supplier<Pose2d> testPoseSupplier = () -> testPose;
    ShotCalculator testShotCalculator =
        new ShotCalculator(
            "test",
            testPoseSupplier,
            () -> new Translation2d(),
            () -> new ChassisSpeeds(),
            RobotShootingInfo);
    ShootingParams cachedShootingParams = testShotCalculator.calculateShot();

    testPose =
        testPose.plus(new Transform2d(430987523953.984213, 9832572384.3249874, new Rotation2d()));

    ShootingParams cachedShootingParams2 = testShotCalculator.calculateShot();

    assertEquals(cachedShootingParams, cachedShootingParams2);
  }

  /**
   * Verifies that iterative TOF convergence produces a self-consistent result: the final
   * timeOfFlight value is consistent with the lookahead distance it implies. Without convergence
   * the initial-distance TOF is used for the lookahead, which under-estimates the effective
   * distance when moving toward a more distant region of the TOF map.
   */
  @Test
  void shotCalculatorConvergesOnMovingTarget() {
    // TOF increases linearly from 0.3 s at 2 m to 0.6 s at 5 m
    final InterpolatingDoubleTreeMap tofMap = new InterpolatingDoubleTreeMap();
    tofMap.put(2.0, 0.3);
    tofMap.put(5.0, 0.6);

    final InterpolatingDoubleTreeMap hoodMap = new InterpolatingDoubleTreeMap();
    hoodMap.put(1.0, 0.0);
    final InterpolatingDoubleTreeMap flywheelMap = new InterpolatingDoubleTreeMap();
    flywheelMap.put(1.0, 0.0);

    RobotShootingInfo info =
        new RobotShootingInfo(hoodMap, flywheelMap, tofMap, new Transform2d(), 0.0, 10.0);

    // Robot at origin, target at (3, 0), moving at 2 m/s in +Y.
    // Single-pass would use TOF at 3 m = 0.4 s → lookahead distance ≈ 3.105 m → TOF ≈ 0.411 s.
    // Converged result should satisfy: tofMap(lookaheadDist) ≈ timeOfFlight (self-consistent).
    ShotCalculator calculator =
        new ShotCalculator(
            "test",
            () -> new Pose2d(0, 0, new Rotation2d()),
            () -> new Translation2d(3, 0),
            () -> new ChassisSpeeds(0, 2.0, 0),
            info);

    ShootingParams result = calculator.calculateShot();
    double tof = result.timeOfFlight();

    // Verify self-consistency: plug the result's TOF back in to re-derive the lookahead distance,
    // then look up TOF at that distance — it should match.
    double lookaheadDist =
        new Translation2d(3, 0).getDistance(new Translation2d(0.0 * tof, 2.0 * tof));
    double verifiedTof = tofMap.get(lookaheadDist);

    assertEquals(
        tof, verifiedTof, 0.002, "Converged TOF must be self-consistent with lookahead distance");

    // Also confirm convergence changed the result vs single-pass (initial TOF at 3 m is 0.4 s).
    double singlePassTof = tofMap.get(3.0);
    assertNotEquals(
        singlePassTof, tof, 0.005, "Convergence should improve on the single-pass estimate");
  }

  @Test
  void shotCalculatorTestCacheCleared() {
    final InterpolatingDoubleTreeMap interpolatingTreeMapTest1 = new InterpolatingDoubleTreeMap();
    final InterpolatingDoubleTreeMap interpolatingTreeMapTest2 = new InterpolatingDoubleTreeMap();
    final InterpolatingDoubleTreeMap interpolatingTreeMapTest3 = new InterpolatingDoubleTreeMap();
    interpolatingTreeMapTest1.put(1.0, 0.0);
    interpolatingTreeMapTest2.put(3.5, 2.0);
    interpolatingTreeMapTest3.put(2.3, 4.25);

    RobotShootingInfo RobotShootingInfo =
        new RobotShootingInfo(
            interpolatingTreeMapTest1,
            interpolatingTreeMapTest2,
            interpolatingTreeMapTest3,
            new Transform2d(),
            0.0,
            5.0);
    testPose = new Pose2d(1.0, 0.0, new Rotation2d());
    Supplier<Pose2d> testPoseSupplier = () -> testPose;
    ShotCalculator testShotCalculator =
        new ShotCalculator(
            "test",
            testPoseSupplier,
            () -> new Translation2d(),
            () -> new ChassisSpeeds(),
            RobotShootingInfo);
    ShootingParams cachedShootingParams = testShotCalculator.calculateShot();

    testShotCalculator.clearShootingParams();
    testPose =
        testPose.plus(new Transform2d(430987523953.984213, 9832572384.3249874, new Rotation2d()));

    ShootingParams cachedShootingParams2 = testShotCalculator.calculateShot();

    assertNotEquals(cachedShootingParams, cachedShootingParams2);
  }
}
