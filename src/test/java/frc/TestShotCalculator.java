package frc;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
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
            // public Pose2d(double x, double y, Rotation2d rotation)
            () -> new Pose2d(1.0, 0.0, new Rotation2d()),
            () -> new Translation2d(),
            RobotShootingInfo);
    ShootingParams cachedShootingParams = testShotCalculator.calculateShot();

    assertEquals(0.0, cachedShootingParams.hoodAngle());

    assertEquals(0.0, cachedShootingParams.flywheelSpeed());

    assertTrue(cachedShootingParams.targetHeading().equals(new Rotation2d()));
  }

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
    Supplier<Pose2d> testPose = () -> new Pose2d(1.0, 0.0, new Rotation2d());
    ShotCalculator testShotCalculator =
        new ShotCalculator(
            // public Pose2d(double x, double y, Rotation2d rotation)
            testPose, () -> new Translation2d(), RobotShootingInfo);
    ShootingParams cachedShootingParams = testShotCalculator.calculateShot();

    testPose = () -> (new Pose2d(430987523953.984213, 9832572384.3249874, new Rotation2d()));

    ShootingParams cashedShootingParams2 = testShotCalculator.calculateShot();

    assertEquals(cachedShootingParams, cashedShootingParams2);
  }
}
