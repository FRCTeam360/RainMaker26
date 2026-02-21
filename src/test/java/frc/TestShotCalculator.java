package frc;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.subsystems.Shooter.ShotCalculator;
import frc.robot.subsystems.Shooter.ShotCalculator.ShootingParams;
import org.junit.jupiter.api.Test;

public class TestShotCalculator {
  @Test
  void shotCalculatorTest() {
    final InterpolatingDoubleTreeMap interpolatingTreeMapTestZero =
        new InterpolatingDoubleTreeMap();
    interpolatingTreeMapTestZero.put(1.0, 0.0);
    ShotCalculator testShotCalculator =
        new ShotCalculator(
            // public Pose2d(double x, double y, Rotation2d rotation)
            () -> new Pose2d(1.0, 0.0, new Rotation2d()),
            () -> new Translation2d(),
            interpolatingTreeMapTestZero,
            interpolatingTreeMapTestZero,
            new Transform2d());
    ShootingParams cachedShootingParams = testShotCalculator.calculateShot();

    assertEquals(0.0, cachedShootingParams.hoodAngle());

    assertEquals(0.0, cachedShootingParams.flywheelSpeed());

    System.out.println(cachedShootingParams.targetHeading().getDegrees());
    System.out.println(new Rotation2d().getDegrees());
    assertTrue(cachedShootingParams.targetHeading().equals(new Rotation2d()));

    ShootingParams cachedShootingParams2 = testShotCalculator.calculateShot();

    assertEquals(0.0, cachedShootingParams2.hoodAngle());
    assertEquals(0.0, cachedShootingParams2.flywheelSpeed());
    assertTrue(cachedShootingParams2.targetHeading().equals(new Rotation2d()));
  }
}
