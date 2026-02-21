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
  void test() {
    ShootingParams expectedShootingParams = new ShootingParams(new Rotation2d(), 0.0, 0.0);
    final InterpolatingDoubleTreeMap interpolatingTreeMapTestZero =
        new InterpolatingDoubleTreeMap();
    interpolatingTreeMapTestZero.put(0.0, 0.0);
    ShotCalculator zeroShotCalculator =
        new ShotCalculator(
            () -> new Pose2d(),
            () -> new Translation2d(),
            interpolatingTreeMapTestZero,
            interpolatingTreeMapTestZero,
            new Transform2d());
    ShootingParams cachedShootingParams = zeroShotCalculator.calculateShot();

    assertEquals(0.0, cachedShootingParams.hoodAngle());
    assertEquals(0.0, cachedShootingParams.flywheelSpeed());
    assertTrue(cachedShootingParams.targetHeading().equals(new Rotation2d()));
  }
}
