package frc;

import frc.robot.subsystems.Shooter.ShotCalculator;
import frc.robot.subsystems.Shooter.ShotCalculator.RobotShootingInfo;
import frc.robot.subsystems.Shooter.TargetSelectionStateMachine.TargetInternalStates;
import frc.robot.subsystems.Shooter.TargetSelectionStateMachine.TargetWantedStates;
import frc.robot.subsystems.Shooter.TargetSelectionStateMachine;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class TestTargetSelectionStateMachine {
  @Test
  void ShouldReturnPassCalculator() {
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
            0.0,
            0.0,
            0);
    ShotCalculator passCalculator =
        new ShotCalculator(
            "test",
            // public Pose2d(double x, double y, Rotation2d rotation)
            () -> new Pose2d(1.0, 0.0, new Rotation2d()),
            () -> new Translation2d(),
            () -> new ChassisSpeeds(),
            RobotShootingInfo);
    TargetSelectionStateMachine testTargetSelectionStateMachine =
        new TargetSelectionStateMachine(null, passCalculator, null);
    testTargetSelectionStateMachine.setWantedState(TargetWantedStates.OUTPOST);
    testTargetSelectionStateMachine.update();
    assertEquals(passCalculator, testTargetSelectionStateMachine.getActiveCalculator());
  }
}
