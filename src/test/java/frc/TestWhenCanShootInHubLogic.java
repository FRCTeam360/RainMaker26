// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc;

import static frc.robot.utils.RobotUtils.ActiveHub.*;
import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.utils.RobotUtils;
import frc.robot.utils.RobotUtils.ActiveHub;
import java.util.Optional;
import org.junit.jupiter.api.Test;

/** Add your docs here. */
public class TestWhenCanShootInHubLogic {
  @Test
  void hubPhaseShouldBeBOTH() {
    ActiveHub activeHub = RobotUtils.getHubPhase(25.0, true);
    assertEquals(BOTH, activeHub);
  }

  @Test
  void hubPhaseShouldBeAUTOLOSER() {
    ActiveHub activeHub = RobotUtils.getHubPhase(75.0, true);
    assertEquals(AUTOLOSER, activeHub);
  }

  @Test
  void hubPhaseShouldBeAUTOWINNER() {
    ActiveHub activeHub = RobotUtils.getHubPhase(50.0, true);
    assertEquals(AUTOWINNER, activeHub);
  }

  @Test
  void autoWinnerShouldBeBlueAlliance() {
    Alliance autoWinner = RobotUtils.getAutoWinner("Blue");
    assertEquals(Alliance.Blue, autoWinner);
  }

  @Test
  void autoWinnerShouldBeRedAlliance() {
    Alliance autoWinner = RobotUtils.getAutoWinner("Red");
    assertEquals(Alliance.Red, autoWinner);
  }

  // Optional<Alliance> alliance, Alliance autoWinner, ActiveHub gamePhase
  @Test
  void hubActiveShouldBeTrue() {
    Boolean hubActive =
        RobotUtils.hubActive(Optional.of(Alliance.Blue), Alliance.Blue, ActiveHub.AUTOWINNER);
    assertEquals(true, hubActive);
  }

  @Test
  void hubActiveShouldBeFalse() {
    Boolean hubActive =
        RobotUtils.hubActive(Optional.of(Alliance.Blue), Alliance.Red, ActiveHub.AUTOWINNER);
    assertEquals(false, hubActive);
  }
}
