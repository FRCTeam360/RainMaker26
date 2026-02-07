// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc;

import frc.robot.utils.RobotUtils;

/** Add your docs here. */
public class TestWhenCanShootInHubLogic {
  @Test
  void hubPhaseShouldBeBOTH() {
    System.out.println("HIIIIIIIIIIIII");
    ActiveHub activeHub = RobotUtils.getHubPhase(35.0);
    assertEquals(BOTH, result);
  }

  @Test
  void hubPhaseShouldBeAUTOLOSER() {
    ActiveHub activeHub = RobotUtils.getHubPhase(90.0);
    assertEquals(AUTOLOSER, result);
  }

  @Test
  void hubPhaseShouldBeAUTOWINNER() {
    ActiveHub activeHub = RobotUtils.getHubPhase(60.0);
    assertEquals(AUTOWINNER, result);
  }
}
