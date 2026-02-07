// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import frc.robot.subsystems.Shooter.Flywheel.Flywheel;
import frc.robot.subsystems.Shooter.Hood.Hood;

public class ShotCalculator {
  private final Flywheel flywheel;
  private final Hood hood;

  public ShotCalculator(Flywheel flywheel, Hood hood) {
    this.flywheel = flywheel;
    this.hood = hood;
  }

  public record ShootingParameters() {}

  public ShootingParameters getParameters() {
    throw new UnsupportedOperationException("getParameters() has not been implemented");
  }

  public void clearShootingParameters() {}
}
