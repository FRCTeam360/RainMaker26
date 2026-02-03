// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Flywheel.FlywheelIOSim;
import frc.robot.subsystems.IntakePivot.IntakePivotIOSim;
import frc.robot.subsystems.Turret.TurretIOSim;

public final class Autos {

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }

  private IntakePivotIOSim pivotDown;
  private IntakePivotIOSim pivotUp;
  private TurretIOSim turretAimHub;
  private TurretIOSim turretShoot;
  private FlywheelIOSim flywheelSpin;

  
}
