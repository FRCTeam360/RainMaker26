// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;

public class ShooterConstants {
  public static Transform2d ROBOT_TO_SHOOTER =
      new Transform2d(Units.inchesToMeters(-7.0), 0.0, new Rotation2d());
}
