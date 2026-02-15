// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;

public class ShooterConstants {
  public static final Transform2d ROBOT_TO_SHOOTER = new Transform2d(0.0, 0.0, new Rotation2d());

  /** NetworkTables keys for the custom dashboard target point. */
  public static final String NT_TABLE = "Shooting";

  public static final String NT_TARGET_X = "TargetX";
  public static final String NT_TARGET_Y = "TargetY";
  public static final String NT_TARGET_ACTIVE = "TargetActive";

  /** Tolerance for heading alignment before auto-firing (radians). */
  public static final double HEADING_TOLERANCE_RAD = Math.toRadians(3.0);
}
