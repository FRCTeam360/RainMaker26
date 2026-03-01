// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;

public class ShooterConstants {
  public static Transform2d WOODBOT_TO_SHOOTER =
      new Transform2d(Units.inchesToMeters(-7.0), 0.0, new Rotation2d());
  public static final double ALLIANCE_PHASE_DURATION_SECONDS = 25.0;
  public static final double ALLIANCE_PHASE_START_MODULO = 5.0;

  public static Transform2d PRACTICEBOT_TO_SHOOTER =
      new Transform2d(Units.inchesToMeters(-7.437), 0.0, new Rotation2d());

  // Test Y offset to verify shooterOffsetY velocity math in sim.
  // Real robot has no Y offset — set back to 0.0 after verification.
  public static Transform2d SIM_TO_SHOOTER =
      new Transform2d(Units.inchesToMeters(0.0), Units.inchesToMeters(6.0), new Rotation2d());
}
