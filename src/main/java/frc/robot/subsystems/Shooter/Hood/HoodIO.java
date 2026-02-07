// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter.Hood;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
  /** Creates a new HoodIO. */
  @AutoLog
  public static class HoodIOInputs {
    public double voltage = 0.0;
    public double supplyCurrent = 0.0;
    public double statorCurrent = 0.0;
    public double velocity = 0.0;
    public double position = 0.0;
  }

  public default void updateInputs(HoodIOInputs inputs) {}

  public void setDutyCycle(double dutyCycle);

  public void setPosition(double position);

  public void setEncoder(double position);
}
