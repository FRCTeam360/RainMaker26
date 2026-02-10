// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.FlywheelKicker;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelKickerIO {
  /** Creates a new FlywheelKickerIO. */
  @AutoLog
  public static class FlywheelKickerIOInputs {
    public double voltage = 0.0;
    public double supplyCurrent = 0.0;
    public double statorCurrent = 0.0;
    public double velocity = 0.0;
    public double position = 0.0;
    public boolean sensor = false;
  }

  public default void updateInputs(FlywheelKickerIOInputs inputs) {}

  public void setDutyCycle(double dutyCycle);

  public void setVelocity(double rpm);
}
