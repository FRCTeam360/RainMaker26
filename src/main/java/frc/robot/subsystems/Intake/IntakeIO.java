// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  /** Creates a new IntakeIO. */
  @AutoLog
  public static class IntakeIOInputs {
    public double statorCurrent = 0.0;
    public double supplyCurrent = 0.0;
    public double voltage = 0.0;
    public double velocity = 0.0;
    public double position = 0.0;
    public boolean sensor = false;
    // insert inputs
  }

  public void setDutyCycle(double value);

  public void stop();

  public default void updateInputs(IntakeIOInputs inputs) {}
}
