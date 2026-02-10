// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  public static class ClimberIOInputs {
    public double climberVelocity = 0.0;
    public double climberPosition = 0.0;
    public double climberDutyCycle = 0.0;
    public double climberCurrent = 0.0;
    public double climberTemp = 0.0;
  }

  public default void updateInputs(ClimberIOInputs inputs) {}

  public void setDutyCycle(double dutyCycle);

  public void setPosition(double position);
}
