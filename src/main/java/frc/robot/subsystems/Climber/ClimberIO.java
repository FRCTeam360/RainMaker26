// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  public static class ClimberIOInputs {
    public double climberLeftVelocity = 0.0;
    public double climberRightVelocity = 0.0;
    public double climberLeftPosition = 0.0;
    public double climberRightPosition = 0.0;
    public double climberLeftDutyCycle = 0.0;
    public double climberRightDutyCycle = 0.0;
    public double climberLeftCurrent = 0.0;
    public double climberRightCurrent = 0.0;
    public double climberLeftTemp = 0.0;
    public double climberRightTemp = 0.0;
  }

  public default void updateInputs(ClimberIOInputs inputs) {}

  public void setLeftDutyCycle(double dutyCycle);
  public void setLeftPosition(double position);

  public void setRightDutyCycle(double dutyCycle);
  public void setRightPosition(double position);
  
  public boolean leftAboveMinHeight();
  public boolean rightAboveMinHeight();

  public void zeroBoth();

  public void updatePIDF(double P, double I, double D, double F);

}
