// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.IntakePivot;

import org.littletonrobotics.junction.AutoLog;

public interface IntakePivotIO {

  @AutoLog
  public static class IntakePivotIOInputs {
    public double statorCurrent = 0.0;
    public double supplyCurrent = 0.0;
    public double voltage = 0.0;
    public double velocity = 0.0;
    public double position = 0.0;
    public boolean brakeMode = true;
  }

  public void setPosition(double position);

  public void setDutyCycle(double value);

  public default void updateInputs(IntakePivotIOInputs inputs) {}

  public void enableBrakeMode();

  public void disableBrakeMode();
}
