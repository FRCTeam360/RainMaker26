// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.IntakePivot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public interface IntakePivotIO  {
  
  public static class IntakePivotIOInputs {
    public double statorCurrent = 0.0;
    public double supplyCurrent = 0.0;
    public double voltage = 0.0;
    public double velocity = 0.0;
    public double position = 0.0;
    public boolean sensor = false;
    // insert inputs
  }

  public void setPosition(double position);

  public void stop();

  public void setEncoder(double value);

}
