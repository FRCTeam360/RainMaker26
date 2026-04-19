// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake.IntakePivot;

public class IntakePivotIONoop implements IntakePivotIO {
  @Override
  public void setZero() {}

  @Override
  public void setPosition(double position) {}

  @Override
  public void setPositionSmooth(double position) {}

  @Override
  public void setPositionAggressive(double position) {}

  @Override
  public void setDutyCycle(double value) {}

  @Override
  public void enableBrakeMode() {}

  @Override
  public void disableBrakeMode() {}
}
