// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climber;

public class ClimberIONoop implements ClimberIO {

  public void setDutyCycle(double value) {}

  public void setLeftDutyCycle(double dutyCycle) {}

  public void setLeftPosition(double position) {}

  public void setRightDutyCycle(double dutyCycle) {}

  public void setRightPosition(double position) {}

  public boolean leftAboveMinHeight() {
    return false;
  }

  public boolean rightAboveMinHeight() {
    return false;
  }

  public void zeroBoth() {}
}
