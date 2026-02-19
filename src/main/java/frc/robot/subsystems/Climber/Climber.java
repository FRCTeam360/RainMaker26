// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  /** Creates a new Climber. */
  public Climber(ClimberIO io) {
    this.io = io;
  }

  public void setLeftDutyCycle(double dutyCycle) {
    io.setLeftDutyCycle(dutyCycle);
  }

  public void setRightDutyCycle(double dutyCycle) {
    io.setRightDutyCycle(dutyCycle);
  }

  public void stop() {
    io.setLeftDutyCycle(0.0);
    io.setRightDutyCycle(0.0);
  }

  public boolean leftAboveMinHeight() {
    return io.leftAboveMinHeight();
  }

  public boolean rightAboveMinHeight(){
    return io.rightAboveMinHeight();
  }

  public void setLeftPosition(double position) {
    io.setLeftPosition(position);
  }

  public void setRightPosition(double position) {
    io.setRightPosition(position);
  }

  public void zeroBoth() {
    io.zeroBoth();
  }


  public void updatePIDF(double P, double I, double D, double F) {
    io.updatePIDF(P, I, D, F);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
  }
}
