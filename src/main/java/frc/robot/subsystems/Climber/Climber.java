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
  }

  public void setLeftPosition(double position) {
    io.setLeftPosition(position);
  }


  @Override
  public void periodic() {
    io.updateInputs(inputs);
  }
}
