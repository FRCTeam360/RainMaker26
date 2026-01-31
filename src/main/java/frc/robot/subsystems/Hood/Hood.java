// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Hood;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Hood extends SubsystemBase {
  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();
  private final double TOLERANCE = 0.5;

  /** Creates a new Hood. */
  public Hood(HoodIO io) {
    this.io = io;
  }

  public void setDutyCycle(double dutyCycle) {
    io.setDutyCycle(dutyCycle);
  }

  public void setPosition(double position) {
    io.setPosition(position);
  }

  public double getPosition() {
    return inputs.position;
  }

  public Command setPositionCmd(double position) {
    return this.runOnce(() -> io.setPosition(position));
  }

  public void setEncoder(double position) {
    io.setEncoder(position);
  }

  public void stop() {
    io.setDutyCycle(0);
  }

  public boolean atSetpoint(double setpoint) {
    return Math.abs(getPosition() - setpoint) < TOLERANCE;
  }

  public Command moveToZeroAndZero() {
    return Commands.waitUntil(
            () -> Math.abs(inputs.supplyCurrent) >= 30.0 && Math.abs(inputs.velocity) == 0.0)
        .deadlineFor(this.runEnd(() -> io.setDutyCycle(0.1), () -> io.setDutyCycle(0.0)))
        .andThen(runOnce(() -> inputs.position = 0.0));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Hood", inputs);
  }

  public Command setDutyCycleCommand(double value) {
    return this.setDutyCycleCommand(() -> value);
  }

  public Command setDutyCycleCommand(DoubleSupplier valueSup) {
    return this.runEnd(() -> io.setDutyCycle(valueSup.getAsDouble()), () -> io.setDutyCycle(0.0));
  }

  public Command zero() {
    return this.runOnce(() -> setEncoder(0.0));
  }
}
