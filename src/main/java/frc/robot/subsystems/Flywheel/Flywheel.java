// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Flywheel;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Flywheel extends SubsystemBase {
  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

  /** Creates a new Flywheel. */
  public Flywheel(FlywheelIO io) {
    this.io = io;
  }

  public void setRPM(double rpm) {
    io.setRPM(rpm);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Flywheel", inputs);
  }

  public void setDutyCycle(double duty) {
    io.setDutyCycle(duty);
  }

  public void stop() {
    io.setDutyCycle(0.0);
  }

  public Command setDutyCycleCommand(DoubleSupplier dutySupplier) {
    return this.runEnd(() -> this.setDutyCycle(dutySupplier.getAsDouble()), () -> this.stop());
  }
}
