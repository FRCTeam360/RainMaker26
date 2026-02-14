// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter.Flywheel;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.FlywheelTuneCommand;

import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Flywheel extends SubsystemBase {
  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();
  private FlywheelTuneCommand tuner;

  public enum FlywheelStates {
    OFF,
    SHOOTING
  }

  /** Creates a new Flywheel. */
  public Flywheel(FlywheelIO io) {
    this.io = io;
  }

  public FlywheelStates getState() {
    return currentState;
  }

  private FlywheelStates wantedState = FlywheelStates.OFF;
  private FlywheelStates currentState = FlywheelStates.OFF;
  private FlywheelStates previousState = FlywheelStates.OFF;

  private void updateState() {
    previousState = currentState;

    switch (wantedState) {
      case SHOOTING:
        currentState = FlywheelStates.SHOOTING;
        break;
      case OFF:
        currentState = FlywheelStates.OFF;
        break;
    }
  }

  public void setVelocity(double velocity) {
    io.setVelocity(velocity);
  }

  public double getVelocity() {
    if (inputs.velocities.length > 0) {
      return inputs.velocities[0];
    }
    return 0.0;
  }

  public boolean atSetpoint(double targetRPM, double tolerance) {
    // TODO: make tolerance a constant in hardware layer
    return Math.abs(getVelocity() - targetRPM) < tolerance;
  }

  private void applyState() {
    switch (currentState) {
      case SPINUP_SHOOTING:
        setVelocity(3000.0);
        break;
      case SHOOTING:
        setRPM(Constants.SPINUP_SHOOTING_FLYWHEEL_RPM);
        break;
      case OFF:
      default:
        stop();
        break;
    }
  }

  public void setWantedState(FlywheelStates state) {
    wantedState = state;
    updateState();
    applyState();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Flywheel", inputs);
    Logger.processInputs("Flywheel", inputs);
    Logger.recordOutput("Subsystems/Flywheel/WantedState", wantedState.toString());
    Logger.recordOutput("Subsystems/Flywheel/CurrentState", currentState.toString());
    Logger.recordOutput("Subsystems/Flywheel/PreviousState", previousState.toString());
  }

  public void setDutyCycle(double duty) {
    io.setDutyCycle(duty);
  }

  public void stop() {
    io.setDutyCycle(0.0);
  }

  public Command setDutyCycleCommand(double value) {
    return this.setDutyCycleCommand(() -> value);
  }

  public Command setDutyCycleCommand(DoubleSupplier valueSup) {
    return this.runEnd(() -> io.setDutyCycle(valueSup.getAsDouble()), () -> io.setDutyCycle(0.0));
  }

  public Command setVelocityCommand(DoubleSupplier supplierVelocity) {
    return this.runEnd(
        () -> io.setVelocity(supplierVelocity.getAsDouble()), () -> io.setDutyCycle(0.0));
  }

  public Command setVelocityCommand(double velocity) {
    return this.setVelocityCommand(() -> velocity);
  }
}
