// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter.FlywheelKicker;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ControlState;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class FlywheelKicker extends SubsystemBase {
  // Constants
  private static final double KICKER_VELOCITY_RPM = 4500.0;

  // IO fields
  private final FlywheelKickerIO io;
  private final FlywheelKickerIOInputsAutoLogged inputs = new FlywheelKickerIOInputsAutoLogged();

  // Enums
  public enum FlywheelKickerStates {
    IDLE,
    KICKING
  }

  // State variables
  private FlywheelKickerStates wantedState = FlywheelKickerStates.IDLE;
  private FlywheelKickerStates currentState = FlywheelKickerStates.IDLE;
  private FlywheelKickerStates previousState = FlywheelKickerStates.IDLE;
  private ControlState controlState = ControlState.SUPERSTRUCTURE;

  // Constructor

  /** Creates a new FlywheelKicker. */
  public FlywheelKicker(FlywheelKickerIO io) {
    this.io = io;
  }

  // State machine methods

  public FlywheelKickerStates getState() {
    return currentState;
  }

  public void setWantedState(FlywheelKickerStates state) {
    wantedState = state;
  }

  public void setControlState(ControlState controlState) {
    this.controlState = controlState;
  }

  private void updateState() {
    previousState = currentState;

    switch (wantedState) {
      case KICKING:
        currentState = FlywheelKickerStates.KICKING;
        break;
      case IDLE:
      default:
        currentState = FlywheelKickerStates.IDLE;
        break;
    }
  }

  private void applyState() {
    switch (currentState) {
      case KICKING:
        setVelocity(KICKER_VELOCITY_RPM);
        break;
      case IDLE:
      default:
        stop();
        break;
    }
  }

  // IO delegation methods

  public void setDutyCycle(double dutyCycle) {
    io.setDutyCycle(dutyCycle);
  }

  public void setVelocity(double veloicty) {
    io.setVelocity(veloicty);
  }

  public void stop() {
    io.setDutyCycle(0.0);
  }

  // Command factory methods

  public Command setDutyCycleCommand(double value) {
    return this.setDutyCycleCommand(() -> value);
  }

  public Command setDutyCycleCommand(DoubleSupplier valueSup) {
    return this.runEnd(() -> io.setDutyCycle(valueSup.getAsDouble()), () -> io.setDutyCycle(0.0));
  }

  public Command setVelocityCommand(double rpm) {
    return this.runEnd(() -> setVelocity(rpm), () -> setVelocity(0.0));
  }

  // periodic

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("FlywheelKicker", inputs);

    if (controlState == ControlState.SUPERSTRUCTURE) {
      updateState();
      applyState();
    }
    Logger.recordOutput("Subsystems/FlywheelKicker/WantedState", wantedState);
    Logger.recordOutput("Subsystems/FlywheelKicker/CurrentState", currentState);
    Logger.recordOutput("Subsystems/FlywheelKicker/PreviousState", previousState);
    Logger.recordOutput("Subsystems/FlywheelKicker/ControlState", controlState);
  }
}
